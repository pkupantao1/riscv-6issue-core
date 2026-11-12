//==============================================================================
// o6_div.sv - 64 位整数除法 (迭代，基 2 恢复余数，每拍 1 位；W 型 32 拍)
//
// RISC-V 特殊情况：除零 -> 商全 1、余数 = 被除数；
//                  溢出 (最小负数 / -1) -> 商 = 被除数、余数 0。
// 忙时 fu_ok 屏蔽 FU_DIV 发射。结果保持到 out_ready (长延迟写口 W6 仲裁) 为止。
// 基 4 SRT 是 v2 性能项。
//==============================================================================
`default_nettype none
module o6_div
  import o6_pkg::*;
(
  input  wire              clk_i,
  input  wire              rst_ni,
  input  wire              flush_i,
  input  var br_update_t   br_upd_i,
  input  wire              valid_i,
  input  var uop_t         uop_i,
  input  wire  [63:0]      a_i,
  input  wire  [63:0]      b_i,
  output wire              busy_o,
  output wire              valid_o,
  output wire              done_o,
  output uop_t             uop_o,
  output logic [63:0]      y_o,
  input  wire              out_ready_i
);
  typedef enum logic [1:0] {S_IDLE, S_RUN, S_DONE} st_e;
  st_e         st;
  uop_t        u;
  logic [6:0]  cnt;
  logic [63:0] q, r, d;
  logic        neg_q, neg_r, special;
  logic [63:0] spec_res;

  wire is_rem = (uop_i.op == DIV_REM) || (uop_i.op == DIV_REMU);
  wire sgn    = (uop_i.op == DIV_DIV) || (uop_i.op == DIV_REM);
  wire [63:0] ax = uop_i.is_w ? (sgn ? {{32{a_i[31]}}, a_i[31:0]} : {32'b0, a_i[31:0]}) : a_i;
  wire [63:0] bx = uop_i.is_w ? (sgn ? {{32{b_i[31]}}, b_i[31:0]} : {32'b0, b_i[31:0]}) : b_i;
  wire        an = sgn && ax[63];
  wire        bn = sgn && bx[63];
  wire [63:0] aa = an ? -ax : ax;
  wire [63:0] ba = bn ? -bx : bx;
  wire        div0 = (bx == 64'd0);
  wire        ovf  = sgn && (bx == 64'hFFFF_FFFF_FFFF_FFFF) &&
                     (uop_i.is_w ? (ax == 64'hFFFF_FFFF_8000_0000) : (ax == 64'h8000_0000_0000_0000));

  wire [64:0] r_sh  = {r, q[63]};
  wire [64:0] r_sub = r_sh - {1'b0, d};
  wire        killed = br_killed(u.br_mask, br_upd_i);
  wire        start  = (st == S_IDLE) && valid_i && !br_killed(uop_i.br_mask, br_upd_i);

  assign busy_o  = (st != S_IDLE);
  assign valid_o = (st == S_DONE) && !killed;
  assign done_o  = (st == S_DONE);           // 结果在手 (不看本拍分支杀)：给写回口占用 / 选择用，有效位仍用 valid_o

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st <= S_IDLE;
    end else if (flush_i) begin
      st <= S_IDLE;
    end else begin
      unique case (st)
        S_IDLE: if (start) st <= S_RUN;
        S_RUN:  if (killed) st <= S_IDLE;
                else if (special || cnt == 7'd0) st <= S_DONE;
        S_DONE: if (killed || out_ready_i) st <= S_IDLE;
        default: st <= S_IDLE;
      endcase
    end
  end

  always_ff @(posedge clk_i) begin
    if (st == S_IDLE) begin
      uop_t n;
      n = uop_i; n.br_mask = uop_i.br_mask & ~br_upd_i.resolve_mask;
      u        <= n;
      q        <= uop_i.is_w ? {aa[31:0], 32'b0} : aa;   // W 型左对齐，只迭代 32 次
      r        <= '0;
      d        <= ba;
      cnt      <= uop_i.is_w ? 7'd32 : 7'd64;
      neg_q    <= an ^ bn;
      neg_r    <= an;
      special  <= div0 || ovf;
      spec_res <= div0 ? (is_rem ? ax : 64'hFFFF_FFFF_FFFF_FFFF) : (is_rem ? 64'd0 : ax);
    end else begin
      uop_t n;
      n = u; n.br_mask = u.br_mask & ~br_upd_i.resolve_mask;
      u <= n;
      if (st == S_RUN) begin
        if (!special && cnt != 7'd0) begin
          if (!r_sub[64]) begin r <= r_sub[63:0]; q <= {q[62:0], 1'b1}; end
          else            begin r <= r_sh[63:0];  q <= {q[62:0], 1'b0}; end
          cnt <= cnt - 1'b1;
        end else begin
          logic [63:0] res;
          if (special) res = spec_res;
          else if (u.op == DIV_REM || u.op == DIV_REMU) res = neg_r ? -r : r;
          else res = neg_q ? -q : q;
          y_o <= u.is_w ? {{32{res[31]}}, res[31:0]} : res;
        end
      end
    end
  end

  always_comb begin
    uop_o = u;
    uop_o.br_mask = u.br_mask & ~br_upd_i.resolve_mask;
  end
endmodule
