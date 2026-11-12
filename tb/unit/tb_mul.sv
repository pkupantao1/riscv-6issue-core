//==============================================================================
// tb_mul.sv - o6_mul (64×64 乘法，分段进位保存 + 4 拍) 单元 TB
//
// 每拍发一条 (valid 随机 ~80 %)：op ∈ MUL / MULH / MULHSU / MULHU，is_w 随机 (只对 MUL 有意义)；
//   操作数取随机值或边界值 (0、±1、最小负数、全 1、单个位、段边界附近的位)。
// 参考模型 (与实现无关)：130 位有符号乘，按 op 取高 / 低 64 位 (MULW 取低 32 位符号扩展)；
//   在 EX4 (发出后第 3 拍) 的 valid_o 上核对 y_o。o6_mul 内部还有"切分积 == 整体积"的断言。
// 用 VDEFS="+define+O6_DW_SIM" 让 DW02_multp / DW02_tree 走 DesignWare 仿真模型 (真的有两个进位保存向量)；
//   段宽用 +define+O6_MUL_SEGW=N 改。+vec=N (默认 200000)
//==============================================================================
`default_nettype none
module tb_mul;
  import o6_pkg::*;

  logic        clk = 0, rst_n = 0;
  always #1 clk = ~clk;

  logic        valid;
  uop_t        uop;
  logic [63:0] a, b;
  logic        vo;
  uop_t        uo;
  logic [63:0] y;
  logic        s1v;
  uop_t        s1u;
  br_update_t  bu;

  o6_mul dut (.clk_i(clk), .rst_ni(rst_n), .flush_i(1'b0), .br_upd_i(bu), .valid_i(valid), .uop_i(uop),
              .a_i(a), .b_i(b), .valid_o(vo), .uop_o(uo), .y_o(y), .s1_v_o(s1v), .s1_u_o(s1u));

  function automatic logic [63:0] ref_mul(input logic [4:0] op, input logic is_w, input logic [63:0] x, input logic [63:0] z);
    logic signed [129:0] xa, zb, p;
    xa = (op == MUL_MULH || op == MUL_MULHSU) ? 130'($signed(x)) : 130'(x);
    zb = (op == MUL_MULH)                     ? 130'($signed(z)) : 130'(z);
    p  = xa * zb;
    if (op == MUL_MUL) return is_w ? {{32{p[31]}}, p[31:0]} : p[63:0];
    return p[127:64];
  endfunction

  function automatic logic [63:0] pick();
    int k;
    k = $urandom_range(0, 11);
    case (k)
      0: return 64'd0;
      1: return 64'd1;
      2: return '1;                              // -1
      3: return 64'h8000_0000_0000_0000;         // 最小负数
      4: return 64'h7FFF_FFFF_FFFF_FFFF;
      5: return 64'd1 << $urandom_range(0, 63);  // 单个位
      6: return ~(64'd1 << $urandom_range(0, 63));
      7: return {$urandom, $urandom} >> $urandom_range(0, 63);
      default: return {$urandom, $urandom};
    endcase
  endfunction

  // 在途参考值 (EX1 发出 -> EX4 valid_o，第 3 拍)
  typedef struct packed { logic v; logic [63:0] y; logic [4:0] op; logic w; logic [63:0] a, b; } exp_t;
  exp_t q [3];
  int errs = 0, nchk = 0, nvec;
  int nop [4];

  initial begin
    if (!$value$plusargs("vec=%d", nvec)) nvec = 200000;
    bu = '0; valid = 0; uop = '0; a = '0; b = '0;
    for (int i = 0; i < 3; i++) q[i] = '0;
    repeat (3) @(posedge clk);
    rst_n = 1;
    for (int n = 0; n < nvec + 3; n++) begin
      @(negedge clk);
      // EX4 核对：本拍 valid_o 对应 3 拍前发出的那条
      if (q[2].v) begin
        nchk++;
        if (!vo) begin errs++; if (errs < 10) $display("错误：第 %0d 拍应出结果但 valid_o = 0", n); end
        else if (y !== q[2].y) begin
          errs++;
          if (errs < 10) $display("错误：op=%0d w=%0d a=%h b=%h  得 %h  应 %h", q[2].op, q[2].w, q[2].a, q[2].b, y, q[2].y);
        end
      end else if (vo) begin errs++; if (errs < 10) $display("错误：第 %0d 拍多出 valid_o", n); end
      q[2] = q[1]; q[1] = q[0];
      // 发下一条
      valid = (n < nvec) && ($urandom_range(0, 9) < 8);
      uop = '0;
      uop.fu = FU_MUL;
      uop.op = 5'($urandom_range(0, 3));
      uop.is_w = (uop.op == MUL_MUL) && $urandom_range(0, 3) == 0;
      a = pick(); b = pick();
      q[0] = '{v: valid, y: ref_mul(uop.op, uop.is_w, a, b), op: uop.op, w: uop.is_w, a: a, b: b};
      if (valid) nop[uop.op]++;
    end
    $display("核对 %0d 条 (MUL %0d / MULH %0d / MULHSU %0d / MULHU %0d)，错误 %0d", nchk, nop[0], nop[1], nop[2], nop[3], errs);
    if (errs == 0 && nchk > nvec / 2) $display("=== UNIT PASS ===");
    else                              $display("=== UNIT FAIL ===");
    $finish;
  end
endmodule
