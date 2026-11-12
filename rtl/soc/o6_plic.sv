//==============================================================================
// o6_plic.sv - 平台级中断控制器 (SiFive PLIC 布局；NSRC 个源，2 个上下文：0 = M，1 = S)
//   0x000000 + 4*s         优先级 (3 位)
//   0x001000               pending 位图 (只读)
//   0x002000 + 0x80*c      使能位图
//   0x200000 + 0x1000*c    阈值
//   0x200004 + 0x1000*c    认领 (读) / 完成 (写)
// 电平触发：源保持为 1 时，完成后若仍为 1 会再次挂起。认领中的源不会再次挂起，直到完成。
//==============================================================================
`default_nettype none
module o6_plic #(
  parameter int NSRC = 31      // 源 1..NSRC (源 0 保留)
) (
  input  wire          clk_i,
  input  wire          rst_ni,
  input  wire  [NSRC:1] src_i,
  input  wire          req_i,
  input  wire          we_i,
  input  wire  [25:0]  addr_i,
  input  wire  [63:0]  wdata_i,
  input  wire  [7:0]   wmask_i,
  output logic         ack_o,
  output logic [63:0]  rdata_o,
  output logic [1:0]   eip_o         // {SEIP, MEIP}
);
  logic [2:0]      prio   [NSRC+1];
  logic [NSRC:0]   pend, claimed;
  logic [NSRC:0]   en     [2];
  logic [2:0]      thr    [2];

  // 每个上下文最高优先级的挂起使能源
  logic [$clog2(NSRC+1)-1:0] best [2];
  logic [2:0]                bestp [2];
  logic [1:0]                eip_c;
  // 中断输出寄存一拍 (31 源优先级比较不进核里的提交/LSU 路径；多一拍中断延迟)
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) eip_o <= '0;
    else         eip_o <= eip_c;
  // 二叉树比较 (5 级)，同优先级取编号小的 (左子树)；原来是 31 个源逐个比较的串行链
  localparam int NL = 32;   // 叶子数 (>= NSRC + 1)
  always_comb begin
    for (int c = 0; c < 2; c++) begin
      logic [2:0] lp [2*NL];
      logic [$clog2(NSRC+1)-1:0] li [2*NL];
      for (int s = 0; s < NL; s++) begin
        lp[NL+s] = (s >= 1 && s <= NSRC && pend[s] && en[c][s]) ? prio[s] : 3'd0;
        li[NL+s] = s[$clog2(NSRC+1)-1:0];
      end
      for (int n = NL - 1; n >= 1; n--) begin
        if (lp[2*n] >= lp[2*n+1]) begin lp[n] = lp[2*n];   li[n] = li[2*n];   end
        else                      begin lp[n] = lp[2*n+1]; li[n] = li[2*n+1]; end
      end
      best[c]  = (lp[1] != 3'd0) ? li[1] : '0;
      bestp[c] = lp[1];
      eip_c[c] = (bestp[c] > thr[c]);
    end
  end

  // 32 位寄存器访问：双字对齐的地址里取高 / 低半字
  wire [25:0] a32  = {addr_i[25:2], 2'b00};
  wire        hi   = addr_i[2];
  wire [31:0] wd32 = hi ? wdata_i[63:32] : wdata_i[31:0];
  wire        wen  = we_i && (hi ? |wmask_i[7:4] : |wmask_i[3:0]);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      pend <= '0; claimed <= '0; ack_o <= 1'b0; rdata_o <= '0;
      for (int s = 0; s <= NSRC; s++) prio[s] <= '0;
      for (int c = 0; c < 2; c++) begin en[c] <= '0; thr[c] <= '0; end
    end else begin
      logic [31:0] r;
      ack_o <= req_i && !ack_o;
      // 挂起：电平触发，认领中的不再挂起
      for (int s = 1; s <= NSRC; s++)
        if (src_i[s] && !claimed[s]) pend[s] <= 1'b1;
      r = '0;
      if (req_i && !ack_o) begin
        if (a32 < 26'h001000) begin
          if (a32[11:2] <= NSRC) begin
            r = {29'b0, prio[a32[11:2]]};
            if (wen) prio[a32[11:2]] <= wd32[2:0];
          end
        end else if (a32 == 26'h001000) begin
          r = 32'(pend);
        end else if (a32 == 26'h002000 || a32 == 26'h002080) begin
          r = 32'(en[a32[7]]);
          if (wen) en[a32[7]] <= wd32[NSRC:0] & ~(NSRC+1)'(1);
        end else if (a32 == 26'h200000 || a32 == 26'h201000) begin
          r = {29'b0, thr[a32[12]]};
          if (wen) thr[a32[12]] <= wd32[2:0];
        end else if (a32 == 26'h200004 || a32 == 26'h201004) begin
          if (we_i) begin
            // 完成
            if (wd32 != 0 && wd32 <= NSRC) claimed[wd32[$clog2(NSRC+1)-1:0]] <= 1'b0;
          end else begin
            // 认领
            r = 32'(best[a32[12]]);
            if (best[a32[12]] != 0) begin
              pend[best[a32[12]]]    <= 1'b0;
              claimed[best[a32[12]]] <= 1'b1;
            end
          end
        end
        rdata_o <= hi ? {r, 32'b0} : {32'b0, r};
      end
    end
  end
endmodule
