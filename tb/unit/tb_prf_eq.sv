//==============================================================================
// tb_prf_eq.sv - 2nm PRF 切片结构 (syn2n/prf_slice{,3,4,5}_wrap.sv) 与平铺参考 (prf_bank_wrap) 的锁步等价 (docs/13 §10 第 2 步)
//   同一串随机激励喂两份，每拍逐读口比 rdata (判据：10⁶ 拍无差异)
//   约束 = 真实核里的用法：同拍各写口写不同的项 (pdst 唯一)；读口任意 (含读本拍正在写的项)
//   先把 N 项全写一遍再开始比 (未写项两边都是 X)
//   编译时 +define+DUT=prf_sliceK_wrap 选被比的结构；+cycles=N (默认 1e6)
//==============================================================================
`default_nettype none
`timescale 1ns/1ps
module tb_prf_eq;
  import o6_pkg::*;
  localparam int N = 128, NR = 12, NW = 5, AW = $clog2(N);
  logic clk = 0, rst_n = 0;
  always #5 clk = ~clk;

  logic [AW-1:0]  ra [NR], wa [NW];
  preg_t [NR-1:0] ra_p;
  preg_t [NW-1:0] wa_p;
  logic [NW-1:0]  wen;
  logic [63:0]    wd [NW];
  logic [63:0]    rd_ref [NR], rd_dut [NR];

  always_comb begin
    for (int r = 0; r < NR; r++) ra_p[r] = preg_t'(ra[r]);
    for (int w = 0; w < NW; w++) wa_p[w] = preg_t'(wa[w]);
  end
  prf_bank_wrap #(.N(N), .NR(NR), .NW(NW)) u_ref (.clk_i(clk), .rst_ni(rst_n), .raddr_i(ra_p), .rdata_o(rd_ref),
                                                  .wen_i(wen), .waddr_i(wa_p), .wdata_i(wd));
  `DUT #(.N(N), .NR(NR), .NW(NW)) u_dut (.clk_i(clk), .rst_ni(rst_n), .raddr_i(ra), .rdata_o(rd_dut),
                                         .wen_i(wen), .waddr_i(wa), .wdata_i(wd));

  int ncyc, errs, nraw;
  initial begin
    if (!$value$plusargs("cycles=%d", ncyc)) ncyc = 1000000;
    errs = 0; nraw = 0;
    wen = '0;
    for (int r = 0; r < NR; r++) ra[r] = '0;
    for (int w = 0; w < NW; w++) begin wa[w] = '0; wd[w] = '0; end
    repeat (3) @(posedge clk);
    rst_n = 1;
    // 先全写一遍
    for (int e = 0; e < N; e += NW) begin
      @(negedge clk);
      for (int w = 0; w < NW; w++) begin wen[w] = (e + w) < N; wa[w] = AW'(e + w); wd[w] = {$urandom, $urandom}; end
    end
    @(negedge clk); wen = '0;
    repeat (4) @(negedge clk);
    for (int c = 0; c < ncyc; c++) begin
      @(negedge clk);
      // 比上一拍发出的读 (两份的读出延迟相同；结果在 negedge 稳定)
      for (int r = 0; r < NR; r++)
        if (rd_dut[r] !== rd_ref[r]) begin
          errs++;
          if (errs <= 10) $display("[%0t] 读口 %0d 不一致：参考 %h  切片 %h", $time, r, rd_ref[r], rd_dut[r]);
        end
      // 新激励：写口地址互不相同
      for (int w = 0; w < NW; w++) begin
        bit dup;
        wen[w] = $urandom_range(99) < 60;
        do begin
          dup = 0;
          wa[w] = AW'($urandom_range(N - 1));
          for (int v = 0; v < w; v++) if (wen[v] && wa[v] == wa[w]) dup = 1;
        end while (dup);
        wd[w] = {$urandom, $urandom};
      end
      for (int r = 0; r < NR; r++) begin
        ra[r] = AW'($urandom_range(N - 1));
        // 1/4 的读口专门读本拍正在写的项 (读写同拍)
        if ($urandom_range(3) == 0 && wen != 0) begin
          for (int w = 0; w < NW; w++) if (wen[w]) begin ra[r] = wa[w]; nraw++; break; end
        end
      end
    end
    if (errs == 0) $display("=== UNIT PASS === prf_eq %s %0d 拍 (读写同拍 %0d 次) 无差异", `"`DUT`", ncyc, nraw);
    else           $display("=== UNIT FAIL === prf_eq %s 不一致 %0d", `"`DUT`", errs);
    $finish;
  end
endmodule
