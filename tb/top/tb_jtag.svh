// tb_jtag.svh - JTAG 驱动与调试模块自检 (tb_top 内 include，SoC 模式，+dbgtest +nocosim)
// 被测程序：verif/dbg/dbg_loop.S (a0 不断加 1 并写到 counter)
// 流程：IDCODE / DTMCS -> dmactive -> 停机 -> 抽象命令读写 GPR/CSR -> progbuf -> 单步 -> 恢复 -> 再停机
localparam int TCK_HALF = 10;     // TCK 半周期 (ns)；核时钟 1 ns

task automatic jtag_clk(input bit tms, input bit tdi, output bit tdo);
  jtag_tms = tms; jtag_tdi = tdi;
  #(TCK_HALF) jtag_tck = 1'b1;
  tdo = jtag_tdo;
  #(TCK_HALF) jtag_tck = 1'b0;
endtask

task automatic jtag_reset();
  bit d;
  jtag_trst_n = 1'b0; #(4 * TCK_HALF); jtag_trst_n = 1'b1;
  repeat (6) jtag_clk(1, 0, d);
  jtag_clk(0, 0, d);                         // -> Run-Test/Idle
endtask

task automatic jtag_ir(input logic [4:0] ir);
  bit d;
  jtag_clk(1, 0, d); jtag_clk(1, 0, d); jtag_clk(0, 0, d); jtag_clk(0, 0, d);   // Select-DR, Select-IR, Capture-IR, Shift-IR
  for (int i = 0; i < 5; i++) jtag_clk(i == 4, ir[i], d);                      // 最后一位同时进 Exit1-IR
  jtag_clk(1, 0, d); jtag_clk(0, 0, d);                                        // Update-IR, RTI
endtask

task automatic jtag_dr(input int n, input logic [63:0] din, output logic [63:0] dout);
  bit d;
  dout = '0;
  jtag_clk(1, 0, d); jtag_clk(0, 0, d); jtag_clk(0, 0, d);                     // Select-DR, Capture-DR, Shift-DR
  for (int i = 0; i < n; i++) begin
    jtag_clk(i == n - 1, din[i], d);
    dout[i] = d;
  end
  jtag_clk(1, 0, d); jtag_clk(0, 0, d);                                        // Update-DR, RTI
  repeat (4) jtag_clk(0, 0, d);                                                // 空闲几拍让 DMI 完成
endtask

int dbg_err = 0;
task automatic dbg_check(input bit ok, input string what);
  if (!ok) begin dbg_err++; $display("[dbg] 失败：%s", what); end
  else $display("[dbg] 通过：%s", what);
endtask

// DMI 访问：op 1 读 / 2 写；忙则 dmireset 后重试
task automatic dmi(input logic [6:0] addr, input logic [31:0] wdata, input logic [1:0] op, output logic [31:0] rdata);
  logic [63:0] o;
  for (int tries = 0; tries < 20; tries++) begin
    jtag_ir(5'h11);
    jtag_dr(41, {23'b0, addr, wdata, op}, o);          // 发请求 (移出的是上一次结果，忽略)
    jtag_dr(41, {23'b0, addr, 32'b0, 2'b00}, o);       // nop：取本次结果
    if (o[1:0] == 2'd0) begin rdata = o[33:2]; return; end
    jtag_ir(5'h10);
    jtag_dr(32, 64'h1_0000, o);                        // dtmcs.dmireset
  end
  $display("[dbg] DMI 一直忙/失败 addr=%h", addr);
  dbg_err++;
  rdata = '0;
endtask

task automatic dmi_w(input logic [6:0] a, input logic [31:0] v);
  logic [31:0] r;
  dmi(a, v, 2'd2, r);
endtask
task automatic dmi_r(input logic [6:0] a, output logic [31:0] v);
  dmi(a, 32'b0, 2'd1, v);
endtask

task automatic wait_abst(output logic [2:0] err);
  logic [31:0] v;
  for (int i = 0; i < 50; i++) begin
    dmi_r(7'h16, v);
    if (!v[12]) begin err = v[10:8]; return; end
  end
  err = 3'd7;
endtask

// 抽象命令：读寄存器 (64 位)
task automatic reg_rd(input logic [15:0] regno, output logic [63:0] val, output logic [2:0] err);
  logic [31:0] lo, hi;
  dmi_w(7'h17, (32'd3 << 20) | (32'd1 << 17) | regno);
  wait_abst(err);
  dmi_r(7'h04, lo); dmi_r(7'h05, hi);
  val = {hi, lo};
endtask
task automatic reg_wr(input logic [15:0] regno, input logic [63:0] val, output logic [2:0] err);
  dmi_w(7'h04, val[31:0]); dmi_w(7'h05, val[63:32]);
  dmi_w(7'h17, (32'd3 << 20) | (32'd1 << 17) | (32'd1 << 16) | regno);
  wait_abst(err);
endtask

task automatic wait_halted(output bit ok);
  logic [31:0] v;
  ok = 1'b0;
  for (int i = 0; i < 100; i++) begin
    dmi_r(7'h11, v);
    if (v[9]) begin ok = 1'b1; return; end            // allhalted
  end
endtask

initial if ($test$plusargs("dbgtest")) begin
  logic [63:0] o, a0, a0b, dpc0, dpc1, dcsr;
  logic [31:0] v;
  logic [2:0]  e;
  bit          ok;
  wait (rst_n);
  repeat (2000) @(posedge clk);                       // 先让程序跑起来
  jtag_reset();
  jtag_ir(5'h01); jtag_dr(32, 64'h0, o);
  dbg_check(o[0] == 1'b1 && o[31:0] != 32'h0, $sformatf("IDCODE = %08h", o[31:0]));
  jtag_ir(5'h10); jtag_dr(32, 64'h0, o);
  dbg_check(o[3:0] == 4'd1 && o[9:4] == 6'd7, $sformatf("DTMCS = %08h (版本 0.13, abits 7)", o[31:0]));

  dmi_w(7'h10, 32'h1);                                // dmactive
  dmi_r(7'h11, v);
  dbg_check(v[3:0] == 4'd2 && v[11] && !v[9], $sformatf("dmstatus = %08h (0.13，运行中)", v));
  dmi_r(7'h16, v);
  dbg_check(v[28:24] == 5'd8 && v[3:0] == 4'd2, $sformatf("abstractcs = %08h (progbuf 8, data 2)", v));

  // 停机
  dmi_w(7'h10, 32'h8000_0001);
  wait_halted(ok);
  dbg_check(ok, "haltreq 后 allhalted");
  dmi_w(7'h10, 32'h1);

  reg_rd(16'h07b0, dcsr, e);
  dbg_check(e == 0 && dcsr[8:6] == 3'd3 && dcsr[1:0] == 2'd3 && dcsr[31:28] == 4'd4,
            $sformatf("dcsr = %08h (cause haltreq, prv M, xdebugver 4)", dcsr[31:0]));
  reg_rd(16'h07b1, dpc0, e);
  dbg_check(e == 0 && dpc0[63:12] == 52'h80000, $sformatf("dpc = %h (在程序里)", dpc0));
  reg_rd(16'h100a, a0, e);
  dbg_check(e == 0 && a0 != 0, $sformatf("读 a0 = %0d (程序在计数)", a0));

  // 写 a0 再读回
  reg_wr(16'h100a, 64'h1234_5678_9abc_def0, e);
  reg_rd(16'h100a, o, e);
  dbg_check(e == 0 && o == 64'h1234_5678_9abc_def0, $sformatf("写/读 a0 = %h", o));

  // 读 misa、写 mscratch (CSR 通路)
  reg_rd(16'h0301, o, e);
  dbg_check(e == 0 && o[63:62] == 2'b10 && o[8] /*I*/, $sformatf("misa = %h", o));
  reg_wr(16'h0340, 64'h5555_aaaa_0000_ffff, e);
  reg_rd(16'h0340, o, e);
  dbg_check(e == 0 && o == 64'h5555_aaaa_0000_ffff, "写/读 mscratch");

  // progbuf：addi a0, a0, 1 (postexec，无 transfer)
  dmi_w(7'h20, 32'h00150513);
  dmi_w(7'h17, (32'd1 << 18));
  wait_abst(e);
  reg_rd(16'h100a, o, e);
  dbg_check(e == 0 && o == 64'h1234_5678_9abc_def1, $sformatf("progbuf 执行后 a0 = %h", o));

  // 不支持的命令 -> cmderr = 2
  dmi_w(7'h17, 32'h0100_0000);
  dmi_r(7'h16, v);
  dbg_check(v[10:8] == 3'd2, "不支持的命令 cmderr = 2");
  dmi_w(7'h16, 32'h0000_0700);                       // 清 cmderr

  // 访问不存在的 CSR -> 异常 -> cmderr = 3
  reg_rd(16'h0fff, o, e);
  dbg_check(e == 3'd3, $sformatf("非法 CSR 读 cmderr = %0d (期望 3)", e));
  dmi_w(7'h16, 32'h0000_0700);

  // 单步：dcsr.step = 1，resume -> 一条后再停
  reg_rd(16'h07b0, dcsr, e);
  reg_wr(16'h07b0, dcsr | 64'h4, e);
  dmi_w(7'h10, 32'h4000_0001);                        // resumereq
  wait_halted(ok);
  dmi_w(7'h10, 32'h1);
  reg_rd(16'h07b0, dcsr, e);
  reg_rd(16'h07b1, dpc1, e);
  dbg_check(ok && dcsr[8:6] == 3'd4 && dpc1 != dpc0,
            $sformatf("单步：cause = %0d，dpc %h -> %h", dcsr[8:6], dpc0, dpc1));

  // 关单步，恢复运行，过一会再停：a0 应继续增长
  reg_rd(16'h100a, a0, e);
  reg_wr(16'h07b0, dcsr & ~64'h4, e);
  dmi_w(7'h10, 32'h4000_0001);
  dmi_r(7'h11, v);
  for (int i = 0; i < 20 && !v[17]; i++) dmi_r(7'h11, v);
  dbg_check(v[17] && v[11], $sformatf("resumeack + allrunning (dmstatus = %08h)", v));
  repeat (3000) @(posedge clk);
  dmi_w(7'h10, 32'h8000_0001);
  wait_halted(ok);
  dmi_w(7'h10, 32'h1);
  reg_rd(16'h100a, a0b, e);
  dbg_check(ok && a0b > a0, $sformatf("恢复后继续运行：a0 %h -> %h", a0, a0b));

  if (dbg_err == 0) finish(1'b1, "调试模块自检");
  else              finish(1'b0, $sformatf("调试模块自检 %0d 项失败", dbg_err));
end

// ---------------------------------------------------------------------------
// MBIST 自检 (+mbisttest)：ndmreset 让系统停在复位，经 JTAG MBIST TDR 启动，等 done，查 fail；
// 第二遍对 D$ bank0 输出强制一位为 0，期望 fail = 1；最后放开复位，程序应能正常跑起来
// ---------------------------------------------------------------------------
task automatic mbist_run(output bit done, output bit fail);
  logic [63:0] o;
  jtag_ir(5'h12); jtag_dr(8, 64'h1, o);                // start
  done = 1'b0;
  for (int i = 0; i < 400 && !done; i++) begin
    repeat (200) @(posedge clk);
    jtag_dr(8, 64'h1, o);
    done = o[1]; fail = o[2];
  end
  jtag_dr(8, 64'h0, o);                                // 清 start (引擎同步复位)
endtask

initial if ($test$plusargs("mbisttest")) begin
  logic [31:0] v;
  logic [63:0] a0, o;
  logic [2:0]  e;
  bit          done, fail, ok;
  int          t0;
  wait (rst_n);
  repeat (500) @(posedge clk);
  jtag_reset();
  dmi_w(7'h10, 32'h3);                                 // dmactive + ndmreset：系统停在复位
  t0 = cyc;
  mbist_run(done, fail);
  dbg_check(done && !fail, $sformatf("MBIST 无故障：done=%0d fail=%0d (%0d 拍)", done, fail, cyc - t0));
  force u_soc.u_core.u_dc.g_bank[0].u_bank.rdata_o[5] = 1'b0;
  mbist_run(done, fail);
  release u_soc.u_core.u_dc.g_bank[0].u_bank.rdata_o[5];
  dbg_check(done && fail, $sformatf("MBIST 注入固定 0 故障：done=%0d fail=%0d", done, fail));
  dmi_w(7'h10, 32'h1);                                 // 放开 ndmreset
  repeat (5000) @(posedge clk);
  dmi_w(7'h10, 32'h8000_0001);
  wait_halted(ok);
  dmi_w(7'h10, 32'h1);
  reg_rd(16'h100a, a0, e);
  dbg_check(ok && e == 0 && a0 > 0, $sformatf("MBIST 后复位重启，程序在跑：a0 = %0d", a0));
  if (dbg_err == 0) finish(1'b1, "MBIST 自检");
  else              finish(1'b0, $sformatf("MBIST 自检 %0d 项失败", dbg_err));
end
