//==============================================================================
// tb_vex.sv - o6_vex (向量执行器 VL，V1.1) 单元 TB
//   每拍送一拍 (随机：整数 / FP32 加乘 / FP64 乘加，各 SEW、.vv / .vx / .vi / .vf、带不带掩码、体内元素数随机)，
//   按送入顺序在固定延迟 LAT 后核对：
//     - 整数：逐元素参考模型 (独立写的，与 o6_vex 内的 fx_calc 无关)
//     - FP32 vfadd / vfmul：shortreal 运算 (单次舍入，RNE)
//     - FP64 vfmacc：操作数取小整数值，a × b + c 没有舍入，real 算即为精确结果
//     - 被掩掉 / 尾部的元素保持 vd 旧值
//   通过：=== UNIT PASS ===
//==============================================================================
`timescale 1ns/1ps
module tb_vex;
  import o6_pkg::*;
  localparam int TW  = 16;
  localparam int NB  = 3000;           // 拍数

  logic clk = 1'b0, rst_n = 1'b0;
  always #1 clk = ~clk;

  logic                in_v;
  logic [31:0]         in_raw;
  logic [1:0]          in_sew;
  logic [7:0]          in_nel;
  logic [CL_B-1:0]     in_m;
  logic [VLEN-1:0]     in_vs2, in_vs1, in_vd;
  logic [63:0]         in_xs;
  logic [2:0]          in_frm;
  logic [TW-1:0]       in_tag;
  logic                out_v;
  logic [VLEN-1:0]     out_d;
  logic [TW-1:0]       out_tag;
  logic [4:0]          out_ff;

  o6_vex #(.TW(TW)) dut (
    .clk_i(clk), .rst_ni(rst_n),
    .in_v_i(in_v), .in_raw_i(in_raw), .in_sew_i(in_sew), .in_nel_i(in_nel), .in_m_i(in_m),
    .in_vs2_i(in_vs2), .in_vs1_i(in_vs1), .in_vd_i(in_vd), .in_xs_i(in_xs), .in_frm_i(in_frm), .in_tag_i(in_tag),
    .out_v_o(out_v), .out_d_o(out_d), .out_tag_o(out_tag), .out_ff_o(out_ff)
  );

  // 期望结果队列 (按送入顺序)
  logic [VLEN-1:0] exp_q [$];
  logic [TW-1:0]   tag_q [$];
  int              t_in_q [$];
  int              cyc = 0, n_ok = 0, n_err = 0, n_int = 0, n_f32 = 0, n_f64 = 0;
  always @(posedge clk) cyc <= cyc + 1;

  // 编码：OP-V，funct6 / vm / vs2 / vs1(rs1) / funct3 / vd，opcode 1010111
  function automatic logic [31:0] enc(input logic [5:0] f6, input logic vm, input logic [2:0] f3, input logic [4:0] rs1);
    return {f6, vm, 5'd8, rs1, f3, 5'd16, 7'b1010111};
  endfunction

  function automatic logic [63:0] iref(input logic [5:0] f6, input logic [63:0] a, input logic [63:0] b, input int w);
    logic [63:0] m, r;
    longint sa, sb;
    int sh;
    m  = (w == 64) ? '1 : ((64'd1 << w) - 1);
    a = a & m; b = b & m;
    sa = (w == 64) ? longint'(a) : longint'(a << (64 - w)) >>> (64 - w);
    sb = (w == 64) ? longint'(b) : longint'(b << (64 - w)) >>> (64 - w);
    sh = int'(b % w);
    case (f6)
      6'b000000: r = a + b;
      6'b000010: r = a - b;
      6'b000011: r = b - a;
      6'b000100: r = (a < b) ? a : b;
      6'b000101: r = (sa < sb) ? a : b;
      6'b000110: r = (a > b) ? a : b;
      6'b000111: r = (sa > sb) ? a : b;
      6'b001001: r = a & b;
      6'b001010: r = a | b;
      6'b001011: r = a ^ b;
      6'b100101: r = a << sh;
      6'b101000: r = a >> sh;
      6'b101001: r = 64'(sa >>> sh);
      default:   r = b;
    endcase
    return r & m;
  endfunction

  task automatic send_one(input int i);
    int k, w, ne, ew;
    logic [5:0] f6;
    logic [2:0] f3;
    logic       vm;
    logic [4:0] rs1;
    logic [VLEN-1:0] e;
    k = $urandom_range(0, 9);
    for (int j = 0; j < VLEN / 32; j++) begin
      in_vs2[32*j +: 32] = $urandom; in_vs1[32*j +: 32] = $urandom; in_vd[32*j +: 32] = $urandom;
    end
    for (int j = 0; j < CL_B / 32; j++) in_m[32*j +: 32] = $urandom;
    in_xs  = {$urandom, $urandom};
    in_frm = 3'd0;
    vm     = $urandom_range(0, 1);
    rs1    = $urandom;
    in_tag = TW'(i);
    if (k < 6) begin
      // 整数
      logic [5:0] ops [14] = '{6'b000000, 6'b000010, 6'b000011, 6'b000100, 6'b000101, 6'b000110, 6'b000111,
                                6'b001001, 6'b001010, 6'b001011, 6'b100101, 6'b101000, 6'b101001, 6'b010111};
      f6 = ops[$urandom_range(0, 13)];
      in_sew = $urandom_range(0, 3);
      f3 = (k < 2) ? 3'b000 : (k < 4) ? 3'b100 : 3'b011;
      if (f6 == 6'b000011 && f3 == 3'b000) f3 = 3'b100;          // vrsub 没有 .vv
      if (f6 == 6'b010111 && vm) rs1 = (f3 == 3'b000) ? 5'd0 : rs1;   // vmv.v 的 vs2 必须是 0 (这里只算 vs2 字段，不影响)
      in_raw = enc(f6, vm, f3, rs1);
      w = 8 << in_sew; ne = VLEN / w;
      in_nel = $urandom_range(0, 1) ? 8'(ne) : 8'($urandom_range(0, ne));
      e = in_vd;
      for (int el = 0; el < ne; el++)
        if (el < int'(in_nel)) begin
          logic [63:0] a, b, x;
          logic shf, mrg, act;
          a = 64'(in_vs2 >> (w * el));
          b = 64'(in_vs1 >> (w * el));
          shf = (f6 == 6'b100101) || (f6 == 6'b101000) || (f6 == 6'b101001);
          mrg = (f6 == 6'b010111);
          if (f3 == 3'b100) b = in_xs;
          else if (f3 == 3'b011) b = shf ? 64'(rs1) : {{59{rs1[4]}}, rs1};
          act = vm || in_m[el] || mrg;
          x = (mrg && !vm && !in_m[el]) ? a : iref(f6, a, b, w);
          if (act)
            for (int bt = 0; bt < w; bt++) e[w * el + bt] = x[bt];
        end
      n_int++;
    end else if (k < 8) begin
      // FP32 vfadd / vfmul (.vv / .vf)
      logic is_mul, vf;
      shortreal sa, sb, sr;
      is_mul = $urandom_range(0, 1);
      vf     = $urandom_range(0, 1);
      in_sew = 2'd2; ne = VLEN / 32;
      f6 = is_mul ? 6'b100100 : 6'b000000;
      f3 = vf ? 3'b101 : 3'b001;
      in_raw = enc(f6, vm, f3, rs1);
      // 操作数：随机的规格化数，指数 120..130 (和在双精度下精确，shortreal 参考只舍入一次)
      for (int el = 0; el < ne; el++) begin
        in_vs2[32*el +: 32] = {1'($urandom), 8'($urandom_range(120, 130)), 23'($urandom)};
        in_vs1[32*el +: 32] = {1'($urandom), 8'($urandom_range(120, 130)), 23'($urandom)};
      end
      in_xs = {32'hFFFF_FFFF, 1'($urandom), 8'($urandom_range(120, 130)), 23'($urandom)};
      in_nel = $urandom_range(0, 1) ? 8'(ne) : 8'($urandom_range(0, ne));
      e = in_vd;
      for (int el = 0; el < ne; el++)
        if (el < int'(in_nel) && (vm || in_m[el])) begin
          sa = $bitstoshortreal(in_vs2[32*el +: 32]);
          sb = $bitstoshortreal(vf ? in_xs[31:0] : in_vs1[32*el +: 32]);
          sr = is_mul ? sa * sb : sa + sb;
          e[32*el +: 32] = $shortrealtobits(sr);
        end
      n_f32++;
    end else begin
      // FP64 vfmacc.vv：vd = vs1 × vs2 + vd，操作数取小整数 (精确)
      real ra, rb, rc;
      in_sew = 2'd3; ne = VLEN / 64;
      in_raw = enc(6'b101100, vm, 3'b001, rs1);
      for (int el = 0; el < ne; el++) begin
        in_vs2[64*el +: 64] = $realtobits(real'($urandom_range(0, 2000)) - 1000.0);
        in_vs1[64*el +: 64] = $realtobits(real'($urandom_range(0, 2000)) - 1000.0);
        in_vd[64*el +: 64]  = $realtobits(real'($urandom_range(0, 2000000)) - 1000000.0);
      end
      in_nel = $urandom_range(0, 1) ? 8'(ne) : 8'($urandom_range(0, ne));
      e = in_vd;
      for (int el = 0; el < ne; el++)
        if (el < int'(in_nel) && (vm || in_m[el])) begin
          ra = $bitstoreal(in_vs2[64*el +: 64]); rb = $bitstoreal(in_vs1[64*el +: 64]); rc = $bitstoreal(in_vd[64*el +: 64]);
          e[64*el +: 64] = $realtobits(ra * rb + rc);
        end
      n_f64++;
    end
    exp_q.push_back(e); tag_q.push_back(in_tag); t_in_q.push_back(cyc);
  endtask

  int lat = -1;
  always @(posedge clk) if (rst_n && out_v) begin
    logic [VLEN-1:0] e;
    logic [TW-1:0]   t;
    int              ti;
    e = exp_q.pop_front(); t = tag_q.pop_front(); ti = t_in_q.pop_front();
    if (lat < 0) lat = cyc - ti;
    if (cyc - ti != lat) begin n_err++; $display("错误：延迟 %0d (首个 %0d) tag %0d", cyc - ti, lat, t); end
    if (out_tag !== t) begin n_err++; $display("错误：tag %0d 应为 %0d", out_tag, t); end
    if (out_d !== e) begin
      n_err++;
      if (n_err < 10)
        for (int j = 0; j < VLEN / 64; j++)
          if (out_d[64*j +: 64] !== e[64*j +: 64]) begin
            $display("错误：tag %0d 双字 %0d 得 %h 应 %h", t, j, out_d[64*j +: 64], e[64*j +: 64]); break;
          end
    end else n_ok++;
  end

  initial begin
    in_v = 1'b0;
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);
    for (int i = 0; i < NB; i++) begin
      @(negedge clk);
      // 大部分拍连续送 (检查每拍都能收)，偶尔空一拍
      if ($urandom_range(0, 9) == 0) begin in_v = 1'b0; @(negedge clk); end
      send_one(i);
      in_v = 1'b1;
    end
    @(negedge clk); in_v = 1'b0;
    repeat (40) @(posedge clk);
    $display("拍数 %0d (整数 %0d、FP32 %0d、FP64 %0d)，正确 %0d，错误 %0d，延迟 %0d 拍，剩 %0d", NB, n_int, n_f32, n_f64,
             n_ok, n_err, lat, exp_q.size());
    if (n_err == 0 && exp_q.size() == 0 && n_ok == NB) $display("=== UNIT PASS ===");
    else $display("=== UNIT FAIL ===");
    $finish;
  end
endmodule
