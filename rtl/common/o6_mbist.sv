//==============================================================================
// o6_mbist.sv - 单口 SRAM 的 March C− 自测引擎 (每个 o6_sram_sp 一个)
//
// 算法 (两种数据背景各跑一遍：P = 全 0，P = 0101…棋盘)：
//   M0 ⇕(wP)  M1 ⇑(rP, w~P)  M2 ⇑(r~P, wP)  M3 ⇓(rP, w~P)  M4 ⇓(r~P, wP)  M5 ⇕(rP)
// 每个地址 2 拍：第 1 拍读 (或空)，第 2 拍写 (或空)，第 2 拍末寄存第 1 拍读出的数据，下一拍再比较
// (2026-10-29，6 GHz：原来第 2 拍直接 宏读出 -> 全宽比较 -> fail，D$ 块级 r2r −13.9；done_o 随之晚一拍)。
// 控制：start_i 为低时同步复位 (不依赖系统复位，核在复位中也能测)；拉高开始，结束后 done_o = 1，
//   fail_o 粘滞记录是否有过比较错误。test 期间 active_o = 1，外面的封装用它把 SRAM 端口切给引擎。
// 写掩码全 1；按位比较全宽。
//==============================================================================
`default_nettype none
module o6_mbist #(
  parameter int DEPTH = 64,
  parameter int WIDTH = 64
) (
  input  wire                      clk_i,
  input  wire                      start_i,
  output logic                     active_o,
  output logic                     done_o,
  output logic                     fail_o,
  // 接 SRAM
  output logic                     ce_o,
  output logic                     we_o,
  output logic [$clog2(DEPTH)-1:0] addr_o,
  output logic [WIDTH-1:0]         wdata_o,
  input  wire  [WIDTH-1:0]         rdata_i
);
  localparam int AW = $clog2(DEPTH);
  localparam logic [AW-1:0] AMAX = AW'(DEPTH - 1);

  logic          run;
  logic [2:0]    elem;       // 0..5
  logic          bg;         // 0：全 0 背景，1：棋盘背景
  logic          ph;         // 0：读拍，1：写拍
  logic [AW-1:0] a;
  logic          cmp_v;      // 本拍读出的数据要比较 (寄存进 rd_q)
  logic [WIDTH-1:0] exp_d;
  logic          cv2;        // 上一拍寄存的读出数据 rd_q 要比较
  logic [WIDTH-1:0] rd_q, exp2;
  logic          fin;        // March 跑完 (done_o 晚一拍，等最后一次比较)

  // 背景图样
  function automatic logic [WIDTH-1:0] pat(input logic b, input logic inv);
    logic [WIDTH-1:0] p;
    for (int i = 0; i < WIDTH; i++) p[i] = b ? i[0] : 1'b0;
    return inv ? ~p : p;
  endfunction

  // 每个元素：是否读、读期望是否取反、是否写、写是否取反、方向
  logic e_rd, e_rinv, e_wr, e_winv, e_down;
  always_comb begin
    e_rd = 1'b1; e_rinv = 1'b0; e_wr = 1'b1; e_winv = 1'b1; e_down = 1'b0;
    unique case (elem)
      3'd0: begin e_rd = 1'b0; e_wr = 1'b1; e_winv = 1'b0; end
      3'd1: begin e_rinv = 1'b0; e_winv = 1'b1; end
      3'd2: begin e_rinv = 1'b1; e_winv = 1'b0; end
      3'd3: begin e_rinv = 1'b0; e_winv = 1'b1; e_down = 1'b1; end
      3'd4: begin e_rinv = 1'b1; e_winv = 1'b0; e_down = 1'b1; end
      default: begin e_rinv = 1'b0; e_wr = 1'b0; end
    endcase
  end

  wire last_a = e_down ? (a == '0) : (a == AMAX);

  assign active_o = run;
  assign ce_o     = run && (ph ? e_wr : e_rd);
  assign we_o     = run && ph;
  assign addr_o   = a;
  assign wdata_o  = pat(bg, e_winv);

  always_ff @(posedge clk_i) begin
    if (!start_i) begin
      run <= 1'b0; done_o <= 1'b0; fail_o <= 1'b0; fin <= 1'b0;
      elem <= '0; bg <= 1'b0; ph <= 1'b0; a <= '0; cmp_v <= 1'b0; exp_d <= '0; cv2 <= 1'b0;
    end else begin
      cmp_v <= 1'b0;
      cv2   <= cmp_v;
      rd_q  <= rdata_i;
      exp2  <= exp_d;
      if (cv2 && rd_q != exp2) fail_o <= 1'b1;
      done_o <= fin;
      if (!run && !fin) begin
        run <= 1'b1; elem <= '0; bg <= 1'b0; ph <= 1'b0; a <= '0;
      end else if (run) begin
        if (!ph) begin
          // 读拍
          cmp_v <= e_rd;
          exp_d <= pat(bg, e_rinv);
          ph    <= 1'b1;
        end else begin
          // 写拍：推进地址 / 元素 / 背景
          ph <= 1'b0;
          if (!last_a) begin
            a <= e_down ? a - 1'b1 : a + 1'b1;
          end else begin
            logic [2:0] ne;
            ne = elem + 3'd1;
            if (elem == 3'd5) begin
              if (bg) begin run <= 1'b0; fin <= 1'b1; end
              else begin bg <= 1'b1; elem <= '0; a <= '0; end
            end else begin
              elem <= ne;
              a    <= (ne == 3'd3 || ne == 3'd4) ? AMAX : '0;
            end
          end
        end
      end
    end
  end
endmodule
