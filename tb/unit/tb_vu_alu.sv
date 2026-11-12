// tb_vu_alu：o6_vu_alu (合并运算器的写法) 与 o6_vu_alu_ref (原写法) 锁步比对，四种宽度同时跑
//   W = 8：全部操作码 x vxrm 4 种 x cin 2 种 x a、b 全空间 (c 随机)；W = 16 / 32 / 64：随机 N 组，操作数偏向边界值
//   (0、1、全 1、最小负数、最大正数、最小负数 + 1、小移位量)；比较 {饱和, 掩码位, 结果} 三个输出
//   plusargs：+n=<每种宽度的随机组数，默认 3000000>
`timescale 1ns/1ps
module tb_vu_alu;
  import o6_vu_pkg::*;
  int nerr = 0;
  int done = 0;
  longint nchk = 0;
  int n_rand;
  initial if (!$value$plusargs("n=%d", n_rand)) n_rand = 3000000;

  for (genvar w = 0; w < 4; w++) begin : g_w
    localparam int W = 8 << w;
    aop_e         op;
    logic [W-1:0] a, b, c;
    logic         cin;
    logic [1:0]   xrm;
    logic [W-1:0] r0, r1;
    logic         m0, m1, s0, s1;
    o6_vu_alu     #(.W(W)) u_dut (.op_i(op), .a_i(a), .b_i(b), .c_i(c), .cin_i(cin), .xrm_i(xrm), .r_o(r0), .m_o(m0), .s_o(s0));
    o6_vu_alu_ref #(.W(W)) u_ref (.op_i(op), .a_i(a), .b_i(b), .c_i(c), .cin_i(cin), .xrm_i(xrm), .r_o(r1), .m_o(m1), .s_o(s1));

    function automatic logic [W-1:0] pick();
      logic [W-1:0] v;
      unique case ($urandom_range(0, 11))
        0:  v = '0;
        1:  v = W'(1);
        2:  v = '1;
        3:  v = {1'b1, {(W-1){1'b0}}};
        4:  v = {1'b0, {(W-1){1'b1}}};
        5:  v = {1'b1, {(W-1){1'b0}}} + 1'b1;
        6:  v = W'($urandom_range(0, W));                   // 小移位量 / 小数
        7:  v = W'({(W/2){1'b1}}) + W'($urandom_range(0, 2)) - 1'b1;   // 收窄饱和边界附近
        8:  v = -W'($urandom_range(0, 3));
        default: v = W'({$urandom, $urandom});
      endcase
      return v;
    endfunction
    task automatic chk();
      #1;
      nchk++;
      if (r0 !== r1 || m0 !== m1 || s0 !== s1) begin
        nerr++;
        if (nerr <= 30)
          $display("错误 W=%0d op=%s a=%h b=%h c=%h cin=%b xrm=%0d：新 {s%b m%b %h}  参考 {s%b m%b %h}",
                   W, op.name(), a, b, c, cin, xrm, s0, m0, r0, s1, m1, r1);
      end
    endtask

    initial begin
      #1;
      if (W == 8) begin
        op = op.first();
        forever begin
          for (int x = 0; x < 4; x++)
            for (int ci = 0; ci < 2; ci++)
              for (int i = 0; i < 256; i++)
                for (int j = 0; j < 256; j++) begin
                  a = W'(i); b = W'(j); c = W'($urandom); cin = ci[0]; xrm = x[1:0];
                  chk();
                end
          if (op == op.last()) break;
          op = op.next();
        end
      end else begin
        repeat (n_rand) begin
          op  = aop_e'($urandom_range(0, op.num() - 1));
          a   = pick(); b = pick(); c = pick();
          cin = $urandom; xrm = $urandom;
          chk();
        end
      end
      $display("W=%0d 完成", W);
      done++;
    end
  end

  initial begin
    wait (done == 4);
    $display("比对 %0d 组，不一致 %0d", nchk, nerr);
    if (nerr == 0) $display("=== UNIT PASS ===");
    else $display("=== UNIT FAIL ===");
    $finish;
  end
endmodule
