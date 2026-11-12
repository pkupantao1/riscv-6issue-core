// 参考模型：o6_vu_alu 重构 (合并加法器 / 比较 / 移位) 之前的原始写法 (每种操作各自的运算器)，只给 tb_vu_alu 锁步比对用

module o6_vu_alu_ref
  import o6_vu_pkg::*;
#(
  parameter int W = 8
) (
  input  aop_e         op_i,
  input  wire [W-1:0]  a_i, b_i, c_i,
  input  wire          cin_i,
  input  wire [1:0]    xrm_i,
  output logic [W-1:0] r_o,
  output logic         m_o, s_o
);
  // 带舍入的右移 (Spike INT_ROUNDING 再右移)：v 是 2W+1 位的有 / 无符号值，右移 d 位
  //   rnu：加回 v[d-1]；rne：v[d-1] 且 (更低位非零 或 v[d]) 时加 1；rdn：截断；rod：低 d 位非零则置最低位

  function automatic logic [2*W:0] rshr(input logic [2*W:0] v, input int d, input logic [1:0] xrm, input logic sg);
    logic [2*W:0] q, lo;
    logic         inc;
    q   = sg ? (2*W+1)'($signed(v) >>> d) : (v >> d);
    lo  = (d == 0) ? '0 : (v & (((2*W+1)'(1) << d) - 1'b1));
    inc = 1'b0;
    if (d != 0)
      unique case (xrm)
        2'd0: inc = v[d - 1];
        2'd1: inc = v[d - 1] && ((lo & ~((2*W+1)'(1) << (d - 1))) != '0 || v[d]);
        default: inc = 1'b0;
      endcase
    if (xrm == 2'd3 && lo != '0) q[0] = 1'b1;
    return q + (2*W+1)'(inc);
  endfunction
  function automatic logic [W+1:0] valu(input aop_e op, input logic [W-1:0] a, input logic [W-1:0] b,
                                          input logic [W-1:0] c, input logic cin, input logic [1:0] xrm);
    logic [W-1:0]   r;
    logic           m, s;
    logic [W:0]     t;
    logic [2*W-1:0] p;
    logic [2*W:0]   v, q;
    logic [2*W+1:0] pm;
    logic [W-1:0]   mx;
    logic           msx, msy;
    int             sh;
    r = '0; m = 1'b0; s = 1'b0; t = '0; p = '0; v = '0; q = '0;
    // 唯一的乘法器：(W+1) x (W+1) 有符号。乘数 x = vd (vmadd / vnmsub 是 vd * vs1) 或 vs2；
    // 符号位按操作扩展 (MULH / vsmul 两边有符号，MULHSU 只有 x)，低 2W 位就是各种符号组合的精确积
    mx  = (op == A_MADD || op == A_NMSUB) ? c : a;
    msx = (op == A_MULH || op == A_MULHSU || op == A_SMUL) && mx[W-1];
    msy = (op == A_MULH || op == A_SMUL) && b[W-1];
    pm  = (2*W+2)'($signed({msx, mx}) * $signed({msy, b}));
    sh = int'(b[$clog2(W)-1:0]);
    unique case (op)
      A_ADD:  r = a + b;
      A_SUB:  r = a - b;
      A_RSUB: r = b - a;
      A_MINU: r = (a < b) ? a : b;
      A_MIN:  r = ($signed(a) < $signed(b)) ? a : b;
      A_MAXU: r = (a > b) ? a : b;
      A_MAX:  r = ($signed(a) > $signed(b)) ? a : b;
      A_AND:  r = a & b;
      A_OR:   r = a | b;
      A_XOR:  r = a ^ b;
      A_SLL:  r = a << sh;
      A_SRL, A_NSRL: r = a >> sh;
      A_SRA, A_NSRA: r = W'($signed(a) >>> sh);
      A_MERGE: r = cin ? b : a;
      A_MV:   r = b;
      A_SGNJ:  r = {b[W-1], a[W-2:0]};
      A_SGNJN: r = {!b[W-1], a[W-2:0]};
      A_SGNJX: r = {a[W-1] ^ b[W-1], a[W-2:0]};
      C_EQ:   m = a == b;
      C_NE:   m = a != b;
      C_LTU:  m = a < b;
      C_LT:   m = $signed(a) < $signed(b);
      C_LEU:  m = a <= b;
      C_LE:   m = $signed(a) <= $signed(b);
      C_GTU:  m = a > b;
      C_GT:   m = $signed(a) > $signed(b);
      A_ADC:  r = a + b + W'(cin);
      A_SBC:  r = a - b - W'(cin);
      A_MADC: begin t = {1'b0, a} + {1'b0, b} + (W+1)'(cin); m = t[W]; end
      A_MSBC: begin t = {1'b0, a} - {1'b0, b} - (W+1)'(cin); m = t[W]; end
      A_SADDU: begin t = {1'b0, a} + {1'b0, b}; s = t[W]; r = s ? '1 : t[W-1:0]; end
      A_SADD:  begin t = {a[W-1], a} + {b[W-1], b}; s = (t[W] != t[W-1]);
                     r = s ? (t[W] ? {1'b1, {(W-1){1'b0}}} : {1'b0, {(W-1){1'b1}}}) : t[W-1:0]; end
      A_SSUBU: begin s = (b > a); r = s ? '0 : a - b; end
      A_SSUB:  begin t = {a[W-1], a} - {b[W-1], b}; s = (t[W] != t[W-1]);
                     r = s ? (t[W] ? {1'b1, {(W-1){1'b0}}} : {1'b0, {(W-1){1'b1}}}) : t[W-1:0]; end
      A_AADDU: begin v = (2*W+1)'(a) + (2*W+1)'(b);                   q = rshr(v, 1, xrm, 1'b0); r = q[W-1:0]; end
      A_AADD:  begin v = (2*W+1)'($signed(a)) + (2*W+1)'($signed(b)); q = rshr(v, 1, xrm, 1'b1); r = q[W-1:0]; end
      A_ASUBU: begin v = (2*W+1)'(a) - (2*W+1)'(b);                   q = rshr(v, 1, xrm, 1'b1); r = q[W-1:0]; end
      A_ASUB:  begin v = (2*W+1)'($signed(a)) - (2*W+1)'($signed(b)); q = rshr(v, 1, xrm, 1'b1); r = q[W-1:0]; end
      A_SMUL: begin
        p = pm[2*W-1:0];
        v = (2*W+1)'($signed(p));
        q = rshr(v, W - 1, xrm, 1'b1);
        if (a == {1'b1, {(W-1){1'b0}}} && b == {1'b1, {(W-1){1'b0}}}) begin s = 1'b1; r = {1'b0, {(W-1){1'b1}}}; end
        else r = q[W-1:0];
      end
      A_SSRL: begin v = (2*W+1)'(a);          q = rshr(v, sh, xrm, 1'b0); r = q[W-1:0]; end
      A_SSRA: begin v = (2*W+1)'($signed(a)); q = rshr(v, sh, xrm, 1'b1); r = q[W-1:0]; end
      A_NCLIPU: begin
        v = (2*W+1)'(a); q = rshr(v, sh, xrm, 1'b0);
        if (q > (2*W+1)'({(W/2){1'b1}})) begin s = 1'b1; r = W'({(W/2){1'b1}}); end else r = q[W-1:0];
      end
      A_NCLIP: begin
        v = (2*W+1)'($signed(a)); q = rshr(v, sh, xrm, 1'b1);
        if ($signed(q) > $signed((2*W+1)'({(W/2-1){1'b1}}))) begin s = 1'b1; r = W'({(W/2-1){1'b1}}); end
        else if ($signed(q) < -$signed((2*W+1)'({1'b1, {(W/2-1){1'b0}}}))) begin s = 1'b1; r = W'({1'b1, {(W/2-1){1'b0}}}); end
        else r = q[W-1:0];
      end
      A_MUL:    r = pm[W-1:0];
      A_MULH:   r = pm[2*W-1:W];
      A_MULHU:  r = pm[2*W-1:W];
      A_MULHSU: r = pm[2*W-1:W];
      A_MACC:  r = c + pm[W-1:0];
      A_NMSAC: r = c - pm[W-1:0];
      A_MADD:  r = pm[W-1:0] + a;
      A_NMSUB: r = a - pm[W-1:0];
      A_WMACC: r = c + pm[W-1:0];
      A_EXT:   r = a;
      // Zvbb (另写一份：位反转 / 字节反转 / 前后导零 / 计数 / 循环移位)
      A_ANDN:  r = a & ~b;
      A_ROL:   r = (sh == 0) ? a : (a << sh) | (a >> (W - sh));
      A_ROR:   r = (sh == 0) ? a : (a >> sh) | (a << (W - sh));
      A_BREV:  r = {<<{a}};
      A_BREV8: for (int k = 0; k < W / 8; k++) r[8*k +: 8] = {<<{a[8*k +: 8]}};
      A_REV8:  r = (W == 8) ? a : {<<8{a}};
      A_CLZ:   begin r = W'(W); for (int k = 0; k < W; k++) if (a[k]) r = W'(W - 1 - k); end
      A_CTZ:   begin r = W'(W); for (int k = W - 1; k >= 0; k--) if (a[k]) r = W'(k); end
      A_CPOP:  r = W'($countones(a));
      default: r = b;
    endcase
    return {s, m, r};
  endfunction
  always_comb begin
    logic [W+1:0] y;
    y = valu(op_i, a_i, b_i, c_i, cin_i, xrm_i);
    r_o = y[W-1:0]; m_o = y[W]; s_o = y[W+1];
  end
endmodule
