//==============================================================================
// o6_uvm_if.sv - UVM 环境与 tb_top 之间的接口 (core 模式)
//   监视量由 tb/uvm/o6_uvm_bind.svh 从核内层次引用连过来；UVM 通过时钟块读 (#1step = 沿前的值，
//   与 tb_top 里 always @(posedge clk) 看到的一致)
//==============================================================================
`timescale 1ns/1ps
interface o6_uvm_if (input logic clk);
  import o6_pkg::*;
  localparam int NEV = 48;

  logic              rst_n;
  // 提交口 (每拍最多 W 条，按槽位 = 程序顺序)
  logic [W-1:0]      cm_v;
  logic [63:0]       cm_pc   [W];
  logic [31:0]       cm_insn [W];
  logic [1:0]        cm_rdt  [W];     // 0 无 1 整数 2 浮点
  logic [4:0]        cm_rd   [W];
  logic [63:0]       cm_wd   [W];
  logic [W-1:0]      cm_vec;          // 向量单元的指令
  // 陷入 (与本拍的提交同拍；陷入在本拍提交的指令之后)
  logic              trap_v, trap_irq;
  logic [63:0]       trap_cause, trap_epc, trap_tval;
  logic [1:0]        priv;
  logic              dbg_mode, dbg_enter;   // 调试模式 / 本拍进入调试模式 (跳调试 ROM，不经陷入)
  // 微架构事件 (每拍；编号见 o6_uvm_pkg::ev_e) 与各发射队列占用
  logic [NEV-1:0]    ev;
  logic [7:0]        chk;             // 独立断言 (每拍)：bit0 = MMIO load 开始时还有比它老的 store 在 SQ
  logic [7:0]        iq_occ  [N_IQ];
  logic [7:0]        iq_size [N_IQ];
  // 中断驱动 (UVM -> TB 的中断设备 0x0200_C000)：置位脉冲；pend 为设备当前挂起位 {SEIP, MEIP}
  logic              meip_set, seip_set;
  logic [1:0]        irq_pend;
  // CLINT 驱动 (UVM -> TB 的 CLINT 模型)：msip_set 置 msip；tmr_set 把 mtimecmp 设为 mtime + tmr_delta (mtime 每 8 拍加 1)
  logic              msip_set, tmr_set;
  logic [15:0]       tmr_delta;
  // tb 状态：结束 (PASS / FAIL) 与对拍错误数
  logic              tb_done, tb_pass;
  int                tb_err;
  longint            n_commit, cyc;

  clocking cb @(posedge clk);
    default input #1step output #0;
    input  rst_n, cm_v, cm_pc, cm_insn, cm_rdt, cm_rd, cm_wd, cm_vec;
    input  trap_v, trap_irq, trap_cause, trap_epc, trap_tval, priv, dbg_mode, dbg_enter;
    input  ev, chk, iq_occ, iq_size, irq_pend, tb_done, tb_pass, tb_err, n_commit, cyc;
    output meip_set, seip_set, msip_set, tmr_set, tmr_delta;
  endclocking
endinterface
