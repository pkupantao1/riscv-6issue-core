//==============================================================================
// o6_uvm_bind.svh - 在 tb_top 里 (+define+O6_UVM，core 模式) include：例化 o6_uvm_if、从核内层次引用连监视量、启动 UVM
//==============================================================================
  import uvm_pkg::*;
  import o6_uvm_pkg::*;
  o6_uvm_if u_vif (clk);

  assign u_vif.rst_n    = rst_n;
  assign u_vif.tb_done  = tb_done;
  assign u_vif.tb_pass  = tb_pass;
  assign u_vif.tb_err   = n_err;
  assign u_vif.n_commit = n_commit;
  assign u_vif.cyc      = cyc;
`ifdef TB_SOC
  assign u_vif.irq_pend = soc_eirq;
`else
  assign u_vif.irq_pend = {seip_r, meip_r};
`endif
  assign u_vif.priv     = 2'(`CORE.u_csr.priv);
  assign u_vif.dbg_mode  = `CORE.dbg_mode;
  assign u_vif.dbg_enter = `CORE.dbg_enter || midrst_evt;   // 运行中复位 (+midrst) 同样让 PC 流不连续
  assign u_vif.trap_v     = `CORE.u_cc.trap_o;
  assign u_vif.trap_irq   = `CORE.u_cc.trap_irq_o;
  assign u_vif.trap_cause = 64'(`CORE.u_cc.trap_cause_o);
  assign u_vif.trap_epc   = `CORE.u_cc.trap_epc_o;
  assign u_vif.trap_tval  = `CORE.u_cc.trap_tval_o;

  // 提交口：与 tb_top 的对拍取同样的量
  always_comb
    for (int c = 0; c < W; c++) begin
      u_vif.cm_v[c]    = `CORE.u_rob.cm_valid_o[c];
      u_vif.cm_pc[c]   = `CORE.`FE.ftq_pc[`CORE.u_rob.hent[c].ftq_idx] + {`CORE.u_rob.hent[c].ftq_off, 1'b0};
      u_vif.cm_insn[c] = `CORE.u_rob.dbg_raw[`CORE.u_rob.head_row][c];
      u_vif.cm_rdt[c]  = !`CORE.u_rob.cm_rd_valid_o[c] ? 2'd0 : (`CORE.u_rob.cm_rd_fp_o[c] ? 2'd2 : 2'd1);
      u_vif.cm_rd[c]   = `CORE.u_rob.cm_lrd_o[c];
      u_vif.cm_wd[c]   = `CORE.u_iprf.mem[`CORE.u_rob.cm_pdst_o[c]];
`ifndef NO_FPU
      if (`CORE.u_rob.cm_rd_fp_o[c]) u_vif.cm_wd[c] = `CORE.g_fpu.u_fpu.u_fprf.mem[`CORE.u_rob.cm_pdst_o[c]];
`endif
      u_vif.cm_vec[c]  = (`CORE.u_rob.hent[c].fu == FU_VEC);
    end

  // 独立断言：MMIO load 开始访问设备时，比它老的 store 必须都已离开 SQ (sq_head 到达它的 sq_bound)
  always_comb begin
    u_vif.chk = '0;
    u_vif.chk[0] = `CORE.u_lsu.start_mmio && (`CORE.u_lsu.sq_head != `CORE.u_lsu.lq[`CORE.u_lsu.head_mmio_i].sq_bound);
  end

  // 发射队列占用 (顺序 A B M F S D C N，同 tb_top 的性能统计)
  always_comb begin
    u_vif.iq_occ[0] = 8'($countones(`CORE.u_iqa.vld)); u_vif.iq_size[0] = 8'(IQ_A_SIZE);
    u_vif.iq_occ[1] = 8'($countones(`CORE.u_iqb.vld)); u_vif.iq_size[1] = 8'(IQ_B_SIZE);
    u_vif.iq_occ[2] = 8'($countones(`CORE.u_iqm.vld)); u_vif.iq_size[2] = 8'(IQ_M_SIZE);
    u_vif.iq_occ[3] = 8'($countones(`CORE.u_iqf.vld)); u_vif.iq_size[3] = 8'(IQ_F_SIZE);
    u_vif.iq_occ[4] = 8'($countones(`CORE.u_iqs.vld)); u_vif.iq_size[4] = 8'(IQ_S_SIZE);
    u_vif.iq_occ[5] = 8'($countones(`CORE.u_iqd.vld)); u_vif.iq_size[5] = 8'(IQ_D_SIZE);
    u_vif.iq_occ[6] = 8'($countones(`CORE.u_iqc.vld)); u_vif.iq_size[6] = 8'(IQ_C_SIZE);
    u_vif.iq_occ[7] = 8'($countones(`CORE.u_iqn.vld)); u_vif.iq_size[7] = 8'(IQ_N_SIZE);
  end

  // 微架构事件
  always_comb begin
    u_vif.ev = '0;
    u_vif.ev[EV_REDIRECT] = `CORE.redirect;
    for (int p = 0; p < N_BRU; p++)
      if (`CORE.u_iexu.br_v[p] && `CORE.u_iexu.bru_mis[p]) begin
        if (`CORE.u_iexu.ex_u[p].op == BRU_JALR) begin
          if (`CORE.u_iexu.ex_u[p].lrd == 5'd0 && (`CORE.u_iexu.ex_u[p].lrs1 == 5'd1 || `CORE.u_iexu.ex_u[p].lrs1 == 5'd5))
            u_vif.ev[EV_MIS_RET] = 1'b1;
          else u_vif.ev[EV_MIS_IND] = 1'b1;
        end else if (`CORE.u_iexu.ex_u[p].op == BRU_JAL) u_vif.ev[EV_MIS_JAL] = 1'b1;
        else begin
          u_vif.ev[EV_MIS_COND] = 1'b1;
          if (`CORE.u_iexu.bru_taken[p]) u_vif.ev[EV_MIS_COND_TK] = 1'b1;
        end
      end
    if (`CORE.flush) begin
      if (`CORE.u_cc.q_replay)    u_vif.ev[EV_FL_REPLAY] = 1'b1;
      else if (`CORE.u_cc.q_trap) u_vif.ev[EV_FL_TRAP]   = 1'b1;
      else                        u_vif.ev[EV_FL_AFTER]  = 1'b1;
    end
    u_vif.ev[EV_VIOL]     = `CORE.viol_any;
    u_vif.ev[EV_LD_FAIL]  = |`CORE.ld_fail;
    u_vif.ev[EV_FE_EMPTY] = (`CORE.ib_valid == '0);
    u_vif.ev[EV_FTQ_FULL] = `CORE.`FE.ftq_full;
    u_vif.ev[EV_IC_MISS]  = (`CORE.u_ic.st >= 2);
`ifdef FE_M2
    u_vif.ev[EV_F3_OVR]   = `CORE.`FE.mispred3;
    u_vif.ev[EV_F1_BTB]   = `CORE.`FE.f1_ovr;
`endif
    if (`CORE.de_valid != '0 && !`CORE.rn_ready) begin
      u_vif.ev[EV_REN_STALL] = 1'b1;
      u_vif.ev[EV_RN_Q]   = |`CORE.u_rn.q1_v;
      u_vif.ev[EV_RN_TAG] = !`CORE.u_rn.tags_ok;
      u_vif.ev[EV_RN_IFL] = (`CORE.u_rn.ifl_cnt < `CORE.u_rn.n_i);
      u_vif.ev[EV_RN_FFL] = (`CORE.u_rn.ffl_cnt < `CORE.u_rn.n_f);
    end
    if (`CORE.rn_valid != '0 && !`CORE.ds_ready) begin
      u_vif.ev[EV_DS_STALL] = 1'b1;
      u_vif.ev[EV_DS_ROB]   = !`CORE.u_ds.rob_ready_i;
      for (int q = 0; q < N_IQ; q++)
        if (7'(`CORE.u_ds.in_need_i.iqn[q]) > `CORE.u_ds.iq_free_i[q]) u_vif.ev[EV_DS_IQ] = 1'b1;
      u_vif.ev[EV_DS_LQ] = ((LQ_W+1)'(`CORE.u_ds.in_need_i.nld) > `CORE.u_ds.lq_free_i);
      u_vif.ev[EV_DS_SQ] = ((SQ_W+1)'(`CORE.u_ds.in_need_i.nst) > `CORE.u_ds.sq_free_i);
    end
    u_vif.ev[EV_DC_LD_MISS] = |`CORE.u_dc.l2_miss;
    u_vif.ev[EV_DC_ST_MISS] = `CORE.u_dc.st_miss_alloc;
    u_vif.ev[EV_DC_NACK]    = |`CORE.u_dc.l2_nack;
    u_vif.ev[EV_MSHR_FULL]  = 1'b1;
    for (int i = 0; i < $size(`CORE.u_dc.m_st); i++) if (`CORE.u_dc.m_st[i] == 0) u_vif.ev[EV_MSHR_FULL] = 1'b0;
    for (int k = 0; k < N_LDP; k++)
      if (`CORE.u_lsu.l1_v[k] && `CORE.u_lsu.l1[k].u.fu == FU_LD && (|`CORE.u_lsu.fw_hit[k])) begin
        if (`CORE.u_lsu.fw_wait[k]) u_vif.ev[EV_STFWD_WAIT] = 1'b1;
        else                        u_vif.ev[EV_STFWD]      = 1'b1;
      end
    u_vif.ev[EV_PTW_I]    = `CORE.ptw_ireq;
    u_vif.ev[EV_PTW_D]    = `CORE.ptw_dreq;
    u_vif.ev[EV_PTW_BUSY] = `CORE.ptw_busy;
    u_vif.ev[EV_DPF]      = `CORE.u_lsu.dpf_v;
    u_vif.ev[EV_MMIO]     = `CORE.u_lsu.mmio_req_o && `CORE.u_lsu.mmio_ack_i;
    u_vif.ev[EV_VU_BUSY]  = !`CORE.u_vu.idle_o;
    u_vif.ev[EV_CMT_FULL] = (`CORE.cm_valid == '1);
    u_vif.ev[EV_SQ_FULL]  = (`CORE.u_lsu.sqv == '1);
    u_vif.ev[EV_LQ_FULL]  = (`CORE.u_lsu.lqv == '1);
    u_vif.ev[EV_SQFULL_FLUSH] = (`CORE.u_lsu.sqv == '1) && `CORE.flush;
  end

  initial begin
    uvm_config_db#(virtual o6_uvm_if)::set(null, "*", "vif", u_vif);
    run_test("o6_base_test");
  end
