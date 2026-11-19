//==============================================================================
// o6_rename.sv - 重命名级 (RN) + RN/DS 流水寄存器
//
// 一拍完成：RAT 读 -> 组内 RAW/WAW 旁路 -> RAT 写；free list 出队；busy 表读；
// 分支标签分配与快照 (RAT / free list head / 标签掩码)。
//
// 组内依赖：槽 i 的源若被同组更早的槽 j 写 (同一寄存器类、同一逻辑号)，
//   取最近的那个 j 的新 pdst，并视为 busy。
// 旧映射 old_pdst 同理取"槽 i 之前最近一次写"。
//
// 分支标签：br_busy 记录已分配未解析的标签。新 uop 的 br_mask = br_busy
//   (都是比它老的分支) | 同组更早槽分到的标签。分支自己的 br_mask 不含自己。
//   误预测 kill_tag 时，比它新的标签 (分配时掩码里含 kill_tag 的) 一并回收。
//==============================================================================
`default_nettype none
module o6_rename
  import o6_pkg::*;
#(
  parameter int N_IWAKE = 7,
  parameter int N_FWAKE = 5
) (
  input  wire              clk_i,
  input  wire              rst_ni,

  // 来自译码级
  input  wire  [W-1:0]     in_valid_i,
  input  var uop_t [W-1:0]     in_uop_i,
  output wire              in_ready_o,      // 本拍整组被重命名

  // 去分派级
  output logic [W-1:0]     out_valid_o,
  output uop_t [W-1:0]     out_uop_o,
  output ds_need_t         out_need_o,      // 这一组分派所需的资源 (随组寄存)
  input  wire  [6:0]       iq_free_i [N_IQ],  // 各发射队列寄存的空位数 (ALU 在 A / B 间分流用)
  input  wire              out_ready_i,     // 分派级本拍消耗整组

  // 广播
  input  var br_update_t   br_upd_i,
  input  wire              flush_i,         // 异常 / 提交级冲刷
  input  var wakeup_t [N_IWAKE-1:0] iwake_i,
  input  var wakeup_t [N_FWAKE-1:0] fwake_i,

  // 提交
  input  wire  [W-1:0]     cm_valid_i,      // 提交且有目的寄存器
  input  wire  [W-1:0]     cm_is_fp_i,
  input  wire  [4:0]       cm_lrd_i [W],
  input  var preg_t [W-1:0]    cm_pdst_i,
  input  var preg_t [W-1:0]    cm_old_pdst_i,
  input  wire  [W-1:0]     cm_mvel_i,       // 提交的是被消除的 move (没有分配新的物理寄存器)

  // O1 核内向量乱序 (docs/11 §9f)
  input  wire  [31:0]      vwpend_i,        // VECU：已提交、还没做完的 "不在核内改名的写" 的寄存器 (组合)
  input  wire  [W-1:0]     vfr_v_i,         // VECU：回收到核内池的向量物理寄存器 (映射切换后旧的)
  input  var preg_t [W-1:0] vfr_p_i,
  input  wire  [NVX-1:0]   vxw_v_i,         // VX 写回 (清忙)，V3：每个执行器一个
  input  var preg_t        vxw_p_i [NVX],
  input  wire  [N_LDP-1:0] vlw_v_i,         // M3：向量 load 写完 (清忙)
  input  var preg_t [N_LDP-1:0] vlw_p_i,
  output logic [N_VPREG-1:0] vbusy_o,       // 向量物理寄存器忙 (VIQ 选择用)
  output logic [31:0]      vpend_o,         // M3：各寄存器有在途的不在核内改名的写 (寄存一拍，给译码级决定展不展开)
  input  wire  [W-1:0]     cmv_f_i,         // 提交 (cmq，与 cm_* 同拍)：乱序 uop / 不在核内改名的写
  input  wire  [W-1:0]     cmv_nw_i,
  input  wire  [4:0]       cmv_wd_i [W],
  input  wire  [3:0]       cmv_wn_i [W],
  input  var preg_t [W-1:0] cmv_pd_i,
  // V1.2b：提交的乱序向量运算的整组 {组大小, 各寄存器的新物理寄存器}
  input  wire  [3:0]       cmv_ng_i [W],
  input  var preg_t [7:0]  cmv_pg_i [W]
);
  localparam int IFL_W = $clog2(N_IPREG);
  localparam int FFL_W = $clog2(N_FPREG);

  //--------------------------------------------------------------------------
  // 组内分类
  //--------------------------------------------------------------------------
  logic [W-1:0] need_i, need_f, need_v, is_br;
  logic [W-1:0] elim, alloc_i; // 被消除的 move (写 RAT、不分配) / 要从整数 free list 分配的
  logic [2:0]   ipos [W];      // 槽 i 之前有几个整数分配
  logic [2:0]   fpos [W];
  logic [3:0]   vpos [W];      // V1.2b：槽 s 之前分配了几个向量物理寄存器 (按组大小计)
  logic [2:0]   n_i, n_f, n_br;
  logic [3:0]   n_v;           // 本组分配的向量物理寄存器数 (<= 8，译码已截组)
  logic [3:0]   vng_s [W];     // 各槽要分配的向量物理寄存器数
  logic [W-1:0] v_f, v_nw;     // O1：本组各槽定为乱序 / 是不在核内改名的向量写 (见向量重命名)
  logic [W-1:0] v_s;           // M3b：向量 store 数据 uop (进 VIQ 读寄存器，不分配目的)
  logic         v_stall;       // M3：本组有向量 load 数据 uop 碰到在途写，停住
  always_comb begin
    logic [2:0] ci, cf, cb;
    logic [3:0] cv;
    ci = '0; cf = '0; cv = '0; cb = '0;
    for (int s = 0; s < W; s++) begin
      need_i[s] = in_valid_i[s] && in_uop_i[s].rd_class == RC_INT;
      need_f[s] = in_valid_i[s] && in_uop_i[s].rd_class == RC_FP;
      need_v[s] = v_f[s];
      vng_s[s]  = v_f[s] ? in_uop_i[s].vng : 4'd0;
      elim[s]   = need_i[s] && in_uop_i[s].mvel;
      alloc_i[s] = need_i[s] && !in_uop_i[s].mvel;
      is_br[s]  = in_valid_i[s] && in_uop_i[s].is_br;
      ipos[s] = ci; fpos[s] = cf; vpos[s] = cv;
      ci += alloc_i[s]; cf += need_f[s]; cv += vng_s[s]; cb += is_br[s];
    end
    n_i = ci; n_f = cf; n_v = cv; n_br = cb;
  end

  //--------------------------------------------------------------------------
  // Free list
  //--------------------------------------------------------------------------
  preg_t [W-1:0] ifl_cand, ffl_cand;
  logic [$clog2(N_IPREG+1)-1:0] ifl_cnt;
  logic [$clog2(N_FPREG+1)-1:0] ffl_cnt;
  logic [W:0]                   ifl_ge, ffl_ge;   // 空闲数 >= k (寄存的温度计码，见 o6_freelist)
  logic [IFL_W:0] ifl_head;
  logic [FFL_W:0] ffl_head;
  // 向量空闲表 (docs/11 §9a)：不进 RAT (I1.2 快路径在 ROB 头按提交态映射读源)，只按程序序分配、按分支快照 / 冲刷恢复
  localparam int VFL_W = $clog2(VCP);
  logic [8:0]     vfl_ge;                // V1.2b：向量空闲表每拍至多出队 8 个
  logic [VFL_W:0] vfl_head, vfl_ahead;
  logic [VFL_W:0] snap_vhead [MAX_BR];

  // 分支标签
  br_mask_t br_busy;
  br_mask_t tag_mask [MAX_BR];   // 每个标签分配时的 br_mask (比它老的分支)
  logic [IFL_W:0] snap_ihead [MAX_BR];
  logic [FFL_W:0] snap_fhead [MAX_BR];

  br_mask_t br_busy_now;   // 去掉本拍已解析的
  assign br_busy_now = br_busy & ~br_upd_i.resolve_mask;

  // 分配最低的两个空闲标签 (空闲 = ~br_busy 寄存值：本拍刚解析的标签下一拍才可再分配，避免同拍复用)
  // 2026-10-25，6 GHz：原来本拍在 br_busy 上串两次 48 位找最低位，再经 tags_ok -> fire -> RAT 写使能 /
  // 快照 (b180u33 194 ps)。改成上一拍按"本拍分配 0 / 1 / 2 个"三种情况各算好下一拍的两个最低空闲标签，
  // 按 fire / n_br 三选一寄存 (tg_q)；与原写法逐拍一致 (断言)
  typedef struct packed { logic k1, k0; br_tag_t t1, t0; } lo2_t;
  function automatic lo2_t lo2(input br_mask_t f);
    lo2_t r;
    r = '0;
    for (int b = 0; b < MAX_BR_PER_GRP; b++) begin
      br_tag_t t;
      logic    k;
      t = '0; k = 1'b0;
      for (int i = MAX_BR-1; i >= 0; i--)
        if (f[i]) begin t = br_tag_t'(i); k = 1'b1; end
      if (k) f[t] = 1'b0;
      if (b == 0) begin r.t0 = t; r.k0 = k; end
      else        begin r.t1 = t; r.k1 = k; end
    end
    return r;
  endfunction
  lo2_t    tg_q;
  br_tag_t new_tag [MAX_BR_PER_GRP];
  logic    tag_ok  [MAX_BR_PER_GRP];
  assign new_tag[0] = tg_q.t0; assign tag_ok[0] = tg_q.k0;
  assign new_tag[1] = tg_q.t1; assign tag_ok[1] = tg_q.k1;

  wire tags_ok  = (n_br == 0) || (n_br == 1 && tag_ok[0]) || (n_br == 2 && tag_ok[0] && tag_ok[1]);
  // 空闲数够不够：按 n_i / n_f 取寄存温度计码的一位 (2026-10-28，6 GHz：原 "tail - head -> 比较" 在 fire 前，b160 −25)
  wire res_ok   = ifl_ge[n_i] && ffl_ge[n_f] && vfl_ge[n_v] && tags_ok && (n_br <= MAX_BR_PER_GRP) &&   // n_i 不含被消除的 move
                  !v_stall;
  // 输出是 2 组队列 (q0 = out_*，q1 在下面)：能否重命名只看寄存的"q1 空"，
  // 不等分派级当拍的就绪 (端口分配 + IQ/ROB/LSQ 空位比较的深逻辑)
  logic [W-1:0] q1_v;
  wire fire     = (|in_valid_i) && res_ok && !(|q1_v) && !flush_i && !br_upd_i.kill;
  assign in_ready_o = fire;

  // 每个槽分到的分支标签与在组内的分支序号
  br_tag_t slot_tag [W];
  logic [1:0] slot_bidx [W];
  always_comb begin
    logic [1:0] c;
    c = '0;
    for (int s = 0; s < W; s++) begin
      slot_bidx[s] = c;
      slot_tag[s]  = new_tag[c[0]];
      c += is_br[s];
    end
  end

  //--------------------------------------------------------------------------
  // RAT
  //--------------------------------------------------------------------------
  logic [4:0] irat_ra [W][4];
  logic [4:0] frat_ra [W][4];
  preg_t      irat_rd [W][4];
  preg_t      frat_rd [W][4];
  logic [4:0] wr_addr [W];
  preg_t [W-1:0] new_pdst;
  // RAT / busy 表按类别各用自己的写数据 (2026-10-28，6 GHz：共用 new_pdst 时 move 消除的 mvel -> mv_ps 选择进了 FRAT 快照，
  // p160u63 in 组 in_uop_i.mvel -> u_frat/snap −57.7；浮点从不被消除)
  preg_t [W-1:0] new_ipdst, new_fpdst;
  logic [W-1:0]  iw_en, fw_en;
  logic [W-1:0]  ibs_en;        // busy 表置位：只有真正分配的 (被消除的 move 沿用源的 busy 状态)
  // 被消除的 move 的源 (RAT 读出，或同组更早普通槽的新 pdst —— 译码级保证不是更早的被消除者)、源的 busy
  preg_t [W-1:0] mv_ps;
  logic [W-1:0]  mv_bz;

  always_comb begin
    for (int s = 0; s < W; s++) begin
      irat_ra[s][0] = in_uop_i[s].lrs1; frat_ra[s][0] = in_uop_i[s].lrs1;
      irat_ra[s][1] = in_uop_i[s].lrs2; frat_ra[s][1] = in_uop_i[s].lrs2;
      irat_ra[s][2] = in_uop_i[s].lrs3; frat_ra[s][2] = in_uop_i[s].lrs3;
      irat_ra[s][3] = in_uop_i[s].lrd;  frat_ra[s][3] = in_uop_i[s].lrd;
      wr_addr[s]    = in_uop_i[s].lrd;
      new_pdst[s]   = need_f[s] ? ffl_cand[fpos[s]] : elim[s] ? mv_ps[s] : ifl_cand[ipos[s]];
      new_ipdst[s]  = elim[s] ? mv_ps[s] : ifl_cand[ipos[s]];
      new_fpdst[s]  = ffl_cand[fpos[s]];
      iw_en[s]      = fire && need_i[s];
      fw_en[s]      = fire && need_f[s];
    end
  end

  // 快照：组内第 b 个分支所在槽
  logic [MAX_BR_PER_GRP-1:0] snap_en;
  br_tag_t [MAX_BR_PER_GRP-1:0] snap_tag;
  logic [2:0] snap_slot [MAX_BR_PER_GRP];
  always_comb begin
    for (int b = 0; b < MAX_BR_PER_GRP; b++) begin
      snap_en[b] = 1'b0; snap_tag[b] = new_tag[b]; snap_slot[b] = '0;
    end
    for (int s = W-1; s >= 0; s--) begin
      if (is_br[s]) begin
        snap_en[slot_bidx[s][0]]   = fire;
        snap_slot[slot_bidx[s][0]] = 3'(s);
      end
    end
  end

  logic [W-1:0] cm_i_en, cm_f_en;
  always_comb begin
    for (int s = 0; s < W; s++) begin
      cm_i_en[s] = cm_valid_i[s] && !cm_is_fp_i[s];
      cm_f_en[s] = cm_valid_i[s] &&  cm_is_fp_i[s];
    end
  end

  o6_rat #(.NARCH(32)) u_irat (
    .clk_i, .rst_ni,
    .rd_addr_i(irat_ra), .rd_data_o(irat_rd),
    .wr_en_i(iw_en), .wr_addr_i(wr_addr), .wr_preg_i(new_ipdst),
    .snap_en_i(snap_en), .snap_tag_i(snap_tag), .snap_slot_i(snap_slot),
    .br_restore_i(br_upd_i.kill), .br_restore_tag_i(br_upd_i.kill_tag),
    .flush_i,
    .cm_en_i(cm_i_en), .cm_addr_i(cm_lrd_i), .cm_preg_i(cm_pdst_i)
  );
  o6_rat #(.NARCH(32)) u_frat (
    .clk_i, .rst_ni,
    .rd_addr_i(frat_ra), .rd_data_o(frat_rd),
    .wr_en_i(fw_en), .wr_addr_i(wr_addr), .wr_preg_i(new_fpdst),
    .snap_en_i(snap_en), .snap_tag_i(snap_tag), .snap_slot_i(snap_slot),
    .br_restore_i(br_upd_i.kill), .br_restore_tag_i(br_upd_i.kill_tag),
    .flush_i,
    .cm_en_i(cm_f_en), .cm_addr_i(cm_lrd_i), .cm_preg_i(cm_pdst_i)
  );

  //--------------------------------------------------------------------------
  // Free list 实例
  //--------------------------------------------------------------------------
  logic [2:0] cm_alloc_i, cm_alloc_f;
  logic [W-1:0] fr_i_vld, fr_f_vld;
  preg_t [W-1:0] fr_preg;
  always_comb begin
    cm_alloc_i = '0; cm_alloc_f = '0;
    for (int s = 0; s < W; s++) begin
      cm_alloc_i += cm_i_en[s] && !cm_mvel_i[s];
      cm_alloc_f += cm_f_en[s];
    end
  end

  //--------------------------------------------------------------------------
  // move 消除的提交侧：dup[p] = 已提交状态里除第一个之外、还有几个架构寄存器映射到 p
  //   提交被消除的 move (新映射 p)：dup[p]++；提交覆盖旧映射 o：dup[o] > 0 则 dup[o]--、不释放，否则释放 o
  //   只随提交 (按序、非投机) 变，分支误预测 / 冲刷都不用恢复：在途的 move 只读源映射，源映射的最后一个
  //   架构引用被覆盖、释放之前，读它的指令都比覆盖者老、已经提交
  //   p0 (x0 的映射，恒为 0) 从不释放、不计数
  // 同一拍几条提交按槽序：槽 s 看到的 dup[o] = 原值 + 槽 0..s 里新映射到 o 的个数 - 槽 0..s-1 里覆盖 o 的个数
  // (组内某槽把 o 释放之后不可能再有槽映射或覆盖 o：o 已没有架构引用)
  //--------------------------------------------------------------------------
  // 2026-10-28 (6 GHz)：改成无条件引用计数 rc[p] = 已提交状态里映射到 p 的架构寄存器数 (= 原 dup + 1)。
  //   提交 (有整数 rd)：rc[新映射]++；覆盖旧映射 o：rc[o]--。更新不再依赖"是否释放"(原写法 dup[o] 读出 -> 组内修正
  //   -> keep -> 按 keep 减，p160u63 r2r cmq_old -> dup 173 ps)。rc 归零即释放：下一拍按寄存的零标志 rz 判定，
  //   同组多个槽覆盖同一 o 时只由最后一个槽释放 (组内某槽释放 o 之后不可能再有槽映射或覆盖 o，所以归零 ⇔ 原写法释放，且只一次)。
  //   提交输入先寄存一拍 (cmq_*)，释放判定再晚一拍 (cmr_*)：整数旧映射比原来晚两拍回空闲表。p0 不计数、不释放
  localparam int DW = 5;           // rc <= 31 (x1..x31 全映射到同一个 p)
  logic [DW-1:0] rc [N_IPREG];
  logic [N_IPREG-1:0] rz;          // rc == 0 (寄存，与 rc 同拍更新)
  logic [W-1:0]  cmq_en, cmq_mvel;
  preg_t [W-1:0] cmq_pdst, cmq_old;
  logic [W-1:0]  cmr_en;
  preg_t [W-1:0] cmr_old;
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) begin cmq_en <= '0; cmr_en <= '0; end
    else         begin cmq_en <= cm_i_en; cmr_en <= cmq_en; end
  always_ff @(posedge clk_i) begin
    cmq_mvel <= cm_mvel_i; cmq_pdst <= cm_pdst_i; cmq_old <= cm_old_pdst_i;
    cmr_old  <= cmq_old;
  end
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int p = 0; p < N_IPREG; p++) begin rc[p] <= (p >= 1 && p < 32) ? DW'(1) : '0; rz[p] <= !(p >= 1 && p < 32); end
    end else begin
      for (int p = 1; p < N_IPREG; p++) begin
        logic [DW+2:0] d;
        d = (DW+3)'(rc[p]);
        for (int s = 0; s < W; s++) begin
          if (cmq_en[s] && cmq_pdst[s] == preg_t'(p)) d = d + 1'b1;
          if (cmq_en[s] && cmq_old[s]  == preg_t'(p)) d = d - 1'b1;
        end
        rc[p] <= DW'(d);
        rz[p] <= (d == '0);
      end
    end
  end
  // 释放判定 (cmr_*：rc 已含这一组的增减)
  logic [W-1:0] cm_free;
  always_comb
    for (int s = 0; s < W; s++) begin
      logic later;
      later = 1'b0;
      for (int j = s + 1; j < W; j++) if (cmr_en[j] && cmr_old[j] == cmr_old[s]) later = 1'b1;
      cm_free[s] = cmr_en[s] && cmr_old[s] != '0 && rz[cmr_old[s]] && !later;
    end
`ifndef SYNTHESIS
  // 参考：原 dup / keep 写法 (按 cmq_* 算，比 cm_free 早一拍)，释放集合逐组一致
  logic [DW-1:0] dup [N_IPREG];
  logic [W-1:0]  rf_free, rf_free_q;
  always_comb
    for (int s = 0; s < W; s++) begin
      logic [DW+2:0] d;
      d = (DW+3)'(dup[cmq_old[s]]);
      for (int j = 0; j < W; j++) begin
        if (j <= s && cmq_en[j] && cmq_mvel[j] && cmq_pdst[j] == cmq_old[s] && cmq_pdst[j] != '0) d = d + 1'b1;
        if (j <  s && cmq_en[j] && cmq_old[j] == cmq_old[s]) d = d - 1'b1;
      end
      rf_free[s] = cmq_en[s] && !(cmq_old[s] == '0 || d != '0);
    end
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) begin
      for (int p = 0; p < N_IPREG; p++) dup[p] <= '0;
      rf_free_q <= '0;
    end else begin
      rf_free_q <= rf_free;
      for (int p = 0; p < N_IPREG; p++) begin
        logic [DW+2:0] d;
        d = (DW+3)'(dup[p]);
        for (int s = 0; s < W; s++) begin
          if (cmq_en[s] && cmq_mvel[s] && cmq_pdst[s] == preg_t'(p) && p != 0) d = d + 1'b1;
          if (cmq_en[s] && !rf_free[s] && cmq_old[s] == preg_t'(p) && p != 0) d = d - 1'b1;
        end
        dup[p] <= DW'(d);
      end
    end
  always_ff @(posedge clk_i) if (rst_ni) begin
    assert (cm_free == rf_free_q) else $error("rename: 引用计数释放 %b 与原 dup 写法 %b 不一致", cm_free, rf_free_q);
    for (int p = 1; p < N_IPREG; p++) begin
      assert (rc[p] < 5'd31) else $error("rename: rc[%0d] 溢出", p);
      assert (rz[p] == (rc[p] == '0)) else $error("rename: rz[%0d] 与 rc 不符", p);
    end
  end
`endif
  // 释放旧映射 (入队到 tail) 寄存一拍：提交判定 -> 压缩 -> 写队列不在一拍里。
  // 冲刷只把 head 恢复到 arch_head (arch_head 仍按当拍提交推进)，不动 tail，所以释放晚几拍不影响恢复，
  // 只是这些物理寄存器晚几拍才能再分配 (整数侧见上，浮点仍按当拍提交)
  preg_t [W-1:0] fr_ipreg;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin fr_i_vld <= '0; fr_f_vld <= '0; end
    else begin fr_i_vld <= cm_free; fr_f_vld <= cm_f_en; end
  end
  always_ff @(posedge clk_i) begin fr_preg <= cm_old_pdst_i; fr_ipreg <= cmr_old; end

  o6_freelist #(.N(N_IPREG), .NARCH(32)) u_ifl (
    .clk_i, .rst_ni,
    .deq_preg_o(ifl_cand), .free_cnt_o(ifl_cnt), .free_ge_o(ifl_ge),
    .deq_fire_i(fire), .deq_cnt_i(n_i),
    .head_o(ifl_head),
    .restore_i(br_upd_i.kill), .restore_head_i(snap_ihead[br_upd_i.kill_tag]),
    .free_vld_i(fr_i_vld), .free_preg_i(fr_ipreg),
    .commit_alloc_cnt_i(cm_alloc_i), .flush_i
  );
  o6_freelist #(.N(N_FPREG), .NARCH(32)) u_ffl (
    .clk_i, .rst_ni,
    .deq_preg_o(ffl_cand), .free_cnt_o(ffl_cnt), .free_ge_o(ffl_ge),
    .deq_fire_i(fire), .deq_cnt_i(n_f),
    .head_o(ffl_head),
    .restore_i(br_upd_i.kill), .restore_head_i(snap_fhead[br_upd_i.kill_tag]),
    .free_vld_i(fr_f_vld), .free_preg_i(fr_preg),
    .commit_alloc_cnt_i(cm_alloc_f), .flush_i
  );

  //--------------------------------------------------------------------------
  // O1 核内向量重命名 (docs/11 §9f)
  //   空闲表：核内池 [0, VCP)，复位映射 v0..v31 -> p0..p31。乱序 uop (vfast) 按程序序分配一个；提交时 arch_head 前进；
  //     旧映射不在提交时回收 (更老的、提交后才在 VECU 里执行的指令可能还要读)，由 VECU 按序切换映射后经 vfr 送回。
  //   VRAT：vr_f[r] = 1 表示 vr_p[r] 是核内改名的生产者 (乱序 uop) 的物理寄存器；= 0 表示最近写 r 的是不在核内改名的指令
  //     (提交后在 VECU 里按序做，原地写 crat 或在 VECU 里改名)，读 r 要按 VECU 的 crat。分支快照 / 提交态 (冲刷恢复) 同整数 RAT。
  //   在途写：不在核内改名的写 (vnw) 按架构寄存器计数 ta (重命名，随分支快照恢复) / tc (提交)；ta != tc 或 VECU 报的
  //     vwpend (已提交未做完) 时该寄存器 "有在途写"。乱序候选读写的寄存器 (vs2 / vs1 / vd / v0) 有在途写时降级为 vea。
  //     所以乱序 uop 读 crat 时，最近写那个寄存器的指令一定已在 VECU 里做完 (映射与数据都已落定)
  //--------------------------------------------------------------------------
  preg_t [7:0]   vfl_cand;
  logic  [$clog2(VCP+1)-1:0] vfl_cnt;
  preg_t [W-1:0] vfr_preg;
  logic  [W-1:0] vfr_vld;
  logic  [5:0]   cmv_nf;                                // 本拍提交的乱序 uop 分配过的向量物理寄存器数 (V1.2b：按组)
  always_comb begin
    vfr_preg = '0; vfr_vld = '0;
    vfr_preg = vfr_p_i; vfr_vld = vfr_v_i;                // V3：W 个
    cmv_nf = '0;
    for (int s = 0; s < W; s++) if (cmv_f_i[s]) cmv_nf += 6'(cmv_ng_i[s]);
  end
  o6_freelist #(.N(VCP), .NARCH(32), .FULL_OK(1), .DQ(8)) u_vfl (
    .clk_i, .rst_ni,
    .deq_preg_o(vfl_cand), .free_cnt_o(vfl_cnt), .free_ge_o(vfl_ge),
    .deq_fire_i(fire), .deq_cnt_i(n_v),
    .head_o(vfl_head), .arch_head_o(vfl_ahead), .arch_preg_o(),
    .restore_i(br_upd_i.kill), .restore_head_i(snap_vhead[br_upd_i.kill_tag]),
    .free_vld_i(vfr_vld), .free_preg_i(vfr_preg),
    .commit_alloc_cnt_i(cmv_nf), .flush_i
  );

  localparam int TW = 9;                                // 在途写计数位宽 (ROB 项数 < 512)
  logic          vr_f  [32];  preg_t vr_p  [32];        // 推测态 VRAT
  logic          cvr_f [32];  preg_t cvr_p [32];        // 提交态
  logic [TW-1:0] ta [32], tc [32];
  logic          vs_f [MAX_BR][32];  preg_t vs_p [MAX_BR][32];
  logic [TW-1:0] tas  [MAX_BR][32];
  logic [N_VPREG-1:0] vbusy;
  assign vbusy_o = vbusy;

  function automatic logic [31:0] vgrp(input logic [4:0] d, input logic [3:0] n);
    return 32'(((64'(1) << n) - 64'(1)) << d);
  endfunction

  // 逐槽：定乱序 / 在途写，源描述，组内各槽之后的 VRAT / 计数 (快照用)
  vsrc_t [3:0]   v_src [W];
  vsrc_t [2:0][7:0] v_sg [W];                            // V1.2b：源 0..2 的整组映射
  preg_t [7:0]   v_pg  [W];                              // V1.2b：目的组新分配的物理寄存器
  preg_t         v_pd  [W];
  logic          va_f [W][32];  preg_t va_p [W][32];    // 槽 0..s 处理之后的 VRAT
  logic [TW-1:0] va_t [W][32];                          // 槽 0..s 处理之后的 ta
  always_comb begin
    logic          cf [32]; preg_t cp [32];
    logic [TW-1:0] ct [32];
    logic [31:0]   pend;
    v_stall = 1'b0;
    for (int r = 0; r < 32; r++) begin
      cf[r] = vr_f[r]; cp[r] = vr_p[r]; ct[r] = ta[r];
    end
    for (int s = 0; s < W; s++) begin
      uop_t        u;
      logic [4:0]  ar [4];
      logic [3:0]  au;
      logic        tnt;
      logic [2:0]  vp;
      u = in_uop_i[s];
      // 当前 (含本组更早的槽) 的在途写
      for (int r = 0; r < 32; r++) pend[r] = (ct[r] != tc[r]) || vwpend_i[r];
      ar[0] = u.raw[24:20]; ar[1] = u.raw[19:15]; ar[2] = u.raw[11:7]; ar[3] = 5'd0;
      // vps 下标：[0] vs2，[1] vs1 (只有 .vv：OPIVV / V1.2a 起 OPFVV)，[2] vd 旧值 (尾部 / 被掩掉的元素保持；乘加的累加源)，[3] v0 (带掩码)
      au    = {!u.raw[25], 1'b1, u.raw[14:12] == 3'b000 || u.raw[14:12] == 3'b001, 1'b1};
      // M3 向量 load 数据 uop：只有 [2] = 寄存器 vwd 的旧值 (有效字节不满一行时补尾部)
      // M4a：带掩码时 [3] = v0 (掩码)，load 的 [2] 一定要 (被掩掉的字节保持旧值)
      if (u.vml) begin ar[2] = u.vwd; au = {u.vmk, u.vnb < 8'(CL_B) || u.vmk, 1'b0, 1'b0}; end
      // M3b 向量 store 数据 uop：只有 [2] = 寄存器 vwd 的当前值 (要写进 SQ 的数据)
      if (u.vms) begin ar[2] = u.vwd; au = {u.vmk, 3'b100}; end
      tnt   = 1'b0;
      for (int k = 0; k < 3; k++)
        for (int j = 0; j < 8; j++) if (au[k] && j < int'((k < 2) ? u.vns : u.vng) && pend[ar[k] + 5'(j)]) tnt = 1'b1;   // V4：源组 vns 个
      if (au[3] && pend[ar[3]]) tnt = 1'b1;
      // M3：已经展开的 uop 不能降级 -> 目的 (store：数据源) 有在途写时本组停住 (等那条在 VECU 里做完)
      if (in_valid_i[s] && u.vml && (tnt || pend[u.vwd])) v_stall = 1'b1;
      if (in_valid_i[s] && u.vms && tnt) v_stall = 1'b1;
      v_f[s]  = in_valid_i[s] && ((u.vfc && !tnt) || u.vml);
      v_s[s]  = in_valid_i[s] && u.vms;
      v_nw[s] = in_valid_i[s] && u.vnw && !v_f[s];
      // V1.2b：目的组 (vng 个) 依次取空闲表的候选；源 0..2 按组查映射 (组内各寄存器)，v0 只有一个
      for (int j = 0; j < 8; j++) begin
        logic [2:0] ix;
        ix = vpos[s][2:0] + 3'(j);
        v_pg[s][j] = vfl_cand[ix];
      end
      v_pd[s] = v_pg[s][0];
      for (int k = 0; k < 3; k++)
        for (int j = 0; j < 8; j++) begin
          logic [4:0] r;
          r = ar[k] + 5'(j);
          v_sg[s][k][j].use_ = au[k] && (j < int'((k < 2) ? u.vns : u.vng)) &&
                               !(k == 2 && u.vnod && !u.vml && !u.vms);   // V3：不读 vd 旧值 (在途写检查照旧算 vd)
          v_sg[s][k][j].ar   = r;
          v_sg[s][k][j].cr   = !cf[r];
          v_sg[s][k][j].p    = cp[r];
        end
      for (int k = 0; k < 3; k++) v_src[s][k] = v_sg[s][k][0];
      v_src[s][3].use_ = au[3]; v_src[s][3].ar = ar[3]; v_src[s][3].cr = !cf[ar[3]]; v_src[s][3].p = cp[ar[3]];
      // 本槽的写 (整组)
      if (v_f[s])
        for (int j = 0; j < 8; j++)
          if (j < int'(u.vng)) begin cf[u.vwd + 5'(j)] = 1'b1; cp[u.vwd + 5'(j)] = v_pg[s][j]; end
      if (v_nw[s]) begin
        logic [31:0] g;
        g = vgrp(u.vwd, u.vwn);
        for (int r = 0; r < 32; r++)
          if (g[r]) begin cf[r] = 1'b0; ct[r] = ct[r] + 1'b1; end
      end
      for (int r = 0; r < 32; r++) begin va_f[s][r] = cf[r]; va_p[s][r] = cp[r]; va_t[s][r] = ct[r]; end
    end
  end

  // 提交态的下一值 (冲刷恢复用含本拍提交的)
  logic          cvr_fn [32]; preg_t cvr_pn [32];
  logic [TW-1:0] tc_n [32];
  always_comb begin
    for (int r = 0; r < 32; r++) begin cvr_fn[r] = cvr_f[r]; cvr_pn[r] = cvr_p[r]; tc_n[r] = tc[r]; end
    for (int s = 0; s < W; s++) begin
      if (cmv_f_i[s])
        for (int j = 0; j < 8; j++)
          if (j < int'(cmv_ng_i[s])) begin cvr_fn[cmv_wd_i[s] + 5'(j)] = 1'b1; cvr_pn[cmv_wd_i[s] + 5'(j)] = cmv_pg_i[s][j]; end
      if (cmv_nw_i[s]) begin
        logic [31:0] g;
        g = vgrp(cmv_wd_i[s], cmv_wn_i[s]);
        for (int r = 0; r < 32; r++)
          if (g[r]) begin cvr_fn[r] = 1'b0; tc_n[r] = tc_n[r] + 1'b1; end
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int r = 0; r < 32; r++) begin
        vr_f[r] <= 1'b1; vr_p[r] <= preg_t'(r); cvr_f[r] <= 1'b1; cvr_p[r] <= preg_t'(r);
        ta[r] <= '0; tc[r] <= '0;
      end
      vbusy <= '0;
    end else begin
      for (int r = 0; r < 32; r++) begin cvr_f[r] <= cvr_fn[r]; cvr_p[r] <= cvr_pn[r]; tc[r] <= tc_n[r]; end
      if (flush_i) begin
        for (int r = 0; r < 32; r++) begin vr_f[r] <= cvr_fn[r]; vr_p[r] <= cvr_pn[r]; ta[r] <= tc_n[r]; end
      end else if (br_upd_i.kill) begin
        for (int r = 0; r < 32; r++) begin
          vr_f[r] <= vs_f[br_upd_i.kill_tag][r]; vr_p[r] <= vs_p[br_upd_i.kill_tag][r]; ta[r] <= tas[br_upd_i.kill_tag][r];
        end
      end else if (fire) begin
        for (int r = 0; r < 32; r++) begin vr_f[r] <= va_f[W-1][r]; vr_p[r] <= va_p[W-1][r]; ta[r] <= va_t[W-1][r]; end
      end
      // 忙：乱序 uop 的目的分配时置、VX 写回时清；冲刷时全清 (在途的都作废)
      if (flush_i) vbusy <= '0;
      else begin
        logic [N_VPREG-1:0] b;
        b = vbusy;
        for (int x = 0; x < NVX; x++) if (vxw_v_i[x]) b[vxw_p_i[x]] = 1'b0;   // V3：各执行器
        for (int k = 0; k < N_LDP; k++) if (vlw_v_i[k]) b[vlw_p_i[k]] = 1'b0;
        if (fire) for (int s = 0; s < W; s++) if (v_f[s]) for (int j = 0; j < 8; j++) if (j < int'(in_uop_i[s].vng)) b[v_pg[s][j]] = 1'b1;
        vbusy <= b;
      end
    end
  end
  // M3：在途写给译码级 (只是选路径的提示：过时了也只影响快慢，展开的 uop 到重命名还会再查、必要时停住)
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) vpend_o <= '0;
    else for (int r = 0; r < 32; r++) vpend_o[r] <= (ta[r] != tc[r]) || vwpend_i[r];
  // 分支快照：分支所在槽 s 之后的 VRAT / ta
  always_ff @(posedge clk_i)
    if (fire)
      for (int s = 0; s < W; s++)
        if (is_br[s])
          for (int r = 0; r < 32; r++) begin
            vs_f[slot_tag[s]][r] <= va_f[s][r]; vs_p[slot_tag[s]][r] <= va_p[s][r]; tas[slot_tag[s]][r] <= va_t[s][r];
          end
`ifndef SYNTHESIS
  always_ff @(posedge clk_i)
    if (rst_ni) begin
      for (int s = 0; s < W; s++)
        // V1.2b：分到的目的组不能有 X (VCS 2018 下 `vfl_cand[3'(int 表达式)]` 曾给出 X，改成显式 3 位加法)
        if (fire && v_f[s])
          assert (!$isunknown(v_pg[s])) else $error("rename: 槽 %0d 目的组 %h 有 X (候选 %h)", s, v_pg[s], vfl_cand);
      for (int s = 0; s < W; s++)
        if (fire && v_f[s]) begin
          // (被误预测作废的乱序 uop 的目的不清忙：重新分配时照样置忙，无妨)
          assert (v_pd[s] < preg_t'(VCP)) else $error("rename: 乱序 uop 分到的 p%0d 不在核内池", v_pd[s]);
          assert (in_uop_i[s].rd_class == RC_NONE) else $error("rename: 乱序 uop 带标量目的");
        end
      for (int k = 0; k < W; k++)
        if (vfr_v_i[k]) assert (vfr_p_i[k] < preg_t'(VCP) && vfr_p_i[k] >= preg_t'(0)) else $error("rename: VECU 回收的 p%0d 不在核内池", vfr_p_i[k]);
    end
`endif

  //--------------------------------------------------------------------------
  // busy 表
  //--------------------------------------------------------------------------
  logic [N_IWAKE-1:0] iclr_en;  preg_t [N_IWAKE-1:0] iclr_p;
  logic [N_FWAKE-1:0] fclr_en;  preg_t [N_FWAKE-1:0] fclr_p;
  always_comb begin
    for (int k = 0; k < N_IWAKE; k++) begin iclr_en[k] = iwake_i[k].valid; iclr_p[k] = iwake_i[k].pdst; end
    for (int k = 0; k < N_FWAKE; k++) begin fclr_en[k] = fwake_i[k].valid; fclr_p[k] = fwake_i[k].pdst; end
  end

  preg_t [3*W-1:0] ibt_rp, fbt_rp;
  logic  [3*W-1:0] ibt_busy, fbt_busy;
  always_comb begin
    for (int s = 0; s < W; s++)
      for (int p = 0; p < 3; p++) begin
        ibt_rp[3*s+p] = irat_rd[s][p];
        fbt_rp[3*s+p] = frat_rd[s][p];
      end
  end

  o6_busytable #(.N(N_IPREG), .NRD(3*W), .NWAKE(N_IWAKE)) u_ibt (
    .clk_i, .rst_ni, .flush_i,
    .set_en_i(ibs_en), .set_preg_i(new_ipdst),
    .clr_en_i(iclr_en), .clr_preg_i(iclr_p),
    .rd_preg_i(ibt_rp), .rd_busy_o(ibt_busy)
  );
  o6_busytable #(.N(N_FPREG), .NRD(3*W), .NWAKE(N_FWAKE)) u_fbt (
    .clk_i, .rst_ni, .flush_i,
    .set_en_i(fw_en), .set_preg_i(new_fpdst),
    .clr_en_i(fclr_en), .clr_preg_i(fclr_p),
    .rd_preg_i(fbt_rp), .rd_busy_o(fbt_busy)
  );

  // 被消除的 move 的源与源的 busy (上面 new_pdst 用)
  always_comb begin
    for (int s = 0; s < W; s++) begin
      logic [4:0] la;
      la = in_uop_i[s].mvsrc ? in_uop_i[s].lrs2 : in_uop_i[s].lrs1;
      mv_ps[s] = irat_rd[s][in_uop_i[s].mvsrc ? 1 : 0];
      mv_bz[s] = ibt_busy[3*s + (in_uop_i[s].mvsrc ? 1 : 0)];
      for (int j = 0; j < s; j++)
        if (alloc_i[j] && in_uop_i[j].lrd == la) begin mv_ps[s] = ifl_cand[ipos[j]]; mv_bz[s] = 1'b1; end
      if (la == 5'd0) begin mv_ps[s] = '0; mv_bz[s] = 1'b0; end
      ibs_en[s] = fire && alloc_i[s];
    end
  end

  //--------------------------------------------------------------------------
  // 组内旁路，生成重命名结果
  //--------------------------------------------------------------------------
  uop_t [W-1:0] rn_uop;
  always_comb begin
    for (int s = 0; s < W; s++) begin
      uop_t u;
      rclass_e cls [3];
      logic [4:0] la [3];
      preg_t  ps [3];
      logic   bz [3];
      preg_t  old;

      u = in_uop_i[s];
      cls[0] = u.rs1_class; cls[1] = u.rs2_class; cls[2] = u.rs3_en ? RC_FP : RC_NONE;
      la[0]  = u.lrs1;      la[1]  = u.lrs2;      la[2]  = u.lrs3;

      for (int p = 0; p < 3; p++) begin
        if (cls[p] == RC_FP) begin ps[p] = frat_rd[s][p]; bz[p] = fbt_busy[3*s+p]; end
        else                 begin ps[p] = irat_rd[s][p]; bz[p] = ibt_busy[3*s+p]; end
        for (int j = 0; j < s; j++) begin
          if (in_valid_i[j] && cls[p] != RC_NONE && in_uop_i[j].rd_class == cls[p] &&
              in_uop_i[j].lrd == la[p]) begin
            ps[p] = new_pdst[j]; bz[p] = elim[j] ? mv_bz[j] : 1'b1;
          end
        end
        if (cls[p] == RC_NONE) begin ps[p] = '0; bz[p] = 1'b0; end
      end

      old = (u.rd_class == RC_FP) ? frat_rd[s][3] : irat_rd[s][3];
      for (int j = 0; j < s; j++)
        if (in_valid_i[j] && u.rd_class != RC_NONE && in_uop_i[j].rd_class == u.rd_class &&
            in_uop_i[j].lrd == u.lrd)
          old = new_pdst[j];

      u.psrc1 = ps[0]; u.src1_busy = bz[0];
      u.psrc2 = ps[1]; u.src2_busy = bz[1];
      u.psrc3 = ps[2]; u.src3_busy = bz[2];
      u.pdst     = (u.rd_class == RC_NONE) ? '0 : new_pdst[s];
      u.old_pdst = (u.rd_class == RC_NONE) ? '0 : old;
      // O1：乱序向量 uop (目的 = 向量物理寄存器，借 pdst 字段；rd_class 是 RC_NONE，整数 / 浮点路径不看它)
      u.vfast = v_f[s] || v_s[s];
      u.vnw   = v_nw[s];
      if (v_f[s]) begin u.pdst = v_pd[s]; u.vps = v_src[s]; u.vpg = v_sg[s]; u.vpd = v_pg[s]; end
      if (v_s[s]) u.vps = v_src[s];

      // 分支掩码
      u.br_mask = br_busy_now;
      for (int j = 0; j < s; j++)
        if (is_br[j]) u.br_mask[slot_tag[j]] = 1'b1;
      u.br_tag = is_br[s] ? slot_tag[s] : '0;
      rn_uop[s] = u;
    end
  end

  //--------------------------------------------------------------------------
  // 分支标签状态
  //--------------------------------------------------------------------------
  br_mask_t nb0;                  // 下一拍的 br_busy，不含本拍分配 (kill 那拍 fire 必为 0)
  lo2_t     tg_n [3];             // 本拍分配 0 / 1 / 2 个标签时下一拍的两个最低空闲标签
  always_comb begin
    br_mask_t f;
    nb0 = br_busy & ~br_upd_i.resolve_mask;
    if (br_upd_i.kill) begin
      for (int t = 0; t < MAX_BR; t++)
        if (tag_mask[t][br_upd_i.kill_tag]) nb0[t] = 1'b0;
      nb0[br_upd_i.kill_tag] = 1'b0;
    end
    // 本拍分配的就是 tg_q.t0 (、tg_q.t1)：它们在 ~br_busy 里空闲，也就在 ~nb0 里
    f = ~nb0;
    tg_n[0] = lo2(f);
    f[tg_q.t0] = 1'b0; tg_n[1] = lo2(f);
    f[tg_q.t1] = 1'b0; tg_n[2] = lo2(f);
  end
  localparam lo2_t TG_RST = {1'b1, 1'b1, br_tag_t'(1), br_tag_t'(0)};   // 全空闲
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      br_busy <= '0; tg_q <= TG_RST;
    end else if (flush_i) begin
      br_busy <= '0; tg_q <= TG_RST;
    end else begin
      br_mask_t nb;
      nb = nb0;
      if (fire)
        for (int s = 0; s < W; s++)
          if (is_br[s]) nb[slot_tag[s]] = 1'b1;
      br_busy <= nb;
      tg_q    <= (!fire || n_br == 3'd0) ? tg_n[0] : (n_br == 3'd1) ? tg_n[1] : tg_n[2];
    end
  end
`ifndef SYNTHESIS
  // 寄存的标签与原写法 (本拍在 ~br_busy 上找两个最低空闲位) 一致
  always_ff @(posedge clk_i) if (rst_ni) begin
    lo2_t rf;
    rf = lo2(~br_busy);
    assert (tg_q == rf) else $error("rename: 寄存的分支标签 tg_q=%h 与 ~br_busy 现找的 %h 不一致", tg_q, rf);
  end
`endif

  // 标签掩码：每拍都要清掉已解析的位。
  // 否则标签 k 解析后被重新分配，某个存活标签 t 的掩码里残留的旧 k 位会让
  // "kill 新 k" 误把比新 k 老的 t 一起回收。下一值集中在一处算。
  always_ff @(posedge clk_i) begin
    for (int t = 0; t < MAX_BR; t++) begin
      br_mask_t m;
      m = tag_mask[t] & ~br_upd_i.resolve_mask;
      for (int s = 0; s < W; s++)
        if (fire && is_br[s] && slot_tag[s] == br_tag_t'(t))
          m = rn_uop[s].br_mask;        // 分支自己的 br_mask (比它老的分支)
      tag_mask[t] <= m;
    end
  end

  // free list 快照：分配到槽 s (含) 为止之后的 head
  always_ff @(posedge clk_i) begin
    if (fire) begin
      for (int s = 0; s < W; s++) begin
        if (is_br[s]) begin
          snap_ihead[slot_tag[s]] <= fl_add_i(ifl_head, ipos[s] + 3'(alloc_i[s]));
          snap_fhead[slot_tag[s]] <= fl_add_f(ffl_head, fpos[s] + 3'(need_f[s]));
          snap_vhead[slot_tag[s]] <= fl_add_v(vfl_head, vpos[s] + vng_s[s]);
        end
      end
    end
  end

  function automatic logic [IFL_W:0] fl_add_i(input logic [IFL_W:0] p, input logic [2:0] k);
    logic [IFL_W:0] s;
    s = {1'b0, p[IFL_W-1:0]} + k;
    if (s >= N_IPREG) return {~p[IFL_W], IFL_W'(s - N_IPREG)};
    else              return {p[IFL_W], s[IFL_W-1:0]};
  endfunction
  function automatic logic [FFL_W:0] fl_add_f(input logic [FFL_W:0] p, input logic [2:0] k);
    logic [FFL_W:0] s;
    s = {1'b0, p[FFL_W-1:0]} + k;
    if (s >= N_FPREG) return {~p[FFL_W], FFL_W'(s - N_FPREG)};
    else              return {p[FFL_W], s[FFL_W-1:0]};
  endfunction
  function automatic logic [VFL_W:0] fl_add_v(input logic [VFL_W:0] p, input logic [3:0] k);
    logic [VFL_W:0] s;
    s = {1'b0, p[VFL_W-1:0]} + k;
    if (s >= VCP) return {~p[VFL_W], VFL_W'(s - VCP)};          // O1：向量空闲表只管核内池
    else              return {p[VFL_W], s[VFL_W-1:0]};
  endfunction

  //--------------------------------------------------------------------------
  // RN/DS 流水寄存器：继续监听唤醒与分支广播
  //--------------------------------------------------------------------------
  function automatic logic woken(input preg_t p, input logic is_fp,
                                 input wakeup_t [N_IWAKE-1:0] iw, input wakeup_t [N_FWAKE-1:0] fw);
    logic r;
    r = 1'b0;
    if (is_fp) begin
      for (int k = 0; k < N_FWAKE; k++) if (fw[k].valid && fw[k].pdst == p) r = 1'b1;
    end else begin
      for (int k = 0; k < N_IWAKE; k++) if (iw[k].valid && iw[k].pdst == p) r = 1'b1;
    end
    return r;
  endfunction

  // 在队列里等待的组：每拍清已解析的分支位、按唤醒清 busy
  function automatic uop_t hold_upd(input uop_t u, input br_update_t bu,
                                    input wakeup_t [N_IWAKE-1:0] iw, input wakeup_t [N_FWAKE-1:0] fw);
    uop_t r;
    r = u;
    r.br_mask = u.br_mask & ~bu.resolve_mask;
    if (woken(u.psrc1, u.rs1_class == RC_FP, iw, fw)) r.src1_busy = 1'b0;
    if (woken(u.psrc2, u.rs2_class == RC_FP, iw, fw)) r.src2_busy = 1'b0;
    if (woken(u.psrc3, 1'b1, iw, fw))                 r.src3_busy = 1'b0;
    return r;
  endfunction

  //--------------------------------------------------------------------------
  // RN1 输出队列 (rq0 / q1，2 项) + RN2 输出级 (2026-10-18，虚拟 2nm 6 GHz)
  //   原来分派路由 grp_need 用本拍的 in_uop_i 算、与重命名串在同一拍 (2nm 综合 75 级 414 ps：
  //   6 条逐条按占用比较分到哪个队列)。现在 rq0 进 RN2 时用已寄存的 uop 算，独占一拍，随组寄存给分派级。
  //   RN2 与队列一样每拍监听唤醒 / 清已解析分支位；kill / 冲刷时一并清空 (都比任何已分派的分支新)。
  //   代价：前端多一拍 (误预测 / 冲刷后重填多 1 拍)，不在任何数据依赖环里。
  //--------------------------------------------------------------------------
  logic [W-1:0] rq0_v;
  uop_t [W-1:0] rq0_u, q1_u;
  gn_cls_t      rq0_cls, q1_cls;                     // 两级队列各组的分类 (grp_need 用，随组寄存)
  logic out_any_q;                                   // = |out_valid_o (寄存副本：infl 门控与 o_mv 不等 6 位或)
  wire o_mv   = !out_any_q || out_ready_i;           // RN2 本拍腾出 (空，或被分派级取走)
  wire q_pop  = (|rq0_v) && o_mv;                    // rq0 进 RN2
  wire q0_mv  = !(|rq0_v) || q_pop;                  // rq0 本拍腾出
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rq0_v <= '0; q1_v <= '0; out_valid_o <= '0; out_any_q <= 1'b0;
    end else if (flush_i || br_upd_i.kill) begin
      rq0_v <= '0; q1_v <= '0; out_valid_o <= '0; out_any_q <= 1'b0;
    end else begin
      if (o_mv) begin out_valid_o <= rq0_v; out_any_q <= |rq0_v; end
      if (q0_mv) begin
        if (|q1_v) begin rq0_v <= q1_v; q1_v <= fire ? in_valid_i : '0; rq0_cls <= q1_cls; end
        else       begin rq0_v <= fire ? in_valid_i : '0; rq0_cls <= gn_cls(fire ? in_valid_i : '0, in_uop_i); end
        if (|q1_v && fire) q1_cls <= gn_cls(in_valid_i, in_uop_i);
      end else if (fire) begin
        q1_v <= in_valid_i; q1_cls <= gn_cls(in_valid_i, in_uop_i);
      end
    end
  end

  // 路由用的空位要扣掉「本拍正被分派走的那一组」：它占的队列空位寄存的 iq_free 还没反映，
  // 不扣的话新一组会被分到实际已满的队列 (u3 实测：IQ_M 分派停顿 6.8 万 -> 19.8 万拍，CoreMark -2.85 %)
  logic [6:0] free_adj [N_IQ];        // 参考模型 / 断言用 (饱和减)
  logic [2:0] infl [N_IQ];            // 在途组的需求 (输出级有效时)，并行版 grp_need 直接用
  always_comb for (int q = 0; q < N_IQ; q++) infl[q] = out_any_q ? out_need_o.iqn[q] : 3'd0;
  always_comb begin
    for (int q = 0; q < N_IQ; q++) begin
      logic [6:0] t;
      t = (|out_valid_o) ? 7'(out_need_o.iqn[q]) : 7'd0;     // o_mv 时输出级若有效，就是本拍被分派走
      free_adj[q] = (iq_free_i[q] > t) ? iq_free_i[q] - t : 7'd0;
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) if (rst_ni)
    assert (out_any_q == (|out_valid_o)) else $error("rename: out_any_q 与 |out_valid_o 不一致");
  // 并行版 grp_need 与原逐条写法 (grp_need_ref) 必须逐位一致
  always_ff @(posedge clk_i)
    if (rst_ni && o_mv && |rq0_v) begin
      ds_need_t gp, gr;
      assert (rq0_cls == gn_cls(rq0_v, rq0_u)) else $error("rename: 寄存的组分类 rq0_cls 与 rq0 不符");
      gp = grp_need    (rq0_cls, iq_free_i[IQ_A], iq_free_i[IQ_B], iq_free_i[IQ_C], iq_free_i[IQ_M], iq_free_i[IQ_N],
                        infl[IQ_A], infl[IQ_B], infl[IQ_C], infl[IQ_M], infl[IQ_N]);
      gr = grp_need_ref(rq0_v, rq0_u, free_adj[IQ_A], free_adj[IQ_B], free_adj[IQ_C], free_adj[IQ_M], free_adj[IQ_N]);
      assert (gp == gr) else $error("rename: grp_need 并行版与参考不一致 par=%h ref=%h", gp, gr);
    end
`endif
  // 比较位预计算一拍 (2026-10-28，6 GHz：b160u55 out_need 环 177 ps，gn_loop 单独综合 179 ps，起点是 IQ 空位)：
  //   pre_q：每拍用本拍的 IQ 空位、输出级在途组 (数值 infl) 与下一拍 rq0 那组的 nbf 算好比较向量，下一拍用；
  //   pv_q：下一拍要补的"上一拍还没算进去的在途组" —— 本拍 o_mv 且 rq0 有组时就是本组 (下一拍它在输出级)，
  //   否则无 (输出级不换组时那组已在 pre_q 里)。门控在寄存前做完，下一拍从寄存器出来只剩一级与或选位
  gn_cls_t rq0_cls_n;                 // rq0_cls 的下一拍值 (与上面 always_ff 的写法一致)
  always_comb begin
    rq0_cls_n = rq0_cls;
    if (!(flush_i || br_upd_i.kill) && q0_mv)
      rq0_cls_n = (|q1_v) ? q1_cls : gn_cls(fire ? in_valid_i : '0, in_uop_i);
  end
  gn_pre_t pre_q;
  gn_inf_t pv_q;
  logic    o_mv_q;
  gn_res_t gn_n;
  always_ff @(posedge clk_i) pre_q <= gn_pre(iq_free_i[IQ_A], iq_free_i[IQ_B], iq_free_i[IQ_C], iq_free_i[IQ_M], iq_free_i[IQ_N],
                                             infl[IQ_A], infl[IQ_B], infl[IQ_C], infl[IQ_M], infl[IQ_N], rq0_cls_n.nbf);
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni)                          begin o_mv_q <= 1'b0; pv_q <= GN_INF_NONE; end
    else if (flush_i || br_upd_i.kill)    begin o_mv_q <= o_mv; pv_q <= GN_INF_NONE; end
    else                                  begin o_mv_q <= o_mv; pv_q <= (o_mv && |rq0_v) ? gn_n.inf : GN_INF_NONE; end
  always_comb gn_n = grp_need_pp(rq0_cls, pre_q, pv_q);
`ifndef SYNTHESIS
  // 参考：数值版 grp_need，用上一拍的空位、上一拍输出级在途 + (上一拍 o_mv 时) 本拍输出级在途
  logic [6:0] free_q [N_IQ];
  logic [2:0] infl_q [N_IQ];
  always_ff @(posedge clk_i) begin free_q <= iq_free_i; infl_q <= infl; end
  always_ff @(posedge clk_i)
    if (rst_ni && o_mv && |rq0_v) begin
      ds_need_t gv;
      logic [3:0] t [N_IQ];
      for (int q = 0; q < N_IQ; q++) t[q] = 4'(infl_q[q]) + ((o_mv_q && out_any_q) ? 4'(out_need_o.iqn[q]) : 4'd0);
      gv = grp_need(rq0_cls, free_q[IQ_A], free_q[IQ_B], free_q[IQ_C], free_q[IQ_M], free_q[IQ_N],
                    t[IQ_A], t[IQ_B], t[IQ_C], t[IQ_M], t[IQ_N]);
      assert (gn_n.need == gv) else $error("rename: grp_need_pp 与数值版 grp_need (上一拍空位 + 两组在途) 不一致 pp=%h val=%h", gn_n.need, gv);
    end
`endif
  always_ff @(posedge clk_i) begin
    if (o_mv) out_need_o <= gn_n.need;
    for (int s = 0; s < W; s++) begin
      if (o_mv) out_uop_o[s] <= hold_upd(rq0_u[s], br_upd_i, iwake_i, fwake_i);
      else      out_uop_o[s] <= hold_upd(out_uop_o[s], br_upd_i, iwake_i, fwake_i);
      if (q0_mv && (|q1_v))  rq0_u[s] <= hold_upd(q1_u[s], br_upd_i, iwake_i, fwake_i);
      else if (q0_mv)        rq0_u[s] <= rn_uop[s];
      else                   rq0_u[s] <= hold_upd(rq0_u[s], br_upd_i, iwake_i, fwake_i);
      if (fire && (!q0_mv || (|q1_v))) q1_u[s] <= rn_uop[s];
      else                              q1_u[s] <= hold_upd(q1_u[s], br_upd_i, iwake_i, fwake_i);
    end
  end
endmodule
