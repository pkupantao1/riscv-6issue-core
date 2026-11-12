#!/usr/bin/env python3
"""单元级变异：给 free list / IQ / 转发搜索植入典型错误，确认单元 TB (tb/unit) 与形式 F3 抓得住 (docs/13 §7)
   python verif/mut/unit_mut.py [ID ...]      (在 132 上；每个变异 = 复制 rtl 树 + 精确串替换 + run_unit.sh / run_fv.sh)
   结果追加到 work/mut/unit_results.txt；每个变异串行跑 (单核)
   RTL 基线取 $SRC (默认本仓库)；转发搜索拆成 o6_sq_fwd 之前用 SRC=/export/master/o6_dev_fwd
"""
import os, re, shutil, subprocess, sys

S = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC = os.environ.get("SRC", S)
M = "/export/master/o6_umut"

# (ID, 说明, 单元, 文件, 旧串, 新串)；单元 = freelist / iq / lsu_fwd / fv:freelist
MUTS = [
    ("UF1", "冲刷时 head 不加本拍提交数", "freelist", "rtl/backend/o6_freelist.sv",
     "        head <= ptr_add(arch_head, {1'b0, commit_alloc_cnt_i});", "        head <= arch_head;"),
    ("UF2", "指针回绕比较 >= 写成 >", "freelist", "rtl/backend/o6_freelist.sv",
     "    if (s >= N) return {~p[AW], AW'(s - N)};", "    if (s > N) return {~p[AW], AW'(s - N)};"),
    ("UF3", "入队不压缩 (按槽号写)", "freelist", "rtl/backend/o6_freelist.sv",
     "          p = ptr_add(tail, off);", "          p = ptr_add(tail, 4'(k));"),
    ("FF1", "F3：冲刷时 head 不加本拍提交数", "fv:freelist", "rtl/backend/o6_freelist.sv",
     "        head <= ptr_add(arch_head, {1'b0, commit_alloc_cnt_i});", "        head <= arch_head;"),
    ("FF3", "F3：入队不压缩", "fv:freelist", "rtl/backend/o6_freelist.sv",
     "          p = ptr_add(tail, off);", "          p = ptr_add(tail, 4'(k));"),
    ("UQ1", "端口 1 取最老而不是最年轻", "iq", "rtl/backend/o6_iq.sv",
     "        g_yng1[i] = req[NP-1][i] && !(|(req[NP-1] & younger(i)));",
     "        g_yng1[i] = req[NP-1][i] && !(|(req[NP-1] & column(i)));"),
    ("UQ2", "入队口不听确认唤醒 cwake", "iq", "rtl/backend/o6_iq.sv",
     "      if (chit(u.psrc1, u.rs1_class == RC_FP, cwake_i)) u.src1_busy = 1'b0;",
     "      if (1'b0) u.src1_busy = 1'b0;"),
    ("UQ3", "判定失败的发射照样释放", "iq", "rtl/backend/o6_iq.sv",
     "      vld <= (vld & ~killed & ~((issued & ~iss_pa & ~iss_pb) & ~fhit) & ~(isw1 & ~fhit)) | enq_ok;",
     "      vld <= (vld & ~killed & ~(issued & ~iss_pa & ~iss_pb) & ~(isw1 & ~fhit)) | enq_ok;"),
    ("UQ4", "入队口不听 load 的 L1 唤醒 ewake", "iq", "rtl/backend/o6_iq.sv",
     "    for (int k = 0; k < N_LDP; k++) iw_enq[N_IWAKE+k] = ewake_i[k];",
     "    for (int k = 0; k < N_LDP; k++) iw_enq[N_IWAKE+k] = '0;"),
    ("UQ5", "发射时不带 poia", "iq", "rtl/backend/o6_iq.sv",
     "iss_uop_o[p].poia = epa[i];", "iss_uop_o[p].poia = '0;"),
    ("UQ6", "入队不看本拍的分支杀", "iq", "rtl/backend/o6_iq.sv",
     "        if (wsel[i][k] && !br_killed(enq_uop_i[k].br_mask, br_upd_i)) enq_ok[i] = 1'b1;",
     "        if (wsel[i][k]) enq_ok[i] = 1'b1;"),
    ("UQ7", "wait_head 项头部判定不排除本拍写入的项", "iq", "rtl/backend/o6_iq.sv",
     "      hd_q[i]  <= vld[i] && !wany[i] && ent[i].rob_idx == rob_head_i;",
     "      hd_q[i]  <= ent[i].rob_idx == rob_head_i;"),
    ("UD1", "#18：load 不合并到 store 本拍新分配的 MSHR", "dcache", "rtl/cache/o6_dcache.sv",
     "      if (st_miss_alloc && spa[PA_W-1:6] == ld_paddr_i[k][PA_W-1:6]) begin",
     "      if (1'b0 && st_miss_alloc && spa[PA_W-1:6] == ld_paddr_i[k][PA_W-1:6]) begin"),
    ("UD2", "预取不查已有同行 MSHR", "dcache", "rtl/cache/o6_dcache.sv",
     "!(|lookup(pf_paddr_i, tag, valid)) && !(|mshr_match(pl, m_st, m_line)) &&",
     "!(|lookup(pf_paddr_i, tag, valid)) &&"),
    ("UD3", "store 命中不置脏", "dcache", "rtl/cache/o6_dcache.sv",
     "                dirty[spa[11:6]][oh2i(s_hit_oh)] <= 1'b1;",
     "                dirty[spa[11:6]][oh2i(s_hit_oh)] <= 1'b0;"),
    ("UD4", "回填写回缓冲取错路", "dcache", "rtl/cache/o6_dcache.sv",
     "            for (int b = 0; b < 8; b++) wb_data[64*b +: 64] <= b_rdata[b][64*f_way +: 64];",
     "            for (int b = 0; b < 8; b++) wb_data[64*b +: 64] <= b_rdata[b][64*(f_way ^ 1'b1) +: 64];"),
    ("FM1", "F4：#18 撤销", "fv:mshr", "rtl/cache/o6_dcache.sv",
     "      if (st_miss_alloc && spa[PA_W-1:6] == ld_paddr_i[k][PA_W-1:6]) begin",
     "      if (1'b0 && st_miss_alloc && spa[PA_W-1:6] == ld_paddr_i[k][PA_W-1:6]) begin"),
    ("FM2", "F4：预取不查已有同行 MSHR", "fv:mshr", "rtl/cache/o6_dcache.sv",
     "!(|lookup(pf_paddr_i, tag, valid)) && !(|mshr_match(pl, m_st, m_line)) &&",
     "!(|lookup(pf_paddr_i, tag, valid)) &&"),
    ("UM1", "#32：非叶 PTE 的 D/A/U 不查", "mmu", "rtl/mmu/o6_ptw.sv",
     "(pte[63:54] != 10'd0) || (!leaf && (pte[7] || pte[6] || pte[4]));", "(pte[63:54] != 10'd0);"),
    ("UM2", "2M 超页不查对齐", "mmu", "rtl/mmu/o6_ptw.sv",
     "if ((lvl == 2'd2 && pte[27:10] != 18'd0) || (lvl == 2'd1 && pte[18:10] != 9'd0)) begin",
     "if (lvl == 2'd2 && pte[27:10] != 18'd0) begin"),
    ("UM3", "TLB 2M 项比较位错", "mmu", "rtl/mmu/o6_tlb.sv",
     "      2'd1:    return a[26:9]  == b[26:9];", "      2'd1:    return a[26:10] == b[26:10];"),
    # 只去掉 P_WAIT 里的 abort 判断是等价变异 (abort 在 mem_req / P_REQ / 填写处还有三道)：要连填写的门控一起去掉
    ("UM4", "PTW 等数据时不理冲刷，且填写不看 abort", "mmu", "rtl/mmu/o6_ptw.sv",
     ("            if (abort || flush_i) begin",
      "assign ifill_o      = (st == P_DONE) && !is_d && !abort && !flush_i;",
      "assign dfill_o      = (st == P_DONE) &&  is_d && !abort && !flush_i;"),
     ("            if (flush_i) begin",
      "assign ifill_o      = (st == P_DONE) && !is_d && !flush_i;",
      "assign dfill_o      = (st == P_DONE) &&  is_d && !flush_i;")),
    ("UM5", "PTW 不查页表访问的 PMP", "mmu", "rtl/mmu/o6_ptw.sv",
     "          else if (!mem_ok(a) || !pmp_ok) begin", "          else if (!mem_ok(a)) begin"),
    ("UR1", "撤销 #35 (只有 wait_head 项启动后才挡中断)", "rob", "rtl/backend/o6_rob.sv",
     "  wire        head_locked = head_ent.started || head_start_i;",
     "  wire        head_locked = head_ent.wait_head && (head_ent.started || head_start_i);"),
    ("UR2", "非法指令候选刚分派那拍也发起陷入", "rob", "rtl/backend/o6_rob.sv",
     "          if (c == head_col && !dx_v) trap_excp = 1'b1;", "          if (c == head_col) trap_excp = 1'b1;"),
    ("UR3", "单步不限每拍 1 条", "rob", "rtl/backend/o6_rob.sv",
     "          if (step_i) go = 1'b0;", "          if (1'b0) go = 1'b0;"),
    ("UR4", "flush_after 不停止提交", "rob", "rtl/backend/o6_rob.sv",
     "          if (hent[c].flush_after) begin", "          if (1'b0) begin"),
    ("FR1", "F6：头行提交完不换行 (row_done 只看列 0)", "fv:rob", "rtl/backend/o6_rob.sv",
     "      if (hvld[c]) row_done = commit[c];", "      if (hvld[c] && c == 0) row_done = commit[c];"),
    ("FB1", "F5：表项不清本拍解析的标签", "fv:brmask", "rtl/backend/o6_iq.sv",
     "    r.br_mask = u.br_mask & ~bu.resolve_mask;", "    r.br_mask = u.br_mask;"),
    ("UL1", "#26 类：sq_older 正好满判错", "lsu_fwd", "rtl/pkg/o6_pkg.sv",
     "  function automatic logic sq_older(input sq_ptr_t a, input sq_ptr_t b);\n    return (a.flag == b.flag) ? (a.idx < b.idx) : (a.idx >= b.idx);",
     "  function automatic logic sq_older(input sq_ptr_t a, input sq_ptr_t b);\n    return (a.flag == b.flag) ? (a.idx < b.idx) : (a.idx > b.idx);"),
    ("UL2", "取最老而不是最年轻的较老 store", "lsu_fwd", "rtl/lsu/o6_sq_fwd.sv",
     "    p = (|lo) ? lo : c;", "    p = (|lo) ? lo & ~(lo - 1'b1) : c & ~(c - 1'b1);"),
    ("UL3", "不比较双字地址", "lsu_fwd", "rtl/lsu/o6_sq_fwd.sv",
     "sq_older(ep, ld_sq_i) && paddr_i[e][PA_W-1:3] == ld_pa_i[PA_W-1:3];", "sq_older(ep, ld_sq_i);"),
    ("UL4", "数据未就绪的 store 也转发 (不等)", "lsu_fwd", "rtl/lsu/o6_sq_fwd.sv",
     "      wohs = wohs | (oh & ~fok_i);", "      wohs = wohs;"),
]


def sh(cmd, cwd=None):
    return subprocess.run(["bash", "-c", cmd], cwd=cwd, capture_output=True, text=True)


def one(mut):
    mid, what, unit, f, old, new = mut
    T = f"{M}/{mid}"
    shutil.rmtree(T, ignore_errors=True)
    os.makedirs(T)
    shutil.copytree(f"{SRC}/rtl", f"{T}/rtl", symlinks=True)
    p = f"{T}/{f}"
    src = open(p, encoding="utf-8").read()
    # 旧串 / 新串可以是等长的元组：一个变异改几处 (例如去掉冗余的两道保护)
    olds, news = (old, new) if isinstance(old, tuple) else ((old,), (new,))
    for o, n in zip(olds, news):
        if src.count(o) != 1:
            return f"{mid} | {unit} | 补丁失败 | 旧串出现 {src.count(o)} 次 ({what})"
        src = src.replace(o, n)
    open(p, "w", encoding="utf-8").write(src)
    if unit.startswith("fv:"):
        fv = "fv_" + unit[3:]
        sh(f"FV_RTL={T} FV_TAG={mid} TIMEOUT=2400 bash verif/formal/run_fv.sh {fv}", cwd=S)
        pf = f"{S}/work/formal/{fv}_{mid}/{fv}.props"      # 每行「性质 结论 长度」(tcl 写)
        props = open(pf, encoding="utf-8").read() if os.path.exists(pf) else ""
        cex = [l.split()[0].split(".")[-1] for l in props.splitlines() if len(l.split()) > 1 and l.split()[1] == "cex"]
        caught = bool(cex)
        return f"{mid} | {unit} | {'抓到' if caught else '没抓到'} | {what}；反例 {' '.join(cex[:4])}"
    args = {"freelist": "3 +cycles=100000", "iq": "3 +cycles=30000", "lsu_fwd": "3 +vec=100000",
            "dcache": "3 +cycles=40000", "mmu": "3 +cycles=20000", "rob": "3 +cycles=40000"}[unit]
    r = sh(f"SRC={T} TAG={mid} timeout 3000 bash scripts/run_unit.sh {unit} {args}", cwd=S)
    m = re.search(r"== \S+：(\d+) / (\d+) 种子通过", r.stdout)
    if not m:
        return f"{mid} | {unit} | 编译 / 运行失败 | {what}；{r.stdout[-200:].strip()}"
    ok, n = int(m.group(1)), int(m.group(2))
    err = re.findall(r"错误：(\S+)", r.stdout)
    return f"{mid} | {unit} | {'抓到' if ok < n else '没抓到'} | {what}；{n - ok} / {n} 种子失败 {' '.join(err[:2])}"


def main():
    want = set(sys.argv[1:])
    os.makedirs(S + "/work/mut", exist_ok=True)
    for m in MUTS:
        if want and m[0] not in want:
            continue
        line = one(m)
        print(line, flush=True)
        with open(S + "/work/mut/unit_results.txt", "a", encoding="utf-8") as fo:
            fo.write(line + "\n")
    print("UMUT_DONE")


if __name__ == "__main__":
    main()
