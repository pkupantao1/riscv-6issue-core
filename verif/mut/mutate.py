#!/usr/bin/env python3
"""mutate.py - 检查器有效性：变异验证 (docs/13_verification_plan.md §7)

用法 (132 上)：mutate.py [变异编号 ...]      不给编号 = 全部；环境变量 MUT_JOBS = 同时编译 / 运行的变异数 (默认 2)
每个变异：在 /export/master/o6_mut/<编号> 建源码树 (rtl / tb 复制，其余符号链接到 dev24)，按「旧串 -> 新串」打补丁
(旧串必须恰好出现一次)，单独编译 simv_mut，跑「当初发现这类缺陷的那组用例」，判定是否被抓到：
  fail    任一用例 tb 判 FAIL / UVM 判 FAIL / 有 RTL 断言或 unique-priority 违例
  perf    基准的误预测率比基线相对升高 > 10% (性能缺陷对拍看不出，§8)
  musthit urg 合并后必达 bin 的 V1 项有未中 (例：中断驱动失效)
结果追加到 work/mut/results.txt：编号 | 判定 | 抓到 / 没抓到 | 细节
"""
import os
import re
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

S = "/export/master/riscv_ooo6_dev24"
M = "/export/master/o6_mut"
E = S + "/work/uvm/elf"
ISA = S + "/work/tests/isa"
ACT = S + "/work/act/o6-rv64gc/elfs"
C1 = ("+define+O6_ROB_ROWS=8 +define+O6_LQ_SIZE=16 +define+O6_SQ_SIZE=8 +define+O6_IQ_AB=12 +define+O6_IQ_M=12 "
      "+define+O6_IQ_C=12 +define+O6_IQ_SD=8 +define+O6_MAX_BR=8 +define+O6_FTQ_SIZE=8")


def rand(kind, s0, n):
    return [(f"{E}/u{kind}{s}", "") for s in range(s0, s0 + n)]


def act(*names):
    out = []
    for n in names:
        for root, _, files in os.walk(ACT):
            out += [(os.path.join(root, f), "+max_cycles=50000000") for f in files if f == n + ".elf"]
    return out


IRQ = [(f"{E}/irq_s{s}", f"+UVM_TESTNAME=o6_irq_test +ntb_random_seed={s}") for s in (1, 2, 3)]
IRQT = [(f"{E}/irqt_s{s}", f"+UVM_TESTNAME=o6_tmr_test +ntb_random_seed={s}") for s in (1, 2, 3)]

# (编号, 对应缺陷 / 检查器, 文件, 旧串, 新串, 用例, 判定, VDEFS)
MUTS = [
    ("M02", "#26 sq_older 正好满", "rtl/pkg/o6_pkg.sv",
     "  function automatic logic sq_older(input sq_ptr_t a, input sq_ptr_t b);\n    return (a.flag == b.flag) ? (a.idx < b.idx) : (a.idx >= b.idx);",
     "  function automatic logic sq_older(input sq_ptr_t a, input sq_ptr_t b);\n    return (a.flag == b.flag) ? (a.idx < b.idx) : (a.idx > b.idx);",
     # 随机程序 (含 C1 配置) 都压不出「SQ 正好被已提交 store 占满」(0/13，2026-10-21)；定向 sq_flush 在默认配置下抓到：
     # SQ 满时进来的 load 转发漏掉最老的 store，读到旧值
     [(f"{E}/sq_flush", "")] + rand("int", 8100, 4), "fail", ""),
    ("M03", "SB4 / SB6：提交漏槽", "rtl/backend/o6_rob.sv",
     "      cm_valid_o[c]    = commit[c];", "      cm_valid_o[c]    = commit[c] && (c != W - 1);",
     [(f"{ISA}/rv64ui-p-add", ""), (f"{E}/uint8100", "")], "fail", ""),
    ("M04", "SB2：中断 epc", "rtl/backend/o6_commit_ctrl.sv",
     "    trap_epc_o   = pc;", "    trap_epc_o   = q_irq ? pc + 64'd4 : pc;", IRQ, "fail", ""),
    ("M05", "SB3：写回 X", "rtl/exu/o6_div.sv",
     "          y_o <= u.is_w ? {{32{res[31]}}, res[31:0]} : res;", "          y_o <= u.is_w ? 64'bx : res;",
     [(f"{ISA}/rv64um-p-divw", ""), (f"{ISA}/rv64um-p-remw", "")] + rand("int", 8100, 4), "fail", ""),
    ("M06", "#5 入队不清本拍 resolve", "rtl/backend/o6_iq.sv",
     "    r.br_mask = u.br_mask & ~bu.resolve_mask;", "    r.br_mask = u.br_mask;",
     rand("int", 8100, 6) + [(f"{S}/work/tests/bench/dhrystone.riscv", "+max_cycles=50000000")], "fail", ""),
    ("M07", "#18 同一行两个 MSHR", "rtl/cache/o6_dcache.sv",
     "      if (st_miss_alloc && spa[PA_W-1:6] == ld_paddr_i[k][PA_W-1:6]) begin",
     "      if (1'b0 && st_miss_alloc && spa[PA_W-1:6] == ld_paddr_i[k][PA_W-1:6]) begin",
     # 随机 10 + memcpy + st_ld_line (core 模式) 与 SoC 模式随机 12 + memcpy (长延迟 / 反压 / 乱序) 都 0：STA 一定就发 store 预取，
     # 排空时不再缺失。定向 st_ld_mshr：先用 13 个冷 load 占住 MSHR 让预取被丢，排空缺失与同行 load 缺失按 d 扫描对齐
     [(f"{E}/st_ld_mshr", ""), (f"{E}/st_ld_line", "")] + rand("int", 8100, 2), "fail", ""),
    ("M08", "浮点 rtz / rdn 互换", "rtl/fpu/o6_fpu_exu.sv",
     "    return fpnew_pkg::roundmode_e'((rm == 3'b111) ? frm : rm);",
     "    return fpnew_pkg::roundmode_e'((((rm == 3'b111) ? frm : rm) == 3'd1) ? 3'd2 : (((rm == 3'b111) ? frm : rm) == 3'd2) ? 3'd1 : ((rm == 3'b111) ? frm : rm));",
     rand("fp", 8200, 6) + [(f"{ISA}/rv64uf-p-fcvt", ""), (f"{ISA}/rv64ud-p-fcvt", "")], "fail", ""),
    ("M09", "#25 TAGE 训练索引错位 (性能)", "rtl/frontend/o6_frontend.sv",
     "    tr_pc  <= ftq_hwm[trn_sel] ? ftq_pc[trn_sel] + 64'd2 : ftq_pc[trn_sel];", "    tr_pc  <= ftq_pc[trn_sel];",
     # 只影响「以跨块 RVI 开头」的取指块 (ftq_hwm)：dhrystone 里这种块上的历史相关分支很少 (0.73% vs 基线 0.72%，没抓到)；
     # 定向 bp_hwm 把三个要靠历史的分支都放进这种块 (基线 0.16%)
     [(f"{E}/bp_hwm", "")], "perf:bp_hwm:0.16", ""),
    ("M10", "必达 bin：中断驱动恒 0", "tb/uvm/o6_uvm_pkg.sv",
     "          vif.cb.meip_set <= req.kind[0]; vif.cb.seip_set <= req.kind[1];",
     "          vif.cb.meip_set <= 1'b0; vif.cb.seip_set <= 1'b0;", IRQ, "musthit", ""),
    ("M11", "#14 SFENCE 不等 SQ 排空", "rtl/backend/o6_commit_ctrl.sv",
     "      end else if (is_sfence && !sq_drained_i) begin", "      end else if (1'b0 && is_sfence && !sq_drained_i) begin",
     # -v 测试 (0/8) 与 rvgen --vm (0/12) 里改 PTE 的 store 都很快排空，窗口碰不上；
     # 定向 sfence_sq：60 个冷行 store 堵住 SQ 后改 PTE、sfence、立即经新映射读
     [(f"{E}/sfence_sq", "")] + rand("vm", 8400, 4), "fail", ""),
    ("M12", "#24 CSR 读译码 priority -> unique", "rtl/csr/o6_csr.sv",
     "    priority casez (ca)", "    unique casez (ca)", rand("csr", 8300, 6) + [(f"{ISA}/rv64mi-p-csr", "")], "fail", ""),
    ("M14", "#29 LR 按 AMO 检查", "rtl/lsu/o6_lsu.sv",
     "        is_ld = (l0r[k].u.fu == FU_LD) || (l0r[k].u.fu == FU_AMO && l0r[k].u.op == AMO_LR);",
     "        is_ld = (l0r[k].u.fu == FU_LD);", rand("vm", 8400, 6), "fail", ""),
    ("M15", "#30 mip 读改写锁存外部 SEIP", "rtl/csr/o6_csr.sv",
     "    if (ca == CSR_MIP || ca == CSR_SIP) rval_rmw[9] = mip_sw[9] & ((ca == CSR_MIP) | mideleg[9]);",
     "    if (1'b0) rval_rmw[9] = 1'b0;", IRQT, "fail", ""),
    ("M16", "#31 F2I 写 x0 死锁", "rtl/fpu/o6_fpu_exu.sv",
     "                       (x == 0) ? (in_u[x].rd_class != RC_FP) : (in_u[x].rd_class == RC_INT),",
     "                       in_u[x].rd_class == RC_INT,", act("F-feq.s-00", "D-fclass.d-00", "F-fcvt.w.s-00"), "fail", ""),
    ("M17", "#32 非叶 PTE 的 D/A/U", "rtl/mmu/o6_ptw.sv",
     " || (!leaf && (pte[7] || pte[6] || pte[4]));", ";",
     act("Sv_sv39_nleaf_pte_DAU_Smode-00", "Sv_sv39_nleaf_pte_DAU_Umode-00"), "fail", ""),
    ("M18", "#33 锁定 TOR 的下界", "rtl/csr/o6_csr.sv",
     "            if (ca[3] == 1'b0 && !pmpcfg[ca[2:0]][7] &&\n                !(ca[2:0] != 3'd7 && pmpcfg[ca[2:0] + 3'd1][7] && pmpcfg[ca[2:0] + 3'd1][4:3] == 2'b01))",
     "            if (ca[3] == 1'b0 && !pmpcfg[ca[2:0]][7])", act("PMPSm_cfg_L_modify_tor-00"), "fail", ""),
    ("M19", "#34 类：NAPOT 不按 64 B 粒度", "rtl/mmu/o6_pmp.sv",
     "  localparam logic [53:0] NAPOT_ONE = (54'd1 << (PMP_G - 1)) - 54'd1;",
     # rvgen --vm 的 NAPOT 都 >= 64 B，低位本来就是 1 (0/6)；ACT 的 PMP 用例按 GRANULARITY 6 生成，也只写合法值 (0/20)；
     # 定向 pmp_g64：pmpaddr 低位写 0 (原始编码 8 B)，G = 64 B 时区域内 +40 处的 store 必须报错 (形式 F2 另抓到)
     "  localparam logic [53:0] NAPOT_ONE = 54'd0;",
     [(f"{E}/pmp_g64", "")] + [(e, "+max_cycles=50000000") for e in sorted((os.path.join(r, f) for r, _, fs in os.walk(ACT) for f in fs
                                                  if f.startswith("PMP") and f.endswith(".elf")),
                                                 key=lambda e: "napot" not in e)[:4]],
     "fail", ""),
]


def sh(cmd, cwd=None):
    return subprocess.run(["bash", "-c", cmd], cwd=cwd, capture_output=True, text=True)


def one(mut):
    mid, what, f, old, new, tests, judge, vdefs, *xenv = mut   # 可选第 9 项：额外环境变量 (如 "SOC=1")
    xenv = xenv[0] if xenv else ""
    rdir = f"runs{'_soc' if 'SOC=1' in xenv else ''}_mut"
    T = f"{M}/{mid}"
    shutil.rmtree(T, ignore_errors=True)
    os.makedirs(T + "/work")
    for d in ("rtl", "tb"):
        shutil.copytree(f"{S}/{d}", f"{T}/{d}", symlinks=True)
    for d in ("scripts", "third_party", "verif"):
        os.symlink(f"{S}/{d}", f"{T}/{d}")
    for d in ("cosim", "tests", "rand", "uvm", "act"):
        os.symlink(f"{S}/work/{d}", f"{T}/work/{d}")
    p = f"{T}/{f}"
    src = open(p, encoding="utf-8").read()
    if src.count(old) != 1:
        return f"{mid} | {judge} | 补丁失败 | 旧串出现 {src.count(old)} 次 ({what})"
    open(p, "w", encoding="utf-8").write(src.replace(old, new))
    cov = "1" if judge == "musthit" else ""
    env = f"UVM=1 SIMTAG=mut COV={cov} VDEFS='{vdefs}' {xenv}"
    b = sh(f"ulimit -v 16000000; {env} nice -n 5 bash scripts/run_vcs.sh build", cwd=T)
    if "vcs rc=0" not in b.stdout:
        return f"{mid} | {judge} | 编译失败 | {b.stdout[-300:].strip()}"
    detail, bad = [], 0
    for elf, args in tests:
        if not os.path.exists(elf):
            detail.append(f"缺 {os.path.basename(elf)}")
            continue
        name = os.path.basename(elf)
        sh(f"{env} TIMEOUT=900 bash scripts/run_vcs.sh run {elf} {args} > /dev/null", cwd=T)
        log = open(f"{T}/work/vcs/{rdir}/{name}/sim.log", errors="ignore").read()
        tb_ok = "=== PASS ===" in log
        uvm_ok = "=== UVM PASS ===" in log
        nas = len(re.findall(r'^Error: "|RT-(MTOCMUCS|NCMUCS|MTOCMPCS|NCMPCS)', log, re.M))
        if not (tb_ok and uvm_ok and nas == 0):
            bad += 1
            why = re.search(r"=== FAIL === (\S+)", log)
            detail.append(f"{name}:{why.group(1) if why else 'UVM/断言'}{'(断言 %d)' % nas if nas else ''}")
        if judge.startswith("perf"):
            m = re.search(r"误预测率=([0-9.]+)%", log)
            rate = float(m.group(1)) if m else 0.0
            base = float(judge.split(":")[2])
            detail.append(f"{name} 误预测率 {rate:.2f}% (基线 {base:.2f}%)")
            caught = rate > base * 1.10
            return f"{mid} | perf | {'抓到' if caught else '没抓到'} | {what}；" + "；".join(detail)
    if judge == "musthit":
        sh(f"urg -dir work/vcs/simv_mut.vdb -report work/cov_text -format text -metric group > /dev/null 2>&1", cwd=T)
        r = sh(f"PYTHONIOENCODING=utf-8 python3 {S}/scripts/musthit.py work/cov_text {S}/scripts/musthit.txt", cwd=T)
        miss = [l.split()[2] + ":" + l.split()[3] for l in r.stdout.splitlines() if l.startswith("未中") and "cg_trap" in l]
        caught = any("mei" in x or "sei" in x for x in miss)
        return f"{mid} | musthit | {'抓到' if caught else '没抓到'} | {what}；未中 {' '.join(miss[:6])}"
    return f"{mid} | fail | {'抓到' if bad else '没抓到'} | {what}；失败 {bad}/{len(tests)} {' '.join(detail[:6])}"


def main():
    want = set(sys.argv[1:])
    todo = [m for m in MUTS if not want or m[0] in want]
    os.makedirs(S + "/work/mut", exist_ok=True)
    with ThreadPoolExecutor(int(os.environ.get("MUT_JOBS", "2"))) as ex:
        for line in ex.map(one, todo):
            print(line, flush=True)
            with open(S + "/work/mut/results.txt", "a", encoding="utf-8") as fo:
                fo.write(line + "\n")
    print("MUT_DONE")


if __name__ == "__main__":
    main()
