#!/usr/bin/env python3
"""musthit.py - 必达 bin 检查：读 urg 文本报告 (grpinfo.txt) 与 scripts/musthit.txt

用法：musthit.py <cov_text 目录> [musthit.txt]
输出每一项的命中次数；阶段 V1 的项未中则退出码 1 (回归失败)，P1 的项未中只标「待补」。
grpinfo.txt 里每个 covergroup 先是类型级汇总 (Group : …)，再是实例 (Group Instance : …)，只读类型级那一份。
coverpoint 的命中表在「Summary for Variable <名>」到下一个「Summary for」之间，交叉在「Summary for Cross <名>」：
表行形如「<bin 名> <次数> <at least> …」，交叉每维一个名字：「hit hit <次数> …」。
"""
import os
import re
import sys


def parse(path):
    """返回 {(group, item): {bin 名: 次数}}"""
    text = open(path, errors="ignore").read()
    res = {}
    # 按类型级 group 切段 (实例段跳过)
    parts = re.split(r"^Group (Instance )?: (\S+)\s*$", text, flags=re.M)
    # re.split 带两个分组：[前文, inst?, name, 正文, inst?, name, 正文, ...]
    for k in range(1, len(parts) - 2, 3):
        inst, name, body = parts[k], parts[k + 1], parts[k + 2]
        if inst:
            continue
        g = name.split("::")[-1]
        for m in re.finditer(r"^Summary for (?:Variable|Cross) (\S+)\s*$(.*?)(?=^Summary for |\Z)", body,
                             flags=re.M | re.S):
            item, sec = m.group(1), m.group(2)
            bins = res.setdefault((g, item), {})
            for line in sec.splitlines():
                t = line.split()
                if len(t) < 2 or "COUNT" in t or t[0] in ("CATEGORY", "NAME"):
                    continue
                # 表行：一个或多个名字 (交叉每维一个，可能带 [ ]) 后跟次数
                k = next((i for i, x in enumerate(t) if x.isdigit()), None)
                if not k:
                    continue
                key = " ".join(x.strip("[]") for x in t[:k])
                bins[key] = max(bins.get(key, 0), int(t[k]))
    return res


def hits(bins, want):
    """bin 名完全相同，或以 _<want> 结尾 (数组 bin：c_C_LD)；交叉项写 hit 表示「每一维都是 hit」那一个 bin"""
    n = None
    for b, c in bins.items():
        names = b.split()
        if b == want or b.endswith("_" + want) or (len(names) > 1 and all(x == want for x in names)):
            n = c if n is None else max(n, c)
    return n


def main():
    cov = sys.argv[1]
    lst = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "musthit.txt")
    data = parse(cov + "/grpinfo.txt")
    fail = todo = ok = 0
    for line in open(lst, encoding="utf-8"):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        g, item, b, stage, *desc = line.split()
        bins = data.get((g, item))
        n = hits(bins, b) if bins is not None else None
        if n is None:
            st = "缺项" if stage == "V1" else "待补"
        elif n > 0:
            st = "命中"
        else:
            st = "未中" if stage == "V1" else "待补"
        if st == "命中":
            ok += 1
        elif stage == "V1":
            fail += 1
        else:
            todo += 1
        print("%-4s %-10s %-15s %-8s %-3s %8s  %s" % (st, g, item, b, stage, "-" if n is None else n, " ".join(desc)))
    print("MUSTHIT 命中 %d，V1 未中 %d，P1 待补 %d" % (ok, fail, todo))
    sys.exit(1 if fail else 0)


if __name__ == "__main__":
    main()
