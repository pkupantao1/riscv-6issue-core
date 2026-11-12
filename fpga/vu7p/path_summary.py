#!/usr/bin/env python3
"""汇总 Vivado report_timing 的路径：按 (起点模块, 终点模块) 分组，给出最差裕量、逻辑级数、条数。
用法：path_summary.py paths.rpt [层级深度=3]"""
import re
import sys
from collections import defaultdict


def mod(pin, depth):
    parts = pin.split("/")[:-2]       # 去掉引脚名和寄存器名
    return "/".join(parts[:depth]) or "(top)"


def main():
    f = sys.argv[1]
    depth = int(sys.argv[2]) if len(sys.argv) > 2 else 3
    groups = defaultdict(lambda: [0.0, 0, 0, ""])
    slack = src = None
    for ln in open(f, errors="replace"):
        m = re.match(r"Slack \(\w+\)\s*:\s*(-?[\d.]+)ns", ln)
        if m:
            slack = float(m.group(1))
            continue
        m = re.match(r"\s+Source:\s+(\S+)", ln)
        if m:
            src = m.group(1)
            continue
        m = re.match(r"\s+Destination:\s+(\S+)", ln)
        if m and slack is not None:
            dst = m.group(1)
            continue
        m = re.match(r"\s+Logic Levels:\s+(\d+)", ln)
        if m and slack is not None:
            k = (mod(src, depth), mod(dst, depth))
            g = groups[k]
            if g[2] == 0 or slack < g[0]:
                g[0], g[1], g[3] = slack, int(m.group(1)), f"{src} -> {dst}"
            g[2] += 1
            slack = None
    rows = sorted(groups.items(), key=lambda kv: kv[1][0])
    print(f"{'裕量ns':>8} {'级数':>4} {'条数':>4}  起点模块 -> 终点模块")
    for (s, d), (sl, lv, n, ex) in rows:
        print(f"{sl:8.2f} {lv:4d} {n:4d}  {s} -> {d}")


if __name__ == "__main__":
    main()
