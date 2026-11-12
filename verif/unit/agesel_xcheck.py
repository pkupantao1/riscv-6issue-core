#!/usr/bin/env python3
"""AGESEL 宏的参考模型 (pdk2n/cam/agesel.py 的 Ref，晶体管级网表就是逐拍对它核的) 与 o6_iq (NP = 2) 选择规格的等价核对
   python verif/unit/agesel_xcheck.py [试验数，默认 200000] [项数，默认 32]

   o6_iq 一侧用「年龄队列」写的规格模型 —— 与 tb/unit/tb_iq.sv 的 I1 检查同一个模型 (单元 TB 里它与 RTL 逐拍比了约 39 万次发射、0 不一致)：
     口 0 = 它能执行的就绪项里最老的；口 1 = 只能走口 1 的就绪项里最老的，没有则两边都能走的里最年轻的，与口 0 同一项则让出
   宏一侧用年龄矩阵 (Ref.O) + 入队时的矩阵更新 (Ref.enq)；输出 G0 / G1 / C1，RTL 的 iss_valid[1] = |G1 & !C1
   两边喂同一串随机操作：每拍若干项入队 (最低的空项，口序 = 年龄序)、若干项释放、随机请求 / 独占位，比对选择与 pdst
   于是 宏参考 ≡ 规格 ≡ RTL (后一半由单元 TB 给出)
"""
import os, random, sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "pdk2n", "cam"))
from agesel import Ref    # noqa: E402


def main():
    ntr = int(sys.argv[1]) if len(sys.argv) > 1 else 200000
    N = int(sys.argv[2]) if len(sys.argv) > 2 else 32
    rnd = random.Random(1)
    ref = Ref(N)
    age = []                       # 在队列里的项，最老在前 (规格模型)
    pd = [0] * N
    bad = 0
    stats = {"g0": 0, "excl": 0, "yng": 0, "yield": 0}
    for t in range(ntr):
        # ---- 本拍请求：在队列里的项随机就绪；独占位 = 只能走口 1 ----
        live = set(age)
        ex = [i in live and rnd.random() < 0.25 for i in range(N)]
        rdy = [i in live and rnd.random() < 0.5 for i in range(N)]
        r0 = [rdy[i] and not ex[i] for i in range(N)]
        r1 = [rdy[i] for i in range(N)]
        g0, g1, c1, q0, q1 = ref.select(r0, r1, ex)
        # ---- 规格 ----
        e0 = next((i for i in age if r0[i]), None)
        exl = [i for i in age if r1[i] and ex[i]]
        if exl:
            e1 = exl[0]; kind = "excl"
        else:
            yl = [i for i in age if r1[i]]
            e1 = yl[-1] if yl else None
            kind = "yng"
            if e1 is not None and e1 == e0:
                e1 = None; kind = "yield"
        # 宏：口 1 的有效 = |G1 & !C1
        m0 = [i for i in range(N) if g0[i]]
        m1 = [i for i in range(N) if g1[i]] if not c1 else []
        ok = (m0 == ([e0] if e0 is not None else [])) and (m1 == ([e1] if e1 is not None else []))
        if ok and e0 is not None and q0 != pd[e0]:
            ok = False
        if ok and e1 is not None and q1 != pd[e1]:
            ok = False
        if not ok:
            bad += 1
            if bad <= 5:
                print(f"不一致 t={t}: 宏 g0={m0} g1={m1} c1={c1} q0={q0:02x} q1={q1:02x}；规格 e0={e0} e1={e1} ({kind})")
        if e0 is not None:
            stats["g0"] += 1
        stats[kind] += 1 if (e1 is not None or kind == "yield") else 0
        # ---- 拍末：发射的项释放 (偶尔不释放，模拟等判定)、随机释放、入队 ----
        gone = set()
        for i in (e0, e1):
            if i is not None and rnd.random() < 0.8:
                gone.add(i)
        for i in age:
            if rnd.random() < 0.03:
                gone.add(i)
        age = [i for i in age if i not in gone]
        free = [i for i in range(N) if i not in set(age)]
        k = min(len(free), rnd.randint(0, 4))
        ents = []
        for port in range(k):
            i = free[port]                     # 最低的空项 (RTL 的入队位置)
            p = rnd.randrange(256)
            ents.append((port, i, p))
            pd[i] = p
        if ents:
            ref.enq(ents)
            age += [i for _, i, _ in ents]     # 口序 = 年龄序，新项最年轻
    print(f"== AGESEL 参考模型 vs o6_iq 选择规格：{ntr} 拍，不一致 {bad}；口 0 发射 {stats['g0']}、"
          f"口 1 独占最老 {stats['excl']}、共享最年轻 {stats['yng']}、让出 {stats['yield']}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
