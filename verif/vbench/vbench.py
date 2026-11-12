#!/usr/bin/env python3
"""向量性能基准生成器：每个核单独一个 .S (strip-mining 循环，数据在 .data)，对拍下跑，看总拍数。

用法：vbench.py <输出目录>
  vvadd    int32  c[i] = a[i] + b[i]                    n = 4096，e32 m8
  saxpy    fp32   y[i] = a * x[i] + y[i]                n = 4096，e32 m8
  memcpy   8 位   16 KB 拷贝                             e8 m8
  dotf     fp32   sum x[i] * y[i] (vfmacc 累加，最后 vfredusum)   n = 4096
  matmul   fp32   C[32x32] += A[32x32] * B[32x32] (按行，vfmacc.vf，vl = 32)
  idot     int8   sum a[i] * b[i] (vqwdotau，每段一条)          n = 8192
  strided  int32  按列读 64x64 矩阵 (跨步 256 B) 求和        e32 m1
"""
import sys
import os

HDR = """    .section .text.init
    .globl _start
_start:
    li    t0, (1 << 9) | (1 << 13)       # VS / FS = Initial
    csrs  mstatus, t0
    li    s9, 2                          # 跑两遍：第一遍热身 (冷缺失)，只测第二遍
9:  li    t6, 1
    bne   s9, t6, 8f
    rdcycle s10
8:
"""
TAIL = """
    addi  s9, s9, -1
    bnez  s9, 9b
    rdcycle s11
    sub   s11, s11, s10
    li    t6, 1
    slli  t6, t6, 63
    or    s11, s11, t6                   # tb 打印 "[bench] 拍数"
    la    t6, tohost
    sd    s11, 0(t6)
    li    t6, 200                        # 隔开两次写 tohost (tb 每拍只看一次)
7:  addi  t6, t6, -1
    bnez  t6, 7b
    li    t0, 1
    la    t1, tohost
    sd    t0, 0(t1)
1:  j     1b
"""
DATA_TAIL = """
    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
"""


def arr(name, nbytes):
    return f"    .align 6\n{name}:\n    .space {nbytes}\n"


K = {}

K["vvadd"] = (r"""
    li    a0, 4096
    la    a1, va
    la    a2, vb
    la    a3, vc
2:  vsetvli t0, a0, e32, m8, ta, ma
    vle32.v v0, (a1)
    vle32.v v8, (a2)
    vadd.vv v16, v0, v8
    vse32.v v16, (a3)
    slli  t1, t0, 2
    add   a1, a1, t1
    add   a2, a2, t1
    add   a3, a3, t1
    sub   a0, a0, t0
    bnez  a0, 2b
""", arr("va", 16384) + arr("vb", 16384) + arr("vc", 16384))

K["saxpy"] = (r"""
    li    a0, 4096
    la    a1, vx
    la    a2, vy
    li    t2, 0x40400000              # 3.0f
    fmv.w.x fa0, t2
2:  vsetvli t0, a0, e32, m8, ta, ma
    vle32.v v0, (a1)
    vle32.v v8, (a2)
    vfmacc.vf v8, fa0, v0
    vse32.v v8, (a2)
    slli  t1, t0, 2
    add   a1, a1, t1
    add   a2, a2, t1
    sub   a0, a0, t0
    bnez  a0, 2b
""", arr("vx", 16384) + arr("vy", 16384))

K["memcpy"] = (r"""
    li    a0, 16384
    la    a1, src
    la    a2, dst
2:  vsetvli t0, a0, e8, m8, ta, ma
    vle8.v v0, (a1)
    vse8.v v0, (a2)
    add   a1, a1, t0
    add   a2, a2, t0
    sub   a0, a0, t0
    bnez  a0, 2b
""", arr("src", 16384) + arr("dst", 16384))

K["dotf"] = (r"""
    li    a0, 4096
    la    a1, vx
    la    a2, vy
    vsetvli t0, zero, e32, m8, ta, ma
    vmv.v.i v16, 0
2:  vsetvli t0, a0, e32, m8, tu, ma
    vle32.v v0, (a1)
    vle32.v v8, (a2)
    vfmacc.vv v16, v0, v8
    slli  t1, t0, 2
    add   a1, a1, t1
    add   a2, a2, t1
    sub   a0, a0, t0
    bnez  a0, 2b
    vsetvli t0, zero, e32, m8, ta, ma
    vmv.s.x v24, zero
    vfredusum.vs v24, v16, v24
    vfmv.f.s fa0, v24
""", arr("vx", 16384) + arr("vy", 16384))

K["matmul"] = (r"""
    li    s2, 32                      # 行
    la    s3, ma_
    la    s4, mc
    li    t0, 32
    vsetvli zero, t0, e32, m1, ta, ma
3:  vle32.v v8, (s4)                  # C 的一行
    la    s5, mb
    li    s6, 32                      # k
    mv    s7, s3
4:  flw   fa0, 0(s7)
    vle32.v v0, (s5)
    vfmacc.vf v8, fa0, v0
    addi  s7, s7, 4
    addi  s5, s5, 128
    addi  s6, s6, -1
    bnez  s6, 4b
    vse32.v v8, (s4)
    addi  s3, s3, 128
    addi  s4, s4, 128
    addi  s2, s2, -1
    bnez  s2, 3b
""", arr("ma_", 4096) + arr("mb", 4096) + arr("mc", 4096))

K["idot"] = (r"""
    li    a0, 8192
    la    a1, va
    la    a2, vb
    vsetvli t0, zero, e32, m1, ta, ma
    vmv.s.x v24, zero
2:  vsetvli t0, a0, e8, m8, ta, ma
    vle8.v v0, (a1)
    vle8.v v8, (a2)
    .word 0x9a800c77                  # vqwdotau.vv v24, v8, v0 (vd = v24，vs2 = v8，vs1 = v0)
    add   a1, a1, t0
    add   a2, a2, t0
    sub   a0, a0, t0
    bnez  a0, 2b
""", arr("va", 8192) + arr("vb", 8192))

K["strided"] = (r"""
    li    s2, 64                      # 64 列
    la    s3, mat
    li    t2, 256                     # 行距
    li    t0, 64
    vsetvli zero, t0, e32, m2, ta, ma
    vmv.v.i v8, 0
3:  vlse32.v v0, (s3), t2
    vadd.vv v8, v8, v0
    addi  s3, s3, 4
    addi  s2, s2, -1
    bnez  s2, 3b
""", arr("mat", 16384))


# 发射开销：64 条互不相关、只要 1 拍的 vadd (e32 m1 vl = 8)，循环 16 次 -> 每条向量指令的拍数
K["vissue"] = ("""
    li    t0, 8
    vsetvli zero, t0, e32, m1, ta, ma
    li    s2, 16
3:
""" + "".join(f"    vadd.vv v{1 + i % 8}, v{9 + i % 8}, v{17 + i % 8}" + chr(10) for i in range(64)) + """
    addi  s2, s2, -1
    bnez  s2, 3b
""", "")


def main():
    od = sys.argv[1]
    os.makedirs(od, exist_ok=True)
    for name, (body, data) in K.items():
        with open(os.path.join(od, name + ".S"), "w") as f:
            f.write(HDR + body + TAIL + "\n    .data\n" + data + DATA_TAIL)
    print(" ".join(K))


if __name__ == "__main__":
    main()
