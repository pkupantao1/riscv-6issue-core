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
  gemm_rb  fp32   C[64x64] += A * B，寄存器分块 8 行 x 32 列 (对标 Ara fmatmul)
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
    fence rw, rw                       # M3b：先等前一遍的 store 排空 (SQ 里积着的向量 store 不算进本遍)
    sfence.vma                         # 普通 fence 提交时不等 SQ 排空、rdcycle 也不受 fence 约束：sfence.vma 提交前等已提交的 store 全写进 D$
    rdcycle s10
8:
"""
TAIL = """
    addi  s9, s9, -1
    bnez  s9, 9b
    fence rw, rw                       # 本遍的 store 排空后才停表
    sfence.vma
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


# 寄存器分块 GEMM (对标 Ara fmatmul)：C[64x64] += A[64x64] * B[64x64]，fp32，行优先 (行距 256 B)。
#   C 按 8 行 x 32 列分块放在 v8..v15 (e32 m1，vl = 32 = 1024 位)；每个 k：读 B 的一行 (1 条 vle)、
#   A 的 8 个元素 (8 条 flw)、8 条 vfmacc.vf。每次向量访存配 8 次向量 FMA，看的是 FMA 利用率。
#   FLOP = 2 * 64^3 = 524288；DLEN = 256 时峰值 16 FLOP/拍 (满利用 32768 拍)
K["gemm_rb"] = (r"""
    li    t0, 32
    vsetvli zero, t0, e32, m1, ta, ma
    li    s2, 0                       # 行块 i = 0, 8, .., 56
5:  li    s8, 0                       # 列块 j = 0, 32
6:  slli  t1, s2, 8                   # i * 256
    slli  t2, s8, 2                   # j * 4
    la    s4, gc
    add   s4, s4, t1
    add   s4, s4, t2                  # &C[i][j]
    mv    t3, s4
    vle32.v v8, (t3)
    addi  t3, t3, 256
    vle32.v v9, (t3)
    addi  t3, t3, 256
    vle32.v v10, (t3)
    addi  t3, t3, 256
    vle32.v v11, (t3)
    addi  t3, t3, 256
    vle32.v v12, (t3)
    addi  t3, t3, 256
    vle32.v v13, (t3)
    addi  t3, t3, 256
    vle32.v v14, (t3)
    addi  t3, t3, 256
    vle32.v v15, (t3)
    addi  t3, t3, 256
    la    s5, gb
    add   s5, s5, t2                  # &B[0][j]
    la    s7, ga
    add   s7, s7, t1                  # &A[i][0]
    li    s6, 64                      # k
7:  vle32.v v0, (s5)
    flw   f0, 0(s7)
    flw   f1, 256(s7)
    flw   f2, 512(s7)
    flw   f3, 768(s7)
    flw   f4, 1024(s7)
    flw   f5, 1280(s7)
    flw   f6, 1536(s7)
    flw   f7, 1792(s7)
    vfmacc.vf v8, f0, v0
    vfmacc.vf v9, f1, v0
    vfmacc.vf v10, f2, v0
    vfmacc.vf v11, f3, v0
    vfmacc.vf v12, f4, v0
    vfmacc.vf v13, f5, v0
    vfmacc.vf v14, f6, v0
    vfmacc.vf v15, f7, v0
    addi  s7, s7, 4
    addi  s5, s5, 256
    addi  s6, s6, -1
    bnez  s6, 7b
    mv    t3, s4
    vse32.v v8, (t3)
    addi  t3, t3, 256
    vse32.v v9, (t3)
    addi  t3, t3, 256
    vse32.v v10, (t3)
    addi  t3, t3, 256
    vse32.v v11, (t3)
    addi  t3, t3, 256
    vse32.v v12, (t3)
    addi  t3, t3, 256
    vse32.v v13, (t3)
    addi  t3, t3, 256
    vse32.v v14, (t3)
    addi  t3, t3, 256
    vse32.v v15, (t3)
    addi  t3, t3, 256
    addi  s8, s8, 32
    li    t4, 64
    blt   s8, t4, 6b
    addi  s2, s2, 8
    blt   s2, t4, 5b
""", arr("ga", 16384) + arr("gb", 16384) + arr("gc", 16384))


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


# 算力峰值 (只用寄存器)：8 条互不相关的点积 (vd = v16..v23，vs2 = v8..v15 组，vs1 = v0..v7 组)，e8 m8 vl = 1024，循环 16 次
#   fp8dot：vfqwdota.vv (FP8 E4M3 -> FP32 累加)；i8dot：vqwdotau.vv。每条 1024 个乘加
K["fp8dot"] = ("""
    li    t0, 1024
    vsetvli t0, t0, e8, m8, ta, ma
    li    s2, 16
3:
    .word 0x9a801877
    .word 0x9a8018f7
    .word 0x9a801977
    .word 0x9a8019f7
    .word 0x9a801a77
    .word 0x9a801af7
    .word 0x9a801b77
    .word 0x9a801bf7
    addi  s2, s2, -1
    bnez  s2, 3b
""", "")

# 算力峰值 (只用寄存器)：8 条互不相关的点积 (vd = v16..v23，vs2 = v8..v15 组，vs1 = v0..v7 组)，e8 m8 vl = 1024，循环 16 次
#   fp8dot：vfqwdota.vv (FP8 E4M3 -> FP32 累加)；i8dot：vqwdotau.vv。每条 1024 个乘加
K["i8dot"] = ("""
    li    t0, 1024
    vsetvli t0, t0, e8, m8, ta, ma
    li    s2, 16
3:
    .word 0x9a800877
    .word 0x9a8008f7
    .word 0x9a800977
    .word 0x9a8009f7
    .word 0x9a800a77
    .word 0x9a800af7
    .word 0x9a800b77
    .word 0x9a800bf7
    addi  s2, s2, -1
    bnez  s2, 3b
""", "")

K["f32fma"] = ("""
    li    t0, 256
    vsetvli t0, t0, e32, m8, ta, ma
    li    s2, 32
3:
    vfmacc.vv v16, v0, v8
    vfmacc.vv v24, v0, v8
    vfmacc.vv v16, v8, v0
    vfmacc.vv v24, v8, v0
    addi  s2, s2, -1
    bnez  s2, 3b
""", "")

# V3 执行器吞吐 (只用寄存器)：8 条互不相关的 vfmul.vv (e32 m8，vl = 256，每条 8 拍)，目的轮流 v16 / v24 (只有 WAW，改名后无关)，
#   循环 32 次 = 256 条。4 个执行器理想 2 拍一条 (FP32 每拍 4 × 32 = 128 个乘)
K["f32mul"] = ("""
    li    t0, 256
    vsetvli t0, t0, e32, m8, ta, ma
    li    s2, 32
3:
""" + "".join(f"    vfmul.vv v{16 + 8 * (i % 2)}, v0, v8" + chr(10) for i in range(8)) + """
    addi  s2, s2, -1
    bnez  s2, 3b
""", "")

# V3 链式累加：FP16 e16 m2 (vl = 128，每条 2 拍)，8 个累加器 v0..v14，源 v16..v30；循环 32 次 = 256 条。
#   4 个执行器理想 2 条 / 拍；每个累加器的下一条要等上一条第 0 个寄存器写回 (约 8 拍)，所以 8 个累加器约 1 条 / 拍
K["f16acc"] = ("""
    li    t0, 128
    vsetvli t0, t0, e16, m2, ta, ma
    li    s2, 32
3:
""" + "".join(f"    vfmacc.vv v{2 * i}, v{16 + 2 * (i % 4)}, v{24 + 2 * (i % 4)}" + chr(10) for i in range(8)) + """
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
