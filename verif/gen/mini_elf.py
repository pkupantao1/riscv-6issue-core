#!/usr/bin/env python3
"""mini_elf.py - 不依赖工具链的极简汇编 + ELF64 生成 (冒烟测试用)

只支持少量 RV64I/M 指令，标签做两遍解析。text 从 0x80000000 开始，tohost 在 0x80001000。
用法: mini_elf.py -o smoke.elf [--prog smoke]
"""
import argparse
import struct

REG = {f"x{i}": i for i in range(32)}
REG.update({"zero": 0, "ra": 1, "sp": 2, "gp": 3, "tp": 4, "t0": 5, "t1": 6, "t2": 7, "s0": 8, "s1": 9,
            "a0": 10, "a1": 11, "a2": 12, "a3": 13, "a4": 14, "a5": 15, "a6": 16, "a7": 17,
            "s2": 18, "s3": 19, "s4": 20, "s5": 21, "s6": 22, "s7": 23, "s8": 24, "s9": 25,
            "s10": 26, "s11": 27, "t3": 28, "t4": 29, "t5": 30, "t6": 31})

def r(f7, rs2, rs1, f3, rd, op): return (f7 << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | op
def i(imm, rs1, f3, rd, op):     return ((imm & 0xfff) << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | op
def s(imm, rs2, rs1, f3, op):    return (((imm >> 5) & 0x7f) << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | ((imm & 0x1f) << 7) | op
def b(imm, rs2, rs1, f3):
    return (((imm >> 12) & 1) << 31) | (((imm >> 5) & 0x3f) << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | \
           (((imm >> 1) & 0xf) << 8) | (((imm >> 11) & 1) << 7) | 0x63
def u(imm, rd, op):              return ((imm & 0xfffff) << 12) | (rd << 7) | op
def j(imm, rd):
    return (((imm >> 20) & 1) << 31) | (((imm >> 1) & 0x3ff) << 21) | (((imm >> 11) & 1) << 20) | \
           (((imm >> 12) & 0xff) << 12) | (rd << 7) | 0x6f

R_OPS = {"add": (0, 0, 0x33), "sub": (0x20, 0, 0x33), "xor": (0, 4, 0x33), "or": (0, 6, 0x33), "and": (0, 7, 0x33),
         "sll": (0, 1, 0x33), "srl": (0, 5, 0x33), "sra": (0x20, 5, 0x33), "slt": (0, 2, 0x33), "sltu": (0, 3, 0x33),
         "mul": (1, 0, 0x33), "mulh": (1, 1, 0x33), "div": (1, 4, 0x33), "divu": (1, 5, 0x33), "rem": (1, 6, 0x33),
         "remu": (1, 7, 0x33), "addw": (0, 0, 0x3b), "mulw": (1, 0, 0x3b), "divw": (1, 4, 0x3b)}
I_OPS = {"addi": (0, 0x13), "xori": (4, 0x13), "ori": (6, 0x13), "andi": (7, 0x13), "slti": (2, 0x13),
         "addiw": (0, 0x1b), "lb": (0, 3), "lh": (1, 3), "lw": (2, 3), "ld": (3, 3), "lbu": (4, 3), "lhu": (5, 3),
         "lwu": (6, 3), "jalr": (0, 0x67)}
S_OPS = {"sb": 0, "sh": 1, "sw": 2, "sd": 3}
CSR = {"mhartid": 0xf14, "minstret": 0xb02, "mscratch": 0x340, "mtvec": 0x305, "mepc": 0x341,
       "mcause": 0x342, "mtval": 0x343, "mstatus": 0x300, "mie": 0x304, "mip": 0x344}
AMO = {"lr": 2, "sc": 3, "amoswap": 1, "amoadd": 0, "amoxor": 4, "amoand": 12, "amoor": 8,
       "amomin": 16, "amomax": 20, "amominu": 24, "amomaxu": 28}
B_OPS = {"beq": 0, "bne": 1, "blt": 4, "bge": 5, "bltu": 6, "bgeu": 7}


def assemble(src, base):
    lines = []
    for ln in src.strip().splitlines():
        ln = ln.split("#")[0].strip()
        if ln:
            lines.append(ln)
    labels, pc = {}, base
    for ln in lines:
        if ln.endswith(":"):
            labels[ln[:-1]] = pc
        else:
            pc += 4
    words, pc = [], base
    for ln in lines:
        if ln.endswith(":"):
            continue
        op, *rest = ln.replace(",", " ").split()
        a = rest

        def reg(t): return REG[t]

        def imm(t):
            return labels[t] - pc if t in labels else int(t, 0)
        if op in R_OPS:
            f7, f3, opc = R_OPS[op]; w = r(f7, reg(a[2]), reg(a[1]), f3, reg(a[0]), opc)
        elif op in ("slli", "srli", "srai"):
            f3 = 1 if op == "slli" else 5
            hi = 0x10 if op == "srai" else 0
            w = i((hi << 6) | int(a[2], 0), reg(a[1]), f3, reg(a[0]), 0x13)
        elif op in I_OPS and op[0] == "l" or op == "jalr" and "(" in a[1]:
            f3, opc = I_OPS[op]
            off, rs = a[1].rstrip(")").split("(")
            w = i(int(off, 0), reg(rs), f3, reg(a[0]), opc)
        elif op in I_OPS:
            f3, opc = I_OPS[op]; w = i(int(a[2], 0), reg(a[1]), f3, reg(a[0]), opc)
        elif op in S_OPS:
            off, rs = a[1].rstrip(")").split("(")
            w = s(int(off, 0), reg(a[0]), reg(rs), S_OPS[op], 0x23)
        elif op in B_OPS:
            w = b(imm(a[2]), reg(a[1]), reg(a[0]), B_OPS[op])
        elif op == "lui":
            w = u(int(a[1], 0), reg(a[0]), 0x37)
        elif op == "auipc":
            w = u(int(a[1], 0), reg(a[0]), 0x17)
        elif op == "jal":
            w = j(imm(a[1]), reg(a[0]))
        elif op == "csrr":
            w = i(CSR[a[1]], 0, 2, reg(a[0]), 0x73)
        elif op == "csrw":
            w = i(CSR[a[0]], reg(a[1]), 1, 0, 0x73)
        elif op == "csrs":
            w = i(CSR[a[0]], reg(a[1]), 2, 0, 0x73)
        elif op.split(".")[0] in AMO:
            base_op, sz = op.split(".")[0], op.split(".")[1]
            f5 = AMO[base_op]
            if base_op == "lr":
                rs2, rs1 = 0, reg(a[1].strip("()"))
            else:
                rs2, rs1 = reg(a[1]), reg(a[2].strip("()"))
            w = r(f5 << 2, rs2, rs1, 3 if sz == "d" else 2, reg(a[0]), 0x2f)
        elif op == "ecall":
            w = 0x73
        elif op == "mret":
            w = 0x30200073
        elif op == ".word":
            w = int(a[0], 0)
        else:
            raise SystemExit(f"不支持: {ln}")
        words.append(w & 0xffffffff)
        pc += 4
    return words


TOHOST_SETUP = """
    auipc s0, 1
    srli s0, s0, 12
    slli s0, s0, 12          # s0 = 0x80001000 (tohost 页)
"""

PROGS = {
    # 陷入：ecall / 未对齐 load / 未对齐 store / 非法指令，处理程序把 mepc+4 后返回
    "trap": """
_start:
    jal  t0, over
handler:
    csrr t4, mepc
    csrr t5, mcause
    add  s3, s3, t5          # 累加 cause，最后写出来让对拍比较
    csrr t5, mtval
    xor  s4, s4, t5
    addi t4, t4, 4
    csrw mepc, t4
    mret
over:
    csrw mtvec, t0
""" + TOHOST_SETUP + """
    ecall
    ld   a0, 65(s0)
    sw   a0, 66(s0)
    lh   a1, 67(s0)
    .word 0x0000707b
    ecall
    addi t3, zero, 1
    sd   t3, 0(s0)
end:
    jal  zero, end
""",
    # 机器定时器中断：mtimecmp = mtime + 50，开 MTIE/MIE，忙等中断计数；处理程序把 mtimecmp 设为最大
    "irq": """
_start:
    jal  t0, over
handler:
    csrr t4, mcause
    addi s5, s5, 1
    lui  t5, 0x2004
    addi t6, zero, -1
    sd   t6, 0(t5)
    mret
over:
    csrw mtvec, t0
    lui  t5, 0x200c
    ld   t6, -8(t5)
    addi t6, t6, 50
    lui  t5, 0x2004
    sd   t6, 0(t5)
    addi t1, zero, 128
    csrw mie, t1
    addi t1, zero, 8
    csrs mstatus, t1
wait:
    addi s6, s6, 1
    beq  s5, zero, wait
""" + TOHOST_SETUP + """
    addi t3, zero, 1
    sd   t3, 0(s0)
end:
    jal  zero, end
""",
    # AMO 与 LR/SC
    "amo": """
_start:
""" + TOHOST_SETUP + """
    addi a0, s0, 128
    addi t0, zero, 5
    sd   t0, 0(a0)
    addi t1, zero, 3
    amoadd.d a1, t1, (a0)
    amoswap.d a2, t1, (a0)
    amomax.d a3, t0, (a0)
    ld   a4, 0(a0)
    amoadd.w a5, t1, (a0)
    lw   a6, 0(a0)
    lr.d a7, (a0)
    addi a7, a7, 1
    sc.d s1, a7, (a0)
    ld   s2, 0(a0)
    sc.d s3, a7, (a0)
    lr.w s4, (a0)
    addi s5, a0, 8
    sc.w s6, s4, (s5)
    addi t3, zero, 1
    sd   t3, 0(s0)
end:
    jal  zero, end
""",

    "smoke": """
_start:
    addi t0, zero, 10
    addi t1, zero, 0
loop:
    add  t1, t1, t0
    addi t0, t0, -1
    bne  t0, zero, loop
    auipc s0, 1              # s0 = 0x80001000 + (pc - 0x80000000) 的页内部分 (pc 在第一页内)
    srli s0, s0, 12
    slli s0, s0, 12          # s0 = 0x80001000
    sd   t1, 64(s0)
    ld   t2, 64(s0)
    addi a0, zero, 7
    addi a1, zero, 6
    mul  a2, a0, a1
    div  a3, a2, a1
    rem  a4, a2, a0
    jal  ra, func
    add  a5, a5, a2
    sw   a5, 72(s0)
    lw   a6, 72(s0)
    lbu  a7, 73(s0)
    csrr s1, mhartid
    csrw mscratch, a5
    csrr s2, mscratch
    addi t3, zero, 1
    sd   t3, 0(s0)           # tohost = 1
end:
    jal  zero, end
func:
    addi a5, zero, 100
    jalr zero, 0(ra)
""",
}


def write_elf(path, words, base=0x80000000, tohost=0x80001000):
    text = b"".join(struct.pack("<I", w) for w in words)
    text += b"\0" * (0x1000 - len(text))           # 补到 tohost 页
    text += b"\0" * 0x100                            # tohost / 数据
    # 段: [ELF 头][程序头][text][strtab][symtab][shstrtab][节头]
    ehsz, phsz, shsz = 64, 56, 64
    off_text = 0x1000
    strtab = b"\0tohost\0fromhost\0"
    sym = struct.pack("<IBBHQQ", 0, 0, 0, 0, 0, 0)
    sym += struct.pack("<IBBHQQ", 1, 0x11, 0, 1, tohost, 8)
    sym += struct.pack("<IBBHQQ", 8, 0x11, 0, 1, tohost + 64, 8)
    shstr = b"\0.text\0.strtab\0.symtab\0.shstrtab\0"
    off_str = off_text + len(text)
    off_sym = off_str + len(strtab)
    off_shs = off_sym + len(sym)
    off_sh = (off_shs + len(shstr) + 7) & ~7
    eh = struct.pack("<16sHHIQQQIHHHHHH", b"\x7fELF\x02\x01\x01" + b"\0" * 9, 2, 0xf3, 1, base, ehsz, off_sh,
                     0, ehsz, phsz, 1, shsz, 5, 4)
    ph = struct.pack("<IIQQQQQQ", 1, 7, off_text, base, base, len(text), len(text), 0x1000)

    def sh(name, typ, flags, addr, off, size, link=0, info=0, align=1, ent=0):
        return struct.pack("<IIQQQQIIQQ", name, typ, flags, addr, off, size, link, info, align, ent)
    shdrs = sh(0, 0, 0, 0, 0, 0)
    shdrs += sh(1, 1, 6, base, off_text, len(text), align=4)
    shdrs += sh(7, 3, 0, 0, off_str, len(strtab))
    shdrs += sh(15, 2, 0, 0, off_sym, len(sym), link=2, info=1, align=8, ent=24)
    shdrs += sh(23, 3, 0, 0, off_shs, len(shstr))
    img = bytearray(off_sh + len(shdrs))
    img[0:ehsz] = eh
    img[ehsz:ehsz + phsz] = ph
    img[off_text:off_text + len(text)] = text
    img[off_str:off_str + len(strtab)] = strtab
    img[off_sym:off_sym + len(sym)] = sym
    img[off_shs:off_shs + len(shstr)] = shstr
    img[off_sh:off_sh + len(shdrs)] = shdrs
    open(path, "wb").write(bytes(img))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", required=True)
    ap.add_argument("--prog", default="smoke")
    a = ap.parse_args()
    write_elf(a.o, assemble(PROGS[a.prog], 0x80000000))


if __name__ == "__main__":
    main()
