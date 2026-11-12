#!/usr/bin/env python3
"""vgen.py - OOO6 随机向量 (RVV 1.0) 程序生成器

生成纯汇编 (as + ld)，在 tb_top 里与 Spike 对拍：标量结果逐条比，向量单元每次跑空时整体比向量寄存器 / 向量 CSR。
生成器自己跟踪 vtype / vl (与 vsetvl 语义一致)，只生成合法的指令 (寄存器组对齐、重叠规则、掩码时 vd != v0)；
少量故意不合法 / 会出异常的 (--traps) 由陷入处理程序跳过，陷入本身也对拍。

类别 (--cls 逗号分隔，默认全部已实现的)：
  vset   vsetvli / vsetivli / vsetvl，随机 SEW / LMUL / ta / ma / AVL
  alu    整数元素运算 vv / vx / vi (add sub rsub min max and or xor sll srl sra merge mv)
  cmp    比较写掩码 (mseq ... msgt)
  mvs    vmv.x.s / vmv.s.x
  mem    单位步长 / 跨步 load / store (各 EEW，掩码)
  fx     定点：饱和加减、平均加减、vsmul、舍入移位 (随机 vxrm)
  mul    乘法 / 乘加
  carry  vadc / vsbc / vmadc / vmsbc
  wide   加宽加减 / 乘 / 乘加 (.vv/.vx/.wv/.wx)
  narrow 收窄移位 / vnclip
  ext    vzext / vsext
  div    除法 / 取余
  red    归约 (含加宽求和)
  mlog   掩码逻辑 vmand.mm ...
  mset   vmsbf / vmsif / vmsof / viota / vid / vcpop / vfirst
  perm   slide / slide1 / gather / gatherei16 / compress
  vmvr   vmv<nr>r.v
  fpa    浮点加减乘除 / min / max / 符号注入 (.vv / .vf)
  fma    浮点乘加 (8 种)
  fwide  浮点加宽加减 / 乘 / 乘加 (含 vfwmaccbf16)
  fcvt   浮点转换 (同宽 / 加宽 / 收窄，含 rtz / rod / BF16)
  fcmp   浮点比较
  fred   浮点归约 (含加宽)
  fmisc  vfsqrt / vfrec7 / vfrsqrt7 / vfclass / vfmerge / vfmv / vfslide1
  mseg   分段访存 (单位步长 / 跨步)
  midx   索引访存 (有序 / 无序，含分段)
  mwhole 整寄存器访存 vl<nr>re<eew> / vs<nr>r
  mmask  vlm / vsm
  mff    fault-only-first (基址靠近禁区页，后面的元素出错只截短 vl)
  fp8    OFP8 转换 (Zvfofp8min：vfwcvtbf16 / vfncvtbf16 (.sat) / vfncvt (.sat).f.f.q，altfmt 选 E5M2)
  dot    点积 (vqwdotau / vqwdotas、vfqwdota (.alt)、vfwdota)；草案扩展，用 .word 直接编码
  bb     Zvbb (RVA23 R2c)：vandn / vrol / vror (.vi 6 位立即数) / vwsll，一元 vbrev / vbrev8 / vrev8 / vclz / vctz / vcpop.v
  (vset 会偶尔用 vsetvl 在 SEW = 8 / 16 时打开 altfmt；altfmt = 1 时其他浮点类不生成)
用法：vgen.py --seed 1 --n 400 -o t.S
"""
import argparse
import random

ABI = ["zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2", "s0", "s1", "a0", "a1", "a2", "a3", "a4", "a5",
       "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9", "s10", "s11", "t3", "t4", "t5", "t6"]
# 随机标量操作数 / 目的用的寄存器 (s0 = 向量数据区基址，s1 = 初始化用，t4..t6 陷入处理)
SREGS = [5, 6, 7, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28]
VDATA = 32768          # 向量访存数据区 (字节)
VLEN = 1024
ELEN = 64

LMULS = {"mf8": -3, "mf4": -2, "mf2": -1, "m1": 0, "m2": 1, "m4": 2, "m8": 3}
SEWS = {"e8": 0, "e16": 1, "e32": 2, "e64": 3}


class VGen:
    def __init__(self, seed, n, cls, traps):
        self.r = random.Random(seed)
        self.n = n
        self.cls = cls
        self.traps = traps
        self.out = []
        self.vill = True
        self.sew = 0        # log2(SEW / 8)
        self.lmul = 0       # log2 LMUL
        self.vl = 0
        self.vl_unknown = False     # fault-only-first 之后 vl 可能被截短
        self.altf = False           # vtype.altfmt
        self.use_alt = any(c in cls for c in ("fp8", "dot"))

    def emit(self, s):
        self.out.append("    " + s)

    # ------------------------------------------------------------------ vtype
    def vlmax(self, sew, lmul):
        return (VLEN >> (sew + 3)) << lmul if lmul >= 0 else (VLEN >> (sew + 3)) >> -lmul

    def legal_vtype(self, sew, lmul):
        return -3 <= lmul <= 3 and (lmul >= 0 or sew + 3 <= 6 + lmul)

    def gen_vset(self):
        r = self.r
        if self.use_alt and r.random() < 0.35:
            return self.gen_vset_alt()
        self.altf = False
        sew = r.choice(list(SEWS.values()))
        lmul = r.choice(list(LMULS.values()))
        if not self.legal_vtype(sew, lmul) and r.random() < 0.85:
            return self.gen_vset()
        ta, ma = r.choice(["ta", "tu"]), r.choice(["ma", "mu"])
        s_name = [k for k, v in SEWS.items() if v == sew][0]
        l_name = [k for k, v in LMULS.items() if v == lmul][0]
        vtype = f"{s_name}, {l_name}, {ta}, {ma}"
        legal = self.legal_vtype(sew, lmul)
        vmax = self.vlmax(sew, lmul) if legal else 0
        kind = r.random()
        rd = r.choice(SREGS)
        if kind < 0.6:
            avl = r.choice([r.randint(0, 40), r.randint(0, 1100), vmax, vmax + 1, 1, 0])
            ra = r.choice(SREGS)
            self.emit(f"li {ABI[ra]}, {avl}")
            self.emit(f"vsetvli {ABI[rd]}, {ABI[ra]}, {vtype}")
            vl = min(avl, vmax)
        elif kind < 0.85:
            avl = r.randint(0, 31)
            self.emit(f"vsetivli {ABI[rd]}, {avl}, {vtype}")
            vl = min(avl, vmax)
        else:
            self.emit(f"vsetvli {ABI[rd]}, zero, {vtype}")
            vl = vmax
        self.vl_unknown = False
        if legal:
            self.vill, self.sew, self.lmul, self.vl = False, sew, lmul, vl
        else:
            self.vill, self.vl = True, 0

    def gen_vset_alt(self):
        """vsetvl 设 vtype (含 altfmt = bit 8)，SEW = 8 / 16"""
        r = self.r
        sew = r.choice([0, 1])
        lmul = r.choice([l for l in LMULS.values() if self.legal_vtype(sew, l)])
        alt = r.random() < 0.7
        vt = (int(alt) << 8) | (r.randint(0, 1) << 7) | (r.randint(0, 1) << 6) | (sew << 3) | (lmul & 7)
        vmax = self.vlmax(sew, lmul)
        avl = r.choice([r.randint(1, 40), vmax, r.randint(1, vmax)])
        ra, rb, rd = r.sample(SREGS, 3)
        self.emit(f"li {ABI[ra]}, {avl}")
        self.emit(f"li {ABI[rb]}, {vt}")
        self.emit(f"vsetvl {ABI[rd]}, {ABI[ra]}, {ABI[rb]}")
        self.vl_unknown = False
        self.vill, self.sew, self.lmul, self.vl, self.altf = False, sew, lmul, min(avl, vmax), alt

    # ------------------------------------------------------------------ 寄存器选择
    def grp(self, lm):
        return 1 << lm if lm > 0 else 1

    def vreg(self, lm, avoid=(), not_v0=False):
        g = self.grp(lm)
        cands = [v for v in range(0, 32, g) if not (not_v0 and v == 0)]
        cands = [v for v in cands if not any(self.ovl(v, g, a, ag) for a, ag in avoid)]
        return self.r.choice(cands)

    @staticmethod
    def ovl(a, asz, b, bsz):
        return max(a + asz, b + bsz) - min(a, b) < asz + bsz

    def mask_suffix(self):
        vm = self.r.random() < 0.65
        return vm, ("" if vm else ", v0.t")

    # ------------------------------------------------------------------ 整数元素运算
    ALU = {  # 名字: 允许的形式
        "vadd": "vxi", "vsub": "vx", "vrsub": "xi", "vminu": "vx", "vmin": "vx", "vmaxu": "vx", "vmax": "vx",
        "vand": "vxi", "vor": "vxi", "vxor": "vxi", "vsll": "vxi", "vsrl": "vxi", "vsra": "vxi",
    }
    CMP = {"vmseq": "vxi", "vmsne": "vxi", "vmsltu": "vx", "vmslt": "vx", "vmsleu": "vxi", "vmsle": "vxi",
           "vmsgtu": "xi", "vmsgt": "xi"}

    def opnd2(self, form, shift=False):
        r = self.r
        if form == "v":
            return None
        if form == "x":
            return ABI[r.choice(SREGS)]
        return str(r.randint(0, 31) if shift else r.randint(-16, 15))

    def gen_alu(self):
        r = self.r
        if r.random() < 0.15:          # merge / mv
            lm = self.lmul
            if r.random() < 0.5:
                vd = self.vreg(lm, not_v0=True)
                vs2 = self.vreg(lm)
                f = r.choice("vxi")
                if f == "v":
                    self.emit(f"vmerge.vvm v{vd}, v{vs2}, v{self.vreg(lm)}, v0")
                elif f == "x":
                    self.emit(f"vmerge.vxm v{vd}, v{vs2}, {ABI[r.choice(SREGS)]}, v0")
                else:
                    self.emit(f"vmerge.vim v{vd}, v{vs2}, {r.randint(-16, 15)}, v0")
            else:
                vd = self.vreg(lm)
                f = r.choice("vxi")
                if f == "v":
                    self.emit(f"vmv.v.v v{vd}, v{self.vreg(lm)}")
                elif f == "x":
                    self.emit(f"vmv.v.x v{vd}, {ABI[r.choice(SREGS)]}")
                else:
                    self.emit(f"vmv.v.i v{vd}, {r.randint(-16, 15)}")
            return
        op = r.choice(list(self.ALU))
        form = r.choice(self.ALU[op])
        vm, ms = self.mask_suffix()
        lm = self.lmul
        vd = self.vreg(lm, not_v0=not vm)
        vs2 = self.vreg(lm)
        sh = op in ("vsll", "vsrl", "vsra")
        if form == "v":
            self.emit(f"{op}.vv v{vd}, v{vs2}, v{self.vreg(lm)}{ms}")
        elif form == "x":
            self.emit(f"{op}.vx v{vd}, v{vs2}, {self.opnd2('x')}{ms}")
        else:
            self.emit(f"{op}.vi v{vd}, v{vs2}, {self.opnd2('i', sh)}{ms}")

    def gen_cmp(self):
        r = self.r
        op = r.choice(list(self.CMP))
        form = r.choice(self.CMP[op])
        vm, ms = self.mask_suffix()
        lm = self.lmul
        g = self.grp(lm)
        vs2 = self.vreg(lm)
        vs1 = self.vreg(lm)
        # 掩码目的：与源寄存器组不重叠 (或恰好等于 vs2 / vs1)
        if r.random() < 0.15:
            vd = vs2
        else:
            av = [(vs2, g)] + ([(vs1, g)] if form == "v" else [])
            vd = self.vreg(0, avoid=av)
        if form == "v":
            if vd != vs1 and self.ovl(vd, 1, vs1, g):
                vd = vs2 if not self.ovl(vs2, 1, vs1, g) or vs2 == vs1 else self.vreg(0, avoid=[(vs2, g), (vs1, g)])
            self.emit(f"{op}.vv v{vd}, v{vs2}, v{vs1}{ms}")
        elif form == "x":
            self.emit(f"{op}.vx v{vd}, v{vs2}, {self.opnd2('x')}{ms}")
        else:
            self.emit(f"{op}.vi v{vd}, v{vs2}, {self.opnd2('i')}{ms}")

    def gen_mvs(self):
        r = self.r
        if r.random() < 0.5:
            self.emit(f"vmv.x.s {ABI[r.choice(SREGS)]}, v{r.randint(0, 31)}")
        else:
            self.emit(f"vmv.s.x v{r.randint(0, 31)}, {ABI[r.choice(SREGS)]}")

    # ------------------------------------------------------------------ V2：同宽元素运算 (定点 / 乘 / 进位)
    FX = {"vsaddu": "vxi", "vsadd": "vxi", "vssubu": "vx", "vssub": "vx", "vaaddu": "vx", "vaadd": "vx",
          "vasubu": "vx", "vasub": "vx", "vsmul": "vx", "vssrl": "vxi", "vssra": "vxi"}
    MUL = {"vmul": "vx", "vmulh": "vx", "vmulhu": "vx", "vmulhsu": "vx"}
    MAC = ["vmacc", "vnmsac", "vmadd", "vnmsub"]

    def sss(self, op, forms, sh=False):
        r = self.r
        form = r.choice(forms)
        vm, ms = self.mask_suffix()
        lm = self.lmul
        vd = self.vreg(lm, not_v0=not vm)
        vs2 = self.vreg(lm)
        if form == "v":
            self.emit(f"{op}.vv v{vd}, v{vs2}, v{self.vreg(lm)}{ms}")
        elif form == "x":
            self.emit(f"{op}.vx v{vd}, v{vs2}, {ABI[r.choice(SREGS)]}{ms}")
        else:
            self.emit(f"{op}.vi v{vd}, v{vs2}, {r.randint(0, 31) if sh else r.randint(-16, 15)}{ms}")

    def gen_fx(self):
        r = self.r
        if r.random() < 0.1:
            self.emit(f"csrwi vxrm, {r.randint(0, 3)}")
        op = r.choice(list(self.FX))
        self.sss(op, self.FX[op], sh=op in ("vssrl", "vssra"))

    def gen_mul(self):
        r = self.r
        if r.random() < 0.6:
            op = r.choice(list(self.MUL))
            self.sss(op, self.MUL[op])
            return
        op = r.choice(self.MAC)
        vm, ms = self.mask_suffix()
        lm = self.lmul
        vd = self.vreg(lm, not_v0=not vm)
        vs2 = self.vreg(lm)
        if r.random() < 0.5:
            self.emit(f"{op}.vv v{vd}, v{self.vreg(lm)}, v{vs2}{ms}")
        else:
            self.emit(f"{op}.vx v{vd}, {ABI[r.choice(SREGS)]}, v{vs2}{ms}")

    def gen_carry(self):
        r = self.r
        lm = self.lmul
        g = self.grp(lm)
        op = r.choice(["vadc", "vsbc", "vmadc", "vmsbc"])
        f = r.choice("vxi" if op in ("vadc", "vmadc") else "vx")
        b = {"v": f"v{self.vreg(lm)}", "x": ABI[r.choice(SREGS)], "i": str(r.randint(-16, 15))}[f]
        if op in ("vadc", "vsbc"):
            vd = self.vreg(lm, not_v0=True)
            vs2 = self.vreg(lm)
            self.emit(f"{op}.v{f}m v{vd}, v{vs2}, {b}, v0")
            return
        vs2 = self.vreg(lm)
        av = [(vs2, g)] + ([(int(b[1:]), g)] if f == "v" else [])
        vd = self.vreg(0, avoid=av)
        if r.random() < 0.5:
            self.emit(f"{op}.v{f}m v{vd}, v{vs2}, {b}, v0")
        else:
            self.emit(f"{op}.v{f} v{vd}, v{vs2}, {b}")

    # ------------------------------------------------------------------ V2：加宽 / 收窄 / 扩展
    WIDE = {"vwaddu": "vxw", "vwadd": "vxw", "vwsubu": "vxw", "vwsub": "vxw",
            "vwmulu": "vx", "vwmul": "vx", "vwmulsu": "vx",
            "vwmaccu": "vx", "vwmacc": "vx", "vwmaccsu": "vx", "vwmaccus": "x"}

    def gen_wide(self):
        r = self.r
        if self.sew > 2 or self.lmul > 2:
            return
        lm = self.lmul
        g, gd = self.grp(lm), self.grp(lm + 1)
        op = r.choice(list(self.WIDE))
        f = r.choice(self.WIDE[op])
        vm, ms = self.mask_suffix()
        vd = self.vreg(lm + 1, not_v0=not vm)
        if "macc" in op:
            vs2 = self.vreg(lm, avoid=[(vd, gd)])
            if f == "v":
                self.emit(f"{op}.vv v{vd}, v{self.vreg(lm, avoid=[(vd, gd)])}, v{vs2}{ms}")
            else:
                self.emit(f"{op}.vx v{vd}, {ABI[r.choice(SREGS)]}, v{vs2}{ms}")
            return
        if f == "w":                    # .wv / .wx：vs2 宽
            vs2 = self.vreg(lm + 1)
            if r.random() < 0.5:
                self.emit(f"{op}.wv v{vd}, v{vs2}, v{self.vreg(lm, avoid=[(vd, gd)])}{ms}")
            else:
                self.emit(f"{op}.wx v{vd}, v{vs2}, {ABI[r.choice(SREGS)]}{ms}")
            return
        vs2 = self.vreg(lm, avoid=[(vd, gd)])
        if f == "v":
            self.emit(f"{op}.vv v{vd}, v{vs2}, v{self.vreg(lm, avoid=[(vd, gd)])}{ms}")
        else:
            self.emit(f"{op}.vx v{vd}, v{vs2}, {ABI[r.choice(SREGS)]}{ms}")

    # ------------------------------------------------------------------ Zvbb
    def gen_bb(self):
        r = self.r
        lm = self.lmul
        vm, ms = self.mask_suffix()
        c = r.random()
        if c < 0.35:                                   # 一元
            op = r.choice(["vbrev", "vbrev8", "vrev8", "vclz", "vctz", "vcpop"])
            vd = self.vreg(lm, not_v0=not vm)
            self.emit(f"{op}.v v{vd}, v{self.vreg(lm)}{ms}")
        elif c < 0.8:
            op, forms = r.choice([("vandn", "vx"), ("vrol", "vx"), ("vror", "vxi")])
            f = r.choice(forms)
            vd = self.vreg(lm, not_v0=not vm)
            vs2 = self.vreg(lm)
            if f == "v":
                self.emit(f"{op}.vv v{vd}, v{vs2}, v{self.vreg(lm)}{ms}")
            elif f == "x":
                self.emit(f"{op}.vx v{vd}, v{vs2}, {ABI[r.choice(SREGS)]}{ms}")
            else:
                self.emit(f"{op}.vi v{vd}, v{vs2}, {r.randint(0, 63)}{ms}")
        else:                                          # vwsll (加宽：SEW <= 32、LMUL <= 4)
            if self.sew > 2 or self.lmul > 2:
                return
            gd = self.grp(lm + 1)
            vd = self.vreg(lm + 1, not_v0=not vm)
            vs2 = self.vreg(lm, avoid=[(vd, gd)])
            f = r.choice("vxi")
            if f == "v":
                self.emit(f"vwsll.vv v{vd}, v{vs2}, v{self.vreg(lm, avoid=[(vd, gd)])}{ms}")
            elif f == "x":
                self.emit(f"vwsll.vx v{vd}, v{vs2}, {ABI[r.choice(SREGS)]}{ms}")
            else:
                self.emit(f"vwsll.vi v{vd}, v{vs2}, {r.randint(0, 31)}{ms}")

    def gen_narrow(self):
        r = self.r
        if self.sew > 2 or self.lmul > 2:
            return
        if r.random() < 0.1:
            self.emit(f"csrwi vxrm, {r.randint(0, 3)}")
        lm = self.lmul
        g2 = self.grp(lm + 1)
        op = r.choice(["vnsrl", "vnsra", "vnclipu", "vnclip"])
        vm, ms = self.mask_suffix()
        vs2 = self.vreg(lm + 1)
        vd = vs2 if r.random() < 0.15 and not (not vm and vs2 == 0) else self.vreg(lm, avoid=[(vs2, g2)], not_v0=not vm)
        f = r.choice("vxi")
        if f == "v":
            self.emit(f"{op}.wv v{vd}, v{vs2}, v{self.vreg(lm)}{ms}")
        elif f == "x":
            self.emit(f"{op}.wx v{vd}, v{vs2}, {ABI[r.choice(SREGS)]}{ms}")
        else:
            self.emit(f"{op}.wi v{vd}, v{vs2}, {r.randint(0, 31)}{ms}")

    def gen_ext(self):
        r = self.r
        dv = r.choice([1, 2, 3])                 # vf2 / vf4 / vf8
        if self.sew < dv or self.lmul - dv < -3:
            return
        lm = self.lmul
        vm, ms = self.mask_suffix()
        vd = self.vreg(lm, not_v0=not vm)
        vs2 = self.vreg(lm - dv, avoid=[(vd, self.grp(lm))])
        self.emit(f"v{r.choice('zs')}ext.vf{1 << dv} v{vd}, v{vs2}{ms}")

    # ------------------------------------------------------------------ V2b/V2c：除法 / 归约 / 掩码 / 置换
    def small_x(self, hi):
        """一个标量寄存器：多数时候先装入小值 (偏移 / 下标)，否则用原来的随机值"""
        r = self.r
        x = ABI[r.choice(SREGS)]
        if r.random() < 0.75:
            self.emit(f"li {x}, {r.randint(0, hi)}")
        return x

    def gen_div(self):
        self.sss(self.r.choice(["vdivu", "vdiv", "vremu", "vrem"]), "vx")

    RED = ["vredsum", "vredand", "vredor", "vredxor", "vredminu", "vredmin", "vredmaxu", "vredmax"]

    def gen_red(self):
        r = self.r
        vm, ms = self.mask_suffix()
        vs2 = self.vreg(self.lmul)
        vd, vs1 = r.randint(0, 31), r.randint(0, 31)
        if r.random() < 0.3 and self.sew <= 2:
            self.emit(f"{r.choice(['vwredsumu', 'vwredsum'])}.vs v{vd}, v{vs2}, v{vs1}{ms}")
        else:
            self.emit(f"{r.choice(self.RED)}.vs v{vd}, v{vs2}, v{vs1}{ms}")

    def gen_mlog(self):
        r = self.r
        op = r.choice(["vmand", "vmnand", "vmandn", "vmxor", "vmor", "vmnor", "vmorn", "vmxnor"])
        self.emit(f"{op}.mm v{r.randint(0, 31)}, v{r.randint(0, 31)}, v{r.randint(0, 31)}")

    def gen_mset(self):
        r = self.r
        vm, ms = self.mask_suffix()
        k = r.random()
        if k < 0.35:
            vs2 = r.randint(0, 31)
            vd = r.choice([v for v in range(32) if v != vs2 and not (not vm and v == 0)])
            self.emit(f"{r.choice(['vmsbf', 'vmsif', 'vmsof'])}.m v{vd}, v{vs2}{ms}")
        elif k < 0.6:
            vs2 = r.randint(0, 31)
            vd = self.vreg(self.lmul, avoid=[(vs2, 1)], not_v0=not vm)
            self.emit(f"viota.m v{vd}, v{vs2}{ms}")
        elif k < 0.75:
            self.emit(f"vid.v v{self.vreg(self.lmul, not_v0=not vm)}{ms}")
        else:
            self.emit(f"{r.choice(['vcpop', 'vfirst'])}.m {ABI[r.choice(SREGS)]}, v{r.randint(0, 31)}{ms}")

    def gen_perm(self):
        r = self.r
        lm = self.lmul
        g = self.grp(lm)
        vm, ms = self.mask_suffix()
        vl = self.vlmax(self.sew, lm)
        k = r.choice(["up", "down", "1up", "1down", "gath", "gath16", "comp"])
        if k in ("up", "down"):
            vs2 = self.vreg(lm)
            vd = self.vreg(lm, avoid=[(vs2, g)] if k == "up" else [], not_v0=not vm)
            if r.random() < 0.5:
                self.emit(f"vslide{k}.vi v{vd}, v{vs2}, {r.randint(0, 31)}{ms}")
            else:
                self.emit(f"vslide{k}.vx v{vd}, v{vs2}, {self.small_x(vl + 4)}{ms}")
        elif k in ("1up", "1down"):
            vs2 = self.vreg(lm)
            vd = self.vreg(lm, avoid=[(vs2, g)] if k == "1up" else [], not_v0=not vm)
            self.emit(f"vslide{k}.vx v{vd}, v{vs2}, {ABI[r.choice(SREGS)]}{ms}")
        elif k == "gath":
            vs2 = self.vreg(lm)
            f = r.choice("vxi")
            if f == "v":
                vs1 = self.vreg(lm)
                vd = self.vreg(lm, avoid=[(vs2, g), (vs1, g)], not_v0=not vm)
                self.emit(f"vrgather.vv v{vd}, v{vs2}, v{vs1}{ms}")
            elif f == "x":
                vd = self.vreg(lm, avoid=[(vs2, g)], not_v0=not vm)
                self.emit(f"vrgather.vx v{vd}, v{vs2}, {self.small_x(vl + 4)}{ms}")
            else:
                vd = self.vreg(lm, avoid=[(vs2, g)], not_v0=not vm)
                self.emit(f"vrgather.vi v{vd}, v{vs2}, {r.randint(0, 31)}{ms}")
        elif k == "gath16":
            l16 = 1 - self.sew + lm
            if l16 < -3 or l16 > 3:
                return
            vs2 = self.vreg(lm)
            vs1 = self.vreg(l16)
            vd = self.vreg(lm, avoid=[(vs2, g), (vs1, self.grp(l16))], not_v0=not vm)
            self.emit(f"vrgatherei16.vv v{vd}, v{vs2}, v{vs1}{ms}")
        else:
            vs2 = self.vreg(lm)
            vs1 = r.randint(0, 31)
            vd = self.vreg(lm, avoid=[(vs2, g), (vs1, 1)])
            self.emit(f"vcompress.vm v{vd}, v{vs2}, v{vs1}")

    def gen_vmvr(self):
        r = self.r
        nr = r.choice([1, 2, 4, 8])
        self.emit(f"vmv{nr}r.v v{r.randrange(0, 32, nr)}, v{r.randrange(0, 32, nr)}")

    def gen_idx(self):
        """把一组寄存器的元素限成小下标 (给 vrgather.vv / vrgatherei16 用)"""
        r = self.r
        lm = self.lmul
        vd = self.vreg(lm)
        self.emit(f"vand.vi v{vd}, v{vd}, {r.choice([7, 15])}")

    # ------------------------------------------------------------------ V3：浮点
    def fp_ok(self, sews=(1, 2, 3)):
        return (not self.vill) and (not self.altf) and self.sew in sews

    def freg(self):
        return f"f{self.r.randint(0, 31)}"

    def fp_rm(self):
        """偶尔换舍入模式 (TRAPS 时偶尔给非法值，之后的浮点向量指令应报非法)"""
        r = self.r
        if r.random() < 0.06:
            if self.traps and r.random() < 0.1:
                self.emit(f"fsrmi {r.randint(5, 7)}")
            else:
                self.emit(f"fsrmi {r.randint(0, 4)}")

    def gen_fpa(self):
        r = self.r
        if not self.fp_ok():
            return
        self.fp_rm()
        op = r.choice(["vfadd", "vfsub", "vfmul", "vfdiv", "vfmin", "vfmax", "vfsgnj", "vfsgnjn", "vfsgnjx",
                       "vfrsub", "vfrdiv"])
        vm, ms = self.mask_suffix()
        lm = self.lmul
        vd = self.vreg(lm, not_v0=not vm)
        vs2 = self.vreg(lm)
        if op in ("vfrsub", "vfrdiv") or r.random() < 0.4:
            self.emit(f"{op}.vf v{vd}, v{vs2}, {self.freg()}{ms}")
        else:
            self.emit(f"{op}.vv v{vd}, v{vs2}, v{self.vreg(lm)}{ms}")

    def gen_fma(self):
        r = self.r
        if not self.fp_ok():
            return
        self.fp_rm()
        op = r.choice(["vfmacc", "vfnmacc", "vfmsac", "vfnmsac", "vfmadd", "vfnmadd", "vfmsub", "vfnmsub"])
        vm, ms = self.mask_suffix()
        lm = self.lmul
        vd = self.vreg(lm, not_v0=not vm)
        vs2 = self.vreg(lm)
        if r.random() < 0.5:
            self.emit(f"{op}.vv v{vd}, v{self.vreg(lm)}, v{vs2}{ms}")
        else:
            self.emit(f"{op}.vf v{vd}, {self.freg()}, v{vs2}{ms}")

    def gen_fwide(self):
        r = self.r
        if not self.fp_ok((1, 2)) or self.lmul > 2:
            return
        self.fp_rm()
        lm = self.lmul
        gd = self.grp(lm + 1)
        vm, ms = self.mask_suffix()
        vd = self.vreg(lm + 1, not_v0=not vm)
        op = r.choice(["vfwadd", "vfwsub", "vfwadd.w", "vfwsub.w", "vfwmul", "vfwmacc", "vfwnmacc", "vfwmsac",
                       "vfwnmsac", "vfwmaccbf16"])
        if op == "vfwmaccbf16" and self.sew != 1:
            return
        if "macc" in op or "msac" in op:
            vs2 = self.vreg(lm, avoid=[(vd, gd)])
            if r.random() < 0.5:
                self.emit(f"{op}.vv v{vd}, v{self.vreg(lm, avoid=[(vd, gd)])}, v{vs2}{ms}")
            else:
                self.emit(f"{op}.vf v{vd}, {self.freg()}, v{vs2}{ms}")
        elif op.endswith(".w"):
            vs2 = self.vreg(lm + 1)
            if r.random() < 0.5:
                self.emit(f"{op[:-2]}.wv v{vd}, v{vs2}, v{self.vreg(lm, avoid=[(vd, gd)])}{ms}")
            else:
                self.emit(f"{op[:-2]}.wf v{vd}, v{vs2}, {self.freg()}{ms}")
        else:
            vs2 = self.vreg(lm, avoid=[(vd, gd)])
            if r.random() < 0.5:
                self.emit(f"{op}.vv v{vd}, v{vs2}, v{self.vreg(lm, avoid=[(vd, gd)])}{ms}")
            else:
                self.emit(f"{op}.vf v{vd}, v{vs2}, {self.freg()}{ms}")

    def gen_fcvt(self):
        r = self.r
        if self.vill or self.altf:
            return
        self.fp_rm()
        lm = self.lmul
        vm, ms = self.mask_suffix()
        k = r.random()
        if k < 0.35:                                     # 同宽
            if not self.fp_ok():
                return
            op = r.choice(["vfcvt.xu.f.v", "vfcvt.x.f.v", "vfcvt.f.xu.v", "vfcvt.f.x.v", "vfcvt.rtz.xu.f.v",
                           "vfcvt.rtz.x.f.v"])
            self.emit(f"{op} v{self.vreg(lm, not_v0=not vm)}, v{self.vreg(lm)}{ms}")
        elif k < 0.7:                                    # 加宽
            if lm > 2:
                return
            op = r.choice(["vfwcvt.xu.f.v", "vfwcvt.x.f.v", "vfwcvt.f.xu.v", "vfwcvt.f.x.v", "vfwcvt.f.f.v",
                           "vfwcvt.rtz.xu.f.v", "vfwcvt.rtz.x.f.v", "vfwcvtbf16.f.f.v"])
            ok = {"vfwcvt.f.xu.v": (0, 1, 2), "vfwcvt.f.x.v": (0, 1, 2), "vfwcvtbf16.f.f.v": (1,)}.get(op, (1, 2))
            if self.sew not in ok:
                return
            vd = self.vreg(lm + 1, not_v0=not vm)
            vs2 = self.vreg(lm, avoid=[(vd, self.grp(lm + 1))])
            self.emit(f"{op} v{vd}, v{vs2}{ms}")
        else:                                            # 收窄
            if lm > 2:
                return
            op = r.choice(["vfncvt.xu.f.w", "vfncvt.x.f.w", "vfncvt.f.xu.w", "vfncvt.f.x.w", "vfncvt.f.f.w",
                           "vfncvt.rod.f.f.w", "vfncvt.rtz.xu.f.w", "vfncvt.rtz.x.f.w", "vfncvtbf16.f.f.w"])
            ok = {"vfncvt.xu.f.w": (0, 1, 2), "vfncvt.x.f.w": (0, 1, 2), "vfncvt.rtz.xu.f.w": (0, 1, 2),
                  "vfncvt.rtz.x.f.w": (0, 1, 2), "vfncvtbf16.f.f.w": (1,)}.get(op, (1, 2))
            if self.sew not in ok:
                return
            vs2 = self.vreg(lm + 1)
            vd = vs2 if r.random() < 0.15 and not (not vm and vs2 == 0) else                 self.vreg(lm, avoid=[(vs2, self.grp(lm + 1))], not_v0=not vm)
            self.emit(f"{op} v{vd}, v{vs2}{ms}")

    def gen_fcmp(self):
        r = self.r
        if not self.fp_ok():
            return
        lm = self.lmul
        g = self.grp(lm)
        vm, ms = self.mask_suffix()
        op = r.choice(["vmfeq", "vmfne", "vmflt", "vmfle", "vmfgt", "vmfge"])
        vs2 = self.vreg(lm)
        if op in ("vmfgt", "vmfge") or r.random() < 0.4:
            vd = self.vreg(0, avoid=[(vs2, g)] if g > 1 else [])
            self.emit(f"{op}.vf v{vd}, v{vs2}, {self.freg()}{ms}")
        else:
            vs1 = self.vreg(lm)
            vd = self.vreg(0, avoid=([(vs2, g), (vs1, g)] if g > 1 else []))
            self.emit(f"{op}.vv v{vd}, v{vs2}, v{vs1}{ms}")

    def gen_fred(self):
        r = self.r
        if not self.fp_ok():
            return
        self.fp_rm()
        vm, ms = self.mask_suffix()
        vs2 = self.vreg(self.lmul)
        vd, vs1 = r.randint(0, 31), r.randint(0, 31)
        if r.random() < 0.3 and self.sew in (1, 2):
            self.emit(f"{r.choice(['vfwredusum', 'vfwredosum'])}.vs v{vd}, v{vs2}, v{vs1}{ms}")
        else:
            self.emit(f"{r.choice(['vfredusum', 'vfredosum', 'vfredmin', 'vfredmax'])}.vs v{vd}, v{vs2}, v{vs1}{ms}")

    def gen_fmisc(self):
        r = self.r
        if not self.fp_ok():
            return
        self.fp_rm()
        lm = self.lmul
        g = self.grp(lm)
        vm, ms = self.mask_suffix()
        k = r.choice(["sqrt", "rec7", "rsqrt7", "class", "merge", "mvvf", "mvsf", "mvfs", "s1up", "s1down"])
        if k in ("sqrt", "rec7", "rsqrt7", "class"):
            self.emit(f"vf{k}.v v{self.vreg(lm, not_v0=not vm)}, v{self.vreg(lm)}{ms}")
        elif k == "merge":
            self.emit(f"vfmerge.vfm v{self.vreg(lm, not_v0=True)}, v{self.vreg(lm)}, {self.freg()}, v0")
        elif k == "mvvf":
            self.emit(f"vfmv.v.f v{self.vreg(lm)}, {self.freg()}")
        elif k == "mvsf":
            self.emit(f"vfmv.s.f v{r.randint(0, 31)}, {self.freg()}")
        elif k == "mvfs":
            self.emit(f"vfmv.f.s {self.freg()}, v{r.randint(0, 31)}")
        else:
            vs2 = self.vreg(lm)
            vd = self.vreg(lm, avoid=[(vs2, g)] if k == "s1up" else [], not_v0=not vm)
            self.emit(f"vfslide1{k[2:]}.vf v{vd}, v{vs2}, {self.freg()}{ms}")

    def gen_finit(self):
        """把一组寄存器换成"好看"的浮点数 (小整数转来)，否则随机位型里 NaN / 极端指数太多"""
        r = self.r
        if not self.fp_ok():
            return
        vd = self.vreg(self.lmul)
        self.emit(f"vand.vi v{vd}, v{vd}, {r.choice([7, 15])}")
        self.emit(f"vfcvt.f.x.v v{vd}, v{vd}")

    # ------------------------------------------------------------------ V4：分段 / 索引 / 整寄存器 / 掩码 / fault-only-first
    EEWS = {0: 8, 1: 16, 2: 32, 3: 64}

    def base_reg(self, lo, hi, align):
        """装一个 vdata 里 [lo, hi] 之间、按 align 对齐的基址，返回寄存器名"""
        r = self.r
        rb = r.choice(SREGS)
        b = r.randint(lo, max(lo, hi))
        b -= b % align
        self.emit(f"li {ABI[rb]}, {b}")
        self.emit(f"add {ABI[rb]}, {ABI[rb]}, s0")
        return ABI[rb]

    def seg_vd(self, nf, em, not_v0):
        """分段的目的寄存器：按 em 对齐，vd + nf*em <= 32"""
        c = [v for v in range(0, 32, em) if v + nf * em <= 32 and not (not_v0 and v == 0)]
        return self.r.choice(c) if c else None

    def gen_mseg(self):
        r = self.r
        if self.vill:
            return
        eb = r.randint(0, 3)
        le = eb - self.sew + self.lmul
        if not -3 <= le <= 3:
            return
        em = 1 << le if le > 0 else 1
        if em > 4:
            return
        nf = r.randint(2, 8 // em)
        st = r.random() < 0.4
        vm, ms = self.mask_suffix()
        vd = self.seg_vd(nf, em, not st and not vm)
        if vd is None:
            return
        w = 1 << eb
        eew = 8 << eb
        if self.traps and r.random() < 0.15 and self.vl > 1:     # 第 k 个元素的第 f 段碰到禁区页：之前的段已写，vstart = k
            k = r.randint(0, min(self.vl - 1, VDATA // (nf * w) - 1))
            f = r.randint(0, nf - 1)
            rb = ABI[r.choice(SREGS)]
            rt = ABI[r.choice([x for x in SREGS if ABI[x] != rb])]
            self.emit(f"la {rb}, vbad")
            self.emit(f"li {rt}, {-(k * nf * w + f * w)}")
            self.emit(f"add {rb}, {rb}, {rt}")
            self.emit(f"v{'s' if st else 'l'}seg{nf}e{eew}.v v{vd}, ({rb}){ms}")
        elif r.random() < 0.6:                                   # 单位步长
            span = max(self.vl, 1) * nf * w
            rb = self.base_reg(0, VDATA - span, w)
            self.emit(f"v{'s' if st else 'l'}seg{nf}e{eew}.v v{vd}, ({rb}){ms}")
        else:                                                    # 跨步
            stride = r.choice([nf * w, (nf + r.randint(0, 3)) * w, -nf * w, 0, w])
            span = abs(stride) * max(self.vl - 1, 0) + nf * w
            if span > VDATA // 2:
                stride = nf * w
                span = nf * w * max(self.vl, 1)
            rb = self.base_reg(span if stride < 0 else 0, VDATA - (0 if stride < 0 else span) - nf * w, w)
            rs = r.choice([x for x in SREGS if ABI[x] != rb])
            self.emit(f"li {ABI[rs]}, {stride}")
            self.emit(f"v{'s' if st else 'l'}sseg{nf}e{eew}.v v{vd}, ({rb}), {ABI[rs]}{ms}")

    def gen_midx(self):
        r = self.r
        if self.vill:
            return
        ib = r.randint(0, 3)                                     # 索引 EEW
        li = ib - self.sew + self.lmul
        if not -3 <= li <= 3 or not self.legal_vtype(ib, li):
            return
        fl = 1 << self.lmul if self.lmul > 0 else 1
        nf = 1 if r.random() < 0.5 else r.randint(2, max(2, 8 // fl))
        if nf * fl > 8:
            nf = 1
        st = r.random() < 0.4
        vm, ms = self.mask_suffix()
        vd = self.seg_vd(nf, fl, not st and not vm)
        if vd is None:
            return
        gi = self.grp(li)
        vi = [v for v in range(0, 32, gi) if not self.ovl(v, gi, vd, nf * fl)]
        if not vi:
            return
        vs2 = r.choice(vi)
        w = 1 << self.sew
        rng = min(4096, 1 << (8 << ib) if ib < 2 else 4096)
        mask = (rng - 1) & ~(w - 1)
        # 把索引寄存器限到 [0, rng) 并按 SEW 对齐 (换到索引的 SEW / EMUL，vl 不变，再换回来)
        names = {-3: "mf8", -2: "mf4", -1: "mf2", 0: "m1", 1: "m2", 2: "m4", 3: "m8"}
        rt = r.choice(SREGS)
        self.emit(f"li {ABI[rt]}, {mask}")
        self.emit(f"vsetvli zero, zero, e{8 << ib}, {names[li]}, tu, mu")
        self.emit(f"vand.vx v{vs2}, v{vs2}, {ABI[rt]}")
        self.emit(f"vsetvli zero, zero, e{8 << self.sew}, {names[self.lmul]}, tu, mu")
        self.altf = False           # vsetvli 没有 altfmt 位：换回来之后 altfmt = 0 (否则后面会按 altfmt = 1 生成非法的 vfwdota 等)
        rb = self.base_reg(0, VDATA - rng - nf * w, 8)
        op = r.choice(["u", "o"])
        seg = f"seg{nf}" if nf > 1 else ""
        self.emit(f"v{'s' if st else 'l'}{op}x{seg}ei{8 << ib}.v v{vd}, ({rb}), v{vs2}{ms}")

    def gen_mwhole(self):
        r = self.r
        nr = r.choice([1, 2, 4, 8])
        vd = r.randrange(0, 32, nr)
        if r.random() < 0.6:
            eb = r.randint(0, 3)
            rb = self.base_reg(0, VDATA - nr * 128, 1 << eb)
            self.emit(f"vl{nr}re{8 << eb}.v v{vd}, ({rb})")
        else:
            rb = self.base_reg(0, VDATA - nr * 128, 1)
            self.emit(f"vs{nr}r.v v{vd}, ({rb})")

    def gen_mmask(self):
        r = self.r
        if self.vill:
            return
        rb = self.base_reg(0, VDATA - 128, 1)
        self.emit(f"v{r.choice('ls')}m.v v{r.randint(0, 31)}, ({rb})")

    def gen_mff(self):
        r = self.r
        if self.vill:
            return
        eb = r.randint(0, 3)
        le = eb - self.sew + self.lmul
        if not -3 <= le <= 3:
            return
        em = 1 << le if le > 0 else 1
        nf = 1 if r.random() < 0.6 else r.randint(2, max(2, 8 // em))
        if nf * em > 8:
            nf = 1
        vm, ms = self.mask_suffix()
        vd = self.seg_vd(nf, em, not vm)
        if vd is None:
            return
        w = 1 << eb
        seg = f"seg{nf}" if nf > 1 else ""
        rb = ABI[r.choice(SREGS)]
        rt = ABI[r.choice([x for x in SREGS if ABI[x] != rb])]
        if r.random() < 0.7 and self.vl > 1:
            # 第 k 个元素 (k >= 1) 落进禁区页：只截短 vl (TRAPS 时偶尔 k = 0，应陷入)
            # 第 k 个元素的第 f 段正好是禁区页的第一个字节 (之前的元素都在下面的有效内存里)
            kmax = min(self.vl - 1, VDATA // (nf * w) - 1)
            if kmax < 1:
                return
            k = r.randint(0 if self.traps and r.random() < 0.2 else 1, kmax)
            f = r.randint(0, nf - 1)
            self.emit(f"la {rb}, vbad")
            self.emit(f"li {rt}, {-(k * nf * w + f * w)}")
            self.emit(f"add {rb}, {rb}, {rt}")
        else:
            rb = self.base_reg(0, VDATA - max(self.vl, 1) * nf * w, w)
        self.emit(f"vl{seg}e{8 << eb}ff.v v{vd}, ({rb}){ms}")
        self.emit(f"csrr {ABI[r.choice(SREGS)]}, vl")
        self.vl_unknown = True

    # ------------------------------------------------------------------ V5：OFP8 转换与点积 (.word 编码)
    def opv(self, f6, vm, vs2, vs1, f3, vd, opc=0x57):
        w = (f6 << 26) | (vm << 25) | (vs2 << 20) | (vs1 << 15) | (f3 << 12) | (vd << 7) | opc
        return f".word 0x{w:08x}"

    def gen_fp8(self):
        r = self.r
        if self.vill or self.sew != 0:
            return
        self.fp_rm()
        lm = self.lmul
        vm, ms = self.mask_suffix()
        k = r.choice(["w", "n", "ns", "q", "qs"])
        if k == "w":                                     # vfwcvtbf16.f.f.v：OFP8 -> BF16 (DSS)
            if lm > 2:
                return
            vd = self.vreg(lm + 1, not_v0=not vm)
            vs2 = self.vreg(lm, avoid=[(vd, self.grp(lm + 1))])
            self.emit(self.opv(0b010010, int(vm), vs2, 0b01101, 1, vd) + f"   # vfwcvtbf16.f.f.v v{vd}, v{vs2}{ms}")
        elif k in ("n", "ns"):                           # vfncvtbf16 (.sat).f.f.w：BF16 -> OFP8 (SDS)
            if lm > 2:
                return
            vs2 = self.vreg(lm + 1)
            vd = vs2 if r.random() < 0.15 and not (not vm and vs2 == 0) else \
                self.vreg(lm, avoid=[(vs2, self.grp(lm + 1))], not_v0=not vm)
            v1 = 0b11101 if k == "n" else 0b11111
            self.emit(self.opv(0b010010, int(vm), vs2, v1, 1, vd) + f"   # vfncvtbf16{'.sat' if k == 'ns' else ''}.f.f.w v{vd}, v{vs2}{ms}")
        else:                                            # vfncvt (.sat).f.f.q：FP32 -> OFP8 (SQS)
            if lm > 1:
                return
            vs2 = self.vreg(lm + 2)
            vd = vs2 if r.random() < 0.15 and not (not vm and vs2 == 0) else \
                self.vreg(lm, avoid=[(vs2, self.grp(lm + 2))], not_v0=not vm)
            v1 = 0b11001 if k == "q" else 0b11011
            self.emit(self.opv(0b010010, int(vm), vs2, v1, 1, vd) + f"   # vfncvt{'.sat' if k == 'qs' else ''}.f.f.q v{vd}, v{vs2}{ms}")

    def gen_dot(self):
        r = self.r
        if self.vill:
            return
        lm = self.lmul
        g = self.grp(lm)
        vm, ms = self.mask_suffix()
        if self.sew == 0:
            kinds = ["iu", "is", "f8", "f8a"]
        elif self.sew == 1:
            kinds = ["iu", "is"] + (["bf"] if self.altf else [])
        else:
            return
        k = r.choice(kinds)
        if k in ("f8", "f8a", "bf"):
            self.fp_rm()
        vs1 = self.vreg(lm)
        vs2 = self.vreg(lm)
        vd = self.vreg(0, avoid=[(vs1, g), (vs2, g)], not_v0=not vm)
        f6, f3 = {"iu": (0b100110, 0), "is": (0b100111, 0), "f8": (0b100110, 1), "f8a": (0b100111, 1),
                  "bf": (0b100100, 1)}[k]
        name = {"iu": "vqwdotau", "is": "vqwdotas", "f8": "vfqwdota", "f8a": "vfqwdota.alt", "bf": "vfwdota"}[k]
        self.emit(self.opv(f6, int(vm), vs2, vs1, f3, vd, 0x77) + f"   # {name}.vv v{vd}, v{vs2}, v{vs1}{ms}")

    # ------------------------------------------------------------------ 访存
    def gen_mem(self):
        r = self.r
        eb = r.randint(0, 3)
        lm = eb - self.sew + self.lmul               # log2 EMUL
        if not (-3 <= lm <= 3):
            return
        eew = 8 << eb
        st = r.random() < 0.4
        vm, ms = self.mask_suffix()
        vd = self.vreg(lm, not_v0=(not vm and not st))
        nbytes = max(self.vl, 1) * (eew // 8)
        strided = r.random() < 0.35
        if strided:
            smax = max(1, (VDATA // 2) // max(self.vl, 1) // (eew // 8))
            stride = r.randint(-min(smax, 16), min(smax, 16)) * (eew // 8)
            span = abs(stride) * max(self.vl - 1, 0) + eew // 8
            base = r.randint(span if stride < 0 else 0, VDATA - (0 if stride < 0 else span) - eew // 8)
            base -= base % (eew // 8)
            if self.traps and r.random() < 0.03:
                base += 1                                  # 不对齐 -> 异常
            rb, rs = r.sample(SREGS, 2)                     # 基址与步长不能同一个寄存器
            self.emit(f"li {ABI[rb]}, {base}")
            self.emit(f"add {ABI[rb]}, {ABI[rb]}, s0")
            self.emit(f"li {ABI[rs]}, {stride}")
            self.emit(f"v{'s' if st else 'l'}se{eew}.v v{vd}, ({ABI[rb]}), {ABI[rs]}{ms}")
        else:
            base = r.randint(0, VDATA - nbytes)
            base -= base % (eew // 8)
            if self.traps and r.random() < 0.03:
                base += 1
            rb = r.choice(SREGS)
            self.emit(f"li {ABI[rb]}, {base}")
            self.emit(f"add {ABI[rb]}, {ABI[rb]}, s0")
            self.emit(f"v{'s' if st else 'l'}e{eew}.v v{vd}, ({ABI[rb]}){ms}")

    # ------------------------------------------------------------------ 主体
    def gen(self):
        r = self.r
        o = self.out
        o += [".section .text.init", ".globl _start", "_start:"]
        self.emit("la t4, trap")
        self.emit("csrw mtvec, t4")
        self.emit("li t4, (1 << 9) | (1 << 13)")         # VS / FS = Initial
        self.emit("csrs mstatus, t4")
        self.emit("la s0, vdata")
        self.emit("la t4, vbad")
        self.emit("srli t4, t4, 2")
        self.emit("ori t4, t4, 0x1FF")
        self.emit("csrw pmpaddr0, t4")
        self.emit("li t4, 0x98")                  # L | NAPOT | ---
        self.emit("csrw pmpcfg0, t4")
        # 向量寄存器初值：从随机数据区装入 (e8 m8 vl = 1024，每组 1 KB)
        self.emit("li s1, 1024")
        self.emit("vsetvli zero, s1, e8, m8, ta, ma")
        for g in range(4):
            self.emit(f"addi s1, s0, {g * 1024}" if g * 1024 < 2048 else f"li s1, {g * 1024}\n    add s1, s1, s0")
            self.emit(f"vle8.v v{g * 8}, (s1)")
        for rr in SREGS:
            self.emit(f"li {ABI[rr]}, {r.getrandbits(64) - (1 << 63) if r.random() < 0.7 else r.randint(-40, 40)}")
        for fr in range(32):
            k = r.random()
            if k < 0.3:
                v = (0xFFFF_FFFF_FFFF << 16) | r.getrandbits(16)
            elif k < 0.6:
                v = (0xFFFF_FFFF << 32) | r.getrandbits(32)
            elif k < 0.8:
                v = r.getrandbits(64)
            else:                                      # 小整数的浮点值 (三种宽度)
                import struct
                x = float(r.randint(-20, 20)) / r.choice([1, 2, 4, 8])
                w = r.choice([16, 32, 64])
                if w == 64:
                    v = struct.unpack("<Q", struct.pack("<d", x))[0]
                elif w == 32:
                    v = (0xFFFF_FFFF << 32) | struct.unpack("<I", struct.pack("<f", x))[0]
                else:
                    v = (0xFFFF_FFFF_FFFF << 16) | struct.unpack("<H", struct.pack("<e", x))[0]
            self.emit(f"li t4, {v - (1 << 64) if v >> 63 else v}")
            self.emit(f"fmv.d.x f{fr}, t4")
        self.gen_vset()
        gens = {"vset": (self.gen_vset, 1), "alu": (self.gen_alu, 6), "cmp": (self.gen_cmp, 3),
                "mvs": (self.gen_mvs, 1), "mem": (self.gen_mem, 3),
                "fx": (self.gen_fx, 3), "mul": (self.gen_mul, 3), "carry": (self.gen_carry, 2),
                "wide": (self.gen_wide, 3), "narrow": (self.gen_narrow, 2), "ext": (self.gen_ext, 1),
                "div": (self.gen_div, 2), "red": (self.gen_red, 3), "mlog": (self.gen_mlog, 2),
                "mset": (self.gen_mset, 3), "perm": (self.gen_perm, 4), "vmvr": (self.gen_vmvr, 1),
                "fpa": (self.gen_fpa, 4), "fma": (self.gen_fma, 3), "fwide": (self.gen_fwide, 3),
                "fcvt": (self.gen_fcvt, 3), "fcmp": (self.gen_fcmp, 2), "fred": (self.gen_fred, 2),
                "fmisc": (self.gen_fmisc, 3),
                "mseg": (self.gen_mseg, 3), "midx": (self.gen_midx, 3), "mwhole": (self.gen_mwhole, 1),
                "mmask": (self.gen_mmask, 1), "mff": (self.gen_mff, 2),
                "fp8": (self.gen_fp8, 3), "dot": (self.gen_dot, 3), "bb": (self.gen_bb, 3)}
        pool = []
        for c in self.cls:
            pool += [gens[c][0]] * gens[c][1]
        if "perm" in self.cls:
            pool += [self.gen_idx] * 2
        if any(c in self.cls for c in ("fpa", "fma", "fwide", "fcvt", "fcmp", "fred", "fmisc")):
            pool += [self.gen_finit] * 2
        for _ in range(self.n):
            g = r.choice(pool)
            if (self.vill or self.vl_unknown) and g != self.gen_vset:
                g = self.gen_vset
            g()
        self.emit("li t4, 1")
        self.emit("la t5, tohost")
        self.emit("sd t4, 0(t5)")
        o.append("1:  j 1b")
        o += ["    .align 2", "trap:"]
        if not self.traps:                    # 不该有异常：报失败 (tohost = (mcause << 1) | 1，mcause 非零)
            self.emit("csrr t4, mcause")
            self.emit("slli t4, t4, 1")
            self.emit("ori t4, t4, 3")
            self.emit("la t5, tohost")
            self.emit("sd t4, 0(t5)")
            o.append("2:  j 2b")
        self.emit("csrr t4, mepc")
        self.emit("addi t4, t4, 4")
        self.emit("csrw mepc, t4")
        self.emit("csrw vstart, zero")        # 访存出错留下的 vstart：清掉，后面的运算才合法
        self.emit("mret")
        o += ["    .data", "    .align 6", "vdata:"]
        for _ in range(VDATA // 8):
            o.append(f"    .dword 0x{r.getrandbits(64):016x}")
        o += ["    .align 12", "vbad:", "    .space 4096"]
        o += ["    .section .tohost, \"aw\", @progbits", "    .align 6", "    .globl tohost", "tohost: .dword 0",
              "    .align 6", "    .globl fromhost", "fromhost: .dword 0"]
        return "\n".join(o) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--n", type=int, default=400)
    ap.add_argument("--cls", default="vset,alu,cmp,mvs,mem,fx,mul,carry,wide,narrow,ext,div,red,mlog,mset,perm,vmvr,"
                                     "fpa,fma,fwide,fcvt,fcmp,fred,fmisc,mseg,midx,mwhole,mmask,mff,fp8,dot")
    ap.add_argument("--traps", action="store_true")
    ap.add_argument("-o", required=True)
    a = ap.parse_args()
    g = VGen(a.seed, a.n, a.cls.split(","), a.traps)
    open(a.o, "w").write(g.gen())


if __name__ == "__main__":
    main()
