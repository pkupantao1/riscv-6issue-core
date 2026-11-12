#!/usr/bin/env python3
"""gen_bootrom.py - 生成 BootROM 镜像 rtl/soc/o6_bootrom_img.svh

布局 (相对 0x1000)：
  0x00  auipc t0, 0
  0x04  addi  a1, t0, 0x40      # a1 = 设备树地址 (0x1040)
  0x08  csrr  a0, mhartid
  0x0c  ld    t0, 24(t0)
  0x10  jr    t0
  0x14  (填充)
  0x18  .dword 0x80000000       # 入口
  0x40  设备树 (DTB，可选：--dtb 给出 dtc 编译好的 .dtb)
用法: gen_bootrom.py [--dtb o6.dtb] -o rtl/soc/o6_bootrom_img.svh
"""
import argparse
import struct

CODE = [0x00000297, 0x04028593, 0xf1402573, 0x0182b283, 0x00028067, 0x00000000]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dtb")
    ap.add_argument("--entry", type=lambda x: int(x, 0), default=0x80000000)
    ap.add_argument("-o", required=True)
    a = ap.parse_args()
    img = b"".join(struct.pack("<I", w) for w in CODE)
    img += struct.pack("<Q", a.entry)
    img += b"\0" * (0x40 - len(img))
    if a.dtb:
        img += open(a.dtb, "rb").read()
    img += b"\0" * ((-len(img)) % 8)
    words = [struct.unpack_from("<Q", img, i)[0] for i in range(0, len(img), 8)]
    with open(a.o, "w", newline="\n", encoding="utf-8") as f:
        f.write("// 由 sw/bootrom/gen_bootrom.py 生成，不要手改\n")
        f.write(f"localparam int ROM_WORDS = {len(words)};\n")
        f.write("localparam logic [63:0] ROM [ROM_WORDS] = '{\n")
        f.write(",\n".join(f"  64'h{w:016x}" for w in words))
        f.write("\n};\n")


if __name__ == "__main__":
    main()
