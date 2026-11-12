#!/usr/bin/env python3
"""mk_sail_json.py - 由 cvw-rv64gc 的 sail.json 派生 OOO6 的 Sail 配置 (riscv-arch-test ACT4 用)

用法：mk_sail_json.py <cvw sail.json> <输出 sail.json>
改动依据：rtl/pkg/o6_pkg.sv (PA_W)、rtl/csr/o6_csr.sv (PMP / 计数器 / mtvec / ID CSR)、rtl/lsu/o6_lsu.sv (未对齐报异常、LR 按读)
"""
import json
import sys

import pyjson5

d = pyjson5.load(open(sys.argv[1]))
m = d["memory"]
m["physaddr_bits"] = 40                                   # o6_pkg::PA_W
m["asidlen"] = 16
m["pmp"].update({"grain": 4, "count": 16, "usable_count": 8,  # G = 4 (64 B，#34)；16 项中 8 项可用
                 "tor_supported": True, "na4_supported": False, "napot_supported": True})
for k in ("load_store", "vector", "amo", "lrsc"):          # 未对齐一律报未对齐异常
    m["misaligned"]["exceptions"][k] = {"Some": "AlignmentException"}

p = d["platform"]
p.update({"vendorid": 0, "archid": 0, "impid": 0, "hartid": 0})   # ID CSR 都读 0
p["wfi_is_nop"] = True                                     # WFI 按空操作实现
p["reservation"].update({"reservation_set_size_exp": 3,
                         "require_exact_reservation_addr": True,   # SC 要与 LR 精确同址 (#17)
                         "invalidate_on_same_hart_store": False})

b = d["base"]
b["mcountinhibit"]["writable_bits"] = {"len": 32, "value": "0x0000_007D"}   # CY / IR / HPM3..6，TM 只读 0
b["writable_hpm_counters"] = {"len": 32, "value": "0x0000_0078"}           # mhpmcounter3..6
b["mtvec"] = {"direct": {"supported": True, "base_alignment": 2}, "vectored": {"supported": True, "base_alignment": 2}}
b["stvec"] = {"direct": {"supported": True}, "vectored": {"supported": True, "base_alignment": 2}}

# RVA23 R1 (2026-10-29)：Zicond / Zcb / Zimop / Zcmop / Zawrs 已实现，从关闭列表去掉并显式打开 (ON)
ON  = ["Zicond", "Zcb", "Zimop", "Zcmop", "Zawrs", "Zicbom", "Zicboz", "Zicbop", "Zic64b", "Zfa",
       "Sstc", "Svinval", "Sv48", "Sv57", "Svnapot", "Svpbmt", "Smstateen", "Ssstateen"]
OFF = ["Zicclsm", "Zfh", "Zbc", "Zbkb", "Zbkc", "Zbkx",
       "Zknd", "Zkne", "Zknh", "Zkt", "Svadu", "Svvptc"]
ex = d["extensions"]
for k in OFF:
    if k in ex and isinstance(ex[k], dict) and "supported" in ex[k]:
        ex[k]["supported"] = False
for k in ON:
    if k in ex and isinstance(ex[k], dict) and "supported" in ex[k]:
        ex[k]["supported"] = True
json.dump(d, open(sys.argv[2], "w"), indent=2)
print("写出", sys.argv[2], "；关闭扩展：", ", ".join(k for k in OFF if k in ex))
