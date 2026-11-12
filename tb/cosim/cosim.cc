// cosim.cc - OOO6 与 Spike 的逐条提交对拍 + 测试平台存储模型 (DPI-C)
//
// 编译：用 conda 的 g++ 12 (Spike 需要 C++20)，静态链接 libstdc++，
//       得到 libcosim.so 交给 VCS 2018 (它自带的 gcc 4.8 编不了 Spike 头文件)。
//
// 两份存储：
//   g_dut : DUT 看到的内存 (TB 的理想 I/D 存储、MMIO 里的 BootROM 读都走这里)
//   spike : Spike 自己的内存 (simif_t::addr_to_mem)
// 两者都从同一个 ELF 装入，各自独立演化；store 是否一致靠后续 load 的值间接比对。
//
// 不确定量 (DUT 说了算，注入 Spike 而不比较)：
//   - 落在设备区的 load 的结果
//   - 读 cycle/time/mip/sip/hpmcounter/marchid/mimpid/mvendorid 的 CSR 指令结果
// 中断：DUT 在哪条指令前进中断由 DUT 决定；对拍时临时置 Spike 的 mip 对应位再 step。

#include "processor.h"
#include "simif.h"
#include "cfg.h"
#include "mmu.h"
#include "disasm.h"
#include "decode.h"
#include "encoding.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cinttypes>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <elf.h>

namespace {

constexpr uint64_t PGSZ = 4096;
constexpr uint64_t DRAM_BASE = 0x80000000ull;
constexpr uint64_t BOOTROM_BASE = 0x1000;

struct SparseMem {
  std::unordered_map<uint64_t, std::unique_ptr<uint8_t[]>> pages;
  uint8_t *page(uint64_t a, bool alloc) {
    uint64_t pn = a / PGSZ;
    auto it = pages.find(pn);
    if (it != pages.end()) return it->second.get();
    if (!alloc) return nullptr;
    auto p = std::make_unique<uint8_t[]>(PGSZ);
    std::memset(p.get(), 0, PGSZ);
    uint8_t *r = p.get();
    pages.emplace(pn, std::move(p));
    return r;
  }
  uint8_t *ptr(uint64_t a, bool alloc) {
    uint8_t *p = page(a, alloc);
    return p ? p + (a % PGSZ) : nullptr;
  }
  void write(uint64_t a, const uint8_t *d, size_t n) {
    for (size_t i = 0; i < n; i++) *ptr(a + i, true) = d[i];
  }
  uint64_t read64(uint64_t a) {
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) {
      uint8_t *p = ptr(a + i, false);
      v |= (uint64_t)(p ? *p : 0) << (8 * i);
    }
    return v;
  }
};

struct Region { uint64_t base, size; };

uint64_t g_dram_size = 0x80000000ull;

bool in_mem(uint64_t a) {
  return (a >= DRAM_BASE && a < DRAM_BASE + g_dram_size) || (a >= BOOTROM_BASE && a < BOOTROM_BASE + 0x10000);
}
bool in_dev(uint64_t a) {
  return (a >= 0x02000000 && a < 0x02010000) || (a >= 0x0C000000 && a < 0x10000000) ||
         (a >= 0x10000000 && a < 0x10001000);
}

SparseMem g_dut;

class SpikeIf : public simif_t {
 public:
  SparseMem mem;
  cfg_t cfg;
  std::map<size_t, processor_t *> harts;
  char *addr_to_mem(reg_t paddr) override {
    if (!in_mem(paddr)) return nullptr;
    return (char *)mem.ptr(paddr, true);
  }
  // 只有 SoC 设备区可访问 (其它地址 Spike 报访问错误，与 DUT 一致)
  bool mmio_load(reg_t a, size_t len, uint8_t *bytes) override {
    if (!in_dev(a)) return false;
    std::memset(bytes, 0, len);     // 设备读的值由 DUT 注入
    return true;
  }
  bool mmio_store(reg_t a, size_t, const uint8_t *) override { return in_dev(a); }
  void proc_reset(unsigned) override {}
  const cfg_t &get_cfg() const override { return cfg; }
  const std::map<size_t, processor_t *> &get_harts() const override { return harts; }
  const char *get_symbol(uint64_t) override { return nullptr; }
};

std::unique_ptr<SpikeIf> g_sif;
processor_t *g_proc = nullptr;
std::ostringstream g_sout;
uint64_t g_tohost = 0, g_fromhost = 0;
bool g_tohost_hit = false;
uint64_t g_tohost_val = 0;
uint64_t g_ncommit = 0;
int   g_vabits = 39;
int g_trap_tgt = -1;     // H：DUT 报的陷入目标 (0 M、1 HS、2 VS)；-1 = 不查 (旧 tb 不调用 cosim_trap_tgt)      // Spike 的最大虚地址位数 (satp 可写模式)；新 tb 经 cosim_vabits 设为 57 (RVA23 R4)
reg_t g_misa0 = 0;        // DUT 的 misa 只读：Spike 若被写改了就写回
int g_verbose = 0;
FILE *g_trace = nullptr;

// Spike 同款 BootROM：a0 = hartid, a1 = dtb (无), 跳到 DRAM_BASE
const uint32_t BOOTROM[] = {
  0x00000297,  // auipc t0, 0
  0x04028593,  // addi  a1, t0, 0x40   (与 SoC 的 BootROM 一致：设备树在 0x1040)
  0xf1402573,  // csrr  a0, mhartid
  0x0182b283,  // ld    t0, 24(t0)
  0x00028067,  // jr    t0
  0x00000000,
  (uint32_t)(DRAM_BASE & 0xffffffff), (uint32_t)(DRAM_BASE >> 32),
};

bool load_elf(const char *path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) { fprintf(stderr, "[cosim] 打不开 ELF %s\n", path); return false; }
  std::vector<uint8_t> b((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  if (b.size() < sizeof(Elf64_Ehdr)) return false;
  auto *eh = (Elf64_Ehdr *)b.data();
  if (std::memcmp(eh->e_ident, ELFMAG, SELFMAG) != 0 || eh->e_ident[EI_CLASS] != ELFCLASS64) {
    fprintf(stderr, "[cosim] 不是 ELF64\n"); return false;
  }
  for (int i = 0; i < eh->e_phnum; i++) {
    auto *ph = (Elf64_Phdr *)(b.data() + eh->e_phoff + i * eh->e_phentsize);
    if (ph->p_type != PT_LOAD || ph->p_memsz == 0) continue;
    std::vector<uint8_t> seg(ph->p_memsz, 0);
    std::memcpy(seg.data(), b.data() + ph->p_offset, ph->p_filesz);
    g_dut.write(ph->p_paddr, seg.data(), seg.size());
    g_sif->mem.write(ph->p_paddr, seg.data(), seg.size());
  }
  // 符号表
  auto *sh = (Elf64_Shdr *)(b.data() + eh->e_shoff);
  for (int i = 0; i < eh->e_shnum; i++) {
    if (sh[i].sh_type != SHT_SYMTAB) continue;
    auto *strtab = (const char *)(b.data() + sh[sh[i].sh_link].sh_offset);
    auto *sym = (Elf64_Sym *)(b.data() + sh[i].sh_offset);
    size_t n = sh[i].sh_size / sizeof(Elf64_Sym);
    for (size_t j = 0; j < n; j++) {
      const char *nm = strtab + sym[j].st_name;
      if (!std::strcmp(nm, "tohost")) g_tohost = sym[j].st_value;
      if (!std::strcmp(nm, "fromhost")) g_fromhost = sym[j].st_value;
    }
  }
  return true;
}

// 不确定 CSR：DUT 说了算
bool nondet_csr(uint32_t csr) {
  switch (csr) {
    case CSR_CYCLE: case CSR_TIME: case CSR_MCYCLE: case CSR_MIP: case CSR_SIP:
    case CSR_HIP: case CSR_VSIP:   // H：VSTIP 由 DUT 的 vstimecmp 比较决定 (Spike 的 time 影子值不走)
    case CSR_MARCHID: case CSR_MIMPID: case CSR_MVENDORID:
    case CSR_SCOUNTOVF:      // Sscofpmf：OF 位由 DUT 计数溢出置位 (Spike 的 hpm 计数器不计事件)
      return true;
    default:
      // mhpmevent：DUT 实现了事件选择 (1..8)，Spike 恒 0 —— 读值以 DUT 为准
      return (csr >= CSR_HPMCOUNTER3 && csr <= CSR_HPMCOUNTER31) ||
             (csr >= CSR_MHPMCOUNTER3 && csr <= CSR_MHPMCOUNTER31) ||
             (csr >= CSR_MHPMEVENT3 && csr <= CSR_MHPMEVENT31);
  }
}

std::string disasm(uint64_t insn) {
  return g_proc->get_disassembler()->disassemble(insn_t(insn));
}

}  // namespace

extern "C" {
void cosim_vabits(int n) { g_vabits = n; }   // 在 cosim_init 之前调用
void cosim_trap_tgt(int t) { g_trap_tgt = t; } // 紧接着的 cosim_trap 比较陷入目标级

// ---------------------------------------------------------------- 初始化
int cosim_init(const char *elf, const char *isa, long long dram_size, int verbose, const char *trace) {
  g_dram_size = (uint64_t)dram_size;
  g_verbose = verbose;
  // 可以再调一次 (tb_top 的 +midrst 运行中复位)：丢掉上一个 Spike，清 tohost / 提交计数
  if (g_proc) { delete g_proc; g_proc = nullptr; }
  if (g_trace) { fclose(g_trace); g_trace = nullptr; }
  g_tohost_hit = false; g_tohost_val = 0; g_ncommit = 0;
  // DUT 一侧的存储也从 ELF 映像重建：否则上一段运行改过的、ELF 段以外的区域 (随机程序的数据区等) 与新 Spike 不一致
  g_dut = SparseMem();
  g_sif = std::make_unique<SpikeIf>();
  g_sif->cfg.isa = isa;
  g_sif->cfg.priv = "MSU";
  g_sif->cfg.pmpregions = 8;
  g_sif->cfg.pmpgranularity = 64;    // DUT 的 PMP 粒度 64 B (o6_pkg::PMP_G = 4，#34)
  g_sif->cfg.hartids = {0};
  g_sif->cfg.trigger_count = 0;      // DUT 不实现调试触发器
  g_sif->cfg.wfi_as_nop = true;      // DUT 的 WFI 按空操作实现 (规范允许)；Spike 默认进 in_wfi 等中断，下一步不前进会误报陷入
  g_dut.write(BOOTROM_BASE, (const uint8_t *)BOOTROM, sizeof(BOOTROM));
  g_sif->mem.write(BOOTROM_BASE, (const uint8_t *)BOOTROM, sizeof(BOOTROM));
  if (!load_elf(elf)) return 1;
  // enable_log_commits() 是取写回值必需的，但 Spike 同时把每条提交打印到日志文件：
  // 非 verbose 时指向 /dev/null (长程序的 sim.log 否则几百 MB，也拖慢仿真)
  FILE *slog = verbose ? stderr : fopen("/dev/null", "w");
  g_proc = new processor_t(isa, "MSU", &g_sif->cfg, g_sif.get(), 0, false, slog ? slog : stderr, g_sout);
  g_sif->harts[0] = g_proc;
  g_proc->enable_log_commits();
  g_proc->set_max_vaddr_bits(g_vabits); // DUT 实现 Sv39 / Sv48 / Sv57 (Spike 默认无 MMU，SUM/MXR/TVM 不可写)
  g_proc->reset();
  g_misa0 = g_proc->get_state()->misa->read();
  // Spike 复位时把 PMP 第 0 项配成全放开 (便利行为)；DUT 按规范复位为 0 (A/L 字段必须为 0)：对齐
  if (g_proc->n_pmp > 0) {
    g_proc->put_csr(CSR_PMPCFG0, 0);
    g_proc->put_csr(CSR_PMPADDR0, 0);
  }
  if (trace && trace[0]) g_trace = fopen(trace, "w");
  fprintf(stderr, "[cosim] ELF=%s isa=%s tohost=0x%" PRIx64 "\n", elf, isa, g_tohost);
  return 0;
}

// ---------------------------------------------------------------- DUT 存储
long long tb_mem_read64(long long addr) { return (long long)g_dut.read64((uint64_t)addr & ~7ull); }

void tb_mem_write64(long long addr, long long data, int mask) {
  uint64_t a = (uint64_t)addr & ~7ull;
  for (int i = 0; i < 8; i++)
    if (mask & (1 << i)) *g_dut.ptr(a + i, true) = (uint8_t)((uint64_t)data >> (8 * i));
  if (g_tohost && a == (g_tohost & ~7ull)) {
    uint64_t v = g_dut.read64(g_tohost);
    if (v != 0) { g_tohost_hit = true; g_tohost_val = v; }
  }
}

int tb_mem_valid(long long addr) { return in_mem((uint64_t)addr) ? 1 : 0; }

int tb_tohost(long long *v) { *v = (long long)g_tohost_val; return g_tohost_hit ? 1 : 0; }

long long tb_tohost_addr() { return (long long)g_tohost; }

// ---------------------------------------------------------------- 对拍
// rd_type: 0 = 无, 1 = 整数, 2 = 浮点
int cosim_commit(long long pc, int insn, int rd_type, int rd, long long wdata) {
  state_t *s = g_proc->get_state();
  uint64_t spc = s->pc;
  if ((uint64_t)pc != spc) {
    fprintf(stderr, "[cosim] 第 %" PRIu64 " 条: PC 不一致 DUT=0x%016llx Spike=0x%016" PRIx64 "\n",
            g_ncommit, (unsigned long long)pc, spc);
    return 1;
  }
  // 指令编码：在执行前按当时的地址翻译取 (执行后页表 / satp 可能已变，再取会缺页)
  uint64_t sinsn = 0;
  bool fetch_ok = true;
  try {
    insn_fetch_t f = g_proc->get_mmu()->load_insn(spc);
    sinsn = f.insn.bits();
  } catch (...) {
    fetch_ok = false;
  }
  uint64_t minstret0 = s->minstret->read();
  uint64_t mcinh0    = s->mcountinhibit->read();     // 执行前的值 (Spike 按它决定本条是否计数)
  // Sstc：Spike 的 time 影子值不走 (没接 CLINT)，按 0 >= stimecmp 自己算 STIP 会抢先进中断；
  // STIP 中断与 MTIP 一样由 DUT 决定 (cosim_trap 临时置位)，这里每步之前清掉
  if (s->menvcfg->read() & MENVCFG_STCE) s->mip->backdoor_write_with_mask(MIP_STIP, 0);
  // H：vstimecmp 驱动的 VSTIP 同理 (hvip.VSTIP 是另存的，不受影响)
  if (g_proc->extension_enabled('H') && (s->henvcfg->read() & HENVCFG_STCE)) s->mip->backdoor_write_with_mask(MIP_VSTIP, 0);
  s->log_reg_write.clear();
  s->log_mem_read.clear();
  s->log_mem_write.clear();
  g_proc->step(1);
  g_ncommit++;
  // DUT 的 misa 只读 (规范允许)：Spike 允许关 C/M/A/F/D，这里写回原值，相当于写入被忽略
  if (s->misa->read() != g_misa0) s->misa->write(g_misa0);

  // Spike 是否也正常退休了 (没有陷入)
  if (s->minstret->read() == minstret0 && !(mcinh0 & 4)) {
    fprintf(stderr, "[cosim] 第 %" PRIu64 " 条 pc=0x%016llx: DUT 正常提交，Spike 发生陷入 mcause=0x%" PRIx64
            " scause=0x%" PRIx64 "\n", g_ncommit, (unsigned long long)pc,
            s->mcause->read(), s->scause->read());
    return 1;
  }

  if (!fetch_ok) {
    fprintf(stderr, "[cosim] 第 %" PRIu64 " 条 pc=0x%016llx: Spike 取指失败但 DUT 正常提交\n",
            g_ncommit, (unsigned long long)pc);
    return 1;
  }
  uint32_t dinsn = (uint32_t)insn;
  if ((sinsn & 3) != 3) sinsn &= 0xffff;
  if ((uint32_t)sinsn != dinsn) {
    fprintf(stderr, "[cosim] 第 %" PRIu64 " 条 pc=0x%016llx: 指令编码不一致 DUT=%08x Spike=%08" PRIx64 "\n",
            g_ncommit, (unsigned long long)pc, dinsn, sinsn);
    return 1;
  }

  // 是否是不确定结果
  bool nondet = false;
  for (auto &m : s->log_mem_read)
    if (in_dev(std::get<0>(m)) || !in_mem(std::get<0>(m))) nondet = true;
  if ((sinsn & 0x7f) == 0x73 && ((sinsn >> 12) & 7) != 0 && nondet_csr((uint32_t)(sinsn >> 20))) nondet = true;

  // 目的寄存器
  int s_type = 0, s_rd = 0;
  uint64_t s_val = 0;
  for (auto &w : s->log_reg_write) {
    int t = w.first & 0xf, r = (int)(w.first >> 4);
    if (t == 0 && r != 0) { s_type = 1; s_rd = r; s_val = w.second.v[0]; }
    if (t == 1)            { s_type = 2; s_rd = r; s_val = w.second.v[0]; }
  }

  if (g_trace)
    fprintf(g_trace, "%" PRIu64 " 0x%016llx (0x%08x) %s%s rd%c%d=0x%016llx\n", g_ncommit,
            (unsigned long long)pc, dinsn, disasm(sinsn).c_str(), nondet ? " [nondet]" : "",
            rd_type == 2 ? 'f' : 'x', rd, (unsigned long long)wdata);

  if (nondet && rd_type == 1 && s_type == 1) {
    s->XPR.write(s_rd, (reg_t)wdata);
    return 0;
  }
  if (rd_type != s_type || (rd_type != 0 && (rd != s_rd || (uint64_t)wdata != s_val))) {
    fprintf(stderr, "[cosim] 第 %" PRIu64 " 条 pc=0x%016llx %s: 写回不一致\n"
                    "        DUT  : type=%d rd=%d val=0x%016llx\n"
                    "        Spike: type=%d rd=%d val=0x%016" PRIx64 "\n",
            g_ncommit, (unsigned long long)pc, disasm(sinsn).c_str(),
            rd_type, rd, (unsigned long long)wdata, s_type, s_rd, s_val);
    return 1;
  }
  return 0;
}

// 陷入：is_irq=1 时先在 Spike 的 mip 上临时置对应位
int cosim_trap(int is_irq, int cause, long long epc, long long tval) {
  state_t *s = g_proc->get_state();
  if ((uint64_t)epc != s->pc) {
    fprintf(stderr, "[cosim] 陷入 cause=%d: epc 不一致 DUT=0x%016llx Spike pc=0x%016" PRIx64 "\n",
            cause, (unsigned long long)epc, s->pc);
    return 1;
  }
  reg_t ext = 0;
  if (is_irq) {
    if (cause == IRQ_M_TIMER) ext = MIP_MTIP;
    if (cause == IRQ_M_SOFT)  ext = MIP_MSIP;
    if (cause == IRQ_M_EXT)   ext = MIP_MEIP;
    if (cause == IRQ_S_EXT)   ext = MIP_SEIP;
    if (cause == IRQ_S_TIMER && (s->menvcfg->read() & MENVCFG_STCE)) ext = MIP_STIP;   // Sstc：STIP 来自 stimecmp
    if (cause == IRQ_LCOF)    ext = MIP_LCOFIP;   // Sscofpmf：DUT 计数溢出置的 LCOFIP
    if (cause == IRQ_VS_TIMER && g_proc->extension_enabled('H') && (s->henvcfg->read() & HENVCFG_STCE))
      ext = MIP_VSTIP;                            // H：vstimecmp 驱动的 VSTIP
    if (ext) s->mip->backdoor_write_with_mask(ext, ext);
  }
  reg_t prv0 = s->prv;
  s->log_reg_write.clear();
  g_proc->step(1);
  if (ext) s->mip->backdoor_write_with_mask(ext, 0);
  // 陷入去了哪一级 (V = 1 时 scause / sepc / stval 读到的是 vs* 那份)
  bool to_s = (s->prv == PRV_S) && (prv0 != PRV_M);
  int  tgt  = !to_s ? 0 : s->v ? 2 : 1;
  int  dtgt = g_trap_tgt;
  g_trap_tgt = -1;
  reg_t c = to_s ? s->scause->read() : s->mcause->read();
  reg_t e = to_s ? s->sepc->read() : s->mepc->read();
  reg_t t = to_s ? s->stval->read() : s->mtval->read();
  // 送 VS 的中断：vscause 是转换后的 (VSSI 2 -> 1、VSTI 6 -> 5、VSEI 10 -> 9)，DUT 报的是原编号
  int  ccause = (tgt == 2 && is_irq && cause <= 10) ? cause - 1 : cause;
  reg_t want_c = ((reg_t)is_irq << 63) | (reg_t)ccause;
  if (dtgt >= 0 && dtgt != tgt) {
    fprintf(stderr, "[cosim] 陷入目标不一致 @pc=0x%016llx cause=%d irq=%d：DUT %s，Spike %s\n",
            (unsigned long long)epc, cause, is_irq, dtgt == 0 ? "M" : dtgt == 1 ? "HS" : "VS",
            tgt == 0 ? "M" : tgt == 1 ? "HS" : "VS");
    return 1;
  }
  if (c != want_c || e != (uint64_t)epc || (!is_irq && t != (uint64_t)tval)) {
    fprintf(stderr, "[cosim] 陷入不一致 @pc=0x%016llx\n"
                    "        DUT  : irq=%d cause=%d tval=0x%016llx\n"
                    "        Spike: %ccause=0x%" PRIx64 " %cepc=0x%" PRIx64 " %ctval=0x%" PRIx64 "\n",
            (unsigned long long)epc, is_irq, cause, (unsigned long long)tval,
            to_s ? 's' : 'm', c, to_s ? 's' : 'm', e, to_s ? 's' : 'm', t);
    return 1;
  }
  if (g_trace)
    fprintf(g_trace, "-- trap irq=%d cause=%d epc=0x%016llx tval=0x%016llx -> pc=0x%016" PRIx64 "\n",
            is_irq, cause, (unsigned long long)epc, (unsigned long long)tval, s->pc);
  return 0;
}

long long cosim_ncommit() { return (long long)g_ncommit; }

// 复位后不确定的架构寄存器 (RISC-V 规定复位后 x1..x31 / f 寄存器的值不确定)：以 DUT 为准写进 Spike
//   (tb_top 的 +midrst：DUT 的 RAT 复位成恒等映射，物理寄存器堆保留复位前的值；Spike 复位清零)
void cosim_set_reg(int fp, int r, long long v) {
  if (!g_proc) return;
  auto *s = g_proc->get_state();
  if (!fp) { if (r != 0) s->XPR.write(r, (reg_t)v); }
  else { freg_t f; f.v[0] = (uint64_t)v; f.v[1] = ~0ull; s->FPR.write(r, f); }
}

// 向量状态 (对拍向量单元用)：寄存器 r 的第 w 个双字；向量 CSR (0 vl 1 vtype 2 vstart 3 vxsat 4 vxrm)
long long cosim_vreg(int r, int w) {
  auto &VU = g_proc->VU;
  const uint64_t *p = (const uint64_t *)VU.reg_file;
  return (long long)p[(size_t)r * (VU.VLEN / 64) + (size_t)w];
}
long long cosim_vcsr(int which) {
  auto &VU = g_proc->VU;
  switch (which) {
  case 0: return (long long)VU.vl->read();
  case 1: return (long long)VU.vtype->read();
  case 2: return (long long)VU.vstart->read();
  case 3: return (long long)g_proc->get_state()->vxsat->read();
  case 5: return (long long)g_proc->get_state()->fflags->read();
  default: return (long long)VU.vxrm->read();
  }
}

void cosim_finish() {
  if (g_trace) fclose(g_trace);
  g_trace = nullptr;
}

}  // extern "C"
