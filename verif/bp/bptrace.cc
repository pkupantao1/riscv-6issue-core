// bptrace.cc - 从 Spike `-l` 的逐指令日志 (标准输入) 抽出控制流记录，写成二进制 trace
//   spike -l --isa=... prog.elf 2>&1 | bptrace out.bpt
// 每条记录 (bp_rec_t)：
//   pc / tgt   指令地址与实际下一 PC
//   ninst      距上一条记录的提交指令数 (含本条；陷入的指令不算提交)
//   kind       0 条件分支  1 直接跳转  2 直接调用  3 返回  4 间接跳转  5 间接调用  6 不可预测重定向 (陷入 / xRET)
//   taken      实际是否跳 (条件分支以外恒 1)
//   ilen       指令长度 (2 / 4)
// 被测程序打印到 UART 的字符会混在行首 (Spike 的 stdout 与日志 stderr 交织)，按 "core" 定位。
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include "bptrace.h"

static bool is_link(unsigned r) { return r == 1 || r == 5; }

// 按编码分类：返回 kind (0..5)，非控制流返回 -1
static int classify(uint32_t insn, int &ilen) {
  if ((insn & 3) != 3) {                     // RVC
    ilen = 2;
    uint32_t op = insn & 3, f3 = (insn >> 13) & 7;
    if (op == 1 && f3 == 5) return 1;                         // c.j
    if (op == 1 && (f3 == 6 || f3 == 7)) return 0;            // c.beqz / c.bnez
    if (op == 2 && f3 == 4 && ((insn >> 2) & 0x1f) == 0) {    // c.jr / c.jalr / c.ebreak
      unsigned rs1 = (insn >> 7) & 0x1f;
      bool link = (insn >> 12) & 1;
      if (rs1 == 0) return -1;                                // c.ebreak (link=1) / 保留
      if (!link) return is_link(rs1) ? 3 : 4;                 // c.jr ra = ret
      return 5;                                               // c.jalr：rd = ra，按调用
    }
    return -1;
  }
  ilen = 4;
  uint32_t opc = insn & 0x7f;
  unsigned rd = (insn >> 7) & 0x1f, rs1 = (insn >> 15) & 0x1f;
  if (opc == 0x63) return 0;
  if (opc == 0x6f) return is_link(rd) ? 2 : 1;
  if (opc == 0x67) {
    if (is_link(rd)) return 5;                                // 调用 (含 rd=ra 且 rs1=ra 的协程切换，按调用)
    if (rd == 0 && is_link(rs1)) return 3;
    return 4;
  }
  return -1;
}

static bool is_xret(uint32_t insn) {
  return insn == 0x30200073 || insn == 0x10200073 || insn == 0x7b200073;   // mret / sret / dret
}

int main(int argc, char **argv) {
  if (argc < 2) { fprintf(stderr, "usage: bptrace out.bpt [max_inst]\n"); return 1; }
  FILE *o = fopen(argv[1], "wb");
  if (!o) { perror(argv[1]); return 1; }
  uint64_t max_inst = argc > 2 ? strtoull(argv[2], nullptr, 0) : 0;
  static char line[4096];
  bool have = false;            // 有一条待定的上一指令
  uint64_t ppc = 0; uint32_t pinsn = 0; bool ptrap = false;
  uint64_t since = 0, total = 0, nrec = 0, ntrap = 0;
  bp_hdr_t h = {}; memcpy(h.magic, "BPT1", 4);
  fwrite(&h, sizeof h, 1, o);
  while (fgets(line, sizeof line, stdin)) {
    char *c = strstr(line, "core");
    if (!c) continue;
    char *p = strchr(c, ':');
    if (!p) continue;
    p++;
    while (*p == ' ') p++;
    if (!strncmp(p, "exception", 9) || !strncmp(p, "interrupt", 9) || !strncmp(p, "trap", 4)) {
      if (have) ptrap = true;          // 上一条陷入 (interrupt 时上一条其实已提交，这里一律当重定向)
      continue;
    }
    if (strncmp(p, "0x", 2)) continue;
    uint64_t pc = strtoull(p, &p, 16);
    char *q = strchr(p, '(');
    if (!q) continue;
    uint32_t insn = (uint32_t)strtoul(q + 1, nullptr, 16);
    if (have) {
      int ilen = 4;
      int k = classify(pinsn, ilen);
      if (!ptrap) { since++; total++; }
      bp_rec_t r = {};
      r.pc = ppc; r.tgt = pc; r.ilen = (uint8_t)ilen;
      bool emit = false;
      if (ptrap || is_xret(pinsn)) {
        r.kind = BP_REDIR; r.taken = 1; emit = true; ntrap++;
      } else if (k >= 0) {
        r.kind = (uint8_t)k;
        r.taken = (k == BP_COND) ? (pc != ppc + (uint64_t)ilen) : 1;
        emit = true;
      } else if (pc != ppc + (uint64_t)ilen) {
        r.kind = BP_REDIR; r.taken = 1; emit = true; ntrap++;   // 其它不连续 (fence.i 后的重取等)
      }
      if (emit) {
        r.ninst = (uint32_t)since; since = 0;
        fwrite(&r, sizeof r, 1, o); nrec++;
      }
      if (max_inst && total >= max_inst) break;
    }
    have = true; ppc = pc; pinsn = insn; ptrap = false;
  }
  h.ninst = total; h.nrec = nrec;
  fseek(o, 0, SEEK_SET);
  fwrite(&h, sizeof h, 1, o);
  fclose(o);
  fprintf(stderr, "bptrace: %llu 条指令, %llu 条记录, %llu 次重定向 -> %s\n",
          (unsigned long long)total, (unsigned long long)nrec, (unsigned long long)ntrap, argv[1]);
  return 0;
}
