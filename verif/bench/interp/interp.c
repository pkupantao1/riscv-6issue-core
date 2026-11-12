// interp.c - 间接跳转基准：小栈机字节码解释器 (switch 编成跳转表，每条字节码一次 JALR)
//   同一条 JALR 的目标随字节码序列变化：只记"上次目标"的预测器几乎每次都错，
//   按路径历史区分的 ITTAGE 能学会 (字节码序列在循环里重复)。
//   用 riscv-tests 基准环境 (util.h 的 setStats 统计测量区的 mcycle / minstret)。
#include <stdint.h>
#include "util.h"

enum { OP_PUSH, OP_ADD, OP_SUB, OP_MUL, OP_DUP, OP_SWAP, OP_DROP, OP_LD, OP_ST, OP_DEC, OP_JNZ, OP_XOR,
       OP_SHL, OP_AND, OP_INC, OP_HALT };

// 程序：acc = 0; for (i = N; i; i--) { t = i * 3 + acc; acc = (t ^ (t << 2)) & 0xffff; }
#define N 2000
static const int32_t prog[] = {
  OP_PUSH, 0, OP_ST, 1,             // acc = 0
  OP_PUSH, N, OP_ST, 0,             // i = N
  /* 8: 循环 */
  OP_LD, 0, OP_PUSH, 3, OP_MUL,     // i * 3
  OP_LD, 1, OP_ADD,                 // + acc
  OP_DUP, OP_PUSH, 2, OP_SHL,       // t, t << 2
  OP_XOR, OP_PUSH, 0xffff, OP_AND,  // (t ^ t<<2) & 0xffff
  OP_ST, 1,
  OP_LD, 0, OP_DEC, OP_DUP, OP_ST, 0,
  OP_JNZ, 8,
  OP_LD, 1, OP_HALT
};

static int64_t run(const int32_t *p)
{
  int64_t st[16], var[4];
  int sp = 0, pc = 0;
  for (;;) {
    int32_t op = p[pc++];
    switch (op) {
      case OP_PUSH: st[sp++] = p[pc++]; break;
      case OP_ADD:  sp--; st[sp-1] += st[sp]; break;
      case OP_SUB:  sp--; st[sp-1] -= st[sp]; break;
      case OP_MUL:  sp--; st[sp-1] *= st[sp]; break;
      case OP_DUP:  st[sp] = st[sp-1]; sp++; break;
      case OP_SWAP: { int64_t t = st[sp-1]; st[sp-1] = st[sp-2]; st[sp-2] = t; } break;
      case OP_DROP: sp--; break;
      case OP_LD:   st[sp++] = var[p[pc++]]; break;
      case OP_ST:   var[p[pc++]] = st[--sp]; break;
      case OP_DEC:  st[sp-1]--; break;
      case OP_INC:  st[sp-1]++; break;
      case OP_JNZ:  { int32_t t = p[pc++]; if (st[--sp]) pc = t; } break;
      case OP_XOR:  sp--; st[sp-1] ^= st[sp]; break;
      case OP_SHL:  sp--; st[sp-1] <<= st[sp]; break;
      case OP_AND:  sp--; st[sp-1] &= st[sp]; break;
      case OP_HALT: return st[sp-1];
      default:      return -1;
    }
  }
}

static int64_t ref(void)
{
  int64_t acc = 0;
  for (int64_t i = N; i; i--) { int64_t t = i * 3 + acc; acc = (t ^ (t << 2)) & 0xffff; }
  return acc;
}

int main(void)
{
  int64_t r;
  setStats(1);
  r = run(prog);
  setStats(0);
  return r == ref() ? 0 : 1;
}
