// bptrace.h - 分支 trace 格式 (bptrace 写，bpsim 读)
#pragma once
#include <cstdint>

enum { BP_COND = 0, BP_JMP = 1, BP_CALL = 2, BP_RET = 3, BP_IND = 4, BP_ICALL = 5, BP_REDIR = 6 };

#pragma pack(push, 1)
struct bp_hdr_t {
  char     magic[4];     // "BPT1"
  uint32_t rsv;
  uint64_t ninst;        // 总提交指令数
  uint64_t nrec;
};
struct bp_rec_t {
  uint64_t pc;
  uint64_t tgt;
  uint32_t ninst;        // 距上一条记录的提交指令数 (含本条)
  uint8_t  kind;
  uint8_t  taken;
  uint8_t  ilen;
  uint8_t  pad;
};
#pragma pack(pop)
