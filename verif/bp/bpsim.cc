// bpsim.cc - 按取指块的分支预测 trace 模拟器 (OOO6 前端分支预测设计空间探索)
//   bpsim -p <预设> [-v] trace1.bpt [trace2.bpt ...]
//   预设：cur (OOO6 现有 RTL)、xs (按香山昆明湖公开文档参数近似的参考)、v2* (OOO6 新设计候选)
//
// 建模：
//   取指块：从起点 PC 起，到块边界 (对齐 32 B，或从起点算 32 B) 为止；遇到第一条"实际跳转"的控制流、
//   或发生误预测的分支即结束；可限制每块分支数 (香山 FTB 每项 2 个分支槽)。
//   同一块内所有分支用块起点那一刻的历史 (硬件一拍并行预测)，表查找可按块起点或分支自身 PC 索引。
//   训练：分支解析后立即更新 (对所有预设相同，略乐观)。RAS 假定推测恢复完美，只在溢出时出错。
//   统计：条件分支方向误预测、间接跳转目标误预测、返回误预测，按每千条指令 (MPKI)。
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <map>
#include "bptrace.h"

typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;

static u32 rng_state = 0x12345678;
static u32 rnd() { rng_state ^= rng_state << 13; rng_state ^= rng_state >> 17; rng_state ^= rng_state << 5; return rng_state; }

//==============================================================================
// 全局历史 (环形位缓冲) 与折叠历史
//==============================================================================
struct Ghist {
  std::vector<uint8_t> b;
  u32 ptr = 0, mask;
  explicit Ghist(int lg = 12) { b.assign(1u << lg, 0); mask = (1u << lg) - 1; }
  int operator[](u32 i) const { return b[(ptr + i) & mask]; }   // i 拍之前进入的位 (0 = 最新)
  void push(int bit) { ptr = (ptr - 1) & mask; b[ptr] = (uint8_t)bit; }
};

struct Folded {
  u32 comp = 0;
  int clen = 1, olen = 0, outp = 0;
  void init(int original, int compressed) { comp = 0; olen = original; clen = compressed; outp = olen % clen; }
  // 在 h.push 之后调用
  void update(const Ghist &h) {
    comp = (comp << 1) ^ (u32)h[0];
    comp ^= (u32)h[olen] << outp;
    comp ^= comp >> clen;
    comp &= (1u << clen) - 1;
  }
};

//==============================================================================
// 历史管理：三种更新方式
//   HM_BLOCK  每块 1 位 (块是否以跳转结束) —— OOO6 现有 RTL
//   HM_DIR    每条条件分支 1 位方向，跳转的无条件分支也推 1 位；每块最多 cap 位 (超出时保留最后 cap 个)
//   HM_PATH   只在跳转时推 S 位：散列 (分支 PC, 目标)；不跳的条件分支不推 (由块结构隐含)
//==============================================================================
enum { HM_BLOCK = 0, HM_DIR = 1, HM_PATH = 2 };

struct HistCfg { int mode = HM_DIR; int cap = 64; int pbits = 2; bool rtl = false; bool tonly = false; };   // tonly：只散列目标地址   // rtl：散列同 RTL bp_phash

struct History {
  HistCfg cfg;
  Ghist g{12};
  u64 path = 0;                       // 路径历史 (跳转分支 PC 低位)
  std::vector<Folded *> folds;
  std::vector<int> pend;              // 本块待推入的位
  void reg(Folded *f) { folds.push_back(f); }
  void push_bit(int b) { g.push(b); for (auto *f : folds) f->update(g); }
  // 块内每个已解析的控制流 (按程序序) 通知一次
  void on_cf(int kind, bool taken, u64 pc, u64 tgt) {
    if (cfg.mode == HM_DIR) {
      if (kind == BP_COND) pend.push_back(taken);
      else if (taken && kind != BP_REDIR) pend.push_back(1);
    } else if (cfg.mode == HM_PATH) {
      if (taken && kind != BP_REDIR) {
        u64 h = (pc >> 1) ^ (pc >> 7) ^ (tgt >> 1) ^ (tgt >> 5);
        if (cfg.rtl) {     // o6_pkg::bp_phash：{s1^s3^t2^t6, s0^s2^t1^t5}，s = 块内槽号
          u64 sl = (pc >> 1) & 15;
          h = (((sl >> 0) ^ (sl >> 2) ^ (tgt >> 1) ^ (tgt >> 5)) & 1) | ((((sl >> 1) ^ (sl >> 3) ^ (tgt >> 2) ^ (tgt >> 6)) & 1) << 1);
          if (cfg.tonly) h = (((tgt >> 1) ^ (tgt >> 5) ^ (tgt >> 9)) & 1) | ((((tgt >> 2) ^ (tgt >> 6) ^ (tgt >> 10)) & 1) << 1);
        }
        for (int i = 0; i < cfg.pbits; i++) pend.push_back((h >> i) & 1);
      }
    }
    if (taken && kind != BP_REDIR) path = (path << 1) ^ ((pc >> 1) & 1) ^ ((pc >> 3) & 2);
  }
  void end_block(bool ended_taken) {
    if (cfg.mode == HM_BLOCK) { push_bit(ended_taken); return; }
    int n = (int)pend.size(), s = std::max(0, n - cfg.cap);
    for (int i = s; i < n; i++) push_bit(pend[i]);
    pend.clear();
  }
};

// 块上下文：块起点与历史快照 (块内各分支预测都用它)
struct BlkCtx {
  u64 start = 0;
  std::vector<u32> fsnap;     // 各折叠历史快照
  u64 path = 0;
  void snap(const History &h) {
    fsnap.resize(h.folds.size());
    for (size_t i = 0; i < h.folds.size(); i++) fsnap[i] = h.folds[i]->comp;
    path = h.path;
  }
};

//==============================================================================
// 方向预测器接口
//==============================================================================
struct DirPred {
  virtual ~DirPred() {}
  virtual void attach(History &h) = 0;
  // pc：分支地址；slot：块内半字号；返回预测方向，内部记下训练所需信息
  virtual bool predict(const BlkCtx &c, u64 pc, int slot) = 0;
  virtual void update(const BlkCtx &c, u64 pc, int slot, bool taken) = 0;
  virtual double kbits() const = 0;
  virtual std::string desc() const = 0;
};

static inline int sat_inc(int v, int mx) { return v < mx ? v + 1 : v; }
static inline int sat_dec(int v, int mn) { return v > mn ? v - 1 : v; }

//------------------------------------------------------------------------------
// 双模态 (2 位，带每块行模式时按块起点 + 槽号索引)
//------------------------------------------------------------------------------
struct Bimodal {
  int lg; std::vector<int8_t> c; bool by_block;
  Bimodal(int lg_, bool blk) : lg(lg_), c(1u << lg_, 1), by_block(blk) {}
  u32 idx(u64 blk, u64 pc, int slot) const {
    if (by_block) { u64 b = blk >> 5; return (u32)((((b ^ (b >> (lg - 4))) << 4) | slot) & ((1u << lg) - 1)); }
    u64 p = pc >> 1; return (u32)((p ^ (p >> lg)) & ((1u << lg) - 1));
  }
  bool pred(u64 blk, u64 pc, int slot) const { return c[idx(blk, pc, slot)] >= 2; }
  void upd(u64 blk, u64 pc, int slot, bool t) { int8_t &v = c[idx(blk, pc, slot)]; v = t ? sat_inc(v, 3) : sat_dec(v, 0); }
  double kbits() const { return (2.0 * (1u << lg)) / 1024; }
};

//------------------------------------------------------------------------------
// TAGE：nt 张带标签表，每行 nway 路 (路用"标签 + 槽号"区分，替代时选 u 最小者)
//   索引基址 = 块起点 (硬件一拍查一行) 或分支 PC (理想)
//------------------------------------------------------------------------------
struct TageCfg {
  int nt = 4;
  std::vector<int> hl;        // 各表历史长度 (位)
  std::vector<int> lgsz;      // 各表行数 log2
  std::vector<int> tagw;      // 各表标签宽度
  int nway = 1;
  int ctrw = 3;
  int base_lg = 12;           // 双模态项数 log2
  bool idx_by_block = true;   // 按块起点索引
  bool slot_in_tag = true;    // 槽号参与匹配 (块索引时必须)
  int u_reset_lg = 18;        // 每 2^n 次更新老化一次 u
  bool use_alt_na = true;
  bool rtl_hash = false;      // 索引 / 标签散列同 RTL o6_tage (按 32 B 块地址)
  int max_alloc = 1;
};

struct Tage : DirPred {
  struct Ent { u32 tag = 0; int8_t ctr = 0; uint8_t u = 0; int8_t slot = -1; bool v = false; };
  TageCfg C;
  std::unique_ptr<Bimodal> base;
  std::vector<std::vector<Ent>> t;       // [表][行*nway + 路]
  std::vector<Folded> fi, ft1, ft2;      // 每表：索引折叠、标签折叠两份
  int fbase = 0;                          // 本预测器折叠历史在 History.folds 里的起点
  int fwd_w = 0;                          // 旧值训练时的写旁路深度 (最近 fwd_w 次训练写过的行改用当前值)
  std::vector<std::vector<u64>> lastw;    // [表][行] 最近一次被训练写的序号
  int alt_na = 0;                         // use_alt_on_na 计数
  u64 nupd = 0;
  // 最近一次 predict 的结果
  int prov = -1, alt = -1, pway = -1, away = -1;
  bool tage_pred = false, alt_pred = false, prov_weak = false;
  std::vector<u32> cidx, ctag;
  struct St { int prov, alt, pway, away; bool tp, ap, pw; std::vector<u32> ci, ct; };
  St save() const { return St{prov, alt, pway, away, tage_pred, alt_pred, prov_weak, cidx, ctag}; }
  void load(const St &x) { prov = x.prov; alt = x.alt; pway = x.pway; away = x.away; tage_pred = x.tp;
                           alt_pred = x.ap; prov_weak = x.pw; cidx = x.ci; ctag = x.ct; }
  int ctrmax, ctrmin;

  explicit Tage(const TageCfg &c) : C(c) {
    base.reset(new Bimodal(C.base_lg, C.idx_by_block));
    t.resize(C.nt);
    for (int i = 0; i < C.nt; i++) t[i].assign((size_t)(1u << C.lgsz[i]) * C.nway, Ent());
    lastw.resize(C.nt);
    for (int i = 0; i < C.nt; i++) lastw[i].assign((size_t)1u << C.lgsz[i], ~0ull >> 1);
    fi.resize(C.nt); ft1.resize(C.nt); ft2.resize(C.nt);
    cidx.resize(C.nt); ctag.resize(C.nt);
    ctrmax = (1 << (C.ctrw - 1)) - 1; ctrmin = -(1 << (C.ctrw - 1));
  }
  void attach(History &h) override {
    fbase = (int)h.folds.size();
    for (int i = 0; i < C.nt; i++) {
      fi[i].init(C.hl[i], C.lgsz[i]); ft1[i].init(C.hl[i], C.tagw[i]); ft2[i].init(C.hl[i], C.tagw[i] - 1);
      h.reg(&fi[i]); h.reg(&ft1[i]); h.reg(&ft2[i]);
    }
  }
  u32 gidx(const BlkCtx &c, u64 pc, int i) const {
    if (C.rtl_hash) {
      u64 a5 = c.start >> 5;
      return (u32)(((a5 ^ (a5 >> 10) ^ (a5 >> (11 + i))) ^ c.fsnap[fbase + 3 * i]) & ((1u << C.lgsz[i]) - 1));
    }
    u64 a = (C.idx_by_block ? c.start : pc) >> 1;
    int lg = C.lgsz[i];
    u64 p = c.path & ((1ull << std::min(C.hl[i], 16)) - 1);
    u64 v = a ^ (a >> lg) ^ (a >> (lg + 1 + i)) ^ c.fsnap[fbase + 3 * i] ^ (p ^ (p >> lg));
    return (u32)(v & ((1u << lg) - 1));
  }
  u32 gtag(const BlkCtx &c, u64 pc, int i) const {
    if (C.rtl_hash) {
      u64 a5 = c.start >> 5;
      return (u32)((a5 ^ (a5 >> 11) ^ c.fsnap[fbase + 3 * i + 1] ^ ((u64)c.fsnap[fbase + 3 * i + 2] << 1)) & ((1u << C.tagw[i]) - 1));
    }
    u64 a = (C.idx_by_block ? c.start : pc) >> 1;
    u64 v = a ^ (a >> 11) ^ c.fsnap[fbase + 3 * i + 1] ^ ((u64)c.fsnap[fbase + 3 * i + 2] << 1);
    return (u32)(v & ((1u << C.tagw[i]) - 1));
  }
  int find(int i, u32 idx, u32 tag, int slot) const {
    const Ent *row = &t[i][(size_t)idx * C.nway];
    for (int w = 0; w < C.nway; w++)
      if (row[w].v && row[w].tag == tag && (!C.slot_in_tag || row[w].slot == slot)) return w;
    return -1;
  }
  bool predict(const BlkCtx &c, u64 pc, int slot) override {
    prov = alt = pway = away = -1;
    for (int i = 0; i < C.nt; i++) { cidx[i] = gidx(c, pc, i); ctag[i] = gtag(c, pc, i); }
    for (int i = C.nt - 1; i >= 0; i--) {
      int w = find(i, cidx[i], ctag[i], slot);
      if (w < 0) continue;
      if (prov < 0) { prov = i; pway = w; } else { alt = i; away = w; break; }
    }
    bool bp = base->pred(c.start, pc, slot);
    alt_pred = alt >= 0 ? (t[alt][(size_t)cidx[alt] * C.nway + away].ctr >= 0) : bp;
    if (prov < 0) { tage_pred = alt_pred = bp; prov_weak = false; return tage_pred; }
    const Ent &e = t[prov][(size_t)cidx[prov] * C.nway + pway];
    prov_weak = (e.ctr == 0 || e.ctr == -1) && e.u == 0;
    bool pp = e.ctr >= 0;
    tage_pred = (C.use_alt_na && prov_weak && alt_na >= 0) ? alt_pred : pp;
    return tage_pred;
  }
  // 置信度：提供者计数器的强弱 (给 SC 用)
  int conf() const {
    if (prov < 0) return 1;
    int v = t[prov][(size_t)cidx[prov] * C.nway + pway].ctr;
    return std::abs(2 * v + 1);
  }
  // 预测时读到的各表行 (2 路) —— RTL 训练用的就是这些旧值
  std::vector<Ent> rows() const {
    std::vector<Ent> r((size_t)C.nt * C.nway);
    for (int i = 0; i < C.nt; i++)
      for (int w = 0; w < C.nway; w++) r[(size_t)i * C.nway + w] = t[i][(size_t)cidx[i] * C.nway + w];
    return r;
  }
  // 按 RTL (o6_tage) 的写法训练：计数器 / u 用预测时的旧值算，直接覆盖到表里
  void update_stale(const St &x, const std::vector<Ent> &rw0, const BlkCtx &c, u64 pc, int slot, bool taken) {
    nupd++;
    // 写旁路：最近 fwd_w 次训练里写过的行，用表里的当前值代替预测时的旧值
    std::vector<Ent> rw = rw0;
    for (int i = 0; i < C.nt; i++) {
      u64 lw = lastw[i][x.ci[i]];
      if (fwd_w && lw <= nupd && nupd - lw <= (u64)fwd_w)
        for (int w = 0; w < C.nway; w++) rw[(size_t)i * C.nway + w] = t[i][(size_t)x.ci[i] * C.nway + w];
    }
    auto mark = [&](int i) { lastw[i][x.ci[i]] = nupd; };
    bool mis = x.tp != taken;
    if (x.prov >= 0) {
      const Ent &o = rw[(size_t)x.prov * C.nway + x.pway];
      Ent &e = t[x.prov][(size_t)x.ci[x.prov] * C.nway + x.pway];
      bool pp = o.ctr >= 0;
      bool wk = (o.ctr == 0 || o.ctr == -1) && o.u == 0;
      if (wk && pp != x.ap) alt_na = (x.ap == taken) ? sat_inc(alt_na, 7) : sat_dec(alt_na, -8);
      e.u   = (pp != x.ap) ? ((pp == taken) ? sat_inc(o.u, 3) : sat_dec(o.u, 0)) : o.u;
      e.ctr = taken ? sat_inc(o.ctr, ctrmax) : sat_dec(o.ctr, ctrmin);
      mark(x.prov);
      if (x.alt < 0 && wk) base->upd(c.start, pc, slot, taken);
    } else {
      base->upd(c.start, pc, slot, taken);
    }
    if (mis && x.prov < C.nt - 1) {
      int start = x.prov + 1;
      if (C.nt - start > 1 && (rnd() & 1)) start++;
      int done = 0;
      for (int i = start; i < C.nt && done < C.max_alloc; i++) {
        int vw = -1;
        for (int w = 0; w < C.nway; w++) {
          const Ent &o = rw[(size_t)i * C.nway + w];
          if (!o.v || o.u == 0) { vw = w; break; }
        }
        if (vw >= 0) {
          Ent &e = t[i][(size_t)x.ci[i] * C.nway + vw];
          e.v = true; e.tag = x.ct[i]; e.slot = (int8_t)slot; e.ctr = taken ? 0 : -1; e.u = 0; done++;
          mark(i);
        }
      }
      if (!done)
        for (int i = start; i < C.nt; i++)
          for (int w = 0; w < C.nway; w++) {
            const Ent &o = rw[(size_t)i * C.nway + w];
            if (o.v && o.u) { t[i][(size_t)x.ci[i] * C.nway + w].u = o.u - 1; mark(i); }
          }
    }
  }
  void update(const BlkCtx &c, u64 pc, int slot, bool taken) override {
    nupd++;
    bool mis = tage_pred != taken;
    if (prov >= 0) {
      Ent &e = t[prov][(size_t)cidx[prov] * C.nway + pway];
      bool pp = e.ctr >= 0;
      if (prov_weak && pp != alt_pred) alt_na = (alt_pred == taken) ? sat_inc(alt_na, 7) : sat_dec(alt_na, -8);
      if (pp != alt_pred) e.u = (pp == taken) ? sat_inc(e.u, 3) : sat_dec(e.u, 0);
      e.ctr = taken ? sat_inc(e.ctr, ctrmax) : sat_dec(e.ctr, ctrmin);
      if (alt < 0 && prov_weak) base->upd(c.start, pc, slot, taken);
    } else {
      base->upd(c.start, pc, slot, taken);
    }
    // 分配：误预测且提供者不是最长表
    if (mis && prov < C.nt - 1) {
      int start = prov + 1;
      if (C.nt - start > 1 && (rnd() & 1)) start++;    // 随机跳过一张，减少乒乓
      int done = 0;
      for (int i = start; i < C.nt && done < C.max_alloc; i++) {
        Ent *row = &t[i][(size_t)cidx[i] * C.nway];
        int vw = -1;
        for (int w = 0; w < C.nway; w++) if (!row[w].v || row[w].u == 0) { vw = w; break; }
        if (vw >= 0) {
          row[vw].v = true; row[vw].tag = ctag[i]; row[vw].slot = (int8_t)slot;
          row[vw].ctr = taken ? 0 : -1; row[vw].u = 0; done++;
        }
      }
      if (!done)
        for (int i = start; i < C.nt; i++) {
          Ent *row = &t[i][(size_t)cidx[i] * C.nway];
          for (int w = 0; w < C.nway; w++) row[w].u = sat_dec(row[w].u, 0);
        }
    }
    if ((nupd & ((1ull << C.u_reset_lg) - 1)) == 0)
      for (auto &tb : t) for (auto &e : tb) e.u >>= 1;
  }
  double kbits() const override {
    double b = base->kbits() * 1024;
    for (int i = 0; i < C.nt; i++) {
      double ent = C.tagw[i] + C.ctrw + 2 + 1 + (C.slot_in_tag ? 4 : 0);
      b += ent * (1u << C.lgsz[i]) * C.nway;
    }
    return b / 1024;
  }
  std::string desc() const override {
    char s[256]; std::string r;
    snprintf(s, sizeof s, "TAGE %d 表 x %d 路 (行 2^%d..2^%d, 历史 %d..%d, 标签 %d..%d)", C.nt, C.nway,
             *std::min_element(C.lgsz.begin(), C.lgsz.end()), *std::max_element(C.lgsz.begin(), C.lgsz.end()),
             C.hl.front(), C.hl.back(), C.tagw.front(), C.tagw.back());
    return s;
  }
};

//------------------------------------------------------------------------------
// 统计校正器 (GEHL 形式)：偏置表 (pc, TAGE 方向) + 若干全局历史表 + TAGE 置信度项
//------------------------------------------------------------------------------
// mode：推翻 TAGE 的条件  0 = 总和定方向；1 = SC 与 TAGE 不同时 |sum| >= theta 才推翻；
//       2 = TAGE 计数器饱和 (conf 7) 时不推翻；3 = 1 且 2。wconf = TAGE 置信度项的权重
struct ScCfg { bool on = false; std::vector<int> hl; int lg = 9; int ctrw = 6; int mode = 0; int wconf = 8; };
struct Sc {
  ScCfg C;
  std::vector<std::vector<int8_t>> tb;    // [0] 偏置；[1..] 历史表
  std::vector<Folded> f;
  int fbase = 0, theta = 20, tc = 0, cmax, cmin;
  std::vector<u32> ci;
  int sum = 0;
  explicit Sc(const ScCfg &c) : C(c) {
    tb.resize(1 + C.hl.size());
    for (auto &x : tb) x.assign(1u << C.lg, 0);
    f.resize(C.hl.size()); ci.resize(tb.size());
    cmax = (1 << (C.ctrw - 1)) - 1; cmin = -(1 << (C.ctrw - 1));
  }
  void attach(History &h) {
    fbase = (int)h.folds.size();
    for (size_t i = 0; i < C.hl.size(); i++) { f[i].init(std::max(C.hl[i], 1), C.lg); h.reg(&f[i]); }
  }
  // 返回最终方向
  bool predict(const BlkCtx &c, u64 pc, bool tp, int tconf) {
    u64 a = pc >> 1;
    u32 m = (1u << C.lg) - 1;
    ci[0] = (u32)(((a ^ (a >> C.lg)) << 1 | tp) & m);
    sum = 0;
    for (size_t i = 0; i < C.hl.size(); i++) {
      u32 fv = C.hl[i] ? c.fsnap[fbase + i] : 0;
      ci[i + 1] = (u32)((a ^ (a >> (C.lg - 2)) ^ fv ^ ((u64)tp << (C.lg - 1))) & m);
    }
    for (size_t i = 0; i < tb.size(); i++) sum += 2 * tb[i][ci[i]] + 1;
    sum += (tp ? 1 : -1) * C.wconf * tconf;    // TAGE 越有把握，SC 越难推翻
    bool d = sum >= 0;
    if (d != tp) {
      if ((C.mode & 1) && std::abs(sum) < theta) d = tp;
      if ((C.mode & 2) && tconf >= 7) d = tp;
    }
    return d;
  }
  void update(bool taken, bool scpred) {
    if (scpred != taken || std::abs(sum) < theta) {
      for (size_t i = 0; i < tb.size(); i++) { int8_t &v = tb[i][ci[i]]; v = taken ? sat_inc(v, cmax) : sat_dec(v, cmin); }
    }
    // 自适应阈值 (Seznec)
    if (scpred != taken) { if (++tc >= 31) { tc = 0; theta++; } }
    else if (std::abs(sum) < theta) { if (--tc <= -32) { tc = 0; if (theta > 6) theta--; } }
  }
  double kbits() const { return C.on ? (double)tb.size() * (1u << C.lg) * C.ctrw / 1024 : 0; }
};

//------------------------------------------------------------------------------
// 循环预测器：记录"连续同向 N 次后反向一次"的分支，置信后覆盖
//------------------------------------------------------------------------------
struct Loop {
  struct E { u32 tag = 0; u16 past = 0, cur = 0; uint8_t conf = 0, age = 0; bool dir = false, v = false; };
  bool on = false; int lg = 6, nway = 4;
  std::vector<E> t;
  int hit = -1; bool lpred = false, valid = false;
  Loop() {}
  void init(bool o, int l) { on = o; lg = l; t.assign(1u << lg, E()); }
  u32 set(u64 pc) const { return (u32)(((pc >> 1) & ((1u << (lg - 2)) - 1)) << 2); }
  u32 tg(u64 pc) const { return (u32)((pc >> (lg + 1)) & 0x3fff); }
  bool predict(u64 pc) {
    hit = -1; valid = false;
    if (!on) return false;
    u32 s = set(pc), g = tg(pc);
    for (int w = 0; w < nway; w++) if (t[s + w].v && t[s + w].tag == g) { hit = (int)(s + w); break; }
    if (hit < 0) return false;
    E &e = t[hit];
    valid = e.conf >= 3 && e.past > 0;
    lpred = (e.cur + 1 == e.past) ? !e.dir : e.dir;
    return lpred;
  }
  void update(u64 pc, bool taken, bool tage_mis) {
    if (!on) return;
    if (hit >= 0) {
      E &e = t[hit];
      if (valid && lpred != taken) { e.v = false; return; }
      if (taken == e.dir) {
        e.cur++;
        if (e.cur > e.past && e.past) { e.conf = 0; e.past = 0; }
        if (e.cur == 0xffff) e.v = false;
      } else {
        if (e.past == e.cur && e.past) { if (e.conf < 3) e.conf++; }
        else { e.past = e.cur; e.conf = 0; }
        e.cur = 0;
      }
      if (e.age < 255) e.age++;
      return;
    }
    if (!tage_mis || !taken) return;       // 只在 TAGE 预测错、且分支跳时分配 (循环体的回跳)
    u32 s = set(pc);
    for (int w = 0; w < nway; w++) {
      E &e = t[s + w];
      if (!e.v || e.age == 0) { e = E(); e.v = true; e.tag = tg(pc); e.dir = taken; e.cur = 0; return; }
    }
    for (int w = 0; w < nway; w++) if (t[s + w].age) t[s + w].age--;
  }
  double kbits() const { return on ? (double)t.size() * (14 + 16 + 16 + 2 + 8 + 1) / 1024 : 0; }
};

//------------------------------------------------------------------------------
// 间接跳转目标：ITTAGE (或退化为按 PC 的"上次目标"表 / OOO6 现有的 32 项全相联)
//------------------------------------------------------------------------------
struct ItCfg { int nt = 0; std::vector<int> hl, lgsz; int tagw = 9; int base_lg = 9; int fa = 0; };
struct Ittage {
  struct E { u32 tag = 0; u64 tgt = 0; uint8_t ctr = 0, u = 0; bool v = false; };
  ItCfg C;
  std::vector<u64> base;                     // 按 PC 的上次目标
  std::vector<std::vector<E>> t;
  std::vector<Folded> fi, ft;
  int fbase = 0;
  std::vector<u32> ci, ct;
  int prov = -1, alt = -1;
  u64 ptgt = 0;
  // 全相联 (fa > 0)：OOO6 现有 uBTB 目标 (LRU)
  std::vector<std::pair<u64, u64>> fa;      // (pc, tgt)，前面最新
  explicit Ittage(const ItCfg &c) : C(c) {
    base.assign(1u << C.base_lg, 0);
    t.resize(C.nt); fi.resize(C.nt); ft.resize(C.nt); ci.resize(C.nt); ct.resize(C.nt);
    for (int i = 0; i < C.nt; i++) t[i].assign(1u << C.lgsz[i], E());
  }
  void attach(History &h) {
    fbase = (int)h.folds.size();
    for (int i = 0; i < C.nt; i++) { fi[i].init(C.hl[i], C.lgsz[i]); ft[i].init(C.hl[i], C.tagw); h.reg(&fi[i]); h.reg(&ft[i]); }
  }
  u64 predict(const BlkCtx &c, u64 pc) {
    if (C.fa) {
      for (auto &e : fa) if (e.first == pc) return ptgt = e.second;
      return ptgt = 0;
    }
    u64 a = pc >> 1;
    prov = alt = -1;
    for (int i = 0; i < C.nt; i++) {
      ci[i] = (u32)((a ^ (a >> C.lgsz[i]) ^ c.fsnap[fbase + 2 * i]) & ((1u << C.lgsz[i]) - 1));
      ct[i] = (u32)((a ^ (a >> 9) ^ c.fsnap[fbase + 2 * i + 1]) & ((1u << C.tagw) - 1));
    }
    for (int i = C.nt - 1; i >= 0; i--) {
      E &e = t[i][ci[i]];
      if (e.v && e.tag == ct[i]) { if (prov < 0) prov = i; else { alt = i; break; } }
    }
    u64 bt = base[(a ^ (a >> C.base_lg)) & ((1u << C.base_lg) - 1)];
    if (prov < 0) return ptgt = bt;
    E &e = t[prov][ci[prov]];
    if (e.ctr == 0 && alt >= 0) return ptgt = t[alt][ci[alt]].tgt;
    if (e.ctr == 0 && alt < 0) return ptgt = bt;
    return ptgt = e.tgt;
  }
  void update(u64 pc, u64 tgt) {
    if (C.fa) {
      for (size_t k = 0; k < fa.size(); k++) if (fa[k].first == pc) { fa.erase(fa.begin() + k); break; }
      fa.insert(fa.begin(), {pc, tgt});
      if ((int)fa.size() > C.fa) fa.pop_back();
      return;
    }
    u64 a = pc >> 1;
    bool mis = ptgt != tgt;
    base[(a ^ (a >> C.base_lg)) & ((1u << C.base_lg) - 1)] = tgt;
    if (prov >= 0) {
      E &e = t[prov][ci[prov]];
      if (e.tgt == tgt) { e.ctr = (uint8_t)sat_inc(e.ctr, 3); if (mis == false) e.u = (uint8_t)sat_inc(e.u, 1); }
      else if (e.ctr) e.ctr--;
      else { e.tgt = tgt; e.ctr = 1; }
    }
    if (mis && prov < C.nt - 1) {
      bool done = false;
      for (int i = prov + 1; i < C.nt && !done; i++) {
        E &e = t[i][ci[i]];
        if (!e.v || e.u == 0) { e.v = true; e.tag = ct[i]; e.tgt = tgt; e.ctr = 0; e.u = 0; done = true; }
      }
      if (!done) for (int i = prov + 1; i < C.nt; i++) if (t[i][ci[i]].u) t[i][ci[i]].u--;
    }
  }
  double kbits() const {
    if (C.fa) return C.fa * (38 + 30) / 1024.0;
    double b = (double)(1u << C.base_lg) * 38;
    for (int i = 0; i < C.nt; i++) b += (double)(1u << C.lgsz[i]) * (C.tagw + 38 + 2 + 1 + 1);
    return b / 1024;
  }
};

//==============================================================================
// 整体配置
//==============================================================================
struct Cfg {
  std::string name;
  int blk_bytes = 32; bool blk_aligned = true; int max_br = 0;   // max_br：每块最多分支数 (0 = 不限)
  int blk_line = 0;               // 非对齐块不跨越的行大小 (0 = 可跨行；64 = 不跨 64 B 缓存行)
  HistCfg hist;
  TageCfg tage;
  bool cur_ooo6 = false;          // OOO6 现有：每块只训练一个槽
  int train_n = 0;                // >0：每块 TAGE 只训练前 train_n 个"候选" (误预测的优先)，0 = 全训练
  bool train_pos = false;         // 候选按块内位置取前 train_n 个 (RTL：预译码给出的前两个条件分支)
  int stale_d = 0;                // >0：TAGE 训练推迟 stale_d 个分支，并用预测时的旧值改写 (同 RTL)
  int tage_fwd = 0;               // 旧值训练的写旁路深度
  ScCfg sc;
  bool loop = false; int loop_lg = 6;
  ItCfg it;
  int ras = 32;
  // 前端跳转识别：uBTB (全相联，0 拍) / BTB (组相联，lat1 拍) / 预译码 (latf 拍)；误预测惩罚 pen 拍
  int ub_n = 32, btb_lgsets = 8, btb_ways = 1;
  int ub_ways = 0;                // uBTB 路数：0 = 全相联；否则 ub_n / ub_ways 组，按块入口 PC 散列选组
  int ub_slots = 1, btb_slots = 1;  // 每项记几个控制流槽 (香山 uFTB/FTB 为 2)
  bool ub_ctr = false;            // uBTB 每槽带 2 位方向计数器 (OOO6 现有没有：有项就预测跳)
  bool l1_full_dir = false;       // BTB 级条件分支方向：false = 只有双模态 (OOO6 现有)，true = 完整 TAGE
  int lat1 = 1, latf = 3, pen = 14;
  int lat2 = 0;                   // >0：三级 (BTB+bimodal 在 lat1，完整 TAGE 在 lat2)
};

static std::vector<int> geom(int n, int lo, int hi) {
  std::vector<int> v(n);
  for (int i = 0; i < n; i++) v[i] = (int)std::lround(lo * std::pow((double)hi / lo, n > 1 ? (double)i / (n - 1) : 0));
  return v;
}

static Cfg preset1(const std::string &p);

// 预设 + 覆盖：  基础预设名[+键=值]...   例：v2+lg=10+lat1=2+blk=ul+maxbr=2+ub=128
//   lg=N     TAGE 每表行数 2^N         nt=N   TAGE 表数 (历史长度按几何级数重排)   hmax=N 最长历史
//   lat1=N   BTB 级改道气泡             ub=N   uBTB 项数                            btb=N  BTB 组数 2^N
//   blk=a|u|ul  对齐 / 入口起可跨行 / 入口起不跨 64 B 行                            maxbr=N 每块最多分支数
//   pb=N     路径历史每次跳转推的位数   sc=0/1  loop=0/1  ways=N TAGE 每行路数
static Cfg preset(const std::string &spec) {
  size_t k = spec.find('+');
  Cfg c = preset1(spec.substr(0, k));
  c.name = spec;
  int nt = c.tage.nt, hmin = c.tage.hl.front(), hmax = c.tage.hl.back();
  bool rehist = false;
  while (k != std::string::npos) {
    size_t e = spec.find('+', k + 1);
    std::string kv = spec.substr(k + 1, e == std::string::npos ? std::string::npos : e - k - 1);
    size_t q = kv.find('=');
    std::string key = kv.substr(0, q), val = q == std::string::npos ? "" : kv.substr(q + 1);
    int v = atoi(val.c_str());
    if (key == "lg") c.tage.lgsz.assign(c.tage.nt, v);
    else if (key == "nt") { nt = v; rehist = true; }
    else if (key == "hmax") { hmax = v; rehist = true; }
    else if (key == "lat1") c.lat1 = v;
    else if (key == "lat2") c.lat2 = v;
    else if (key == "ub") c.ub_n = v;
    else if (key == "btb") c.btb_lgsets = v;
    else if (key == "blk") { c.blk_aligned = val == "a"; c.blk_line = val == "ul" ? 64 : 0; }
    else if (key == "maxbr") c.max_br = v;
    else if (key == "pb") c.hist.pbits = v;
    else if (key == "sc") {                                     // 1 = 5 表 (偏置 + 历史 6/13/27/40)；2 = 香山式 4 表 (0/4/10/16)
      c.sc.on = v != 0;
      if (v == 1) { c.sc.hl = {0, 6, 13, 27, 40}; c.sc.lg = 10; }
      if (v == 2) { c.sc.hl = {0, 4, 10, 16};     c.sc.lg = 9;  }
      if (v == 3) { c.sc.hl = {0, 4, 8, 16, 32, 64}; c.sc.lg = 10; }
    }
    else if (key == "sclg") c.sc.lg = v;                        // SC 每表行数 2^N
    else if (key == "scm") c.sc.mode = v;                       // SC 推翻条件 (见 ScCfg)
    else if (key == "scwc") c.sc.wconf = v;                     // TAGE 置信度项权重
    else if (key == "scw") c.sc.ctrw = v;                       // SC 计数器位宽
    else if (key == "loop") c.loop = v != 0;
    else if (key == "ways") c.tage.nway = v;
    else if (key == "tr") c.train_n = v;
    else if (key == "rh") c.tage.rtl_hash = v != 0;
    else if (key == "hr") c.hist.rtl = v != 0;
    else if (key == "tp") c.train_pos = v != 0;
    else if (key == "sd") c.stale_d = v;
    else if (key == "ht") c.hist.tonly = v != 0;
    else if (key == "fw") c.tage_fwd = v;
    else if (key == "bw") c.btb_ways = v;                       // BTB 路数
    else if (key == "ubw") c.ub_ways = v;                       // uBTB 路数 (0 = 全相联)
    else if (key == "bs") c.btb_slots = v;                      // BTB 每项槽数
    else if (key == "ubs") c.ub_slots = v;                      // uBTB 每项槽数
    else if (key == "l1d") c.l1_full_dir = v != 0;              // BTB 级用完整 TAGE 方向
    else if (key == "itt") {                                    // ITTAGE (v2 的配置)；0 = 只有 uBTB 记的上次目标
      if (v == 1) { c.it.fa = 0; c.it.nt = 6; c.it.hl = {4, 10, 20, 40, 80, 160}; c.it.lgsz = {8, 8, 9, 9, 9, 9};
                    c.it.tagw = 10; c.it.base_lg = 9; }
      if (v == 2) { c.it.fa = 0; c.it.nt = 5; c.it.hl = {4, 8, 13, 16, 32}; c.it.lgsz = {8, 8, 9, 9, 9};   // 香山式
                    c.it.tagw = 9; c.it.base_lg = 9; }
      if (v == 3) { c.it.fa = 0; c.it.nt = 4; c.it.hl = {6, 18, 56, 172}; c.it.lgsz = {8, 8, 8, 8};        // 小号，沿用 TAGE 的历史
                    c.it.tagw = 9; c.it.base_lg = 8; }
    }
    else if (key == "tw") c.tage.tagw.assign(c.tage.nt, v);
    else if (key == "blg") c.tage.base_lg = v;
    else if (key == "ur") c.tage.u_reset_lg = v ? v : 62;   // u 位周期老化 2^v 次更新一次；0 = 不老化
    else { fprintf(stderr, "未知覆盖项 %s\n", key.c_str()); exit(1); }
    k = e;
  }
  if (rehist) {
    int lg = c.tage.lgsz.front();
    c.tage.nt = nt; c.tage.hl = geom(nt, hmin, hmax); c.tage.lgsz.assign(nt, lg);
    c.tage.tagw.resize(nt);
    for (int i = 0; i < nt; i++) c.tage.tagw[i] = 9 + (3 * i) / std::max(1, nt - 1);
  }
  return c;
}

static Cfg preset1(const std::string &p) {
  Cfg c; c.name = p;
  if (p == "cur") {
    // OOO6 现有 RTL：32 B 对齐块；双模态 256 行 x 16 槽；TAGE 4 x 256 项 (1 路，槽号匹配)，
    // 历史每块 1 位 8/16/32/64；间接跳转只有 uBTB (32 项全相联) 的上次目标；RAS 32
    c.hist.mode = HM_BLOCK;
    c.tage.nt = 4; c.tage.hl = {8, 16, 32, 64}; c.tage.lgsz = {8, 8, 8, 8}; c.tage.tagw = {9, 9, 9, 9};
    c.tage.nway = 1; c.tage.base_lg = 12; c.tage.idx_by_block = true; c.tage.slot_in_tag = true;
    c.tage.use_alt_na = false;
    c.cur_ooo6 = true; c.train_n = 1;
    c.it.fa = 32;
    c.ub_n = 32; c.btb_lgsets = 8; c.btb_ways = 1; c.l1_full_dir = false;
  } else if (p == "xs") {
    // 香山昆明湖 (按公开设计文档的参数近似，非 RTL 精确模型)：
    //   取指块 32 B (从起点算，不对齐)、FTB 每项 2 个分支槽；历史按分支方向 (每块至多 2 位)
    //   TAGE 4 表 x 4096 项 (每行 2 槽)，历史 8/13/32/119，标签 8；基表 2048 x 2
    //   SC 4 表 x 512 (每槽)，历史 0/4/10/16；ITTAGE 5 表 (256/256/512/512/512，历史 4/8/13/16/32)；RAS 32
    c.blk_aligned = false; c.max_br = 2;
    c.hist.mode = HM_DIR; c.hist.cap = 2;
    c.tage.nt = 4; c.tage.hl = {8, 13, 32, 119}; c.tage.lgsz = {11, 11, 11, 11}; c.tage.tagw = {8, 8, 8, 8};
    c.tage.nway = 2; c.tage.base_lg = 12; c.tage.idx_by_block = true; c.tage.slot_in_tag = true;
    c.sc.on = true; c.sc.hl = {0, 4, 10, 16}; c.sc.lg = 10;
    c.it.nt = 5; c.it.hl = {4, 8, 13, 16, 32}; c.it.lgsz = {8, 8, 9, 9, 9}; c.it.tagw = 9; c.it.base_lg = 9;
    // uFTB 取 256 项 (偏向对香山有利的上限)，FTB 2048 项 4 路，s2 出 TAGE 结果
    c.ub_n = 256; c.btb_lgsets = 9; c.btb_ways = 4; c.l1_full_dir = true;
    c.ub_slots = 2; c.btb_slots = 2; c.ub_ctr = true;
  } else if (p == "t40") {
    // RTL t40：对齐 32 B 块；bimodal 256 行 x 16 槽 (触发器)；TAGE 8 x 1024 行 x 2 路，12 位标签，
    // 历史 6..300 (路径历史，散列同 RTL)，不老化，每块训前两个条件分支；无 SC / 循环 / ITTAGE；
    // uBTB 32 项 2 槽 + 计数器；BTB 256 项直接映射 (方向只看 bimodal)
    c.hist.mode = HM_PATH; c.hist.pbits = 2; c.hist.rtl = true; c.hist.tonly = true;
    c.tage.nt = 8; c.tage.hl = {6, 10, 18, 32, 56, 98, 172, 300}; c.tage.lgsz = std::vector<int>(8, 10);
    c.tage.tagw = std::vector<int>(8, 12); c.tage.nway = 2; c.tage.base_lg = 12; c.tage.idx_by_block = true;
    c.tage.slot_in_tag = true; c.tage.max_alloc = 2; c.tage.rtl_hash = true; c.tage.u_reset_lg = 62;
    c.train_n = 2; c.train_pos = false;          // 提交时挑：误预测的优先
    c.it.fa = 32;
    c.ub_n = 32; c.ub_slots = 2; c.ub_ctr = true; c.btb_lgsets = 8; c.btb_ways = 1; c.btb_slots = 1; c.l1_full_dir = false;
  } else if (p.rfind("v2", 0) == 0) {
    // OOO6 新设计候选：32 B 对齐块 (保持现有取指结构)，按块起点索引、每行多路
    c.hist.mode = HM_PATH; c.hist.pbits = 2;
    c.tage.nt = 8; c.tage.hl = geom(8, 6, 640); c.tage.lgsz = std::vector<int>(8, 11);
    c.tage.tagw = {9, 9, 10, 10, 11, 11, 12, 12};
    c.tage.nway = 2; c.tage.base_lg = 13; c.tage.idx_by_block = true; c.tage.slot_in_tag = true;
    c.tage.max_alloc = 2;
    c.sc.on = true; c.sc.hl = {0, 6, 13, 27, 40}; c.sc.lg = 10;
    c.loop = true; c.loop_lg = 6;
    c.it.nt = 6; c.it.hl = {4, 10, 20, 40, 80, 160}; c.it.lgsz = {8, 8, 9, 9, 9, 9}; c.it.tagw = 10; c.it.base_lg = 9;
    c.ub_n = 64; c.btb_lgsets = 10; c.btb_ways = 4; c.l1_full_dir = true;
    c.ub_slots = 2; c.btb_slots = 2; c.ub_ctr = true;
    if (p == "v2ub128") c.ub_n = 128;
    if (p == "v2u")  { c.blk_aligned = false; }                    // 从入口起 32 B，可跨行 (同香山)
    if (p == "v2ul") { c.blk_aligned = false; c.blk_line = 64; }   // 从入口起 32 B，不跨 64 B 行
    if (p == "v2ul2") { c.blk_aligned = false; c.blk_line = 64; c.max_br = 2; }
    if (p == "v2ub256") c.ub_n = 256;
    if (p == "v2b2k") c.btb_lgsets = 9;
    if (p == "v2bim") c.l1_full_dir = false;
    if (p == "v2dir") { c.hist.mode = HM_DIR; c.hist.cap = 4; }
    if (p == "v2dir2") { c.hist.mode = HM_DIR; c.hist.cap = 2; }
    if (p == "v2p3") { c.hist.pbits = 3; c.tage.hl = geom(8, 6, 960); }
    if (p == "v2t12") { c.tage.nt = 12; c.tage.hl = geom(12, 4, 1000); c.tage.lgsz = std::vector<int>(12, 10);
                        c.tage.tagw = {8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 13}; }
    if (p == "v2pc") { c.tage.idx_by_block = false; c.tage.slot_in_tag = false; }   // 理想：按分支 PC 索引
    if (p == "v2nosc") c.sc.on = false;
    if (p == "v2noloop") c.loop = false;
  } else {
    fprintf(stderr, "未知预设 %s\n", p.c_str()); exit(1);
  }
  return c;
}

//==============================================================================
// 模拟
//==============================================================================
struct Stats {
  u64 inst = 0, cond = 0, cond_mis = 0, ind = 0, ind_mis = 0, ret = 0, ret_mis = 0, blocks = 0;
  u64 bubbles = 0, lost = 0;      // 气泡周期；损失周期 = 气泡 + 误预测 x 惩罚
  void add(const Stats &o) { inst += o.inst; cond += o.cond; cond_mis += o.cond_mis; ind += o.ind; ind_mis += o.ind_mis;
                             ret += o.ret; ret_mis += o.ret_mis; blocks += o.blocks; bubbles += o.bubbles; lost += o.lost; }
  double mpki(u64 m) const { return inst ? 1000.0 * m / inst : 0; }
};

struct Sim {
  Cfg C;
  History H;
  std::unique_ptr<Tage> tage;
  std::unique_ptr<Sc> sc;
  Loop loop;
  std::unique_ptr<Ittage> it;
  std::vector<u64> ras;
  BlkCtx ctx;
  u64 blk_end = 0;
  int blk_nbr = 0;
  bool in_blk = false;
  // 前端跳转识别：uBTB / BTB 每项 = 块入口 PC 下的若干控制流槽 (按槽号升序)
  struct Slot { int slot = -1; u64 tgt = 0; bool cond = false; int8_t ctr = 2; };
  struct BE { u64 key = 0; bool v = false; u64 lru = 0; int n = 0; Slot s[2]; };
  std::vector<BE> ub, btb;
  u64 tick = 0;
  u64 bpc = 0;                     // 本块入口 PC (uBTB / BTB 的键)
  BE p0, p1;                       // 块开始时两级读出的项
  std::vector<Slot> bl;            // 本块经过的控制流 (按序；taken 的记 ctr=3，不跳的 ctr=0)
  Stats *SP = nullptr;
  // uBTB 的候选范围：全相联为整张表，组相联为一组
  std::pair<int, int> ub_range(u64 key) const {
    if (!C.ub_ways) return {0, (int)ub.size()};
    int ns = (int)ub.size() / C.ub_ways;
    u64 a = key >> 1;
    int si = (int)((a ^ (a >> 5) ^ (a >> 10)) % (u64)ns);
    return {si * C.ub_ways, si * C.ub_ways + C.ub_ways};
  }
  BE *ub_find(u64 key) { auto r = ub_range(key); for (int i = r.first; i < r.second; i++) if (ub[i].v && ub[i].key == key) return &ub[i]; return nullptr; }
  BE *btb_set(u64 key) { u64 a = key >> 1; return &btb[(size_t)(((a ^ (a >> C.btb_lgsets)) & ((1u << C.btb_lgsets) - 1)) * C.btb_ways)]; }
  BE *btb_find(u64 key) { BE *s = btb_set(key); for (int w = 0; w < C.btb_ways; w++) if (s[w].v && s[w].key == key) return &s[w]; return nullptr; }
  BE *ub_alloc(u64 key) {
    auto rg = ub_range(key);
    BE *x = &ub[rg.first];
    for (int i = rg.first; i < rg.second; i++) { BE &y = ub[i]; if (!y.v) { x = &y; break; } if (y.lru < x->lru) x = &y; }
    *x = BE(); x->v = true; x->key = key; return x;
  }
  BE *btb_alloc(u64 key) {
    BE *s = btb_set(key), *x = &s[0];
    for (int w = 0; w < C.btb_ways; w++) { if (!s[w].v) { x = &s[w]; break; } if (s[w].lru < x->lru) x = &s[w]; }
    *x = BE(); x->v = true; x->key = key; return x;
  }
  // 由项给出预测：按槽序第一个"无条件"或"条件且 dir 说跳"的槽；返回槽号 (-1 = 顺序) 与目标
  template <class F> static int epred(const BE &e, F dir, u64 &tgt) {
    if (!e.v) return -1;
    for (int i = 0; i < e.n; i++)
      if (!e.s[i].cond || dir(e.s[i])) { tgt = e.s[i].tgt; return e.s[i].slot; }
    return -1;
  }
  // 项已满 (e.n == K，槽按槽号升序)、又来了一个不在项里的跳转 (槽号 fslot) 时，选被替换的槽，返回 0..K-1。
  // 可用信息：e.s[i].slot (块内位置)、e.s[i].cond (是否条件分支)、e.s[i].ctr (0..3 方向计数器，3 = 强跳)、
  // fslot 与已有槽的先后 (fslot 比所有已有槽都靠前时，块会在它那里结束，后面的槽本次根本走不到)
  // 现行策略：替换块里最靠后的槽 (硬件最简单；靠前的槽决定块在哪结束，更值得留)
  static int pick_victim(const BE &e, int K, int fslot) {
    (void)e; (void)fslot;
    return K - 1;
  }
  // 训练一项：经过的条件分支更新计数器；最终跳转的槽若不在项里则插入 (满了按 pick_victim 替换)
  void etrain(BE &e, int K, int fslot, u64 ftgt, bool fcond) {
    for (auto &c : bl)
      for (int i = 0; i < e.n; i++)
        if (e.s[i].slot == c.slot && e.s[i].cond) e.s[i].ctr = c.ctr == 3 ? (int8_t)sat_inc(e.s[i].ctr, 3) : (int8_t)sat_dec(e.s[i].ctr, 0);
    if (fslot < 0) return;
    for (int i = 0; i < e.n; i++) if (e.s[i].slot == fslot) { e.s[i].tgt = ftgt; return; }
    Slot ns; ns.slot = fslot; ns.tgt = ftgt; ns.cond = fcond; ns.ctr = 2;
    if (e.n < K) e.s[e.n++] = ns; else e.s[pick_victim(e, K, fslot)] = ns;
    std::sort(e.s, e.s + e.n, [](const Slot &x, const Slot &y) { return x.slot < y.slot; });
  }

  bool pcstat = false;
  std::map<u64, std::pair<u64, u64>> pcs;     // 分支 PC -> (次数, 误预测)
  explicit Sim(const Cfg &c) : C(c) {
    H.cfg = C.hist;
    tage.reset(new Tage(C.tage)); tage->attach(H); tage->fwd_w = C.tage_fwd;
    sc.reset(new Sc(C.sc)); if (C.sc.on) sc->attach(H);
    loop.init(C.loop, C.loop_lg);
    it.reset(new Ittage(C.it)); it->attach(H);
    ub.assign(C.ub_n, BE()); btb.assign((size_t)(1u << C.btb_lgsets) * C.btb_ways, BE());
  }
  double kbits() const { return tage->kbits() + sc->kbits() + loop.kbits() + it->kbits(); }

  void start_block(u64 pc) {
    // 对齐块：表按 32 B 块基址索引 (同一分支不论从哪进块都落在同一项，槽号区分)；非对齐块 (香山)：按入口 PC
    ctx.start = C.blk_aligned ? (pc & ~31ull) : pc;
    ctx.snap(H);
    blk_end = C.blk_aligned ? ((pc | (u64)(C.blk_bytes - 1)) + 1) : pc + C.blk_bytes;
    if (!C.blk_aligned && C.blk_line) blk_end = std::min(blk_end, (pc | (u64)(C.blk_line - 1)) + 1);
    blk_nbr = 0; in_blk = true;
    bpc = pc;
    BE *a = ub_find(pc); p0 = a ? *a : BE(); if (a) a->lru = ++tick;
    BE *b = btb_find(pc); p1 = b ? *b : BE(); if (b) b->lru = ++tick;
    bl.clear();
  }
  // 每块限训练数时：块内条件分支的预测状态先存起来，块结束时挑 train_n 个训练 TAGE (其余只训练 bimodal)
  struct PT { Tage::St st; BlkCtx c; u64 pc; int slot; bool taken, mis; std::vector<Tage::Ent> rw; };
  std::vector<PT> pend_t;
  std::vector<PT> dq;              // 延迟训练队列 (sd > 0)
  size_t dq_h = 0;
  void flush_train() {
    if (pend_t.empty()) return;
    std::vector<int> ord;
    for (int i = 0; i < (int)pend_t.size(); i++) if (C.train_pos || pend_t[i].mis) ord.push_back(i);
    if (!C.train_pos) for (int i = 0; i < (int)pend_t.size(); i++) if (!pend_t[i].mis) ord.push_back(i);
    std::vector<bool> chosen(pend_t.size(), false);
    for (int k = 0; k < (int)ord.size() && k < C.train_n; k++) chosen[ord[k]] = true;
    for (int i = 0; i < (int)pend_t.size(); i++) {
      PT &x = pend_t[i];
      if (chosen[i]) {
        if (C.stale_d) dq.push_back(x);
        else { tage->load(x.st); tage->update(x.c, x.pc, x.slot, x.taken); }
      } else tage->base->upd(x.c.start, x.pc, x.slot, x.taken);
    }
    pend_t.clear();
    while (C.stale_d && dq.size() - dq_h > (size_t)C.stale_d) {
      PT &y = dq[dq_h++];
      tage->update_stale(y.st, y.rw, y.c, y.pc, y.slot, y.taken);
    }
    if (dq_h > 4096) { dq.erase(dq.begin(), dq.begin() + dq_h); dq_h = 0; }
  }
  void end_block(bool taken) { flush_train(); H.end_block(taken); in_blk = false; }
  // 块结束时的前端记账：fslot/ftgt = 最终 (正确) 预测的跳转槽与目标，fslot<0 表示顺序；mis = 本块以误预测结束
  void fe_account(int fslot, u64 ftgt, bool fcond, bool mis) {
    u64 t0 = 0, t1 = 0;
    int s0 = epred(p0, [&](const Slot &x) { return C.ub_ctr ? x.ctr >= 2 : true; }, t0);
    u64 bb = C.blk_aligned ? (bpc & ~31ull) : bpc;
    int s1 = epred(p1, [&](const Slot &x) {
      if (C.l1_full_dir) return fslot == x.slot;            // 完整预测 (本块无误预测时即实际方向)
      return tage->base->pred(ctx.start, bb + 2 * (u64)x.slot, (int)(((bb >> 1) + x.slot) & 15));
    }, t1);
    auto same = [&](int sl, u64 tg) { return sl < 0 ? fslot < 0 : (fslot == sl && tg == ftgt); };
    // 三级 (lat2 > 0)：BTB 级先用 bimodal 方向在 lat1 改道，完整 TAGE 在 lat2 再纠正
    u64 t1b = 0;
    int s1b = C.lat2 ? epred(p1, [&](const Slot &x) {
      return tage->base->pred(ctx.start, bb + 2 * (u64)x.slot, (int)(((bb >> 1) + x.slot) & 15)); }, t1b) : -2;
    int b = 0;
    if (mis) { SP->lost += C.pen; }
    else if (same(s0, t0)) { }
    else if (C.lat2 && same(s1b, t1b)) b = C.lat1;
    else if (same(s1, t1)) b = C.lat2 ? C.lat2 : C.lat1;
    else b = C.latf;
    SP->bubbles += b; SP->lost += b;
    if (mis) { fslot = -1; }       // 误预测结束的块：只训练计数器，不插入
    // uBTB
    if (C.ub_ctr || C.ub_slots > 1) {
      BE *e = ub_find(bpc);
      if (!e && fslot >= 0) e = ub_alloc(bpc);
      if (e) { etrain(*e, C.ub_slots, fslot, ftgt, fcond); e->lru = ++tick; }
    } else if (!same(s0, t0) && !mis) {
      // OOO6 现有：只在不一致时改写；最终顺序则作废
      BE *e = ub_find(bpc);
      if (fslot >= 0) { if (!e) e = ub_alloc(bpc); e->n = 0; etrain(*e, 1, fslot, ftgt, fcond); e->lru = ++tick; }
      else if (e) e->v = false;
    }
    // BTB
    if (C.btb_slots > 1) {
      BE *e = btb_find(bpc);
      if (!e && fslot >= 0) e = btb_alloc(bpc);
      if (e) { etrain(*e, C.btb_slots, fslot, ftgt, fcond); e->lru = ++tick; }
    } else if (!same(s1, t1) && !mis) {
      BE *e = btb_find(bpc);
      if (fslot >= 0) { if (!e) e = btb_alloc(bpc); e->n = 0; etrain(*e, 1, fslot, ftgt, fcond); e->lru = ++tick; }
      else if (e) e->v = false;
    }
  }

  void run(FILE *f, Stats &S) {
    SP = &S;
    bp_hdr_t h;
    if (fread(&h, sizeof h, 1, f) != 1 || memcmp(h.magic, "BPT1", 4)) { fprintf(stderr, "坏 trace\n"); exit(1); }
    std::vector<bp_rec_t> buf(1 << 16);
    u64 pcur = 0; bool first = true;
    size_t n;
    while ((n = fread(buf.data(), sizeof(bp_rec_t), buf.size(), f)) > 0) {
      for (size_t k = 0; k < n; k++) {
        const bp_rec_t &r = buf[k];
        S.inst += r.ninst;
        if (first) { start_block(r.pc); first = false; }
        // 顺序越过块边界：中间是不含控制流的整块 (超过 64 块的长直线代码只按 64 块推历史)
        if (r.pc < ctx.start) { end_block(false); start_block(r.pc); }
        for (int g = 0; r.pc >= blk_end; g++) {
          fe_account(-1, 0, false, false);
          end_block(false); S.blocks++;
          start_block(g < 64 ? blk_end : r.pc);
        }
        (void)pcur;
        if (r.kind == BP_REDIR) { end_block(true); S.blocks++; start_block(r.tgt); continue; }   // 陷入：不记前端账
        int slot = (int)((r.pc >> 1) & 15);
        bool mis = false, ptaken = true;
        if (r.kind == BP_COND) {
          S.cond++;
          bool tp = tage->predict(ctx, r.pc, slot);
          bool pred = tp;
          bool scp = tp;
          if (C.sc.on) { scp = sc->predict(ctx, r.pc, tp, tage->conf()); pred = scp; }
          bool lp = loop.predict(r.pc);
          if (loop.valid) pred = lp;
          ptaken = pred;
          mis = pred != r.taken;
          if (mis) S.cond_mis++;
          if (pcstat) { auto &q = pcs[r.pc]; q.first++; if (mis) q.second++; }
          if (C.train_n) pend_t.push_back(PT{tage->save(), ctx, r.pc, slot, r.taken, tp != r.taken,
                                             C.stale_d ? tage->rows() : std::vector<Tage::Ent>()});
          else tage->update(ctx, r.pc, slot, r.taken);
          if (C.sc.on) sc->update(r.taken, scp);
          loop.update(r.pc, r.taken, tp != r.taken);
        } else if (r.kind == BP_RET) {
          S.ret++;
          u64 pt = 0;
          if (!ras.empty()) { pt = ras.back(); ras.pop_back(); }
          if (pt != r.tgt) { mis = true; S.ret_mis++; }
        } else if (r.kind == BP_IND || r.kind == BP_ICALL) {
          S.ind++;
          u64 pt = it->predict(ctx, r.pc);
          if (pt != r.tgt) { mis = true; S.ind_mis++; }
          it->update(r.pc, r.tgt);
        }
        if (r.kind == BP_CALL || r.kind == BP_ICALL) {
          ras.push_back(r.pc + r.ilen);
          if ((int)ras.size() > C.ras) ras.erase(ras.begin());
        }
        H.on_cf(r.kind, r.taken, r.pc, r.tgt);
        {
          Slot c; c.slot = (int)((r.pc - (C.blk_aligned ? (r.pc & ~31ull) : bpc)) >> 1); c.tgt = r.tgt;
          c.cond = r.kind == BP_COND; c.ctr = r.taken ? 3 : 0;
          bl.push_back(c);
        }
        blk_nbr++;
        bool ends = r.taken || mis || (C.max_br && blk_nbr >= C.max_br);
        if (ends) {
          if (mis) fe_account(-1, 0, false, true);
          else if (r.taken) fe_account((int)((r.pc - (C.blk_aligned ? (r.pc & ~31ull) : bpc)) >> 1), r.tgt, r.kind == BP_COND, false);
          else fe_account(-1, 0, false, false);
          end_block(r.taken);
          S.blocks++;
          start_block(r.taken ? r.tgt : r.pc + r.ilen);
        }
        (void)ptaken;
      }
    }
  }
};

int main(int argc, char **argv) {
  std::vector<std::string> presets; std::vector<std::string> files; bool verbose = false, pcstat = false;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "-p") && i + 1 < argc) presets.push_back(argv[++i]);
    else if (!strcmp(argv[i], "-v")) verbose = true;
    else if (!strcmp(argv[i], "-pcs")) pcstat = true;
    else files.push_back(argv[i]);
  }
  if (presets.empty() || files.empty()) { fprintf(stderr, "usage: bpsim -p 预设 [-p 预设 ...] [-v] trace.bpt ...\n"); return 1; }
  printf("%-10s %-24s %9s %8s %8s %8s %8s %8s %8s %8s %8s\n", "预设", "trace", "Minst", "条件MPKI", "间接MPKI", "返回MPKI", "总MPKI",
         "气泡/KI", "损失/KI", "前端IPC", "存储KB");
  for (auto &p : presets) {
    Stats tot; double kb = 0;
    std::vector<double> ratios;
    for (auto &fn : files) {
      Cfg c = preset(p);
      Sim s(c);
      s.pcstat = pcstat;
      kb = s.kbits() / 8;
      FILE *f = fopen(fn.c_str(), "rb");
      if (!f) { perror(fn.c_str()); return 1; }
      Stats S;
      s.run(f, S);
      fclose(f);
      if (pcstat) {
        std::vector<std::pair<u64, u64>> v;
        for (auto &kv : s.pcs) v.push_back({kv.second.second, kv.first});
        std::sort(v.rbegin(), v.rend());
        for (int k = 0; k < 15 && k < (int)v.size(); k++)
          printf("  误预测 %8llu / %8llu  pc=%llx\n", (unsigned long long)v[k].first,
                 (unsigned long long)s.pcs[v[k].second].first, (unsigned long long)v[k].second);
      }
      tot.add(S);
      std::string base = fn.substr(fn.find_last_of('/') + 1);
      double all = S.mpki(S.cond_mis + S.ind_mis + S.ret_mis);
      printf("%-10s %-24s %9.2f %8.3f %8.3f %8.3f %8.3f %8.1f %8.1f %8.2f %8.1f\n", p.c_str(), base.c_str(), S.inst / 1e6,
             S.mpki(S.cond_mis), S.mpki(S.ind_mis), S.mpki(S.ret_mis), all, S.mpki(S.bubbles), S.mpki(S.lost),
             (double)S.inst / (S.blocks + S.lost), kb);
      if (verbose) printf("           条件 %llu (错 %llu)  间接 %llu (错 %llu)  返回 %llu (错 %llu)  块 %llu\n",
                          (unsigned long long)S.cond, (unsigned long long)S.cond_mis, (unsigned long long)S.ind,
                          (unsigned long long)S.ind_mis, (unsigned long long)S.ret, (unsigned long long)S.ret_mis,
                          (unsigned long long)S.blocks);
    }
    printf("%-10s %-24s %9.2f %8.3f %8.3f %8.3f %8.3f %8.1f %8.1f %8.2f %8.1f\n\n", p.c_str(), "== 合计", tot.inst / 1e6,
           tot.mpki(tot.cond_mis), tot.mpki(tot.ind_mis), tot.mpki(tot.ret_mis),
           tot.mpki(tot.cond_mis + tot.ind_mis + tot.ret_mis), tot.mpki(tot.bubbles), tot.mpki(tot.lost),
           (double)tot.inst / (tot.blocks + tot.lost), kb);
  }
  return 0;
}
