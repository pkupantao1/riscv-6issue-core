/* 仅用于在 libcosim.so 还没编好时检查 VCS 能否编译 RTL + TB */
long long tb_mem_read64(long long a) { (void)a; return 0; }
void tb_mem_write64(long long a, long long d, int m) { (void)a; (void)d; (void)m; }
int tb_mem_valid(long long a) { (void)a; return 1; }
int tb_tohost(long long *v) { *v = 0; return 0; }
int cosim_init(const char *e, const char *i, long long s, int v, const char *t) { return 0; }
int cosim_commit(long long pc, int insn, int t, int rd, long long w) { return 0; }
int cosim_trap(int i, int c, long long e, long long t) { return 0; }
long long cosim_ncommit(void) { return 0; }
void cosim_vabits(int n) { (void)n; }
void cosim_trap_tgt(int t) { (void)t; }
void cosim_finish(void) {}
long long tb_tohost_addr(void) { return 0x80001000; }
