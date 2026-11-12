# OOO6 对 cvfpu 的本地补丁

基线：openhwgroup/cvfpu master @ 77811635 (2026-09-14)，common_cells @ 6aeee85，fpu_div_sqrt_mvp @ 86e1f55。
全部文件已转为 LF 换行 (Windows 克隆时被转成 CRLF，VCS 2018 解析带反斜杠续行的字符串会失败)。

| 文件 | 改动 | 原因 |
|---|---|---|
| src/common_cells/src/cf_math_pkg.sv | `idx_width` 用循环代替 `$clog2(num_idx)` | DC 2018 VER-720：不支持对非常量表达式求 $clog2 |
| src/fpnew_divsqrt_th_64_multi.sv | `FFLARNC` 的清零条件 `x \| flush_i` 先赋给中间信号 | 宏把清零表达式拼进 `synopsys sync_set_reset` 注释，带运算符时 DC 2018 语法错误 |
| src/fpnew_fma.sv | 数据流水寄存器 `FFL(q, d, reg_ena, rst)` 改为无使能无复位的 `O6_PIPE_REG(q, d)`，有效位 `FFLARNC` 不变 | 带握手使能 / 复位的寄存器 DC 重定时推不动 (RTDC-139 "not a pipeline")，FMA 加到 5 级流水仍有一级 3 ns。OOO6 只在 U6/U7 用 FMA，两者从不反压，去掉使能不改变行为 |
