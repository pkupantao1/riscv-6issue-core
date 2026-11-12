//==============================================================================
// o6_tlb.sv - 全相联 TLB (Sv39 / Sv48 / Sv57：4K / 2M / 1G / 512G / 256T 页，Svnapot 64K)
//
// 每项：VPN (45 位)、页级 (0..4)、NAPOT 标记 (0 级 64K)、PPN、权限位 (R/W/X/U/A/D)、PBMT，
//       以及"故障项"标记 (PTW 遇到无效 PTE / 访问错误时填入，下次查到就直接报错，
//       否则依赖"缺失 -> 重执行"的路径会无限循环)。
// 三种模式共用：satp 写入 (含改 MODE) 与 sfence.vma 一律全部清空，所以表内只有当前模式的项；
//   Sv39 / Sv48 下 VPN 高位是规范地址的符号扩展，全 45 位比较照样对。
// 4 级 (256T) 页的物理地址低 48 位来自 VA，超出 PA_W 的部分 (VA[47:40]) 非零 -> 访问错误 (查找时判)。
// 不带 ASID / VMID (规范允许多清)；H：每项带 V 标记 (V = 0 按 satp、V = 1 按 vsatp 翻译的项共存，查找比 V)。替换：轮转。
// H 两级：项是 VA -> PA 的合并结果 (页级取两级较小者)，另存 G-stage 叶子的 {D, X, W, R} (查找方按访问类型判 guest page fault)
//   与 GPA：普通 / 最终 GPA 的 guest page fault 项存 GPA 页号 (查找时按页级拼 VA 低位)；隐式 (VS PTE 地址) 的存完整 PTE GPA。
//==============================================================================
`default_nettype none
module o6_tlb
  import o6_pkg::*;
#(
  parameter int N  = 32,
  parameter int NL = 2          // 查找口数
) (
  input  wire                 clk_i,
  input  wire                 rst_ni,
  input  wire                 flush_i,

  input  wire  [VPN_W-1:0]    lk_vpn_i [NL],
  input  wire  [NL-1:0]       lk_v_i,
  output logic [NL-1:0]       lk_hit_o,
  output logic [PA_W-13:0]    lk_ppn_o [NL],     // 已按页级拼好低位 VPN
  output logic [7:0]          lk_perm_o [NL],    // {D, A, G, U, X, W, R, V}
  output logic [1:0]          lk_fault_o [NL],   // 0 = 无，1 = 页错误，2 = 访问错误，3 = guest page fault
  output logic [3:0]          lk_gperm_o [NL],   // G-stage {D, X, W, R}
  output logic [56:0]         lk_gpa_o [NL],     // GPA[58:2] (非隐式时 [9:0] = 0，由查找方填 VA[11:2])
  output logic                lk_gimp_o [NL],    // 隐式 guest page fault (tinst = 0x3000)
  output logic [1:0]          lk_pbmt_o [NL],    // Svpbmt：0 = PMA，1 = NC，2 = IO

  input  wire                 fill_i,
  input  wire                 fill_v_i,
  input  wire  [VPN_W-1:0]    fill_vpn_i,
  input  wire  [2:0]          fill_level_i,
  input  wire                 fill_nap_i,
  input  wire  [PA_W-13:0]    fill_ppn_i,
  input  wire  [7:0]          fill_perm_i,
  input  wire  [1:0]          fill_pbmt_i,
  input  wire  [1:0]          fill_fault_i,
  input  wire  [3:0]          fill_gperm_i,
  input  wire  [56:0]         fill_gpa_i,
  input  wire                 fill_gimp_i
);
  localparam int PW = PA_W - 12;
  logic [N-1:0]     v;
  logic [N-1:0]     tv;           // H：V 标记
  logic [VPN_W-1:0] vpn   [N];
  logic [2:0]       lvl   [N];
  logic             nap   [N];
  logic [PW-1:0]    ppn   [N];
  logic [7:0]       perm  [N];
  logic [1:0]       pbmt  [N];
  logic [1:0]       flt   [N];
  logic [3:0]       gperm [N];
  logic [56:0]      gpa   [N];
  logic             gimp  [N];
  logic [$clog2(N)-1:0] rr;

  // 页内 VPN 位 (来自 VA 的位)
  function automatic logic [35:0] lmask(input logic [2:0] l, input logic n);
    unique case (l)
      3'd4:    return 36'hF_FFFF_FFFF;
      3'd3:    return 36'h0_07FF_FFFF;
      3'd2:    return 36'h0_0003_FFFF;
      3'd1:    return 36'h0_0000_01FF;
      default: return n ? 36'h0_0000_000F : 36'h0;
    endcase
  endfunction

  always_comb begin
    for (int p = 0; p < NL; p++) begin
      lk_hit_o[p] = 1'b0; lk_ppn_o[p] = '0; lk_perm_o[p] = '0; lk_fault_o[p] = '0; lk_pbmt_o[p] = '0;
      lk_gperm_o[p] = '0; lk_gpa_o[p] = '0; lk_gimp_o[p] = 1'b0;
      for (int i = 0; i < N; i++) begin
        logic [35:0] m;
        m = lmask(lvl[i], nap[i]);
        if (v[i] && tv[i] == lk_v_i[p] && (lk_vpn_i[p][VPN_W-1:36] == vpn[i][VPN_W-1:36]) && (((lk_vpn_i[p][35:0] ^ vpn[i][35:0]) & ~m) == '0)) begin
          lk_hit_o[p]   = 1'b1;
          lk_perm_o[p]  = perm[i];
          lk_pbmt_o[p]  = pbmt[i];
          lk_fault_o[p] = (flt[i] == 2'd0 && lvl[i] == 3'd4 && lk_vpn_i[p][35:PW] != '0) ? 2'd2 : flt[i];
          lk_ppn_o[p]   = (ppn[i] & ~m[PW-1:0]) | (lk_vpn_i[p][PW-1:0] & m[PW-1:0]);
          lk_gperm_o[p] = gperm[i];
          lk_gimp_o[p]  = gimp[i];
          lk_gpa_o[p]   = gimp[i] ? gpa[i] : {(gpa[i][56:10] & ~{11'b0, m}) | (lk_vpn_i[p] & {11'b0, m}), 10'b0};
        end
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      v <= '0; rr <= '0;
    end else if (flush_i) begin
      v <= '0;
    end else if (fill_i) begin
      v[rr] <= 1'b1;
      rr <= rr + 1'b1;
    end
  end

  always_ff @(posedge clk_i) begin
    if (fill_i && !flush_i) begin
      vpn[rr]  <= fill_vpn_i;
      tv[rr]   <= fill_v_i;
      lvl[rr]  <= fill_level_i;
      nap[rr]  <= fill_nap_i;
      ppn[rr]  <= fill_ppn_i;
      perm[rr] <= fill_perm_i;
      pbmt[rr] <= fill_pbmt_i;
      flt[rr]  <= fill_fault_i;
      gperm[rr] <= fill_gperm_i;
      gpa[rr]   <= fill_gpa_i;
      gimp[rr]  <= fill_gimp_i;
    end
  end
endmodule
`default_nettype wire
