//==============================================================================
// o6_dtm_jtag.sv - RISC-V JTAG 调试传输模块 (Debug Spec 0.13.2 §6)
//
// TAP：IR 5 位；IDCODE (0x01，复位后默认)、DTMCS (0x10)、DMI (0x11)、MBIST (0x12)、BYPASS (0x1F 及其它)
// MBIST TDR (8 位)：写 bit0 = 启动 (电平，保持到测完再清)；读 {5'b0, fail, done, start}
// DMI：41 位 {addr[6:0], data[31:0], op[1:0]}；Update-DR 发请求，Capture-DR 取结果
//   op 结果：0 成功，2 失败 (粘滞)，3 忙 (上一请求未完成，粘滞，dtmcs.dmireset 清)
// 跨时钟域：TCK 侧翻转 req_tog (载荷保持)，核时钟侧两级同步检出翻转 -> 发一拍 DMI 请求，
//   拿到响应后翻转 ack_tog；TCK 侧两级同步，req_tog == ack_sync 即空闲。
//==============================================================================
`default_nettype none
module o6_dtm_jtag #(
  parameter logic [31:0] IDCODE = 32'h0000_0001 | (32'h06E5 << 1)   // 版本 0，部件号 0，厂商号占位
) (
  // JTAG
  input  wire          tck_i,
  input  wire          tms_i,
  input  wire          tdi_i,
  output logic         tdo_o,
  output logic         tdo_oe_o,
  input  wire          trst_ni,          // TAP 复位 (可接上电复位)

  // DMI (核时钟域)
  input  wire          clk_i,
  input  wire          rst_ni,
  output logic         dmi_req_o,
  output logic [6:0]   dmi_addr_o,
  output logic [31:0]  dmi_wdata_o,
  output logic [1:0]   dmi_op_o,
  input  wire          dmi_resp_i,
  input  wire  [31:0]  dmi_rdata_i,

  // MBIST (核时钟域)
  output logic         mbist_start_o,
  input  wire          mbist_done_i,
  input  wire          mbist_fail_i
);
  //---------------- TAP 状态机
  typedef enum logic [3:0] {
    TLR, RTI, SEL_DR, CAP_DR, SH_DR, EX1_DR, PA_DR, EX2_DR, UPD_DR,
    SEL_IR, CAP_IR, SH_IR, EX1_IR, PA_IR, EX2_IR, UPD_IR
  } tap_e;
  tap_e st;

  always_ff @(posedge tck_i or negedge trst_ni) begin
    if (!trst_ni) st <= TLR;
    else unique case (st)
      TLR:    st <= tms_i ? TLR    : RTI;
      RTI:    st <= tms_i ? SEL_DR : RTI;
      SEL_DR: st <= tms_i ? SEL_IR : CAP_DR;
      CAP_DR: st <= tms_i ? EX1_DR : SH_DR;
      SH_DR:  st <= tms_i ? EX1_DR : SH_DR;
      EX1_DR: st <= tms_i ? UPD_DR : PA_DR;
      PA_DR:  st <= tms_i ? EX2_DR : PA_DR;
      EX2_DR: st <= tms_i ? UPD_DR : SH_DR;
      UPD_DR: st <= tms_i ? SEL_DR : RTI;
      SEL_IR: st <= tms_i ? TLR    : CAP_IR;
      CAP_IR: st <= tms_i ? EX1_IR : SH_IR;
      SH_IR:  st <= tms_i ? EX1_IR : SH_IR;
      EX1_IR: st <= tms_i ? UPD_IR : PA_IR;
      PA_IR:  st <= tms_i ? EX2_IR : PA_IR;
      EX2_IR: st <= tms_i ? UPD_IR : SH_IR;
      UPD_IR: st <= tms_i ? SEL_DR : RTI;
      default: st <= TLR;
    endcase
  end

  localparam logic [4:0] IR_IDCODE = 5'h01, IR_DTMCS = 5'h10, IR_DMI = 5'h11, IR_MBIST = 5'h12, IR_BYPASS = 5'h1F;
  // MBIST：TCK 侧启动位，核时钟侧同步；结果同步回 TCK 侧
  logic       mb_start;
  logic [1:0] mb_s1, mb_s2;            // {fail, done}
  always_ff @(posedge tck_i or negedge trst_ni) begin
    if (!trst_ni) begin mb_s1 <= '0; mb_s2 <= '0; end
    else begin mb_s1 <= {mbist_fail_i, mbist_done_i}; mb_s2 <= mb_s1; end
  end
  logic [4:0]  ir, ir_sh;
  logic [40:0] dr;

  //---------------- 跨时钟域状态
  logic        req_tog, ack_s1, ack_s2;     // TCK 侧
  logic [6:0]  q_addr;
  logic [31:0] q_wdata;
  logic [1:0]  q_op;
  logic        ack_tog;                     // 核时钟侧
  logic [31:0] rsp_data;                    // 核时钟侧，ack 翻转后稳定
  wire         busy = (req_tog != ack_s2);
  logic [1:0]  dmistat;                     // 粘滞错误

  always_ff @(posedge tck_i or negedge trst_ni) begin
    if (!trst_ni) begin ack_s1 <= 1'b0; ack_s2 <= 1'b0; end
    else begin ack_s1 <= ack_tog; ack_s2 <= ack_s1; end
  end

  //---------------- IR / DR
  always_ff @(posedge tck_i or negedge trst_ni) begin
    if (!trst_ni) begin
      ir <= IR_IDCODE; ir_sh <= '0; dr <= '0;
      req_tog <= 1'b0; q_addr <= '0; q_wdata <= '0; q_op <= '0; dmistat <= '0; mb_start <= 1'b0;
    end else begin
      unique case (st)
        TLR:    ir <= IR_IDCODE;
        CAP_IR: ir_sh <= 5'b00001;
        SH_IR:  ir_sh <= {tdi_i, ir_sh[4:1]};
        UPD_IR: ir <= ir_sh;
        CAP_DR: begin
          unique case (ir)
            IR_IDCODE: dr <= {9'b0, IDCODE};
            IR_DTMCS:  dr <= {9'b0, 14'b0, 1'b0, 1'b0, 1'b0, 3'd1 /*idle*/, dmistat, 6'd7 /*abits*/, 4'd1 /*0.13*/};
            IR_DMI:    dr <= {q_addr, rsp_data, (busy ? 2'd3 : dmistat)};
            IR_MBIST:  dr <= {33'b0, 5'b0, mb_s2, mb_start};
            default:   dr <= '0;
          endcase
        end
        SH_DR: begin
          unique case (ir)
            IR_IDCODE, IR_DTMCS: dr <= {9'b0, tdi_i, dr[31:1]};
            IR_MBIST:            dr <= {33'b0, tdi_i, dr[7:1]};
            IR_DMI:              dr <= {tdi_i, dr[40:1]};
            default:             dr <= {40'b0, tdi_i};
          endcase
        end
        UPD_DR: begin
          if (ir == IR_MBIST) mb_start <= dr[0];
          if (ir == IR_DTMCS) begin
            if (dr[16]) dmistat <= '0;                       // dmireset
            if (dr[17]) begin dmistat <= '0; end             // dmihardreset (在途请求的结果丢弃)
          end else if (ir == IR_DMI) begin
            if (busy) begin
              if (dmistat == 2'd0) dmistat <= 2'd3;
            end else if (dmistat == 2'd0 && (dr[1:0] == 2'd1 || dr[1:0] == 2'd2)) begin
              q_addr <= dr[40:34]; q_wdata <= dr[33:2]; q_op <= dr[1:0];
              req_tog <= ~req_tog;
            end
          end
        end
        default: ;
      endcase
    end
  end

  // TDO 在 TCK 下降沿更新
  always_ff @(negedge tck_i or negedge trst_ni) begin
    if (!trst_ni) begin tdo_o <= 1'b0; tdo_oe_o <= 1'b0; end
    else begin
      tdo_oe_o <= (st == SH_IR) || (st == SH_DR);
      if (st == SH_IR) tdo_o <= ir_sh[0];
      else             tdo_o <= dr[0];
    end
  end

  //---------------- 核时钟侧
  logic mbs1, mbs2;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin mbs1 <= 1'b0; mbs2 <= 1'b0; end
    else begin mbs1 <= mb_start; mbs2 <= mbs1; end
  end
  assign mbist_start_o = mbs2;

  logic req_s1, req_s2, req_s3;
  logic wait_rsp;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      req_s1 <= 1'b0; req_s2 <= 1'b0; req_s3 <= 1'b0; ack_tog <= 1'b0; wait_rsp <= 1'b0;
      dmi_req_o <= 1'b0; dmi_addr_o <= '0; dmi_wdata_o <= '0; dmi_op_o <= '0; rsp_data <= '0;
    end else begin
      req_s1 <= req_tog; req_s2 <= req_s1; req_s3 <= req_s2;
      dmi_req_o <= 1'b0;
      if (req_s2 != req_s3) begin
        dmi_req_o <= 1'b1; dmi_addr_o <= q_addr; dmi_wdata_o <= q_wdata; dmi_op_o <= q_op;
        wait_rsp <= 1'b1;
      end
      if (wait_rsp && dmi_resp_i) begin
        rsp_data <= dmi_rdata_i;
        ack_tog  <= ~ack_tog;
        wait_rsp <= 1'b0;
      end
    end
  end
endmodule
