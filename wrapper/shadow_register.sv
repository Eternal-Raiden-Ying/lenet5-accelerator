// ═══════════════════════════════════════════════════════════════════════════════
// shadow_register — 512-bit descriptor, shift-reg write, bit-slice read
// ═══════════════════════════════════════════════════════════════════════════════
// 写侧: 移位寄存器, AHB INCR16 顺序读回 16 words, Word 0 先到 → 16 拍后自然就位.
// 读侧: bit-slice assign (xsim 嵌套 packed struct cast 有 bug, 暂用传统方式).
//
// Bit 布局 (desc_full_t 定义见 top/lenet5_pkg.sv):
//   Word 0-5:  desc_cfg_t (192b)
//   Word 6-14: desc_dma_t (288b)
//   Word 15:   reserved    (32b)
// ═══════════════════════════════════════════════════════════════════════════════
`default_nettype none

import wrapper_pkg::*;

module shadow_register (
    input  wire        clk, rst_n,
    input  wire        sr_wr_en,
    input  wire [31:0] sr_wr_data,
    output desc_cfg_t  cfg_out,
    output desc_dma_t  dma_out
);

    // ── 写侧: 移位寄存器 ──
    logic [511:0] desc_raw;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)  desc_raw <= 512'd0;
        else if (sr_wr_en) desc_raw <= {sr_wr_data, desc_raw[511:32]};
    end

    // ── 读侧: bit-slice assign ──
    // (以下为 desc_full_t packed struct cast 的等效展开.
    //  xsim 2024.2 嵌套 packed struct cast 有 bit 错位 bug,
    //  故保留逐 bit assign. 未来 Vivado 修复后可切回 3 行 cast 版本.)

    // ── Word 0 [31:0]: in_h, in_w, out_ch, in_ch ──
    assign cfg_out.in_ch  = desc_raw[7:0];
    assign cfg_out.out_ch = desc_raw[15:8];
    assign cfg_out.in_w   = desc_raw[23:16];
    assign cfg_out.in_h   = desc_raw[31:24];

    // ── Word 1 [63:32]: stride_h, stride_w, kernel_h, kernel_w, out_h, out_w ──
    assign cfg_out.out_w    = desc_raw[39:32];
    assign cfg_out.out_h    = desc_raw[47:40];
    assign cfg_out.kernel_w = desc_raw[51:48];
    assign cfg_out.kernel_h = desc_raw[55:52];
    assign cfg_out.stride_w = desc_raw[59:56];
    assign cfg_out.stride_h = desc_raw[63:60];

    // ── Word 2 [95:64]: im_total_writes, zp_y, pad_h, pad_w ──
    assign cfg_out.pad_w        = desc_raw[67:64];
    assign cfg_out.pad_h        = desc_raw[71:68];
    assign cfg_out.zp_y         = desc_raw[79:72];
    assign cfg_out.im_total_writes = desc_raw[95:80];

    // ── Word 3 [127:96]: reserved_w1, im_read_base, im_ch_stride, im_row_stride ──
    assign cfg_out.im_row_stride = desc_raw[103:96];
    assign cfg_out.im_ch_stride  = desc_raw[113:104];
    assign cfg_out.im_read_base  = desc_raw[122:114];
    assign cfg_out.reserved_w1   = desc_raw[127:123];

    // ── Word 4 [159:128]: reserved_w2, im_write_base, out_ch_stride, out_row_stride ──
    assign cfg_out.out_row_stride = desc_raw[135:128];
    assign cfg_out.out_ch_stride  = desc_raw[145:136];
    assign cfg_out.im_write_base  = desc_raw[154:146];
    assign cfg_out.reserved_w2    = desc_raw[159:155];

    // ── Word 5 [191:160]: reserved_w3, ct_mode, disable_flush, keep_accum,
    //                       pool_bypass, mode, wt_ocg_stride, wt_row_stride, wt_ch_stride ──
    assign cfg_out.wt_ch_stride    = desc_raw[167:160];
    assign cfg_out.wt_row_stride   = desc_raw[175:168];
    assign cfg_out.wt_ocg_stride   = desc_raw[185:176];
    assign cfg_out.mode            = desc_raw[186];
    assign cfg_out.pool_bypass     = desc_raw[187];
    assign cfg_out.keep_accum      = desc_raw[188];
    assign cfg_out.disable_flush   = desc_raw[189];
    assign cfg_out.ct_mode         = desc_raw[190];
    assign cfg_out.reserved_w3     = desc_raw[191];

    // ── Word 6~14: desc_dma_t ──
    assign dma_out.weight_bytes    = desc_raw[223:192];
    assign dma_out.requant_bytes   = desc_raw[255:224];
    assign dma_out.weight_ddr_ptr  = desc_raw[287:256];
    assign dma_out.requant_ddr_ptr = desc_raw[319:288];
    assign dma_out.feature_ddr_ptr = desc_raw[351:320];
    assign dma_out.result_ddr_ptr  = desc_raw[383:352];
    assign dma_out.feature_bytes   = desc_raw[415:384];
    assign dma_out.next_desc       = desc_raw[447:416];
    assign dma_out.is_last         = desc_raw[448];
    assign dma_out.reserved_dma    = desc_raw[479:449];

    // ═══════════════════════════════════════════════════════════════════
    // 理想版本 (xsim 修复嵌套 packed struct bug 后启用):
    //   desc_full_t desc_full;
    //   assign desc_full = desc_full_t'(desc_raw);
    //   assign cfg_out   = desc_full.cfg;
    //   assign dma_out   = desc_full.dma;
    // ═══════════════════════════════════════════════════════════════════

endmodule

`default_nettype wire
