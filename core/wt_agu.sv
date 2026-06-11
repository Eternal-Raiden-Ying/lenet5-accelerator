// ═══════════════════════════════════════════════════════════════════════════════
// wt_agu — Weight SRAM 读地址生成 (纯组合, 双模 8x8/1x64)
// ═══════════════════════════════════════════════════════════════════════════════
// MODE_8x8 (Conv):
//   行索引 = ocg * in_ch * kh + ic * kh + ky
//   8 个 kx 组打包在同一 512b 行, kx MUX 选组不重读 SRAM
//
// MODE_1x64 (FC):
//   行索引 = ic * wt_ch_stride + ky * wt_row_stride + kx
//   wt_ch_stride = kh * kw (compiler pre-computed, rows per ic)
//   wt_row_stride = kw       (compiler pre-computed, rows per ky)
//   每 (ic,ky,kx) 三元组读一行 64×int8=512b, kx 每次变化需新行
//
// in_ch ≤ 255 (编译器切分保证), max wt_rd_addr ≤ 255 (C_CHECK 校验)
// ═══════════════════════════════════════════════════════════════════════════════
`default_nettype none

import wrapper_pkg::*;
import core_pkg::*;

module wt_agu (
    input  desc_cfg_t   cfg,            // 使用 in_ch, kernel_h, mode, wt_ch_stride, wt_row_stride
    input  layer_cnt_t  cnt,            // 使用 ocg, ic, ky, kx
    output logic  [7:0] wt_rd_addr
);

    // ── MODE_8x8: wt_rd_addr = ocg * in_ch * kh + ic * kh + ky ──
    //   ocg*in_ch*kh 最大: 7*255*5=8925 → 14b, 远小于 16b
    logic [15:0] addr_8x8;
    assign addr_8x8 = cnt.ocg * cfg.wt_ocg_stride
                    + cnt.ic  * cfg.wt_ch_stride
                    + cnt.ky;

    // ── MODE_1x64: wt_rd_addr = ic * wt_ch_stride + ky * wt_row_stride + kx ──
    //   ic*25 + ky*5 + kx 最大: 255*25 + 4*5 + 4 = 6399 → 13b
    //   编译器切分保证 ≤ 8'hFF, C_CHECK 校验
    logic [15:0] addr_1x64;
    assign addr_1x64 = cnt.ocg * cfg.wt_ocg_stride
                     + cnt.ic  * cfg.wt_ch_stride
                     + cnt.ky  * cfg.wt_row_stride
                     + cnt.kx;

    assign wt_rd_addr = (cfg.mode == MODE_1x64) ? addr_1x64[7:0] : addr_8x8[7:0];

endmodule

`default_nettype wire
