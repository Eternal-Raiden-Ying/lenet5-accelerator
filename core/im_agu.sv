// ═══════════════════════════════════════════════════════════════════════════════
// im_agu — IM SRAM 读地址 + pp_valid (纯组合, wt_agu 风格: 只出地址+valid, 无数据路径)
// ═══════════════════════════════════════════════════════════════════════════════
//   addr    = im_read_base + x_off + y_off + ic*im_ch_stride
//   x_off   = ((ox*stride_w - pad_w) & ~7) >> 3
//   y_off   = (oy*stride_h - pad_h + ky) * im_row_stride
//   pp_valid = y_ok && xi 在边界内
//
//   mux_shift (= ((base_x[2:0]+kx)&7)*8) 移至 pe_array 内部自算
//   MODE_1x64: ox=oy=0, L_kw=in_w, L_kh=in_h → 公式自然退化
// ═══════════════════════════════════════════════════════════════════════════════
`default_nettype none

import wrapper_pkg::*;
import core_pkg::*;

module im_agu (
    input  desc_cfg_t   cfg,
    input  layer_cnt_t  cnt,
    output logic  [8:0] im_rd_addr_lo, im_rd_addr_hi,
    output logic  [7:0] pp_valid
);

    // ── x / y 基址 ──
    logic signed [9:0] base_x, base_y;
    assign base_x = $signed({1'b0, cnt.ox}) * $signed({6'b0, cfg.stride_w})
                  - $signed({6'b0, cfg.pad_w});
    assign base_y = $signed({1'b0, cnt.oy}) * $signed({6'b0, cfg.stride_h})
                  - $signed({6'b0, cfg.pad_h});

    // ── 地址: im_read_base + x_off + y_off + ic*im_ch_stride + ky*im_row_stride ──
    logic [8:0]  x_off;
    logic [15:0] y_off, ch_off, ky_off;
    logic [16:0] addr;

    assign x_off  = ((base_x & (~10'sd7)) >> 3) & 9'h1FF;
    assign y_off  = (base_y + $signed({6'b0, cnt.ky})) * cfg.im_row_stride;
    assign ch_off = cnt.ic * cfg.im_ch_stride;
    assign ky_off = cnt.ky * cfg.im_row_stride;          // 冗余保护: y_off 已含 ky

    assign addr   = {1'b0, cfg.im_read_base}
                  + {8'b0, x_off}
                  + y_off
                  + ch_off;

    assign im_rd_addr_lo = addr[8:0];
    assign im_rd_addr_hi = addr[8:0] + 9'd1;

    // ── Valid 掩码 ──
    logic y_ok;
    assign y_ok = (base_y >= 0) && (base_y + $signed({6'b0, cnt.ky})) < cfg.in_h;

    genvar gi;
    generate for (gi = 0; gi < 8; gi = gi + 1) begin : gv
        logic signed [9:0] xi;
        assign xi = base_x + $signed({6'b0, cnt.kx}) + $signed({7'b0, gi});
        assign pp_valid[gi] = y_ok && (xi >= 0) && (xi < cfg.in_w);
    end endgenerate

endmodule

`default_nettype wire
