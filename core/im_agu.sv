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
    output logic        im_rd_addr_lo_vld, im_rd_addr_hi_vld,
    output logic  [7:0] pp_valid
);

    // ── x / y 基址 ──
    logic signed [9:0] base_x, base_y;
    assign base_x = $signed({1'b0, cnt.ox}) * $signed({6'b0, cfg.stride_w})
                  - $signed({6'b0, cfg.pad_w});
    assign base_y = $signed({1'b0, cnt.oy}) * $signed({6'b0, cfg.stride_h})
                  - $signed({6'b0, cfg.pad_h});

    // ── 【新增】引入动态 x 坐标和修复后的 y 坐标 ──
    logic signed [9:0] cur_base_x;
    logic signed [9:0] cur_y;
    assign cur_base_x = base_x + $signed({6'b0, cnt.kx});
    assign cur_y      = base_y + $signed({6'b0, cnt.ky});

    // ── 偏移量保留原有位宽 ──
    logic [15:0] y_off, ch_off;
    assign y_off  = cur_y * $signed({1'b0, cfg.im_row_stride});
    assign ch_off = cnt.ic * cfg.im_ch_stride;

    // ── 【修改点 2】使用带符号的高位宽进行地址累加 ──
    // 将所有加数安全转换为有符号数 (防止 Verilog 隐式无符号化)
    logic signed [17:0] signed_shift;
    logic signed [9:0]  signed_addr;
    assign signed_shift = $signed(cur_base_x >>> 3)     // 算术右移，保留负数
                       + $signed({2'b0, y_off}) 
                       + $signed({2'b0, ch_off});
    assign signed_addr = $signed(signed_shift[9:0]) + $signed({1'b0, cfg.im_read_base});

    // ── 【修改点 3】截取低 10 位输出 ──
    // bit [9] 是符号位，bit [8:0] 是满血的 512 深度物理地址
    assign im_rd_addr_lo = signed_addr[8:0];
    assign im_rd_addr_hi = signed_addr[8:0] + 9'd1;

    // ── Valid 掩码保持不变 (上一轮修复后的有符号比较版本) ──
    logic y_ok;
    assign y_ok = (cur_y >= 0) && (cur_y < $signed({2'b0, cfg.in_h}));

    logic signed [9:0] x_chunk;
    assign x_chunk = $signed(cur_base_x >>> 3);

    // ── 精准的 VLD 判断 ──
    // lo 块的索引就是 x_chunk。
    // hi 块的索引是 x_chunk + 1。
    // 一个块有效，只需满足：1. 不在左侧负数区 (>=0)  2. 起点没超过图像右边界
    assign im_rd_addr_lo_vld = (x_chunk >= 0)  && ((x_chunk * 8) < $signed({2'b0, cfg.in_w})) && y_ok;
    
    // 注意：hi 块的索引比 lo 块大 1，所以当 x_chunk = -1 时，hi 块的索引是 0，恰好有效！
    assign im_rd_addr_hi_vld = (x_chunk >= -1) && (((x_chunk + 1) * 8) < $signed({2'b0, cfg.in_w})) && y_ok;

    
    genvar gi;
    generate for (gi = 0; gi < 8; gi = gi + 1) begin : gv
        logic signed [9:0] xi;
        assign xi = base_x + $signed({6'b0, cnt.kx}) + $signed({7'b0, gi});
        assign pp_valid[gi] = y_ok && (xi >= 0) && (xi < $signed({2'b0, cfg.in_w}));
    end endgenerate

endmodule

`default_nettype wire
