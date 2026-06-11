// ═══════════════════════════════════════════════════════════════════════════════
// Corner-Turn — Dual-mode: spatial transpose (CT_SPATIAL) / time gearbox (CT_GEARBOX)
// ═══════════════════════════════════════════════════════════════════════════════
// ct_mode=CT_SPATIAL: 双缓冲 4×8 矩阵 → scatter write CHW planar + ready 反压
// ct_mode=CT_GEARBOX: 32b 直通 → sequential write, 不拼装
//
// 【IM 写地址 (空间转置)】
//   addr = im_write_base
//        + scatter_ch × out_ch_stride       ← 通道组偏移 (编译器预填)
//        + spatial_oy × out_row_stride      ← 行偏移 (编译器预填)
//        + spatial_oxg                       ← 行内 64b word 偏移
//   spatial_oy / spatial_oxg corner_turn 内部自增, layer_start 复位
//
// 【双缓冲设计】
//   两份 4×8 矩阵 matrix_buf[0] 和 matrix_buf[1]:
//     - fill_bank: 正在接收 pool 输出的矩阵
//     - scatter_bank: 正在 scatter-write 到 IM SRAM 的矩阵(已填满的)
//   填满一份(fill_w_idx==7)→ 翻转 fill_bank,启动 scatter(4 拍,每通道一拍)
//   scatter 4 拍 ≤ fill 容量(8 entries), 无需反压。
// ═══════════════════════════════════════════════════════════════════════════════
`default_nettype none

import wrapper_pkg::*;

module corner_turn (
    input  wire        clk,
    input  wire        rst_n,
    input  desc_cfg_t   cfg,                // 使用 ct_mode,out_h,out_ch_stride,out_row_stride,im_write_base
    input  wire        layer_start,        // 层启动脉冲: sram_write_cnt + spatial 计数器清零
    input  wire [31:0] pp_data,
    input  wire        pp_valid,
    output logic        im_wr_en,
    output logic  [8:0] im_wr_addr,
    output logic [63:0] im_wr_data,
    output logic [15:0] sram_write_cnt
);

    // ═══════════════════════════════════════════════════════════════
    // Spatial Transpose (pool_bypass=0) — 双缓冲矩阵
    // ═══════════════════════════════════════════════════════════════
    // 2 banks × 4 ch_groups × 8 spatial positions
    logic [7:0] matrix_buf [0:1][0:3][0:7];
    logic       fill_bank;         // 0 or 1, 当前接收 pool 输出的矩阵
    logic [2:0] fill_w_idx;        // 0..7, fill_bank 的写入列指针
    logic       scatter_active;    // 1 = scatter-write 进行中
    logic [1:0] scatter_ch;        // 0..3, 当前 scatter 输出的通道组
    logic       scatter_bank;      // 正在 scatter 的矩阵(在 scatter 启动时锁存)

    // ── IM 写地址 spatial 计数器 (内部自增, layer_start 清零) ──
    logic [7:0] spatial_oy;        // 当前输出行 (0..out_h-1)
    logic [7:0] spatial_oxg;       // 当前行内 64b word 偏移 (0..out_row_stride-1)

    // ═══════════════════════════════════════════════════════════════
    // Time Gearbox (ct_mode=CT_GEARBOX): 32b→64b 拼装
    //   half_full=0: 暂存 pp_data → gearbox_reg[31:0]
    //   half_full=1: {pp_data, gearbox_reg} → 64b write
    // ═══════════════════════════════════════════════════════════════
    logic [31:0] gearbox_reg;
    logic        half_full;        // 0=等待 lo32, 1=lo32 已填,等待 hi32

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            im_wr_en <= 0; im_wr_addr <= 0; im_wr_data <= 0;
            sram_write_cnt <= 0;
            fill_bank <= 0; fill_w_idx <= 0;
            scatter_active <= 0; scatter_ch <= 0; scatter_bank <= 0;
            spatial_oy <= 0; spatial_oxg <= 0;
            gearbox_reg <= 0; half_full <= 0;
            for (int b = 0; b < 2; b++)
                for (int i = 0; i < 4; i++)
                    for (int j = 0; j < 8; j++)
                        matrix_buf[b][i][j] <= 0;
        end else begin
            im_wr_en <= 0;

            // 每层启动时清零写计数 + spatial 计数器
            if (layer_start) begin
                sram_write_cnt <= 0;
                spatial_oy  <= 0;
                spatial_oxg <= 0;
            end

            if (cfg.ct_mode) begin
                // ── Time Gearbox (32b→64b 拼装) ──
                if (pp_valid) begin
                    if (!half_full) begin
                        gearbox_reg <= pp_data;
                        half_full <= 1;
                    end else begin
                        im_wr_en   <= 1;
                        im_wr_data <= {pp_data, gearbox_reg};
                        im_wr_addr <= cfg.im_write_base + sram_write_cnt[8:0];
                        sram_write_cnt <= sram_write_cnt + 1;
                        half_full <= 0;
                    end
                end

            end else begin
                // ── Spatial Transpose (双缓冲) ──

                // Scatter-write: 每拍输出一个 ch_group 的 8 spatial 值(64b)
                if (scatter_active) begin
                    im_wr_en <= 1;
                    im_wr_data <= {matrix_buf[scatter_bank][scatter_ch][7],
                                   matrix_buf[scatter_bank][scatter_ch][6],
                                   matrix_buf[scatter_bank][scatter_ch][5],
                                   matrix_buf[scatter_bank][scatter_ch][4],
                                   matrix_buf[scatter_bank][scatter_ch][3],
                                   matrix_buf[scatter_bank][scatter_ch][2],
                                   matrix_buf[scatter_bank][scatter_ch][1],
                                   matrix_buf[scatter_bank][scatter_ch][0]};
                    // IM addr = base + ch×ch_stride + oy×row_stride + oxg
                    im_wr_addr <= cfg.im_write_base
                                + scatter_ch * cfg.out_ch_stride
                                + spatial_oy * cfg.out_row_stride
                                + spatial_oxg;
                    sram_write_cnt <= sram_write_cnt + 1;

                    if (scatter_ch == 2'd3) begin
                        // Scatter 4 拍完成 → 推进 spatial 坐标
                        scatter_active <= 0;
                        scatter_ch <= 0;
                        if (spatial_oxg == cfg.out_row_stride - 8'd1) begin
                            spatial_oxg <= 0;
                            if (spatial_oy == cfg.out_h - 8'd1)
                                spatial_oy <= 0;
                            else
                                spatial_oy <= spatial_oy + 8'd1;
                        end else begin
                            spatial_oxg <= spatial_oxg + 8'd1;
                        end
                    end else begin
                        scatter_ch <= scatter_ch + 1;
                    end
                end

                // 接收 pool 输出,填入当前 fill_bank
                if (pp_valid) begin
                    matrix_buf[fill_bank][0][fill_w_idx] <= pp_data[7:0];
                    matrix_buf[fill_bank][1][fill_w_idx] <= pp_data[15:8];
                    matrix_buf[fill_bank][2][fill_w_idx] <= pp_data[23:16];
                    matrix_buf[fill_bank][3][fill_w_idx] <= pp_data[31:24];

                    if (fill_w_idx == 3'd7) begin
                        // 当前矩阵填满 → 翻转 fill_bank,启动 scatter
                        scatter_active <= 1;
                        scatter_bank  <= fill_bank;
                        scatter_ch    <= 0;
                        fill_bank     <= ~fill_bank;
                        fill_w_idx    <= 0;
                    end else begin
                        fill_w_idx <= fill_w_idx + 1;
                    end
                end
            end
        end
    end

endmodule

`default_nettype wire
