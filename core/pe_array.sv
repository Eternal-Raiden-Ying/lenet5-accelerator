// ═══════════════════════════════════════════════════════════════════════════════
// PE Array — 8×8 MAC (8 pix × 8 out_ch), Output Stationary, 双模 8x8/1x64
//
// 【v10 重构】 mac_en/clear_accum 打拍移至 compute_core, pe_array 用 _sync 信号
// ═══════════════════════════════════════════════════════════════════════════════
`default_nettype none

import wrapper_pkg::*;
import core_pkg::*;

module pe_array (
    input  wire        clk,
    input  wire        rst_n,
    input  desc_cfg_t   cfg,                // 使用 mode, keep_accum
    input  wire  [3:0] cnt_kx,
    input  wire  [7:0] cnt_ox,             // 来自 FSM: 当前输出列 (pe_array 自算 mux_shift)
    input  wire [63:0] im_rd_data_lo,      // 来自 im_sram (1 拍 SRAM 延迟)
    input  wire [63:0] im_rd_data_hi,
    input  wire  [7:0] pp_valid,           // 来自 im_agu: 坐标边界掩码
    input  wire [511:0] wt_rd_data,        // 来自 weight_sram (1 拍 SRAM 延迟)
    input  wire        mac_en_sync,        // 来自 compute_core: mac_en 打 2 拍
    input  wire        clear_accum_sync,   // 来自 compute_core: clear_accum 打 2 拍
    output logic [2047:0] pe_output        // 组合逻辑直出 (= acc 展开), FIFO 在 flush 周期锁存
);

    // 8×8 accumulators (唯一的 2048b 寄存器组)
    logic signed [31:0] acc [0:7][0:7];

    // pe_output: 纯组合 assign, 实时反映 acc 值
    //   FSM 保证 flush_strobe=1 时 mac_en_sync=0 (flush 周期无 MAC 更新),
    //   FIFO 在 flush_strobe posedge 采样 pe_output = 稳定的最终累加结果。
    always_comb begin
        for (int p_ox = 0; p_ox < 8; p_ox = p_ox + 1)
            for (int p_oc = 0; p_oc < 8; p_oc = p_oc + 1)
                pe_output[p_ox*256 + p_oc*32 +: 32] = acc[p_ox][p_oc];
    end

    // ═══════════════════════════════════════════════════════════════
    // 控制信号 1 拍延迟 (对齐 SRAM 读延迟)
    //   cnt_kx@N, cnt_ox@N 在 FSM 更新, im_rd_req@N → SRAM 数据@N+1 到达
    //   kx_d1=cnt_kx@N, mux_shift_d1 由 pe_array 组合自算, 在 N+1 拍用于 MUX
    // ═══════════════════════════════════════════════════════════════
    logic [3:0] kx_d1;
    logic [6:0] mux_shift_d1;
    logic [7:0] pp_valid_d1;
    logic       mode_d1;

    // ── mux_shift 组合计算 (原 im_agu 逻辑, 现移至 pe_array 内部) ──
    logic signed [9:0] base_x;
    logic [6:0] mux_shift_comb;
    assign base_x = $signed({1'b0, cnt_ox}) * $signed({6'b0, cfg.stride_w})
                  - $signed({6'b0, cfg.pad_w});
    assign mux_shift_comb = {4'd0, (base_x[2:0] + cnt_kx[2:0]) & 3'd7} * 7'd8;

    always_ff @(posedge clk) begin
        kx_d1        <= cnt_kx;
        mux_shift_d1 <= mux_shift_comb;
        pp_valid_d1  <= pp_valid;
        mode_d1      <= cfg.mode;
    end

    // ═══════════════════════════════════════════════════════════════
    // 组合 MUX (使用 d1 延迟后的控制信号 + 刚到达的 SRAM 数据)
    //
    //   IM: 128b 窗口右移, mux_shift_d1 = ((base_x[2:0]+kx)&7)*8  (pe_array 自算)
    //   WT: MODE_8x8 → 按 kx 从 512b 行中选 8 个 weight
    //       MODE_1x64 → 全 64 weight 直通 (无 kx 选择)
    // ═══════════════════════════════════════════════════════════════

    // ── IM 窗口 + MUX ──
    logic [127:0] im_window;
    assign im_window = {im_rd_data_hi, im_rd_data_lo};

    logic [63:0] pp_data_comb;
    assign pp_data_comb = im_window >> mux_shift_d1;   // pe_array 内部组合自算

    // ── WT 选组 (MODE_8x8) ──
    logic [7:0] wt_sel_comb [0:7];
    genvar gi;
    generate
        for (gi = 0; gi < 8; gi = gi + 1) begin : gw
            assign wt_sel_comb[gi] = wt_rd_data[kx_d1*64 + gi*8 +: 8];
        end
    endgenerate

    // ═══════════════════════════════════════════════════════════════
    // 寄存器打拍: 组合 MUX 结果 → 寄存器 → MAC
    //   IM: 64b pixel 数据
    //   WT: 8×8b (MODE_8x8) 或 512b 全通 (MODE_1x64)
    //   valid / mode 一并打拍对齐
    // ═══════════════════════════════════════════════════════════════
    logic [63:0] pp_data;
    logic  [7:0] wt_8x8 [0:7];
    logic [511:0] wt_data_full;       // MODE_1x64: 全 64 weight
    logic  [7:0] pp_valid_reg;
    logic        mode_reg;

    always_ff @(posedge clk) begin
        pp_data      <= pp_data_comb;
        pp_valid_reg <= pp_valid_d1;
        mode_reg     <= mode_d1;
        wt_data_full <= wt_rd_data;           // MODE_1x64: 全量寄存
        for (int i = 0; i < 8; i++)
            wt_8x8[i] <= wt_sel_comb[i];      // MODE_8x8: 选组后寄存
    end

    // 解包 pixel (从寄存后的 pp_data)
    logic [7:0] pixel [0:7];
    assign pixel[0] = pp_data[7:0];
    assign pixel[1] = pp_data[15:8];
    assign pixel[2] = pp_data[23:16];
    assign pixel[3] = pp_data[31:24];
    assign pixel[4] = pp_data[39:32];
    assign pixel[5] = pp_data[47:40];
    assign pixel[6] = pp_data[55:48];
    assign pixel[7] = pp_data[63:56];

    // ═══════════════════════════════════════════════════════════════
    // MAC 累加器 (mac_en_sync + 寄存器输出数据)
    // ═══════════════════════════════════════════════════════════════
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int p_ox = 0; p_ox < 8; p_ox = p_ox + 1)
                for (int p_oc = 0; p_oc < 8; p_oc = p_oc + 1)
                    acc[p_ox][p_oc] <= 0;
        end else if (mac_en_sync) begin
            if (mode_reg == MODE_8x8) begin
                for (int p_ox = 0; p_ox < 8; p_ox = p_ox + 1) begin
                    if (pp_valid_reg[p_ox]) begin
                        for (int p_oc = 0; p_oc < 8; p_oc = p_oc + 1) begin
                            logic signed [31:0] prod;
                            prod = $signed({1'b0, pixel[p_ox]})
                                 * $signed(wt_8x8[p_oc]);
                            // clear_accum_sync → 重新开始累加 (= prod), 否则累加 (+= prod)
                            acc[p_ox][p_oc] <= (clear_accum_sync && !cfg.keep_accum)
                                ? prod
                                : acc[p_ox][p_oc] + prod;
                        end
                    end
                end
            end else begin  // MODE_1x64
                if (pp_valid_reg[0]) begin
                    for (int p_oc = 0; p_oc < 64; p_oc = p_oc + 1) begin
                        logic signed [31:0] prod;
                        prod = $signed({1'b0, pixel[0]})
                             * $signed(wt_data_full[p_oc*8 +: 8]);
                        acc[p_oc/8][p_oc%8] <= (clear_accum_sync && !cfg.keep_accum)
                            ? prod
                            : acc[p_oc/8][p_oc%8] + prod;
                    end
                end
            end
        end else if (clear_accum_sync && !cfg.keep_accum) begin
            // mac_en_sync=0 时单独清零 (层结束时用)
            for (int p_ox = 0; p_ox < 8; p_ox = p_ox + 1)
                for (int p_oc = 0; p_oc < 8; p_oc = p_oc + 1)
                    acc[p_ox][p_oc] <= 0;
        end
    end

endmodule

`default_nettype wire
