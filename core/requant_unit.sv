// ═══════════════════════════════════════════════════════════════════════════════
// Requant — ×4 TDM: clamp(round(y×M>>shift)+zp_y, 0, 255), 3-stage pipe
// ═══════════════════════════════════════════════════════════════════════════════
//
// 【v5: cfg 统一 packed struct, 去 mode/cfg_zp_y/cfg_out_ch 散口】
//
//   rq_ocg: ocg_changed_sync 时递增, 对齐 flush_d1 时序
//   fg_ch_group: (发送req时，需要比fg_valid早一拍) 递增
//   rq_rd_req = fg_will_valid || (fg_valid && (fg_ch_group != 0))
//
//   时序 (flush_d2@T+2 = fg_will_valid):
//     T+1: ocg_changed_sync=1 → rq_ocg++
//     T+2: will_valid=1 → req(ch0)
//     T+3: fg_valid=1, fg=chunk0, rq_data=ch0
//          s1 ← (chunk0, ch0) ✓
//          fg_valid=1 → req(ch1), fg_ch_group++
//     T+4: fg=chunk1, rq_data=ch1
//          s1 ← (chunk1, ch1) ✓
//
// ═══════════════════════════════════════════════════════════════════════════════
`default_nettype none

import wrapper_pkg::*;

module requant_unit (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        layer_start,
    input  desc_cfg_t   cfg,
    input  wire [127:0] fg_data,
    input  wire        fg_valid,
    input  wire        fg_will_valid,
    input  wire        ocg_changed_sync,
    input  wire [287:0] rq_rd_data,
    output logic        rq_rd_req,
    output logic  [3:0] rq_rd_addr,
    output logic [31:0] rq_data,
    output logic        rq_valid
);

    logic [4:0] cur_rq_ocg, prev_rq_ocg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_rq_ocg <= 0; prev_rq_ocg <= 0;
        end else if (layer_start) begin
            cur_rq_ocg <= 0; prev_rq_ocg <= 0;
        end else if (ocg_changed_sync) begin
            cur_rq_ocg <= cur_rq_ocg + 5'd1;
            prev_rq_ocg <= cur_rq_ocg;
        end
    end

    logic [3:0] fg_ch_group;
    logic [3:0] ch_group_max;
    logic       req_trig;

    assign req_trig = fg_will_valid || (fg_valid && (fg_ch_group != 0));

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fg_ch_group  <= 0;
            ch_group_max <= 4'd1;
        end else if (layer_start) begin
            fg_ch_group  <= 0;
            // temporarily for simplifying logic, needs to consider cfg.mode, cfg.out_ch and cnt.ocg
            // while it should use ch group max to filter those uneffective channel, it could not be done,
            // because fifo's output is (W,C), which means uneffective channel scatter in the total output
            // thus, we locked the ch group max to 16, equals to a counter, it doesn't influence the logic at all
            ch_group_max <= 4'hf; 
        end else if (req_trig) begin
            fg_ch_group <= (fg_ch_group == ch_group_max) ? 4'd0
                                                        : fg_ch_group + 4'd1;
        end
    end

    assign rq_rd_req  = req_trig;
    assign rq_rd_addr = cfg.mode ? fg_ch_group
                             : {prev_rq_ocg[2:0], fg_ch_group[0]};

    // Stage 1: MUL — explicit per-channel (xsim generate bug workaround)
    logic [127:0] s1_data; logic s1_valid;
    logic [287:0] s1_rq;
    logic signed [63:0] s1_prod0, s1_prod1, s1_prod2, s1_prod3;
    logic signed [31:0] s1_y0, s1_y1, s1_y2, s1_y3;
    logic signed [31:0] s1_M0, s1_M1, s1_M2, s1_M3;
    assign s1_y0 = s1_data[31:0];
    assign s1_y1 = s1_data[63:32];
    assign s1_y2 = s1_data[95:64];
    assign s1_y3 = s1_data[127:96];
    assign s1_M0 = s1_rq[31:0];
    assign s1_M1 = s1_rq[103:72];
    assign s1_M2 = s1_rq[175:144];
    assign s1_M3 = s1_rq[247:216];
    assign s1_prod0 = s1_y0 * s1_M0;
    assign s1_prod1 = s1_y1 * s1_M1;
    assign s1_prod2 = s1_y2 * s1_M2;
    assign s1_prod3 = s1_y3 * s1_M3;

    always_ff @(posedge clk) begin
        s1_data  <= fg_data;
        s1_valid <= fg_valid;
        s1_rq    <= rq_rd_data;
    end

    // Stage 2: ROUND + SHR + ADD b_fused — explicit per-channel
    logic [127:0] s2_y; logic s2_valid; logic [7:0] s2_zp;
    logic  [7:0] s1_shift0, s1_shift1, s1_shift2, s1_shift3;
    logic signed [31:0] s1_b0, s1_b1, s1_b2, s1_b3;
    logic signed [63:0] s1_rounded0, s1_rounded1, s1_rounded2, s1_rounded3;
    logic signed [31:0] s2_part0, s2_part1, s2_part2, s2_part3;
    assign s1_shift0 = s1_rq[39:32];
    assign s1_shift1 = s1_rq[111:104];
    assign s1_shift2 = s1_rq[183:176];
    assign s1_shift3 = s1_rq[255:248];
    assign s1_b0     = s1_rq[71:40];
    assign s1_b1     = s1_rq[143:112];
    assign s1_b2     = s1_rq[215:184];
    assign s1_b3     = s1_rq[287:256];
    assign s1_rounded0 = (s1_shift0 > 0) ? s1_prod0 + (64'sd1 << (s1_shift0 - 1)) : s1_prod0;
    assign s1_rounded1 = (s1_shift1 > 0) ? s1_prod1 + (64'sd1 << (s1_shift1 - 1)) : s1_prod1;
    assign s1_rounded2 = (s1_shift2 > 0) ? s1_prod2 + (64'sd1 << (s1_shift2 - 1)) : s1_prod2;
    assign s1_rounded3 = (s1_shift3 > 0) ? s1_prod3 + (64'sd1 << (s1_shift3 - 1)) : s1_prod3;
    assign s2_part0 = (s1_rounded0 >>> s1_shift0) + s1_b0;
    assign s2_part1 = (s1_rounded1 >>> s1_shift1) + s1_b1;
    assign s2_part2 = (s1_rounded2 >>> s1_shift2) + s1_b2;
    assign s2_part3 = (s1_rounded3 >>> s1_shift3) + s1_b3;

    always_ff @(posedge clk) begin
        s2_y[31:0]   <= s2_part0;
        s2_y[63:32]  <= s2_part1;
        s2_y[95:64]  <= s2_part2;
        s2_y[127:96] <= s2_part3;
        s2_valid <= s1_valid; s2_zp <= cfg.zp_y;
    end

    // Stage 3: ADD zp_y + CLAMP
    always_ff @(posedge clk) begin
        if (s2_valid) begin
            rq_data <= {clamp8(s2_y[127:96] + s2_zp),
                        clamp8(s2_y[95:64]  + s2_zp),
                        clamp8(s2_y[63:32]  + s2_zp),
                        clamp8(s2_y[31:0]   + s2_zp)};
            rq_valid <= 1;
        end else rq_valid <= 0;
    end

    function [7:0] clamp8;
        input [31:0] v;
        clamp8 = (v > 32'd255) ? 8'd255 : ((v[31]) ? 8'd0 : v[7:0]);
    endfunction

endmodule

`default_nettype wire
