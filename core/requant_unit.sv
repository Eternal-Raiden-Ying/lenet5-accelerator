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
            ch_group_max <= cfg.mode ? ((cfg.out_ch + 9'd3) >> 2) - 4'd1 : 4'd1;
        end else if (req_trig) begin
            fg_ch_group <= (fg_ch_group == ch_group_max) ? 4'd0
                                                        : fg_ch_group + 4'd1;
        end
    end

    assign rq_rd_req  = req_trig;
    assign rq_rd_addr = cfg.mode ? fg_ch_group
                             : {prev_rq_ocg[2:0], fg_ch_group[0]};

    // Stage 1: MUL
    logic [127:0] s1_data; logic s1_valid;
    logic [287:0] s1_rq;
    logic signed [31:0] s1_y [0:3];
    logic signed [31:0] s1_M  [0:3];
    logic signed [63:0] s1_prod [0:3];
    genvar gi;
    generate for (gi = 0; gi < 4; gi = gi + 1) begin : gs1
        assign s1_y[gi]    = s1_data[gi*32 +: 32];
        assign s1_M[gi]    = s1_rq[gi*72 +: 32];
        assign s1_prod[gi] = s1_y[gi] * s1_M[gi];
    end endgenerate

    always_ff @(posedge clk) begin
        s1_data  <= fg_data;
        s1_valid <= fg_valid;
        s1_rq    <= rq_rd_data;
    end

    // Stage 2: ROUND + SHR + ADD b_fused
    logic [127:0] s2_y; logic s2_valid; logic [7:0] s2_zp;
    logic  [7:0] s1_shift [0:3];
    logic signed [31:0] s1_b [0:3];
    logic signed [63:0] s1_rounded [0:3];
    logic signed [31:0] s2_part [0:3];
    generate for (gi = 0; gi < 4; gi = gi + 1) begin : gs2
        assign s1_shift[gi] = s1_rq[gi*72 + 32 +: 8];
        assign s1_b[gi]     = s1_rq[gi*72 + 40 +: 32];
        assign s1_rounded[gi] = (s1_shift[gi] > 0)
            ? s1_prod[gi] + (64'sd1 << (s1_shift[gi] - 1))
            : s1_prod[gi];
        assign s2_part[gi] = s1_rounded[gi] >>> s1_shift[gi] + s1_b[gi];
    end endgenerate

    always_ff @(posedge clk) begin
        for (int i = 0; i < 4; i++) s2_y[i*32 +: 32] <= s2_part[i];
        s2_valid <= s1_valid; s2_zp <= cfg.zp_y;
    end

    // Stage 3: ADD zp_y + CLAMP
    always_ff @(posedge clk) begin
        if (s2_valid) begin
            rq_data[7:0]   <= clamp8(s2_y[31:0]   + s2_zp);
            rq_data[15:8]  <= clamp8(s2_y[63:32]  + s2_zp);
            rq_data[23:16] <= clamp8(s2_y[95:64]  + s2_zp);
            rq_data[31:24] <= clamp8(s2_y[127:96] + s2_zp);
            rq_valid <= 1;
        end else rq_valid <= 0;
    end

    function [7:0] clamp8;
        input [31:0] v;
        clamp8 = (v > 32'd255) ? 8'd255 : ((v[31]) ? 8'd0 : v[7:0]);
    endfunction

endmodule

`default_nettype wire
