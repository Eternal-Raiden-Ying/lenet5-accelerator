// ═══════════════════════════════════════════════════════════════════════════════
// PingPong + 2×2 MaxPool v2 — 4-Bank BRAM, group-major output
// ═══════════════════════════════════════════════════════════════════════════════
//
// 【存储】 2 交替 buffer × 4 bank × 2 行(even/odd) × MAX_W×16b LUTRAM
//   Bank 0: {ch4,ch0}  Bank 1: {ch5,ch1}  Bank 2: {ch6,ch2}  Bank 3: {ch7,ch3}
//
// 【写】 shadow 拼 2 拍 requant(32b→16b/bank), 并发写 4 bank
// 【读】 rd_cnt 驱动 3 级流水: 读BRAM → max4 → 输出, group-major(ch-layer0→1)
// ═══════════════════════════════════════════════════════════════════════════════
`default_nettype none

import wrapper_pkg::*;

module pingpong_pool #(parameter int MAX_W = 32) (
    input  wire        clk,
    input  wire        rst_n,
    input  desc_cfg_t   cfg,                // 使用 pool_bypass, out_w
    input  wire [31:0] rq_data,       // {ch3,ch2,ch1,ch0}
    input  wire        rq_valid,
    output logic [31:0] pp_data,
    output logic        pp_valid
);

    // ═══════════════════════════════════════════════════════════════
    // BRAM: [buf][bank][addr] — 2 buffers × 4 banks × 2 rows
    //   每 entry 16b = {ch_hi(15:8), ch_lo(7:0)}
    //   even 行: addr = pixel_col
    //   odd  行: addr = pixel_col
    // ═══════════════════════════════════════════════════════════════
    logic [15:0] bank_even [0:1][0:3][0:MAX_W-1];
    logic [15:0] bank_odd  [0:1][0:3][0:MAX_W-1];

    // ═══════════════════════════════════════════════════════════════
    // 写侧
    // ═══════════════════════════════════════════════════════════════
    logic        buf_sel;           // 0=写buf0, 1=写buf1
    logic [31:0] shadow;           // ch-layer 0 暂存 {ch3,ch2,ch1,ch0}
    logic        shadow_full;
    logic [7:0]  wr_col;
    logic        wr_row;           // 0=even, 1=odd
    logic        buf_sel_d;        // 边沿检测

    always_ff @(posedge clk) buf_sel_d <= buf_sel;
    wire buf_flip = (buf_sel != buf_sel_d);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            buf_sel <= 0; shadow <= 0; shadow_full <= 0; wr_col <= 0; wr_row <= 0;
        end else if (rq_valid && !cfg.pool_bypass) begin
            if (!shadow_full) begin
                shadow      <= rq_data;                        // ch0-3
                shadow_full <= 1;
            end else begin
                shadow_full <= 0;
                // 拼 ch7-4 + ch3-0 → 16b/bank, 并发写 4 bank
                if (!wr_row) begin
                    bank_even[buf_sel][0][wr_col] <= {rq_data[7:0],   shadow[7:0]};
                    bank_even[buf_sel][1][wr_col] <= {rq_data[15:8],  shadow[15:8]};
                    bank_even[buf_sel][2][wr_col] <= {rq_data[23:16], shadow[23:16]};
                    bank_even[buf_sel][3][wr_col] <= {rq_data[31:24], shadow[31:24]};
                end else begin
                    bank_odd[buf_sel][0][wr_col]  <= {rq_data[7:0],   shadow[7:0]};
                    bank_odd[buf_sel][1][wr_col]  <= {rq_data[15:8],  shadow[15:8]};
                    bank_odd[buf_sel][2][wr_col]  <= {rq_data[23:16], shadow[23:16]};
                    bank_odd[buf_sel][3][wr_col]  <= {rq_data[31:24], shadow[31:24]};
                end

                if (wr_col == cfg.out_w - 1) begin
                    wr_col <= 0;
                    if (wr_row) begin wr_row <= 0; buf_sel <= ~buf_sel; end
                    else         wr_row <= 1;
                end else wr_col <= wr_col + 1;
            end
        end
    end

    // ═══════════════════════════════════════════════════════════════
    // 读侧 — rd_cnt 驱动流水线
    // ═══════════════════════════════════════════════════════════════
    typedef enum logic [1:0] { RD_IDLE, RD_RUN, RD_DONE } rd_state_t;
    rd_state_t rd_state;

    logic        rd_buf;
    logic        ch_layer;
    logic [7:0]  rd_cnt;
    logic [7:0]  pw;
    logic [7:0]  pw2;
    logic [7:0]  rd_pcol_i;
    logic        rd_sub_i;

    logic        s1_valid;
    logic [7:0]  s1_pcol;
    logic        s1_sub;
    logic [7:0]  s1_e0, s1_e1, s1_e2, s1_e3;
    logic [7:0]  s1_o0, s1_o1, s1_o2, s1_o3;

    logic        s1b_waiting;
    logic [7:0]  s1b_pcol;
    logic [7:0]  s1b_e0, s1b_e1, s1b_e2, s1b_e3;
    logic [7:0]  s1b_o0, s1b_o1, s1b_o2, s1b_o3;

    logic        s2_valid;
    logic [7:0]  s2_pcol;
    logic [7:0]  s2_r0, s2_r1, s2_r2, s2_r3;
    logic        s3_valid;

    logic        do_read, do_max4, do_output;

    assign rd_pcol_i = (rd_cnt - 8'd1) >> 1;
    assign rd_sub_i  = (rd_cnt[0] == 1'b1);

    always_comb begin
        do_read   = (rd_state == RD_RUN) && (rd_cnt >= 8'd1) && (rd_cnt <= pw2);
        do_max4   = (rd_state == RD_RUN) && (rd_cnt >= 8'd2) && (rd_cnt <= pw2 + 8'd1);
        do_output = (rd_state == RD_RUN) && (rd_cnt >= 8'd3) && (rd_cnt <= pw2 + 8'd2);
    end

    function automatic [7:0] max4;
        input [7:0] a, b, c, d;
        logic [7:0] ab, cd;
        ab = (a > b) ? a : b;
        cd = (c > d) ? c : d;
        max4 = (ab > cd) ? ab : cd;
    endfunction

    function automatic [7:0] pick;
        input [15:0] entry;
        input        layer;
        pick = layer ? entry[15:8] : entry[7:0];
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_state <= RD_IDLE; rd_buf <= 0; ch_layer <= 0;
            rd_cnt <= 0; pw <= 0; pw2 <= 0;
            s1_valid <= 0; s1b_waiting <= 0; s2_valid <= 0; s3_valid <= 0;
            pp_data <= 0; pp_valid <= 0;
        end else if (cfg.pool_bypass) begin
            pp_data  <= rq_data;
            pp_valid <= rq_valid;
            rd_state <= RD_IDLE; s1_valid <= 0; s1b_waiting <= 0; s2_valid <= 0; s3_valid <= 0;
        end else begin
            pp_valid <= 0;

            case (rd_state)
                RD_IDLE: begin
                    if (buf_flip) begin
                        rd_buf   <= ~buf_sel;
                        ch_layer <= 0;
                        rd_cnt   <= 8'd1;
                        pw       <= cfg.out_w[7:1];
                        pw2      <= {cfg.out_w[7:1], 1'b0};
                        rd_state <= RD_RUN;
                    end
                end

                RD_RUN: begin
                    if (rd_cnt < pw2 + 8'd2) begin
                            rd_cnt <= rd_cnt + 8'd1;
                        end else if (ch_layer == 0) begin
                            ch_layer <= 1; rd_cnt <= 8'd1; rd_state <= RD_RUN;
                            s1b_waiting <= 0; s2_valid <= 0; s3_valid <= 0;
                        end else begin
                            rd_state <= RD_IDLE;
                            s3_valid <= 0;
                        end

                    if (do_read) begin
                        automatic logic [7:0] pix = rd_sub_i ? (rd_pcol_i*2 + 8'd1) : (rd_pcol_i*2);
                        s1_valid <= 1;
                        s1_pcol  <= rd_pcol_i;
                        s1_sub   <= rd_sub_i;
                        s1_e0 <= pick(bank_even[rd_buf][0][pix], ch_layer);
                        s1_e1 <= pick(bank_even[rd_buf][1][pix], ch_layer);
                        s1_e2 <= pick(bank_even[rd_buf][2][pix], ch_layer);
                        s1_e3 <= pick(bank_even[rd_buf][3][pix], ch_layer);
                        s1_o0 <= pick(bank_odd[rd_buf][0][pix], ch_layer);
                        s1_o1 <= pick(bank_odd[rd_buf][1][pix], ch_layer);
                        s1_o2 <= pick(bank_odd[rd_buf][2][pix], ch_layer);
                        s1_o3 <= pick(bank_odd[rd_buf][3][pix], ch_layer);
                    end else begin
                        s1_valid <= 0;
                    end

                    if (s1_valid && s1_sub == 0) begin
                        s1b_waiting <= 1;
                        s1b_pcol    <= s1_pcol;
                        s1b_e0 <= s1_e0; s1b_e1 <= s1_e1; s1b_e2 <= s1_e2; s1b_e3 <= s1_e3;
                        s1b_o0 <= s1_o0; s1b_o1 <= s1_o1; s1b_o2 <= s1_o2; s1b_o3 <= s1_o3;
                    end else if (s1_valid && s1_sub == 1) begin
                        s1b_waiting <= 0;
                    end

                    if (s1_valid && s1_sub == 1 && s1b_waiting) begin
                        s2_valid <= 1;
                        s2_pcol  <= s1_pcol;
                        s2_r0 <= max4(s1b_e0, s1_e0, s1b_o0, s1_o0);
                        s2_r1 <= max4(s1b_e1, s1_e1, s1b_o1, s1_o1);
                        s2_r2 <= max4(s1b_e2, s1_e2, s1b_o2, s1_o2);
                        s2_r3 <= max4(s1b_e3, s1_e3, s1b_o3, s1_o3);
                    end else begin
                        s2_valid <= 0;
                    end

                    s3_valid <= s2_valid;
                    if (s2_valid) begin
                        pp_data  <= {s2_r3, s2_r2, s2_r1, s2_r0};
                        pp_valid <= 1;
                    end
                end

                RD_DONE: begin
                    rd_state <= RD_IDLE;
                end
            endcase
        end
    end

endmodule

`default_nettype wire
