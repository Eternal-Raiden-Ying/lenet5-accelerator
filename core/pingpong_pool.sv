`default_nettype none

import wrapper_pkg::*;

module pingpong_pool #(parameter int MAX_W = 32) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        layer_start,
    input  desc_cfg_t  cfg,
    input  wire [31:0] rq_data,
    input  wire        rq_valid,
    output logic [31:0] pp_data,
    output logic        pp_valid
);
    logic [15:0] bank_even [0:1][0:3][0:MAX_W-1];
    logic [15:0] bank_odd  [0:1][0:3][0:MAX_W-1];

    // ── 定义对齐后的宽度 ──
    wire  [7:0]  out_w_padded = (cfg.out_w + 8'd7) & 8'hF8; // e.g., 10 -> 16

    logic        buf_sel;
    logic [31:0] shadow;
    logic        shadow_full;
    logic [7:0]  wr_col;
    logic        wr_row;
    logic        buf_sel_d;

    always_ff @(posedge clk) buf_sel_d <= buf_sel;
    wire buf_flip = (buf_sel != buf_sel_d);

    // ── 写侧 (无需过滤，全盘接收) ──
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            buf_sel <= 0; shadow <= 0; shadow_full <= 0; wr_col <= 0; wr_row <= 0;
        end else if (layer_start) begin
            buf_sel <= 0; shadow <= 0; shadow_full <= 0; wr_col <= 0; wr_row <= 0;
        end else if (rq_valid && !cfg.pool_bypass) begin
            if (!shadow_full) begin
                shadow <= rq_data; 
                shadow_full <= 1;
            end else begin
                shadow_full <= 0;
                
                // 将所有像素（包含 padding 垃圾）写入 RAM
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

                // 【核心修复】：使用 out_w_padded 换行！
                if (wr_col == out_w_padded - 1) begin
                    wr_col <= 0;
                    if (wr_row) begin 
                        wr_row <= 0; 
                        buf_sel <= ~buf_sel; 
                    end
                    else wr_row <= 1;
                end else begin
                    wr_col <= wr_col + 1;
                end
            end
        end
    end

    // ── max4 函数 ──
    function automatic logic [7:0] max4(input logic [7:0] a, b, c, d);
        logic [7:0] ab, cd;
        ab = (a > b) ? a : b;
        cd = (c > d) ? c : d;
        return (ab > cd) ? ab : cd;
    endfunction

    // ── 读侧 v5 (2-phase, same-pixel ch groups) ──
    logic       rd_running;
    logic [7:0] rd_col;
    logic       rd_layer;
    logic       rd_buf;
    logic       rd_phase;
    logic [7:0] p_even_left [0:3];
    logic [7:0] p_odd_left  [0:3];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_running <= 0; rd_col <= 0; rd_layer <= 0; rd_buf <= 0; rd_phase <= 0;
            pp_valid <= 0; pp_data <= 0;
            for (int i=0; i<4; i++) begin p_even_left[i] <= 0; p_odd_left[i] <= 0; end
        end else if (cfg.pool_bypass) begin
            pp_data    <= rq_data;
            pp_valid   <= rq_valid;
            rd_running <= 0;
        end else begin
            if (buf_flip) begin
                rd_running <= 1; rd_col <= 0; rd_layer <= 0;
                rd_buf <= ~buf_sel; rd_phase <= 0;
            end

            pp_valid <= 0;
            if (rd_running) begin
                if (rd_phase == 0) begin
                    for (int i = 0; i < 4; i++) begin
                        p_even_left[i] <= rd_layer ? bank_even[rd_buf][i][rd_col][15:8] : bank_even[rd_buf][i][rd_col][7:0];
                        p_odd_left[i]  <= rd_layer ? bank_odd[rd_buf][i][rd_col][15:8]  : bank_odd[rd_buf][i][rd_col][7:0];
                    end
                    rd_phase <= 1;
                end else begin
                    pp_valid <= 1;
                    for (int i = 0; i < 4; i++) begin
                        logic [7:0] re, ro;
                        re = rd_layer ? bank_even[rd_buf][i][rd_col+1][15:8] : bank_even[rd_buf][i][rd_col+1][7:0];
                        ro = rd_layer ? bank_odd[rd_buf][i][rd_col+1][15:8]  : bank_odd[rd_buf][i][rd_col+1][7:0];
                        pp_data[8*i +: 8] <= max4(p_even_left[i], p_odd_left[i], re, ro);
                    end
                    
                    // 【核心修复】：使用 out_w_padded 推进，吐出池化后的垃圾数据！
                    if (rd_col + 2 >= out_w_padded) begin
                        rd_col <= 0;
                        if (rd_layer == 1) rd_running <= 0;
                        else               rd_layer <= 1;
                    end else begin
                        rd_col <= rd_col + 2;
                    end
                    rd_phase <= 0;
                end
            end
        end
    end

endmodule