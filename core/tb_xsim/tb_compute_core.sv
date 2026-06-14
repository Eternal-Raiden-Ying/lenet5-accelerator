// ═══════════════════════════════════════════════════════════════════
// tb_compute_core — full pipeline testbench with behavioral golden model
//
// Behavioral models: weight_sram, requant_sram, + software reference conv
//
// Test #1: MODE_8x8 5×5 kernel, 12×10 input, pad=1, 1→12ch (2 ocg)
//   out_w=10, out_h=8, 32 flushes, 256 gearbox writes
//   Bit-exact golden comparison against software reference model.
// ═══════════════════════════════════════════════════════════════════
`timescale 1ns / 100ps

import wrapper_pkg::*;
import core_pkg::*;

// ── Test parameters ──
localparam int  TN_IN_W       = 12;
localparam int  TN_IN_H       = 10;
localparam int  TN_IN_CH      = 1;
localparam int  TN_OUT_CH     = 12;
localparam int  TN_KERNEL     = 5;
localparam int  TN_PAD        = 0;
localparam int  TN_STRIDE     = 1;
localparam int  TN_OUT_W      = (TN_IN_W + 2*TN_PAD - TN_KERNEL) / TN_STRIDE + 1;  // 8
localparam int  TN_OUT_H      = (TN_IN_H + 2*TN_PAD - TN_KERNEL) / TN_STRIDE + 1;  // 6
localparam int  TN_OCG        = (TN_OUT_CH + 7) / 8;    // 2
localparam int  TN_OX_STEPS   = (TN_OUT_W + 7) / 8;      // 1
localparam int  TN_FLUSHES    = TN_OCG * TN_OUT_H * TN_OX_STEPS;  // 12
localparam int  TN_WRITES     = TN_FLUSHES * 8;           // 96
localparam int  TN_IM_ROW_STRIDE = (TN_IN_W + 7) / 8;    // 2
localparam int  TN_IM_CH_STRIDE  = TN_IN_H * TN_IM_ROW_STRIDE;  // 20
localparam int  TN_OUT_ROW_STRIDE = (TN_OUT_W + 7) / 8;  // 1
localparam int  TN_OUT_CH_STRIDE  = TN_OUT_H * TN_OUT_ROW_STRIDE;  // 6
localparam int  TN_IM_WR_BASE  = 128;

module tb_compute_core;
    logic        core_clk, core_rst_n;
    logic        cfg_valid;
    desc_cfg_t   cfg;
    logic        im_ext_cs, im_ext_wr;
    logic  [8:0] im_ext_addr;
    logic [63:0] im_ext_wdata;
    logic [63:0] im_ext_rdata;
    logic        im_ext_ready;
    logic        core_busy, layer_done;
    logic  [1:0] core_error;
    core_fsm_state_t core_fsm_state;
    logic        wt_rd_req;
    logic  [7:0] wt_rd_addr;
    logic [511:0] wt_rd_data;
    logic        rq_rd_req;
    logic  [3:0] rq_rd_addr;
    logic [287:0] rq_rd_data;

    integer errors = 0;

    // ═══════════════════════════════════════════════════════════════
    // DUT
    // ═══════════════════════════════════════════════════════════════
    compute_core u_dut (
        .core_clk(core_clk), .core_rst_n(core_rst_n),
        .cfg_valid(cfg_valid), .cfg(cfg),
        .im_ext_cs(im_ext_cs), .im_ext_wr(im_ext_wr),
        .im_ext_addr(im_ext_addr), .im_ext_wdata(im_ext_wdata),
        .im_ext_rdata(im_ext_rdata), .im_ext_ready(im_ext_ready),
        .core_busy(core_busy), .layer_done(layer_done),
        .core_error(core_error), .core_fsm_state(core_fsm_state),
        .wt_rd_req(wt_rd_req), .wt_rd_addr(wt_rd_addr),
        .wt_rd_data(wt_rd_data),
        .rq_rd_req(rq_rd_req), .rq_rd_addr(rq_rd_addr),
        .rq_rd_data(rq_rd_data)
    );

    // ═══════════════════════════════════════════════════════════════
    // Clock & Reset
    // ═══════════════════════════════════════════════════════════════
    initial core_clk = 0;
    always #2.5 core_clk = ~core_clk;

    initial begin core_rst_n = 0; #50 core_rst_n = 1; end

    initial begin
        $dumpfile("tb_compute_core.vcd");
        $dumpvars(0, tb_compute_core);
    end

    // ═══════════════════════════════════════════════════════════════
    // Behavioral Weight SRAM
    // ═══════════════════════════════════════════════════════════════
    logic [511:0] wt_mem [0:255];
    always_ff @(posedge core_clk)
        if (wt_rd_req) wt_rd_data <= wt_mem[wt_rd_addr];

    // ═══════════════════════════════════════════════════════════════
    // Behavioral Requant SRAM
    // ═══════════════════════════════════════════════════════════════
    logic [287:0] rq_mem [0:15];
    always_ff @(posedge core_clk)
        if (rq_rd_req) rq_rd_data <= rq_mem[rq_rd_addr];

    // ═══════════════════════════════════════════════════════════════
    // IM SRAM external access helpers
    // ═══════════════════════════════════════════════════════════════
    task im_write_word(input [8:0] addr, input [63:0] data);
        @(posedge core_clk);
        im_ext_cs <= 1'b1; im_ext_wr <= 1'b1; im_ext_addr <= addr; im_ext_wdata <= data;
        @(posedge core_clk);
        im_ext_cs <= 1'b0; im_ext_wr <= 1'b0;
        @(posedge core_clk);
    endtask

    task im_read_word(input [8:0] addr, output [63:0] data);
        @(posedge core_clk);
        im_ext_cs <= 1'b1; im_ext_wr <= 1'b0; im_ext_addr <= addr;
        @(posedge core_clk);  // BRAM read starts
        @(posedge core_clk);  // rdata_raw ready
        @(posedge core_clk);  // im_ext_rdata valid
        data = im_ext_rdata;
        im_ext_cs <= 1'b0;
    endtask

    task tick(input integer n);
        for (int i = 0; i < n; i++) @(posedge core_clk);
    endtask

    task wait_layer_done;
        while (!layer_done) @(posedge core_clk);
    endtask

    // ═══════════════════════════════════════════════════════════════
    // Behavioral Reference Model — software golden convolution
    //
    // Computes conv5x5 with pad/stride, identity requant.
    // Output ordering matches hardware: ocg→oy→ox(step8).
    // Each flush produces 8 gearbox writes (64b each, 2 rq outputs/word).
    // ═══════════════════════════════════════════════════════════════

    // ── Image: 2D array [0:TN_IN_H-1][0:TN_IN_W-1] ──
    logic [7:0] img [0:TN_IN_H-1][0:TN_IN_W-1];

    // ── Weights: 4D [oc][ic][ky][kx] ──
    logic signed [7:0] wt_ref [0:TN_OUT_CH-1][0:TN_IN_CH-1][0:TN_KERNEL-1][0:TN_KERNEL-1];

    // ── Output: 3D [oc][oy][ox] ──
    integer out_ref [0:TN_OUT_CH-1][0:TN_OUT_H-1][0:TN_OUT_W-1];

    // ── Expected gearbox write buffer (linear, 256 entries) ──
    logic [63:0] exp_write [0:TN_WRITES-1];

    // Compute software golden
    task compute_golden;
        integer oc, ic, oy, ox, ky, kx;
        integer ix, iy;    // input coordinates
        integer sum;
        integer ocg, oyg, oxg;
        integer flush_idx, w_idx;
        integer ch_in_ocg;  // channel within ocg (0..7)
        integer ox_in_step;  // ox position within 8-pixel step (0..7)
        integer actual_ox;   // absolute ox = oxg*8 + ox_in_step
        integer rq_pair;     // pair index within a flush (0..7)

        // Step 1: generate image and weight values
        //   img[y][x] = y * TN_IN_W + x + 1  (1..120)
        //   weight[oc][ic][ky][kx] = 1 (all ones, for simple golden)
        for (oy = 0; oy < TN_IN_H; oy = oy + 1)
            for (ox = 0; ox < TN_IN_W; ox = ox + 1)
                img[oy][ox] = 8'(oy * TN_IN_W + ox + 1);

        for (oc = 0; oc < TN_OUT_CH; oc = oc + 1)
            for (ic = 0; ic < TN_IN_CH; ic = ic + 1)
                for (ky = 0; ky < TN_KERNEL; ky = ky + 1)
                    for (kx = 0; kx < TN_KERNEL; kx = kx + 1)
                        wt_ref[oc][ic][ky][kx] = 8'sd1;

        // Step 2: compute output via convolution
        for (oc = 0; oc < TN_OUT_CH; oc = oc + 1) begin
            for (oy = 0; oy < TN_OUT_H; oy = oy + 1) begin
                for (ox = 0; ox < TN_OUT_W; ox = ox + 1) begin
                    sum = 0;
                    for (ky = 0; ky < TN_KERNEL; ky = ky + 1) begin
                        for (kx = 0; kx < TN_KERNEL; kx = kx + 1) begin
                            iy = oy * TN_STRIDE + ky - TN_PAD;
                            ix = ox * TN_STRIDE + kx - TN_PAD;
                            if (iy >= 0 && iy < TN_IN_H && ix >= 0 && ix < TN_IN_W)
                                sum = sum + img[iy][ix] * wt_ref[oc][0][ky][kx];
                        end
                    end
                    // identity requant: clamp(sum, 0, 255)
                    if (sum < 0) sum = 0;
                    if (sum > 255) sum = 255;
                    out_ref[oc][oy][ox] = sum;
                end
            end
        end

        // Step 3: pack into gearbox write order
        //   Hardware order: ocg(step8) → oy → ox(step8) → flush → gearbox
        //   Within one flush: 8 gearbox writes
        //     write[0] = {rq[1], rq[0]} = {ox0_ch4-7, ox0_ch0-3}
        //     write[1] = {rq[3], rq[2]} = {ox1_ch4-7, ox1_ch0-3}
        //     ...
        //     write[7] = {rq[15], rq[14]} = {ox7_ch4-7, ox7_ch0-3}
        flush_idx = 0;
        for (ocg = 0; ocg < TN_OCG; ocg = ocg + 1) begin
            for (oyg = 0; oyg < TN_OUT_H; oyg = oyg + 1) begin
                for (oxg = 0; oxg < TN_OX_STEPS; oxg = oxg + 1) begin
                    // One flush per (ocg, oyg, oxg)
                    for (rq_pair = 0; rq_pair < 8; rq_pair = rq_pair + 1) begin
                        automatic logic [31:0] rq_lo, rq_hi;
                        // gearbox write[rq_pair] = pixel position rq_pair, 8 channels
                        //   rq_lo = channels 0-3 (= FIFO chunk rq_pair*2)
                        //   rq_hi = channels 4-7 (= FIFO chunk rq_pair*2+1)
                        ox_in_step = rq_pair;
                        actual_ox = oxg * 8 + ox_in_step;
                        rq_lo = 32'd0;
                        rq_hi = 32'd0;
                        if (actual_ox < TN_OUT_W) begin
                            // rq_lo = {ch3,ch2,ch1,ch0}, each 8b clamped
                            for (ch_in_ocg = 0; ch_in_ocg < 4; ch_in_ocg = ch_in_ocg + 1) begin
                                oc = ocg * 8 + ch_in_ocg;
                                if (oc < TN_OUT_CH)
                                    rq_lo[ch_in_ocg*8 +: 8] = 8'(out_ref[oc][oyg][actual_ox]);
                            end
                            // rq_hi = {ch7,ch6,ch5,ch4}
                            for (ch_in_ocg = 0; ch_in_ocg < 4; ch_in_ocg = ch_in_ocg + 1) begin
                                oc = ocg * 8 + 4 + ch_in_ocg;
                                if (oc < TN_OUT_CH)
                                    rq_hi[ch_in_ocg*8 +: 8] = 8'(out_ref[oc][oyg][actual_ox]);
                            end
                        end
                        // Gearbox write = {rq_hi, rq_lo}
                        w_idx = flush_idx * 8 + rq_pair;
                        exp_write[w_idx] = {rq_hi, rq_lo};
                    end
                    flush_idx = flush_idx + 1;
                end
            end
        end
    endtask

    // ═══════════════════════════════════════════════════════════════
    // Test #1: 5×5 conv, 12×10 input, pad=1, 1→12ch, bit-exact check
    // ═══════════════════════════════════════════════════════════════
    task test1_conv5x5_golden;
        automatic integer k, oy_i, oc_i;
        automatic logic [63:0] rd;
        automatic integer local_errs;

        $display("\n==== TEST #1: 5x5 conv, 12x10 in, pad=0, 1->12ch, golden check ====");
        $display("    out=%0dx%0d, %0d ocg, %0d flushes, %0d gearbox writes",
                 TN_OUT_W, TN_OUT_H, TN_OCG, TN_FLUSHES, TN_WRITES);

        cfg_valid <= 1'b0;
        im_ext_cs <= 1'b0; im_ext_wr <= 1'b0;
        tick(5);

        // ── Compute golden reference ──
        compute_golden();

        // ── Pre-load IM SRAM: 10 rows × 12 pixels ──
        //   im_row_stride=2: each row occupies 2 × 64b words
        //   Row r: addr 2r → pixels 0..7, addr 2r+1 → pixels 8..11
        for (oy_i = 0; oy_i < TN_IN_H; oy_i = oy_i + 1) begin
            // even bank: pixels 0..7
            im_write_word(9'(oy_i * 2), {
                img[oy_i][7], img[oy_i][6], img[oy_i][5], img[oy_i][4],
                img[oy_i][3], img[oy_i][2], img[oy_i][1], img[oy_i][0]
            });
            // odd bank: pixels 8..11 (upper 4 bytes zero)
            im_write_word(9'(oy_i * 2 + 1), {
                8'd0, 8'd0, 8'd0, 8'd0,
                img[oy_i][11], img[oy_i][10], img[oy_i][9], img[oy_i][8]
            });
        end

        // ── Pre-load Weight SRAM ──
        //   MODE_8x8 wt_rd_addr = ocg*wt_ocg_stride + ic*wt_ch_stride + ky
        //   wt_ocg_stride=5, wt_ch_stride=5, ky=0..4
        //   For ocg=0: addr = ic*5 + ky. ic=0: addr = ky (0..4)
        //   For ocg=1: addr = 1*5 + 0 + ky = 5 + ky (5..9)
        //   Each row: 5 kx groups × 8 weights/group = 40 bytes
        for (oc_i = 0; oc_i < TN_OUT_CH; oc_i = oc_i + 8) begin
            automatic integer ocg_base;
            ocg_base = (oc_i / 8) * TN_IN_CH * TN_KERNEL;  // ocg * in_ch * kh
            for (k = 0; k < TN_KERNEL; k = k + 1) begin
                automatic logic [511:0] row_val = 512'd0;
                automatic integer addr;
                addr = ocg_base + k;  // ocg*wt_ocg_stride + ic*wt_ch_stride + ky
                for (int kxi = 0; kxi < TN_KERNEL; kxi = kxi + 1)
                    for (int oci = 0; oci < 8; oci = oci + 1)
                        if ((oc_i + oci) < TN_OUT_CH)
                            row_val[kxi*64 + oci*8 +: 8] = wt_ref[oc_i+oci][0][k][kxi];
                wt_mem[addr] = row_val;
            end
        end

        // ── Pre-load Requant SRAM: identity (M=1, shift=0, b=0) ──
        for (k = 0; k < 16; k = k + 1) begin
            automatic logic [71:0] ident;
            ident = {32'd0, 8'd0, 32'd1};  // {b, shift, M}
            rq_mem[k] = {ident, ident, ident, ident};
        end

        // ── Configure layer ──
        cfg.in_ch           <= 8'(TN_IN_CH);
        cfg.out_ch          <= 8'(TN_OUT_CH);
        cfg.in_w            <= 8'(TN_IN_W);
        cfg.in_h            <= 8'(TN_IN_H);
        cfg.out_w           <= 8'(TN_OUT_W);
        cfg.out_h           <= 8'(TN_OUT_H);
        cfg.kernel_w        <= 4'(TN_KERNEL);
        cfg.kernel_h        <= 4'(TN_KERNEL);
        cfg.stride_w        <= 4'(TN_STRIDE);
        cfg.stride_h        <= 4'(TN_STRIDE);
        cfg.pad_w           <= 4'(TN_PAD);
        cfg.pad_h           <= 4'(TN_PAD);
        cfg.zp_y            <= 8'd0;
        cfg.im_total_writes <= 16'(TN_WRITES);
        cfg.im_row_stride   <= 8'(TN_IM_ROW_STRIDE);
        cfg.im_ch_stride    <= 10'(TN_IM_CH_STRIDE);
        cfg.im_read_base    <= 9'd0;
        cfg.out_row_stride  <= 8'(TN_OUT_ROW_STRIDE);
        cfg.out_ch_stride   <= 10'(TN_OUT_CH_STRIDE);
        cfg.im_write_base   <= 9'(TN_IM_WR_BASE);
        cfg.wt_ch_stride    <= 8'(TN_KERNEL);
        cfg.wt_row_stride   <= 8'd1;
        cfg.wt_ocg_stride   <= 10'(TN_KERNEL);
        cfg.mode            <= MODE_8x8;
        cfg.pool_bypass     <= 1'b1;
        cfg.keep_accum      <= 1'b0;
        cfg.disable_flush   <= 1'b0;
        cfg.ct_mode         <= CT_GEARBOX;
        cfg.reserved_w1     <= 5'd0;
        cfg.reserved_w2     <= 5'd0;
        cfg.reserved_w3     <= 1'b0;

        // ── Issue cfg_valid ──
        cfg_valid <= 1'b1;
        @(posedge core_clk);
        cfg_valid <= 1'b0;

        // ── Wait for layer_done ──
        wait_layer_done();
        $display("  layer_done at t=%0t", $time);

        // ── Read back all gearbox writes and compare ──
        local_errs = 0;
        for (k = 0; k < TN_WRITES; k = k + 1) begin
            im_read_word(9'(TN_IM_WR_BASE + k), rd);
            if (rd !== exp_write[k]) begin
                if (local_errs < 10)  // limit printed errors
                    $display("  FAIL write[%0d]: got=%h exp=%h", k, rd, exp_write[k]);
                local_errs = local_errs + 1;
            end
        end

        if (local_errs == 0) begin
            $display("  All %0d writes match golden ✓", TN_WRITES);
        end else begin
            $display("  %0d / %0d MISMATCHES", local_errs, TN_WRITES);
            errors = errors + local_errs;
        end
    endtask

    // ═══════════════════════════════════════════════════════════════
    // Test #2: Pool + CT_SPATIAL — same conv, pool_bypass=0
    //
    //   2×2 MaxPool reduces 8×6 → 4×3 output.
    //   CT_SPATIAL scatter-writes to IM with CHW-planar layout.
    //   Golden model simulates exact pool + spatial write ordering.
    // ═══════════════════════════════════════════════════════════════

    localparam int TP_OUT_W = TN_OUT_W;   // 8
    localparam int TP_OUT_H = TN_OUT_H;   // 6
    localparam int TP_POOL_W = TP_OUT_W / 2;  // 4
    localparam int TP_POOL_H = TP_OUT_H / 2;  // 3
    localparam int TP_IM_WR_BASE = 256;
    // out_row_stride=1 → spatial_oxg wraps at 0, spatial_oy increments per matrix
    localparam int TP_OUT_ROW_STRIDE = 1;
    localparam int TP_OUT_CH_STRIDE  = TP_POOL_H * TN_OCG * TP_OUT_ROW_STRIDE;  // 3*2*1=6
    // 2 ocg × 3 pool rows × 4 scatter writes/row = 24 total writes
    localparam int TP_WRITES = TN_OCG * TP_POOL_H * 4;

    logic [63:0] exp_pool_write [0:TP_WRITES-1];

    task compute_pool_golden;
        integer oy, ox, oc;
        integer iy, ix;
        integer pool_oy, pool_ox;  // pool output coordinates
        integer ch_grp, s_idx;
        integer val00, val01, val10, val11, max_val;
        integer conv_out [0:TP_OUT_H-1][0:TP_OUT_W-1][0:TN_OUT_CH-1];
        integer pool_out [0:TP_POOL_H-1][0:TP_POOL_W-1][0:TN_OUT_CH-1];
        integer matrix_idx, scatter_oy, scatter_oxg;

        // Step 1: same conv output as test1 (identity weights, pad=0)
        for (oy = 0; oy < TP_OUT_H; oy = oy + 1) begin
            for (ox = 0; ox < TP_OUT_W; ox = ox + 1) begin
                for (oc = 0; oc < TN_OUT_CH; oc = oc + 1) begin
                    integer sum_val;
                    sum_val = 0;
                    for (int ky = 0; ky < TN_KERNEL; ky = ky + 1) begin
                        for (int kx = 0; kx < TN_KERNEL; kx = kx + 1) begin
                            iy = oy * TN_STRIDE + ky;
                            ix = ox * TN_STRIDE + kx;
                            if (iy < TN_IN_H && ix < TN_IN_W)
                                sum_val = sum_val + img[iy][ix];
                        end
                    end
                    if (sum_val < 0) sum_val = 0;
                    if (sum_val > 255) sum_val = 255;
                    conv_out[oy][ox][oc] = sum_val;
                end
            end
        end

        // Step 2: 2×2 maxpool
        for (pool_oy = 0; pool_oy < TP_POOL_H; pool_oy = pool_oy + 1) begin
            for (pool_ox = 0; pool_ox < TP_POOL_W; pool_ox = pool_ox + 1) begin
                for (oc = 0; oc < TN_OUT_CH; oc = oc + 1) begin
                    val00 = conv_out[pool_oy*2  ][pool_ox*2  ][oc];
                    val01 = conv_out[pool_oy*2  ][pool_ox*2+1][oc];
                    val10 = conv_out[pool_oy*2+1][pool_ox*2  ][oc];
                    val11 = conv_out[pool_oy*2+1][pool_ox*2+1][oc];
                    max_val = val00;
                    if (val01 > max_val) max_val = val01;
                    if (val10 > max_val) max_val = val10;
                    if (val11 > max_val) max_val = val11;
                    pool_out[pool_oy][pool_ox][oc] = max_val;
                end
            end
        end

        // Step 3: simulate CT_SPATIAL matrix assembly + scatter writes
        //   Hardware loop: ocg→oy→ox. Pool sees data from ALL ocg groups sequentially.
        //   Per pool row: 8 pp_valid (4 ch_layer0 + 4 ch_layer1) → 4 scatter writes.
        //   2 ocg × 3 pool rows = 6 matrices → 24 scatter writes.
        //   ocg=1 (ch8-11): bytes 0-3 valid output, bytes 4-7 = 0 (ch12-15 unused).
        scatter_oy  = 0;
        scatter_oxg = 0;
        matrix_idx  = 0;

        for (int ocg_i = 0; ocg_i < TN_OCG; ocg_i = ocg_i + 1) begin
            for (pool_oy = 0; pool_oy < TP_POOL_H; pool_oy = pool_oy + 1) begin
                for (ch_grp = 0; ch_grp < 4; ch_grp = ch_grp + 1) begin
                    automatic logic [63:0] wdata_64 = 64'd0;
                    // Bytes 0-3: pool_out[pool_oy][pool_ox][ocg*8 + ch_grp]
                    for (s_idx = 0; s_idx < 4; s_idx = s_idx + 1)
                        wdata_64[s_idx*8 +: 8] = 8'(pool_out[pool_oy][s_idx][ocg_i*8 + ch_grp]);
                    // Bytes 4-7: pool_out[pool_oy][pool_ox][ocg*8 + ch_grp+4]
                    //   For ocg=1, ch_grp+4+8 > 11 (out_ch=12), these are zero
                    for (s_idx = 0; s_idx < 4; s_idx = s_idx + 1) begin
                        oc = ocg_i * 8 + ch_grp + 4;
                        wdata_64[(s_idx+4)*8 +: 8] = (oc < TN_OUT_CH) ? 8'(pool_out[pool_oy][s_idx][oc]) : 8'd0;
                    end
                    exp_pool_write[matrix_idx] = wdata_64;
                    matrix_idx = matrix_idx + 1;
                end

                if (scatter_oxg == TP_OUT_ROW_STRIDE - 1) begin
                    scatter_oxg = 0;
                    scatter_oy  = scatter_oy + 1;
                end else begin
                    scatter_oxg = scatter_oxg + 1;
                end
            end
        end
    endtask

    task test2_pool_spatial;
        automatic integer oy_i, k, oc_i;
        automatic logic [63:0] rd;
        automatic integer local_errs;
        // Address lookup: for pool row r (0..2), ch_grp c (0..3):
        //   addr = TP_IM_WR_BASE + c*TP_OUT_CH_STRIDE + r*TP_OUT_ROW_STRIDE + scatter_oxg
        // scatter_oxg = 0 for all (wraps at out_row_stride=1, scatter_oy advances instead)

        $display("\n==== TEST #2: Pool(2x2) + CT_SPATIAL, 5x5 conv, 12x10 in, 1->12ch (2 ocg) ====");
        $display("    pool out=%0dx%0d, %0d scatter writes", TP_POOL_W, TP_POOL_H, TP_WRITES);

        cfg_valid <= 1'b0;
        im_ext_cs <= 1'b0; im_ext_wr <= 1'b0;
        tick(5);

        // ── Compute golden ──
        compute_pool_golden();

        // ── Pre-load same IM and weights as test1 ──
        for (oy_i = 0; oy_i < TN_IN_H; oy_i = oy_i + 1) begin
            im_write_word(9'(oy_i * 2), {
                img[oy_i][7], img[oy_i][6], img[oy_i][5], img[oy_i][4],
                img[oy_i][3], img[oy_i][2], img[oy_i][1], img[oy_i][0]
            });
            im_write_word(9'(oy_i * 2 + 1), {
                8'd0, 8'd0, 8'd0, 8'd0,
                img[oy_i][11], img[oy_i][10], img[oy_i][9], img[oy_i][8]
            });
        end

        for (oc_i = 0; oc_i < TN_OUT_CH; oc_i = oc_i + 8) begin
            automatic integer ocg_base;
            ocg_base = (oc_i / 8) * TN_IN_CH * TN_KERNEL;
            for (k = 0; k < TN_KERNEL; k = k + 1) begin
                automatic logic [511:0] row_val = 512'd0;
                automatic integer addr;
                addr = ocg_base + k;
                for (int kxi = 0; kxi < TN_KERNEL; kxi = kxi + 1)
                    for (int oci = 0; oci < 8; oci = oci + 1)
                        if ((oc_i + oci) < TN_OUT_CH)
                            row_val[kxi*64 + oci*8 +: 8] = wt_ref[oc_i+oci][0][k][kxi];
                wt_mem[addr] = row_val;
            end
        end

        for (k = 0; k < 16; k = k + 1) begin
            automatic logic [71:0] ident;
            ident = {32'd0, 8'd0, 32'd1};
            rq_mem[k] = {ident, ident, ident, ident};
        end

        // ── Configure: same conv, but pool_bypass=0, ct_mode=CT_SPATIAL ──
        cfg.in_ch           <= 8'(TN_IN_CH);
        cfg.out_ch          <= 8'(TN_OUT_CH);
        cfg.in_w            <= 8'(TN_IN_W);
        cfg.in_h            <= 8'(TN_IN_H);
        cfg.out_w           <= 8'(TP_OUT_W);
        cfg.out_h           <= 8'(TP_OUT_H);
        cfg.kernel_w        <= 4'(TN_KERNEL);
        cfg.kernel_h        <= 4'(TN_KERNEL);
        cfg.stride_w        <= 4'(TN_STRIDE);
        cfg.stride_h        <= 4'(TN_STRIDE);
        cfg.pad_w           <= 4'(TN_PAD);
        cfg.pad_h           <= 4'(TN_PAD);
        cfg.zp_y            <= 8'd0;
        cfg.im_total_writes <= 16'(TP_WRITES);
        cfg.im_row_stride   <= 8'(TN_IM_ROW_STRIDE);
        cfg.im_ch_stride    <= 10'(TN_IM_CH_STRIDE);
        cfg.im_read_base    <= 9'd0;
        cfg.out_row_stride  <= 8'(TP_OUT_ROW_STRIDE);
        cfg.out_ch_stride   <= 10'(TP_OUT_CH_STRIDE);
        cfg.im_write_base   <= 9'(TP_IM_WR_BASE);
        cfg.wt_ch_stride    <= 8'(TN_KERNEL);
        cfg.wt_row_stride   <= 8'd1;
        cfg.wt_ocg_stride   <= 10'(TN_KERNEL);
        cfg.mode            <= MODE_8x8;
        cfg.pool_bypass     <= 1'b0;          // ← enable pool
        cfg.keep_accum      <= 1'b0;
        cfg.disable_flush   <= 1'b0;
        cfg.ct_mode         <= CT_SPATIAL;    // ← enable spatial transpose
        cfg.reserved_w1     <= 5'd0;
        cfg.reserved_w2     <= 5'd0;
        cfg.reserved_w3     <= 1'b0;

        cfg_valid <= 1'b1;
        @(posedge core_clk);
        cfg_valid <= 1'b0;

        wait_layer_done();
        $display("  layer_done at t=%0t", $time);

        // ── Read back scatter writes and compare ──
        //   Address: base + ch_grp * out_ch_stride + scatter_oy * out_row_stride
        //   scatter_oy cycles 0,1,2 for 3 pool rows
        local_errs = 0;
        for (k = 0; k < TP_WRITES; k = k + 1) begin
            // scatter_oy = k / 4 (since 4 ch_groups per matrix, one matrix per pool row)
            // ch_grp = k % 4
            automatic integer scatter_oy_addr = k / 4;
            automatic integer ch_grp_addr = k % 4;
            automatic integer addr;
            addr = TP_IM_WR_BASE + ch_grp_addr * TP_OUT_CH_STRIDE + scatter_oy_addr * TP_OUT_ROW_STRIDE;
            im_read_word(9'(addr), rd);
            if (rd !== exp_pool_write[k]) begin
                if (local_errs < 10)
                    $display("  FAIL write[%0d] addr=%0d: got=%h exp=%h", k, addr, rd, exp_pool_write[k]);
                local_errs = local_errs + 1;
            end
        end

        if (local_errs == 0)
            $display("  All %0d scatter writes match golden ✓", TP_WRITES);
        else begin
            $display("  %0d / %0d MISMATCHES", local_errs, TP_WRITES);
            errors = errors + local_errs;
        end
    endtask

    // ═══════════════════════════════════════════════════════════════
    // Test #3: pad=(1,1), bypass+gearbox, 5×5 conv, 12×10 in, 1→8ch
    //
    //   pad=1 ⇒ out_w=10, out_h=8, 1 ocg, 2 ox steps, 16 flushes.
    //   im_read_base=3: compensates for negative x_off/y_off wrapping.
    //   128 gearbox writes, bit-exact golden check.
    // ═══════════════════════════════════════════════════════════════
    localparam int T3_IN_W  = 12;
    localparam int T3_IN_H  = 10;
    localparam int T3_PAD   = 1;
    localparam int T3_OUT_W = (T3_IN_W + 2*T3_PAD - TN_KERNEL)/TN_STRIDE + 1;  // 10
    localparam int T3_OUT_H = (T3_IN_H + 2*T3_PAD - TN_KERNEL)/TN_STRIDE + 1;  // 8
    localparam int T3_OCG       = 1;  // out_ch=8
    localparam int T3_OX_STEPS  = (T3_OUT_W + 7) / 8;    // 2
    localparam int T3_FLUSHES   = T3_OCG * T3_OUT_H * T3_OX_STEPS;  // 16
    localparam int T3_WRITES    = T3_FLUSHES * 8;          // 128
    localparam int T3_OX_ALL    = T3_OX_STEPS * 8;         // 16 — HW computes all 8 pos/step
    localparam int T3_IM_ROW_STRIDE = (T3_IN_W + 7) / 8;  // 2
    localparam int T3_IM_CH_STRIDE  = T3_IN_H * T3_IM_ROW_STRIDE;  // 20
    localparam int T3_IM_RD_BASE = 0;   // vld signals handle pad, no base offset needed
    localparam int T3_IM_WR_BASE = 192;

    logic [63:0] exp_pad_write [0:T3_WRITES-1];

    task compute_pad_golden;
        integer oy, ox, oc, ky, kx, iy, ix, sum_val;
        // HW computes all 8 positions per ox step → need ox = 0..T3_OX_ALL-1
        integer conv_out [0:T3_OUT_H-1][0:T3_OX_ALL-1][0:7];  // 8 channels, 16 ox positions
        integer ocg, oxg, flush_idx;
        integer rq_pair, ox_in_step, actual_ox;

        // Step 1: convolution with pad for ALL ox positions (0..T3_OX_ALL-1)
        // HW PE array computes 8 values per ox step, even beyond out_w
        for (oy = 0; oy < T3_OUT_H; oy = oy + 1)
            for (ox = 0; ox < T3_OX_ALL; ox = ox + 1)
                for (oc = 0; oc < 8; oc = oc + 1) begin
                    sum_val = 0;
                    for (ky = 0; ky < TN_KERNEL; ky = ky + 1)
                        for (kx = 0; kx < TN_KERNEL; kx = kx + 1) begin
                            iy = oy * TN_STRIDE + ky - T3_PAD;
                            ix = ox * TN_STRIDE + kx - T3_PAD;
                            if (iy >= 0 && iy < T3_IN_H && ix >= 0 && ix < T3_IN_W)
                                sum_val = sum_val + img[iy][ix];  // all weights=1
                        end
                    if (sum_val < 0) sum_val = 0;
                    if (sum_val > 255) sum_val = 255;
                    conv_out[oy][ox][oc] = sum_val;
                end

        // Step 2: pack into gearbox write order — ALL 8 pos/step, no out_w clipping
        flush_idx = 0;
        for (ocg = 0; ocg < T3_OCG; ocg = ocg + 1)
            for (oy = 0; oy < T3_OUT_H; oy = oy + 1)
                for (oxg = 0; oxg < T3_OX_STEPS; oxg = oxg + 1) begin
                    for (rq_pair = 0; rq_pair < 8; rq_pair = rq_pair + 1) begin
                        automatic logic [31:0] rq_lo = 32'd0;
                        automatic logic [31:0] rq_hi = 32'd0;
                        ox_in_step = rq_pair;
                        actual_ox = oxg * 8 + ox_in_step;
                        // HW computes full conv for all 8 pos — no out_w boundary mask
                        for (int ci = 0; ci < 4; ci = ci + 1) begin
                            rq_lo[ci*8 +: 8] = 8'(conv_out[oy][actual_ox][ci]);
                            rq_hi[ci*8 +: 8] = 8'(conv_out[oy][actual_ox][ci+4]);
                        end
                        exp_pad_write[flush_idx*8 + rq_pair] = {rq_hi, rq_lo};
                    end
                    flush_idx = flush_idx + 1;
                end
    endtask

    task test3_pad_bypass;
        automatic integer oy_i, k, oc_i;
        automatic logic [63:0] rd;
        automatic integer local_errs;

        $display("\n==== TEST #3: pad=(1,1), bypass+gearbox, 5x5 conv, 12x10 in, 1->8ch ====");
        $display("    out=%0dx%0d, %0d flushes, %0d gearbox writes, im_rd_base=%0d",
                 T3_OUT_W, T3_OUT_H, T3_FLUSHES, T3_WRITES, T3_IM_RD_BASE);

        cfg_valid <= 1'b0;
        im_ext_cs <= 1'b0; im_ext_wr <= 1'b0;
        tick(5);

        compute_pad_golden();

        // IM SRAM: 10 rows × 12 pixels at addresses 0..19
        // + zero-filled guard rows at 20..31 for pad=1 edge reads
        for (oy_i = 0; oy_i < T3_IN_H; oy_i = oy_i + 1) begin
            im_write_word(9'(oy_i * 2), {
                img[oy_i][7], img[oy_i][6], img[oy_i][5], img[oy_i][4],
                img[oy_i][3], img[oy_i][2], img[oy_i][1], img[oy_i][0]
            });
            im_write_word(9'(oy_i * 2 + 1), {
                8'd0, 8'd0, 8'd0, 8'd0,
                img[oy_i][11], img[oy_i][10], img[oy_i][9], img[oy_i][8]
            });
        end
        // Guard rows: zero-fill addresses 20..31 so window reads at bottom edge don't hit X
        for (oy_i = 20; oy_i < 32; oy_i = oy_i + 1)
            im_write_word(9'(oy_i), 64'd0);

        // Weight: 5 ky rows, all 1's
        for (oc_i = 0; oc_i < 8; oc_i = oc_i + 8) begin
            for (k = 0; k < TN_KERNEL; k = k + 1) begin
                automatic logic [511:0] row_val = 512'd0;
                for (int kxi = 0; kxi < TN_KERNEL; kxi = kxi + 1)
                    for (int oci = 0; oci < 8; oci = oci + 1)
                        row_val[kxi*64 + oci*8 +: 8] = 8'd1;
                wt_mem[k] = row_val;
            end
        end

        // Requant: identity
        for (k = 0; k < 16; k = k + 1) begin
            automatic logic [71:0] ident = {32'd0, 8'd0, 32'd1};
            rq_mem[k] = {ident, ident, ident, ident};
        end

        // ── Diagnostic: verify IM data at address 0 before compute ──
        begin
            automatic logic [63:0] d0, d1;
            im_read_word(9'd0, d0);
            im_read_word(9'd1, d1);
            $display("  [DIAG] IM[0]=%h IM[1]=%h (expect pixels 1-8 and 9-12)", d0, d1);
        end

        // Configure
        cfg.in_ch           <= 8'd1;
        cfg.out_ch          <= 8'd8;
        cfg.in_w            <= 8'(T3_IN_W);
        cfg.in_h            <= 8'(T3_IN_H);
        cfg.out_w           <= 8'(T3_OUT_W);
        cfg.out_h           <= 8'(T3_OUT_H);
        cfg.kernel_w        <= 4'(TN_KERNEL);
        cfg.kernel_h        <= 4'(TN_KERNEL);
        cfg.stride_w        <= 4'd1;
        cfg.stride_h        <= 4'd1;
        cfg.pad_w           <= 4'(T3_PAD);
        cfg.pad_h           <= 4'(T3_PAD);
        cfg.zp_y            <= 8'd0;
        cfg.im_total_writes <= 16'(T3_WRITES);
        cfg.im_row_stride   <= 8'(T3_IM_ROW_STRIDE);
        cfg.im_ch_stride    <= 10'(T3_IM_CH_STRIDE);
        cfg.im_read_base    <= 9'(T3_IM_RD_BASE);
        cfg.out_row_stride  <= 8'd1;
        cfg.out_ch_stride   <= 10'd8;
        cfg.im_write_base   <= 9'(T3_IM_WR_BASE);
        cfg.wt_ch_stride    <= 8'(TN_KERNEL);
        cfg.wt_row_stride   <= 8'd1;
        cfg.wt_ocg_stride   <= 10'(TN_KERNEL);
        cfg.mode            <= MODE_8x8;
        cfg.pool_bypass     <= 1'b1;
        cfg.keep_accum      <= 1'b0;
        cfg.disable_flush   <= 1'b0;
        cfg.ct_mode         <= CT_GEARBOX;
        cfg.reserved_w1     <= 5'd0;
        cfg.reserved_w2     <= 5'd0;
        cfg.reserved_w3     <= 1'b0;

        cfg_valid <= 1'b1;
        @(posedge core_clk);
        cfg_valid <= 1'b0;

        wait_layer_done();
        $display("  layer_done at t=%0t", $time);

        // Read back and compare
        local_errs = 0;
        for (k = 0; k < T3_WRITES; k = k + 1) begin
            im_read_word(9'(T3_IM_WR_BASE + k), rd);
            if (rd !== exp_pad_write[k]) begin
                if (local_errs < 10)
                    $display("  FAIL write[%0d]: got=%h exp=%h", k, rd, exp_pad_write[k]);
                local_errs = local_errs + 1;
            end
        end

        if (local_errs == 0)
            $display("  All %0d writes match golden ✓", T3_WRITES);
        else begin
            $display("  %0d / %0d MISMATCHES", local_errs, T3_WRITES);
            errors = errors + local_errs;
        end
    endtask

    // ═══════════════════════════════════════════════════════════════
    // Main
    // ═══════════════════════════════════════════════════════════════
    initial begin
        $display("\n==== TB_COMPUTE_CORE (GOLDEN) STARTED ====");

        @(posedge core_rst_n);
        tick(3);

        cfg_valid   <= 1'b0;
        cfg         <= '0;
        im_ext_cs   <= 1'b0;
        im_ext_wr   <= 1'b0;
        im_ext_addr <= 9'd0;
        im_ext_wdata <= 64'd0;
        tick(2);

        // Quick diag
        begin
            automatic logic [63:0] diag_w, diag_r;
            diag_w = 64'hDEADBEEFCAFE1234;
            im_write_word(9'd200, diag_w);
            im_read_word(9'd200, diag_r);
            if (diag_r !== diag_w) begin
                $display("[DIAG] IM ext FAIL W=%h R=%h", diag_w, diag_r);
                $finish;
            end
        end

        test1_conv5x5_golden();
        test2_pool_spatial();
        test3_pad_bypass();

        $display("\n═══════════════════════════════════════");
        if (errors == 0)
            $display("  PASSED — all golden checks match");
        else
            $display("  %0d TOTAL ERRORS", errors);
        $display("═══════════════════════════════════════\n");
        $finish;
    end

    initial begin
        #2000000;
        $display("GLOBAL TIMEOUT (state=%0d)", core_fsm_state);
        $finish;
    end

endmodule

`default_nettype wire
