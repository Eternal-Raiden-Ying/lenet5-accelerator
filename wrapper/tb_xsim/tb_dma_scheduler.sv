// ═══════════════════════════════════════════════════════════════════
// tb_dma_scheduler — verify FETCH addr increment (#2), WRITEBACK ×8 step
//   (#3), and feature_bytes-derived counts (#4) over a 2-layer desc chain.
//
// Behavioral models (inlined): AHB slave, IM SRAM, descriptor builder.
// Adapted for struct-ported DUT (dma_req_t, dma_rsp_t, gb_wr_t, desc_dma_t).
// ═══════════════════════════════════════════════════════════════════
`default_nettype none
`timescale 1ns / 100ps

import wrapper_pkg::*;

module tb_dma_scheduler;
    logic clk, rst_n;

    // ── DUT control/handshake ──
    logic        csr_start_pulse;
    logic [31:0] desc_head_ptr;
    logic        dma_ready;
    logic        compute_done;
    logic        bank_toggle;
    logic        core_busy;
    logic        inference_done;
    logic        error_flag;
    logic  [3:0] error_code;
    dma_state_t  dma_fsm_state;

    // ── AHB Master interface (struct-ported) ──
    dma_req_t    dma_req;
    dma_rsp_t    dma_rsp;

    // ── Shadow Register ──
    logic        sr_wr_en;
    logic [31:0] sr_wr_data;
    desc_cfg_t   cfg_out;
    desc_dma_t   sr_dma;

    // ── Gearbox + IM ext ──
    gb_wr_t      wt_gb, rq_gb;
    logic        gearbox_rst;
    logic        im_ext_cs, im_ext_wr;
    logic  [8:0] im_ext_addr;
    logic [63:0] im_ext_wdata;
    logic [63:0] im_ext_rdata;
    logic        im_ext_ready;

    integer errors = 0;

    // ═══════════════════════════════════════════════════════════════
    // DUTs
    // ═══════════════════════════════════════════════════════════════
    dma_scheduler u_dut (
        .clk(clk), .rst_n(rst_n),
        .csr_start_pulse(csr_start_pulse), .desc_head_ptr(desc_head_ptr),
        .dma_req(dma_req), .dma_rsp(dma_rsp),
        .sr_wr_en(sr_wr_en), .sr_wr_data(sr_wr_data),
        .sr_dma(sr_dma),
        .sr_cfg(cfg_out),
        .dma_ready(dma_ready), .compute_done(compute_done), .bank_toggle(bank_toggle),
        .core_busy(core_busy),
        .wt_gb(wt_gb), .rq_gb(rq_gb), .gearbox_rst(gearbox_rst),
        .im_ext_cs(im_ext_cs), .im_ext_wr(im_ext_wr),
        .im_ext_addr(im_ext_addr), .im_ext_wdata(im_ext_wdata),
        .im_ext_rdata(im_ext_rdata), .im_ext_ready(im_ext_ready),
        .inference_done(inference_done), .error_flag(error_flag),
        .error_code(error_code), .dma_fsm_state(dma_fsm_state)
    );

    shadow_register u_sr (
        .clk(clk), .rst_n(rst_n),
        .sr_wr_en(sr_wr_en), .sr_wr_data(sr_wr_data),
        .cfg_out(cfg_out), .dma_out(sr_dma)
    );

    initial clk=0; always #2.5 clk=~clk;

    // Waveform dump
    initial begin
        $dumpfile("tb_dma_scheduler.vcd");
        $dumpvars(0, tb_dma_scheduler);
    end

    // ═══════════════════════════════════════════════════════════════
    // Behavioral models
    // ═══════════════════════════════════════════════════════════════

    // ── DDR memory ──
    logic [31:0] mem [0:16383];
    function [13:0] widx(input [31:0] byte_addr); widx = byte_addr[15:2]; endfunction

    // ── AHB slave: serves reads, captures writes, drives dma_rsp struct ──
    logic        ahb_busy;
    logic [31:0] ahb_addr;
    logic [15:0] ahb_left;
    logic        ahb_wr;
    logic        ahb_dvalid;
    logic [31:0] ahb_daddr;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ahb_busy<=0; ahb_left<=0; ahb_wr<=0;
            dma_rsp.rdata<=0; dma_rsp.rdata_valid<=0; dma_rsp.done<=0;
            dma_rsp.active<=0; dma_rsp.error<=0; ahb_dvalid<=0;
        end else begin
            dma_rsp.rdata_valid<=0; dma_rsp.done<=0;
            if (dma_req.valid && !ahb_busy) begin
                ahb_busy<=1; dma_rsp.active<=1;
                ahb_addr<=dma_req.addr+32'd4; ahb_left<=dma_req.word_count-16'd1;
                ahb_wr<=dma_req.dir; ahb_daddr<=dma_req.addr; ahb_dvalid<=1;
                $display("[AHB] START addr=%h wcnt=%0d dir=%0d", dma_req.addr, dma_req.word_count, dma_req.dir);
                if (dma_req.dir) mem[widx(dma_req.addr)]<=dma_req.wdata;
            end else if (ahb_busy) begin
                if (ahb_dvalid && !ahb_wr) begin
                    dma_rsp.rdata<=mem[widx(ahb_daddr)]; dma_rsp.rdata_valid<=1;
                end
                if (ahb_left!=0) begin
                    ahb_daddr<=ahb_addr; ahb_dvalid<=1;
                    if (ahb_wr) mem[widx(ahb_addr)]<=dma_req.wdata;
                    ahb_addr<=ahb_addr+32'd4; ahb_left<=ahb_left-16'd1;
                end else begin
                    dma_rsp.done<=1; ahb_busy<=0; dma_rsp.active<=0; ahb_dvalid<=0;
                    $display("[AHB] DONE");
                end
            end
        end
    end

    // ── IM SRAM model ──
    logic [63:0] imram [0:511];
    logic [63:0] im_ext_rdata_r; logic im_ext_ready_r;
    assign im_ext_rdata = im_ext_rdata_r;
    assign im_ext_ready = im_ext_ready_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin im_ext_rdata_r<=0; im_ext_ready_r<=0; end
        else begin
            if (im_ext_cs && im_ext_wr) begin
                imram[im_ext_addr]<=im_ext_wdata;
                $display("  IM_WR addr=%0d data=%h", im_ext_addr, im_ext_wdata);
            end
            if (im_ext_cs && !im_ext_wr) begin
                im_ext_rdata_r<=imram[im_ext_addr]; im_ext_ready_r<=1;
            end else im_ext_ready_r<=0;
        end
    end

    // ── Descriptor builder (v4.2 packed struct layout, word-by-word) ──
    //   Word 0: in_h, in_w, out_ch, in_ch
    //   Word 1: stride_h, stride_w, kernel_h, kernel_w, out_h, out_w
    //   Word 2: im_total_writes, zp_y, reserved, pad_h, pad_w
    //   Word 3: reserved_w1, im_read_base, im_ch_stride, im_row_stride
    //   Word 4: reserved_w2, im_write_base, out_ch_stride, out_row_stride
    //   Word 5: reserved_w3, ct_mode, disable_flush, keep_accum, pool_bypass, mode,
    //            wt_ocg_stride, wt_row_stride, wt_ch_stride
    //   Word 6-14: DMA fields (weight_bytes..is_last+reserved_dma)
    task put_desc(input [31:0] base,
                  input [31:0] wt_bytes, input [31:0] rq_bytes,
                  input [31:0] wt_ptr,   input [31:0] rq_ptr,
                  input [31:0] feat_ptr, input [31:0] res_ptr,
                  input [31:0] feat_bytes, input [31:0] next_desc,
                  input        is_last,
                  input  [8:0] im_rd, input  [8:0] im_wr, input [15:0] im_tw);
        integer b;
        begin
            for (b=0;b<16;b=b+1) mem[widx(base)+b]=32'd0;
            // Word 0: in_h(8), in_w(8), out_ch(8), in_ch(8)
            mem[widx(base)+0] = {8'd28, 8'd28, 8'd8, 8'd1};
            // Word 1: stride_h(4), stride_w(4), kernel_h(4), kernel_w(4), out_h(8), out_w(8)
            mem[widx(base)+1] = {4'd1, 4'd1, 4'd5, 4'd5, 8'd24, 8'd24};
            // Word 2: im_total_writes(16), zp_y(8), pad_h(4), pad_w(4)
            mem[widx(base)+2] = {im_tw, 8'd0, 4'd0, 4'd0};
            // Word 3: reserved_w1(5), im_read_base(9), im_ch_stride(10), im_row_stride(8)
            mem[widx(base)+3] = {5'd0, im_rd, 10'd0, 8'd0};
            // Word 4: reserved_w2(5), im_write_base(9), out_ch_stride(10), out_row_stride(8)
            mem[widx(base)+4] = {5'd0, im_wr, 10'd0, 8'd3};
            // Word 5: reserved_w3(1), ct_mode(1), disable_flush(1), keep_accum(1),
            //          pool_bypass(1), mode(1), wt_ocg_stride(10), wt_row_stride(8), wt_ch_stride(8)
            mem[widx(base)+5] = {1'd0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 10'd0, 8'd0, 8'd0};
            // Word 6-14: DMA fields
            mem[widx(base)+6]  = wt_bytes;
            mem[widx(base)+7]  = rq_bytes;
            mem[widx(base)+8]  = wt_ptr;
            mem[widx(base)+9]  = rq_ptr;
            mem[widx(base)+10] = feat_ptr;
            mem[widx(base)+11] = res_ptr;
            mem[widx(base)+12] = feat_bytes;
            mem[widx(base)+13] = next_desc;
            mem[widx(base)+14] = {31'd0, is_last};
        end
    endtask

    // ── Helpers ──
    task wait_ready; begin while(!dma_ready) @(posedge clk); end endtask
    task tick(input integer n); integer i; begin for(i=0;i<n;i=i+1) @(posedge clk); end endtask

    // emulate bus_wrapper bank_toggle: (compute_done && dma_ready) → ~bank_toggle
    // Use always_ff with reset to avoid multi-driver on bank_toggle logic
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            bank_toggle <= 1'b0;
        else if (compute_done && dma_ready)
            bank_toggle <= ~bank_toggle;
    end

    // ═══════════════════════════════════════════════════════════════
    // Stimulus — 2-layer chain
    // ═══════════════════════════════════════════════════════════════
    localparam D0=32'h0100, D1=32'h0200;
    localparam FEAT=32'h1000, RES=32'h2000;
    localparam WT=32'h3000, RQ=32'h4000;
    integer i; integer fcnt;

    initial begin
        rst_n=0; csr_start_pulse=0; desc_head_ptr=D0;
        compute_done=0; core_busy=0;
        for(i=0;i<16384;i=i+1) mem[i]=32'd0;
        for(i=0;i<512;i=i+1) imram[i]=64'd0;

        put_desc(D0, 32'd64, 32'd36, WT, RQ, FEAT, 32'd0, 32'd32, D1,
                 1'b0, 9'd0, 9'd0, 16'd0);
        $display("[INIT] D0[0]=%h", mem[widx(D0)+0]);
        $display("[INIT] D0[6]=%h (wt_bytes=40)", mem[widx(D0)+6]);
        $display("[INIT] D0[8]=%h (wt_ptr=3000)", mem[widx(D0)+8]);
        for(i=0;i<8;i=i+1) mem[widx(FEAT)+i]=32'hA0000000+i;

        put_desc(D1, 32'd64, 32'd36, WT, RQ, 32'd0, RES, 32'd0, 32'd0,
                 1'b1, 9'd0, 9'd4, 16'd4);
        for(i=0;i<4;i=i+1) imram[4+i]={32'hBB000000+i, 32'hAA000000+i};

        tick(3); rst_n=1; tick(2);

        // ── Start ──
        csr_start_pulse=1; @(posedge clk); #0.1 csr_start_pulse=0;

        // ── PREFILL: wait dma_ready, check FETCH ──
        wait_ready();
        $display("[T] PREFILL dma_ready at t=%0t", $time);
        check_fetch();

        // ── Hand off to Core ──
        core_busy=1; tick(2);

        // ── PREFETCH: layer1 prefetch ──
        wait_ready();
        $display("[T] PREFETCH dma_ready at t=%0t", $time);
        compute_done=1; @(posedge clk); #0.1;
        @(posedge clk); compute_done=0;
        tick(2);

        // ── D_TAIL: writeback ──
        while(dma_fsm_state!=D_TAIL) @(posedge clk);
        $display("[T] entered D_TAIL at t=%0t", $time);
        compute_done=1;
        check_writeback();

        if (errors==0) $display("\n==== DMA SCHEDULER: ALL CHECKS PASSED ====");
        else           $display("\n==== %0d ERRORS ====", errors);
        $finish;
    end

    task check_fetch;
        integer k; logic [63:0] exp;
        begin
            fcnt = sr_dma.feature_bytes >> 3;
            $display("  FETCH expected %0d IM words", fcnt);
            for (k=0;k<fcnt;k=k+1) begin
                exp = {mem[widx(FEAT)+2*k+1], mem[widx(FEAT)+2*k]};
                if (imram[k] !== exp) begin
                    $display("  FAIL FETCH word%0d: imram=%h exp=%h", k, imram[k], exp);
                    errors=errors+1;
                end
            end
            if (errors==0) $display("  FETCH ok: %0d words", fcnt);
        end
    endtask

    task check_writeback;
        integer k; logic [31:0] lo, hi;
        begin
            while(!inference_done) @(posedge clk);
            $display("  WRITEBACK done (inference_done) at t=%0t", $time);
            for (k=0;k<4;k=k+1) begin
                lo = mem[widx(RES)+2*k];
                hi = mem[widx(RES)+2*k+1];
                if (lo!==(32'hAA000000+k) || hi!==(32'hBB000000+k)) begin
                    $display("  FAIL WB word%0d: lo=%h hi=%h", k, lo, hi); errors=errors+1;
                end
            end
            if (errors==0) $display("  WRITEBACK ok: 4×64b @ ×8 step, no overlap");
        end
    endtask

    initial begin #200000; $display("GLOBAL TIMEOUT (state=%0d)", dma_fsm_state); $finish; end
endmodule

`default_nettype wire
