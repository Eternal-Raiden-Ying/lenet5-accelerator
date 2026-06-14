// ═══════════════════════════════════════════════════════════════════
// tb_dma_integ — Full wrapper integration: APB + DMA + AHB + SRAM
//   dma_scheduler + ahb_master + shadow_register + apb_slave + compute_mgmt
//   over a 2-layer descriptor chain with APB CSR access for start/status.
// ═══════════════════════════════════════════════════════════════════
`default_nettype none
`timescale 1ns / 100ps

import wrapper_pkg::*;
import core_pkg::*;

module tb_dma_integ;
    logic clk, rst_n;

    // ── APB bus ──
    logic        PSEL, PENABLE, PWRITE;
    logic [11:0] PADDR;
    logic [31:0] PWDATA, PRDATA;
    logic        soft_reset;

    // ── APB → dma_scheduler ──
    logic        csr_start_pulse;
    logic [31:0] desc_head_ptr;

    // ── DMA handshake ──
    logic        dma_ready, compute_done;
    logic        bank_toggle;
    logic        core_busy;
    logic        inference_done, error_flag;
    logic  [3:0] error_code;
    dma_state_t  dma_fsm_state;
    comp_state_t comp_fsm_state;
    ahb_state_t  ahb_fsm_state;

    // ── compute_mgmt → Core ──
    logic        cfg_valid;
    logic        layer_done;

    // ── AHB Master (struct-ported) ──
    dma_req_t    dma_req;
    dma_rsp_t    dma_rsp;

    // ── AHB bus ──
    logic [31:0] HADDRM;
    htrans_t     HTRANSM;
    logic        HWRITEM;
    logic  [2:0] HSIZEM;
    hburst_t     HBURSTM;
    logic [31:0] HWDATAM;
    logic [31:0] HRDATAM;
    logic        HREADYM;
    logic        HMASTLOCKM;

    // ── Shadow Register ──
    logic        sr_wr_en;
    logic [31:0] sr_wr_data;
    desc_cfg_t   cfg_out;
    desc_dma_t   sr_dma;

    // ── Gearbox / IM ext ──
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
    apb_slave u_apb (
        .PCLK(clk), .PRESETn(rst_n),
        .PSEL(PSEL), .PENABLE(PENABLE), .PWRITE(PWRITE),
        .PADDR(PADDR), .PWDATA(PWDATA),
        .PRDATA(PRDATA), .PREADY(),
        .csr_start_pulse(csr_start_pulse),
        .desc_head_ptr(desc_head_ptr),
        .soft_reset(soft_reset),
        .inference_done(inference_done),
        .error_flag(error_flag),
        .error_code(error_code),
        .dma_fsm_state(dma_fsm_state),
        .comp_fsm_state(comp_fsm_state),
        .ahb_fsm_state(ahb_fsm_state),
        .bank_toggle(bank_toggle),
        .core_busy(core_busy),
        .core_fsm_state(CORE_F_IDLE)
    );

    compute_mgmt u_comp (
        .clk(clk), .rst_n(rst_n),
        .dma_ready(dma_ready), .compute_done(compute_done),
        .bank_toggle(bank_toggle),
        .sr_is_last(sr_dma.is_last),
        .cfg_valid(cfg_valid),
        .layer_done(layer_done), .core_error(2'b00),
        .inference_done(inference_done),
        .comp_fsm_state(comp_fsm_state),
        .error_flag(), .error_code()
    );

    dma_scheduler u_dma (
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

    ahb_master u_ahb (
        .HCLK(clk), .HRESETn(rst_n),
        .HADDRM(HADDRM), .HTRANSM(HTRANSM), .HWRITEM(HWRITEM),
        .HSIZEM(HSIZEM), .HBURSTM(HBURSTM), .HWDATAM(HWDATAM),
        .HRDATAM(HRDATAM), .HREADYM(HREADYM), .HRESP(1'b0),
        .HMASTLOCKM(HMASTLOCKM),
        .req(dma_req), .rsp(dma_rsp),
        .ahb_fsm_state(ahb_fsm_state)
    );

    shadow_register u_sr (
        .clk(clk), .rst_n(rst_n),
        .sr_wr_en(sr_wr_en), .sr_wr_data(sr_wr_data),
        .cfg_out(cfg_out), .dma_out(sr_dma)
    );

    initial clk=0; always #2.5 clk=~clk;

    // Waveform dump
    initial begin
        $dumpfile("tb_dma_integ.vcd");
        $dumpvars(0, tb_dma_integ);
    end

    // ═══════════════════════════════════════════════════════════════
    // Behavioral models
    // ═══════════════════════════════════════════════════════════════

    // ── AHB-Lite memory slave ──
    logic [31:0] mem [0:16383];
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) HRDATAM <= 32'd0;
        else if (HREADYM && (HTRANSM == HTRANS_NONSEQ || HTRANSM == HTRANS_SEQ)) begin
            if (HWRITEM) mem[HADDRM[15:2]] <= HWDATAM;
            HRDATAM <= mem[HADDRM[15:2]];
        end
    end

    // ── IM SRAM model ──
    logic [63:0] imram [0:511];
    logic [63:0] im_rd_r; logic im_rdy_r;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin im_rd_r<=0; im_rdy_r<=0; end
        else begin
            if (im_ext_cs && im_ext_wr) imram[im_ext_addr]<=im_ext_wdata;
            if (im_ext_cs && !im_ext_wr) begin im_rd_r<=imram[im_ext_addr]; im_rdy_r<=1; end
            else im_rdy_r<=0;
        end
    end
    assign im_ext_rdata=im_rd_r; assign im_ext_ready=im_rdy_r;

    // ── emulate bus_wrapper bank_toggle ──
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) bank_toggle <= 1'b0;
        else if (compute_done && dma_ready) bank_toggle <= ~bank_toggle;
    end

    // ── Helpers ──
    function [13:0] widx(input [31:0] ba); widx = ba[15:2]; endfunction

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
            // Word 0 [31:0]: in_h[31:24], in_w[23:16], out_ch[15:8], in_ch[7:0]
            mem[widx(base)+0] = {8'd28, 8'd28, 8'd8, 8'd1};
            // Word 1 [63:32]: stride_h, stride_w, kernel_h, kernel_w, out_h, out_w
            mem[widx(base)+1] = {8'd1, 8'd1, 4'd5, 4'd5, 8'd24, 8'd24};
            // Word 2 [95:64]: im_total_writes[95:80], zp_y[79:72], pad_h[71:68], pad_w[67:64]
            mem[widx(base)+2] = {im_tw, 8'd0, 4'd0, 4'd0};
            // Word 3 [127:96]: reserved_w1[127:123], im_read_base[122:114],
            //                  im_ch_stride[113:104], im_row_stride[103:96]
            mem[widx(base)+3] = {5'd0, im_rd, 10'd0, 8'd0};
            // Word 4 [159:128]: reserved_w2[159:155], im_write_base[154:146],
            //                  out_ch_stride[145:136], out_row_stride[135:128]
            mem[widx(base)+4] = {5'd0, im_wr, 10'd0, 8'd3};
            // Word 5 [191:160]: reserved_w3[191], ct_mode[190], disable_flush[189],
            //   keep_accum[188], pool_bypass[187], mode[186],
            //   wt_ocg_stride[185:176], wt_row_stride[175:168], wt_ch_stride[167:160]
            mem[widx(base)+5] = 32'd0;
            // DMA fields (shadow_register shift-reg order = AHB INCR16 order):
            //   Word  6: weight_bytes    [223:192]
            //   Word  7: requant_bytes   [255:224]
            //   Word  8: weight_ddr_ptr  [287:256]
            //   Word  9: requant_ddr_ptr [319:288]
            //   Word 10: feature_ddr_ptr [351:320]
            //   Word 11: result_ddr_ptr  [383:352]
            //   Word 12: feature_bytes   [415:384]
            //   Word 13: next_desc       [447:416]
            //   Word 14: reserved[479:449], is_last[448]
            mem[widx(base)+6]  = wt_bytes;
            mem[widx(base)+7]  = rq_bytes;
            mem[widx(base)+8]  = wt_ptr;
            mem[widx(base)+9]  = rq_ptr;
            mem[widx(base)+10] = feat_ptr;
            mem[widx(base)+11] = res_ptr;
            mem[widx(base)+12] = feat_bytes;
            mem[widx(base)+13] = next_desc;
            mem[widx(base)+14] = {31'd0, is_last};  // is_last @ desc_raw[448] = Word14 bit0
        end
    endtask

    // ── APB tasks ──
    task apb_write(input [11:0] addr, input [31:0] data);
        begin
            @(posedge clk);
            PSEL=1; PENABLE=0; PWRITE=1; PADDR=addr; PWDATA=data;
            @(posedge clk);
            PENABLE=1;
            @(posedge clk);
            PSEL=0; PENABLE=0;
        end
    endtask

    task apb_read(input [11:0] addr, output [31:0] data);
        begin
            @(posedge clk);
            PSEL=1; PENABLE=0; PWRITE=0; PADDR=addr;
            @(posedge clk);
            PENABLE=1;
            @(posedge clk);
            data = PRDATA;
            PSEL=0; PENABLE=0;
        end
    endtask

    task wait_ready; integer g; begin g=0;
        while(!dma_ready && g<5000) begin @(posedge clk); g=g+1; end end endtask
    task tick(input integer n); integer i; begin for(i=0;i<n;i=i+1) @(posedge clk); end endtask

    // ═══════════════════════════════════════════════════════════════
    // Stimulus — 2-layer chain via APB + real ahb_master
    // ═══════════════════════════════════════════════════════════════
    localparam D0=32'h0100, D1=32'h0200, FEAT=32'h1000, RES=32'h2000, WT=32'h3000, RQ=32'h4000;
    integer i, k, fcnt;
    logic [63:0] exp;
    logic [31:0] lo, hi;
    logic [31:0] rd;

    initial begin
        // Init
        PSEL=0; PENABLE=0; PWRITE=0; PADDR=0; PWDATA=0;
        rst_n=0; core_busy=0; layer_done=0; HREADYM=1;
        for(i=0;i<16384;i=i+1) mem[i]=32'd0;
        for(i=0;i<512;i=i+1) imram[i]=64'd0;

        // Build descriptors in AHB memory
        put_desc(D0, 32'd64, 32'd36, WT, RQ, FEAT, 32'd0, 32'd128, D1,
                 1'b0, 9'd0, 9'd0, 16'd0);
        for(i=0;i<32;i=i+1) mem[widx(FEAT)+i]=32'hA0000000+i;

        put_desc(D1, 32'd64, 32'd36, WT, RQ, 32'd0, RES, 32'd0, 32'd0,
                 1'b1, 9'd0, 9'd20, 16'd5);
        for(i=0;i<5;i=i+1) imram[20+i]={32'hBB000000+i, 32'hAA000000+i};

        tick(3); rst_n=1; tick(2);

        // ── APB: write DESC_HEAD_PTR, then CTRL.start ──
        $display("[APB] Write DESC_HEAD_PTR=0x%08h", D0);
        apb_write({4'h0, APB_DESC_HEAD_PTR}, D0);

        // Read back to verify
        apb_read({4'h0, APB_DESC_HEAD_PTR}, rd);
        if (rd !== D0) begin
            $display("FAIL: DESC_HEAD_PTR readback=%h expect %h", rd, D0); errors=errors+1;
        end

        $display("[APB] Write CTRL.start=1");
        apb_write({4'h0, APB_CTRL}, 32'd1);
        // Wait for 2-FF synchronizer to propagate → csr_start_pulse
        tick(5);

        // Read VERSION
        apb_read({4'h0, APB_VERSION}, rd);
        $display("[APB] VERSION=0x%08h", rd);

        // ── PREFILL ──
        wait_ready();
        fcnt = sr_dma.feature_bytes >> 3;
        $display("[T] PREFILL ready t=%0t, FETCH=%0d IM words", $time, fcnt);
        for (k=0;k<fcnt;k=k+1) begin
            exp = {mem[widx(FEAT)+2*k+1], mem[widx(FEAT)+2*k]};
            if (imram[k] !== exp) begin
                $display("  FAIL FETCH w%0d: got=%h exp=%h",k,imram[k],exp); errors=errors+1; end
        end
        if (fcnt!=16) begin $display("  FAIL FETCH count=%0d exp 16",fcnt); errors=errors+1; end
        if (errors==0) $display("  FETCH ok: 16 words via real ahb_master INCR16");

        // Read DEBUG_STATE
        apb_read({4'h0, APB_DEBUG_STATE}, rd);
        $display("[APB] DEBUG_STATE=0x%08h (dma_fsm=%0d)", rd, rd[1:0]);

        // ── Hand off to Core (emulate layer compute) ──
        core_busy=1; tick(2);

        // ── PREFETCH layer1 ──
        wait_ready();
        $display("[T] PREFETCH ready (layer1) t=%0t", $time);

        // Emulate Core computing: layer_done pulse triggers compute_mgmt
        tick(3);
        layer_done=1; @(posedge clk); #0.1 layer_done=0;
        tick(2);

        // Wait for bank_toggle flip (compute_mgmt handles the sequence)
        tick(5);

        // ── PREFETCH phase0 detects saved_is_last=1 → D_TAIL ──
        while(dma_fsm_state!=D_TAIL) @(posedge clk);
        $display("[T] entered D_TAIL at t=%0t", $time);

        // Emulate last layer compute done
        layer_done=1; @(posedge clk); #0.1 layer_done=0;
        tick(3);

        // ── WRITEBACK complete → check APB INT_STATUS ──
        while(!inference_done) @(posedge clk);
        $display("[T] WRITEBACK done (inference_done) at t=%0t", $time);

        apb_read({4'h0, APB_INT_STATUS}, rd);
        $display("[APB] INT_STATUS=0x%08h (inf_done=%0d)", rd, rd[0]);
        if (rd[0] !== 1'b1) begin
            $display("FAIL: INT_STATUS[0] not set"); errors=errors+1;
        end

        // Verify WRITEBACK data
        for (k=0;k<5;k=k+1) begin
            lo=mem[widx(RES)+2*k]; hi=mem[widx(RES)+2*k+1];
            if (lo!==(32'hAA000000+k) || hi!==(32'hBB000000+k)) begin
                $display("  FAIL WB w%0d: lo=%h hi=%h",k,lo,hi); errors=errors+1; end
        end
        if (errors==0) $display("  WRITEBACK ok: 5×64b @ ×8 step");

        // Clear INT_STATUS via APB INT_CLEAR
        apb_write({4'h0, APB_INT_CLEAR}, 32'd1);
        apb_read({4'h0, APB_INT_STATUS}, rd);
        if (rd[0] !== 1'b0) begin
            $display("FAIL: INT_STATUS[0] not cleared"); errors=errors+1;
        end
        $display("[APB] INT_STATUS cleared ok");

        if (errors==0) $display("\n==== DMA+APB INTEG: ALL CHECKS PASSED ====");
        else           $display("\n==== %0d ERRORS ====", errors);
        $finish;
    end

    initial begin #500000; $display("GLOBAL TIMEOUT st=%0d",dma_fsm_state); $finish; end
endmodule

`default_nettype wire
