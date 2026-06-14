// ═══════════════════════════════════════════════════════════════════════════════
// dma_scheduler — DMA FSM (4 states), owns all AHB + im_ext traffic
//
// D_IDLE → D_PREFILL(desc[0]+wt[0]+rq[0]+FEATURE→BankA, wait core_busy)
//        → D_PREFETCH(wait core_busy→is_last?→desc+wt+rq→~bank_toggle→dma_ready
//                     →wait bank_toggle flip→loop)
//        → D_TAIL(wait compute_done→WRITEBACK→inference_done)
//
// FIXES vs prior version:
//   #2 FETCH_FEATURE now increments im_ext_addr on every 64-bit IM write.
//   #3 WRITEBACK result address steps by 8 bytes per 64-bit word ({cnt,3'b0});
//      single-point WRITEBACK sub-FSM (wb_sub) replaces the duplicated/conflicting
//      AHB-write logic; clean im_ext-read → 2×32b-write → advance sequence.
//   #4 FETCH word counts derived from dma_feature_bytes (padded, compiler-filled):
//      AHB 32b words = bytes>>2, IM 64b writes = bytes>>3 (exact: 8-byte aligned).
//      sr_in_ch/in_w/in_h no longer used → ports removed.
//
// Ports use packed structs (dma_req_t, dma_rsp_t, gb_wr_t, desc_dma_t) from
// wrapper_pkg. See wrapper_pkg.sv for per-field comments.
//
// Style: single-process FSM — state/phase transitions inline in the clocked
//   datapath block (avoids phase-vs-next_state race).
// ═══════════════════════════════════════════════════════════════════════════════
`default_nettype none

import wrapper_pkg::*;

module dma_scheduler (
    input  wire        clk,
    input  wire        rst_n,

    // ── From APB Slave (2-FF synced) ──
    input  wire        csr_start_pulse,
    input  wire [31:0] desc_head_ptr,

    // ── AHB Master ──
    //   dma_req / dma_rsp are packed structs (see wrapper_pkg: dma_req_t, dma_rsp_t)
    output dma_req_t   dma_req,
    input  dma_rsp_t   dma_rsp,

    // ── Shadow Register write ──
    output logic       sr_wr_en,
    output logic [31:0] sr_wr_data,

    // ── From Shadow Register (combinational unpack) ──
    //   sr_dma is a packed struct (see wrapper_pkg: desc_dma_t)
    //   sr_cfg is a packed struct (see wrapper_pkg: desc_cfg_t)
    input  desc_dma_t  sr_dma,
    input  desc_cfg_t  sr_cfg,            // 使用 im_read_base, im_write_base, im_total_writes

    // ── Handshake (dma_scheduler ↔ compute_mgmt) ──
    output logic       dma_ready,     // layer N+1 pre-fetched, Core can issue cfg_valid
    input  wire        compute_done,  // Core finished layer N
    input  wire        bank_toggle,   // 0=Core reads BankA, 1=Core reads BankB

    // ── From Core ──
    input  wire        core_busy,     // 1=Core computing, 0=Core IDLE

    // ── Weight Gearbox (32b→512b S2P) ──
    output gb_wr_t     wt_gb,         // packed struct: wr_en, wdata[31:0], bank_sel
    // ── Requant Gearbox (32b→288b S2P) ──
    output gb_wr_t     rq_gb,         // packed struct: wr_en, wdata[31:0], bank_sel
    // ── Gearbox reset (pulsed on new-layer entry) ──
    output logic       gearbox_rst,

    // ── IM SRAM External Access ──
    output logic       im_ext_cs,     // chip select (1=Wrapper owns IM)
    output logic       im_ext_wr,     // 1=write (FETCH), 0=read (WRITEBACK)
    output logic [8:0] im_ext_addr,   // 64-bit word address (0~511)
    output logic [63:0] im_ext_wdata, // write data (8 pixels × uint8)
    input  wire [63:0] im_ext_rdata,  // read data
    input  wire        im_ext_ready,  // access complete (1 cycle after request)

    // ── Status (to APB Slave) ──
    output logic       inference_done,// pulse: all layers complete
    output logic       error_flag,    // 1=error detected
    output logic [3:0] error_code,    // 0=OK, 1=AHB error
    output dma_state_t dma_fsm_state  // current FSM state for DEBUG_STATE
);

    dma_state_t state;
    dma_phase_t phase;   // sub-step counter; semantics differ per state (see per-state case)

    wb_sub_t  wb_sub;
    logic [31:0] wb_hi_data;

    logic [31:0] saved_next_desc, saved_weight_ptr, saved_requant_ptr;
    logic [31:0] saved_weight_bytes, saved_requant_bytes;
    logic [31:0] saved_feature_ptr, saved_feature_bytes;
    logic [31:0] saved_result_ptr;
    logic        saved_is_last;

    logic        desc_settle, desc_settle_d;  // 2-cycle settle: allow shift register to fully populate before reading sr_dma

    logic [15:0] im_ext_cnt, im_ext_total;
    logic        im_ext_phase;

    logic        bank_toggle_d;
    wire         bank_toggle_changed = (bank_toggle != bank_toggle_d);
    always_ff @(posedge clk) bank_toggle_d <= bank_toggle;

    logic [8:0]  im_base;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= D_IDLE;
            phase          <= PH_DESC;
            dma_req.valid  <= 1'b0;
            dma_req.addr   <= 32'd0;
            dma_req.word_count <= 16'd0;
            dma_req.dir    <= 1'b0;
            dma_req.wdata  <= 32'd0;
            dma_req.burst_mode <= BM_ALLOW_INCR;
            sr_wr_en       <= 1'b0;
            sr_wr_data     <= 32'd0;
            dma_ready      <= 1'b0;
            wt_gb.wr_en    <= 1'b0;
            wt_gb.wdata    <= 32'd0;
            wt_gb.bank_sel <= 1'b0;
            rq_gb.wr_en    <= 1'b0;
            rq_gb.wdata    <= 32'd0;
            rq_gb.bank_sel <= 1'b0;
            gearbox_rst    <= 1'b0;
            im_ext_cs      <= 1'b0;
            im_ext_wr      <= 1'b0;
            im_ext_addr    <= 9'd0;
            im_ext_wdata   <= 64'd0;
            im_base        <= 9'd0;
            inference_done <= 1'b0;
            error_flag     <= 1'b0;
            error_code     <= 4'd0;
            desc_settle    <= 1'b0;
            desc_settle_d  <= 1'b0;
            im_ext_cnt     <= 16'd0;
            im_ext_total   <= 16'd0;
            im_ext_phase   <= 1'b0;
            wb_sub         <= WB_READ;
            wb_hi_data     <= 32'd0;
            saved_next_desc     <= 32'd0;
            saved_weight_ptr    <= 32'd0;
            saved_requant_ptr   <= 32'd0;
            saved_weight_bytes  <= 32'd0;
            saved_requant_bytes <= 32'd0;
            saved_feature_ptr   <= 32'd0;
            saved_feature_bytes <= 32'd0;
            saved_result_ptr    <= 32'd0;
            saved_is_last       <= 1'b0;
        end else begin
            dma_req.valid <= 1'b0;
            sr_wr_en    <= 1'b0;
            wt_gb.wr_en <= 1'b0;
            rq_gb.wr_en <= 1'b0;
            gearbox_rst <= 1'b0;
            im_ext_wr   <= 1'b0;

            if (dma_rsp.error) begin
                error_flag <= 1'b1;
                error_code <= 4'd1;
                state      <= D_IDLE;
                phase      <= PH_DESC;
            end

            if (dma_rsp.rdata_valid) begin
                if ((state==D_PREFILL  && phase==PH_DESC) ||
                    (state==D_PREFETCH && phase==PH_WEIGHT)) begin
                    sr_wr_en       <= 1'b1;
                    sr_wr_data     <= dma_rsp.rdata;
                end else if ((state==D_PREFILL  && phase==PH_WEIGHT) ||
                             (state==D_PREFETCH && phase==PH_REQUANT)) begin
                    wt_gb.wr_en <= 1'b1;
                    wt_gb.wdata <= dma_rsp.rdata;
                end else if ((state==D_PREFILL  && phase==PH_REQUANT) ||
                             (state==D_PREFETCH && phase==PH_FETCH)) begin
                    rq_gb.wr_en <= 1'b1;
                    rq_gb.wdata <= dma_rsp.rdata;
                end else if (state==D_PREFILL && phase==PH_FETCH) begin
                    if (im_ext_phase == 1'b0) begin
                        im_ext_wdata[31:0] <= dma_rsp.rdata;
                        im_ext_phase       <= 1'b1;
                    end else begin
                        im_ext_wdata[63:32] <= dma_rsp.rdata;
                        im_ext_wr           <= 1'b1;
                        im_ext_addr         <= im_base + im_ext_cnt[8:0];
                        im_ext_cnt          <= im_ext_cnt + 16'd1;
                        im_ext_phase        <= 1'b0;
                    end
                end
            end

            case (state)
                D_IDLE: begin
                    dma_ready      <= 1'b0;
                    inference_done <= 1'b0;
                    if (csr_start_pulse) begin
                        phase          <= PH_DESC;
                        dma_req.valid  <= 1'b1;
                        dma_req.addr   <= desc_head_ptr;
                        dma_req.word_count <= 16'd16;
                        dma_req.dir    <= 1'b0;
                        dma_req.burst_mode <= BM_ALLOW_INCR;
                        state          <= D_PREFILL;
                    end
                end

                D_PREFILL: begin
                    case (phase)
                        PH_DESC: if (dma_rsp.done)      desc_settle <= 1'b1;
                              else if (desc_settle) begin
                            // 1st settle: wait 1 extra cycle for shift register to settle
                            desc_settle   <= 1'b0;
                            desc_settle_d <= 1'b1;
                        end else if (desc_settle_d) begin
                            // 2nd settle: read sr_dma, start weight read
                            desc_settle_d <= 1'b0;
                            saved_next_desc     <= sr_dma.next_desc;
                            saved_weight_ptr    <= sr_dma.weight_ddr_ptr;
                            saved_requant_ptr   <= sr_dma.requant_ddr_ptr;
                            saved_weight_bytes  <= sr_dma.weight_bytes;
                            saved_requant_bytes <= sr_dma.requant_bytes;
                            saved_feature_ptr   <= sr_dma.feature_ddr_ptr;
                            saved_feature_bytes <= sr_dma.feature_bytes;
                            saved_result_ptr    <= sr_dma.result_ddr_ptr;
                            saved_is_last       <= sr_dma.is_last;
                            dma_req.valid  <= 1'b1;
                            dma_req.addr   <= sr_dma.weight_ddr_ptr;
                            dma_req.word_count <= sr_dma.weight_bytes[17:2];
                            dma_req.dir    <= 1'b0;
                            dma_req.burst_mode <= BM_ALLOW_INCR;
                            gearbox_rst    <= 1'b1;
                            wt_gb.bank_sel <= 1'b0;
                            phase          <= PH_WEIGHT;
                        end
                        PH_WEIGHT: if (dma_rsp.done) begin
                            // gearbox_rst    <= 1'b1;
                            rq_gb.bank_sel <= 1'b0;
                            dma_req.valid  <= 1'b1;
                            dma_req.addr   <= saved_requant_ptr;
                            dma_req.word_count <= saved_requant_bytes[17:2];
                            dma_req.dir    <= 1'b0;
                            dma_req.burst_mode <= BM_ALLOW_INCR;
                            phase          <= PH_REQUANT;
                        end
                        PH_REQUANT: if (dma_rsp.done) begin
                            im_ext_cs      <= 1'b1;
                            im_ext_wr      <= 1'b0;
                            im_base        <= sr_cfg.im_read_base;
                            im_ext_total   <= sr_dma.feature_bytes[18:3];
                            im_ext_cnt     <= 16'd0;
                            im_ext_phase   <= 1'b0;
                            dma_req.valid  <= 1'b1;
                            dma_req.addr   <= saved_feature_ptr;
                            dma_req.word_count <= sr_dma.feature_bytes[17:2];
                            dma_req.dir    <= 1'b0;
                            dma_req.burst_mode <= BM_ALLOW_INCR;
                            phase          <= PH_FETCH;
                        end
                        PH_FETCH: if (im_ext_cnt == im_ext_total &&
                                  im_ext_total != 16'd0) begin
                            im_ext_cs <= 1'b0;
                            dma_ready <= 1'b1;
                            phase     <= PH_DONE;
                        end
                        PH_DONE: if (core_busy) begin
                            dma_ready <= 1'b0;
                            phase     <= PH_DESC;
                            state     <= saved_is_last ? D_TAIL : D_PREFETCH;
                        end
                    endcase
                end

                D_PREFETCH: begin
                    case (phase)
                        PH_DESC: begin
                            dma_ready <= 1'b0;
                            if (core_busy) begin
                                if (saved_is_last) begin
                                    phase <= PH_DESC;
                                    state <= D_TAIL;
                                end else begin
                                    dma_req.valid  <= 1'b1;
                                    dma_req.addr   <= saved_next_desc;
                                    dma_req.word_count <= 16'd16;
                                    dma_req.dir    <= 1'b0;
                                    dma_req.burst_mode <= BM_ALLOW_INCR;
                                    phase          <= PH_WEIGHT;
                                end
                            end
                        end
                        PH_WEIGHT: if (dma_rsp.done) desc_settle <= 1'b1;
                              else if (desc_settle) begin
                            desc_settle   <= 1'b0;
                            desc_settle_d <= 1'b1;
                        end else if (desc_settle_d) begin
                            desc_settle_d <= 1'b0;
                            saved_next_desc     <= sr_dma.next_desc;
                            saved_weight_ptr    <= sr_dma.weight_ddr_ptr;
                            saved_requant_ptr   <= sr_dma.requant_ddr_ptr;
                            saved_weight_bytes  <= sr_dma.weight_bytes;
                            saved_requant_bytes <= sr_dma.requant_bytes;
                            saved_is_last       <= sr_dma.is_last;
                            saved_feature_ptr   <= sr_dma.feature_ddr_ptr;
                            saved_feature_bytes <= sr_dma.feature_bytes;
                            if (sr_dma.is_last)
                                saved_result_ptr <= sr_dma.result_ddr_ptr;
                            dma_req.valid  <= 1'b1;
                            dma_req.addr   <= sr_dma.weight_ddr_ptr;
                            dma_req.word_count <= sr_dma.weight_bytes[17:2];
                            dma_req.dir    <= 1'b0;
                            dma_req.burst_mode <= BM_ALLOW_INCR;
                            gearbox_rst    <= 1'b1;
                            wt_gb.bank_sel <= ~bank_toggle;
                            phase          <= PH_REQUANT;
                        end
                        PH_REQUANT: if (dma_rsp.done) begin
                            // gearbox_rst    <= 1'b1;
                            rq_gb.bank_sel <= ~bank_toggle;
                            dma_req.valid  <= 1'b1;
                            dma_req.addr   <= saved_requant_ptr;
                            dma_req.word_count <= saved_requant_bytes[17:2];
                            dma_req.dir    <= 1'b0;
                            dma_req.burst_mode <= BM_ALLOW_INCR;
                            phase          <= PH_FETCH;
                        end
                        PH_FETCH: if (dma_rsp.done) begin
                            dma_ready <= 1'b1;
                            phase     <= PH_DONE;
                        end
                        PH_DONE: begin
                            dma_ready <= 1'b1;
                            if (bank_toggle_changed) begin
                                dma_ready <= 1'b0;
                                phase     <= PH_DESC;
                            end
                        end
                    endcase
                end

                D_TAIL: begin
                    dma_ready <= 1'b0;
                    case (phase)
                        PH_DESC: if (compute_done) begin
                            im_ext_cs    <= 1'b1;
                            im_ext_wr    <= 1'b0;
                            im_ext_addr  <= sr_cfg.im_write_base;
                            im_ext_cnt   <= 16'd0;
                            im_ext_total <= sr_cfg.im_total_writes;
                            wb_sub       <= WB_READ;
                            phase        <= PH_WEIGHT;
                        end
                        PH_WEIGHT: begin
                            case (wb_sub)
                                WB_READ: if (im_ext_ready) begin
                                    wb_hi_data     <= im_ext_rdata[63:32];
                                    dma_req.valid  <= 1'b1;
                                    dma_req.addr   <= saved_result_ptr +
                                                      {im_ext_cnt, 3'b000};
                                    dma_req.word_count <= 16'd1;
                                    dma_req.dir    <= 1'b1;
                                    dma_req.wdata  <= im_ext_rdata[31:0];
                                    dma_req.burst_mode <= BM_SINGLE_ONLY;
                                    im_ext_cs      <= 1'b0;
                                    wb_sub         <= WB_LO;
                                end
                                WB_LO: if (dma_rsp.done) begin
                                    dma_req.valid  <= 1'b1;
                                    dma_req.addr   <= saved_result_ptr +
                                                      {im_ext_cnt, 3'b000} + 32'd4;
                                    dma_req.word_count <= 16'd1;
                                    dma_req.dir    <= 1'b1;
                                    dma_req.wdata  <= wb_hi_data;
                                    dma_req.burst_mode <= BM_SINGLE_ONLY;
                                    wb_sub         <= WB_HI;
                                end
                                WB_HI: if (dma_rsp.done) begin
                                    im_ext_cnt <= im_ext_cnt + 16'd1;
                                    if (im_ext_cnt + 16'd1 >= im_ext_total) begin
                                        inference_done <= 1'b1;
                                        phase          <= PH_DESC;
                                        state          <= D_IDLE;
                                    end else begin
                                        im_ext_cs   <= 1'b1;
                                        im_ext_wr   <= 1'b0;
                                        im_ext_addr <= sr_cfg.im_write_base +
                                                       (im_ext_cnt[8:0] + 9'd1);
                                        wb_sub      <= WB_READ;
                                    end
                                end
                            endcase
                        end
                    endcase
                end
            endcase
        end
    end

    always_ff @(posedge clk) dma_fsm_state <= state;

endmodule

`default_nettype wire
