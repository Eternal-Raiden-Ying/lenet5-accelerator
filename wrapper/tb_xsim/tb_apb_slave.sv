// ═══════════════════════════════════════════════════════════════════
// tb_apb_slave — verify APB CSR read/write, self-clearing, W1C, 2-FF sync
//
// Tests: VERSION read, DESC_HEAD_PTR r/w, CTRL start self-clr +
//   soft_reset level, INT_STATUS hardware-set + W1C clear,
//   DEBUG_STATE bit-field mapping, RO write protection.
// ═══════════════════════════════════════════════════════════════════
`default_nettype none
`timescale 1ns / 100ps

import wrapper_pkg::*;

module tb_apb_slave;
    logic        PCLK, PRESETn;
    logic        PSEL, PENABLE, PWRITE;
    logic [11:0] PADDR;
    logic [31:0] PWDATA;
    logic [31:0] PRDATA;
    logic        PREADY;

    logic        csr_start_pulse;
    logic [31:0] desc_head_ptr;
    logic        soft_reset;

    logic        inference_done;
    logic        error_flag;
    logic  [3:0] error_code;
    dma_state_t  dma_fsm_state;
    comp_state_t comp_fsm_state;
    ahb_state_t  ahb_fsm_state;
    core_fsm_state_t core_fsm_state;
    logic        bank_toggle;
    logic        core_busy;

    integer errors = 0;

    apb_slave u_dut (
        .PCLK(PCLK), .PRESETn(PRESETn),
        .PSEL(PSEL), .PENABLE(PENABLE), .PWRITE(PWRITE),
        .PADDR(PADDR), .PWDATA(PWDATA),
        .PRDATA(PRDATA), .PREADY(PREADY),
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
        .core_fsm_state(core_fsm_state)
    );

    initial PCLK = 0;
    always #5 PCLK = ~PCLK;  // 100 MHz

    // Waveform dump
    initial begin
        $dumpfile("tb_apb_slave.vcd");
        $dumpvars(0, tb_apb_slave);
    end

    // ── APB write ──
    task apb_write(input [11:0] addr, input [31:0] data);
        begin
            @(posedge PCLK);
            PSEL=1; PENABLE=0; PWRITE=1; PADDR=addr; PWDATA=data;
            @(posedge PCLK);
            PENABLE=1;
            @(posedge PCLK);
            PSEL=0; PENABLE=0;
        end
    endtask

    // ── APB read ──
    task apb_read(input [11:0] addr, output [31:0] data);
        begin
            @(posedge PCLK);
            PSEL=1; PENABLE=0; PWRITE=0; PADDR=addr;
            @(posedge PCLK);
            PENABLE=1;
            @(posedge PCLK);  // data valid this cycle (PREADY=1)
            data = PRDATA;
            PSEL=0; PENABLE=0;
        end
    endtask

    // ── Wait for csr_start_pulse ──
    task wait_start_pulse;
        integer g;
        begin
            g=0;
            while (!csr_start_pulse && g<20) begin @(posedge PCLK); g=g+1; end
            if (g>=20) begin
                $display("FAIL: timeout waiting csr_start_pulse");
                errors=errors+1;
            end
        end
    endtask

    // ── Assertion helper ──
    task check(input [31:0] got, input [31:0] exp, input string msg);
        begin
            if (got !== exp) begin
                $display("FAIL: %s — got 0x%08h, expected 0x%08h", msg, got, exp);
                errors=errors+1;
            end
        end
    endtask

    logic [31:0] rd;

    // ═══════════════════════════════════════════════════════════════
    // Test sequence
    // ═══════════════════════════════════════════════════════════════
    initial begin
        // Init
        PSEL=0; PENABLE=0; PWRITE=0; PADDR=0; PWDATA=0;
        inference_done=0; error_flag=0; error_code=0;
        dma_fsm_state=D_IDLE; comp_fsm_state=C_IDLE; ahb_fsm_state=A_IDLE; core_fsm_state=CORE_F_IDLE;
        bank_toggle=0; core_busy=0;
        PRESETn=0;
        repeat(4) @(posedge PCLK);
        PRESETn=1;
        @(posedge PCLK);

        // ── T1: Read VERSION (0xFC) → expect 0x0000_0100 ──
        $display("[T1] Read VERSION");
        apb_read({4'h0, APB_VERSION}, rd);
        check(rd, 32'h0000_0100, "VERSION");
        $display("  T1 ok: VERSION=0x%08h", rd);

        // ── T2: Write/Read DESC_HEAD_PTR (0x04) ──
        $display("[T2] DESC_HEAD_PTR r/w");
        apb_write({4'h0, APB_DESC_HEAD_PTR}, 32'hA5A5_5A5A);
        apb_read ({4'h0, APB_DESC_HEAD_PTR}, rd);
        check(rd, 32'hA5A5_5A5A, "DESC_HEAD_PTR readback");
        apb_write({4'h0, APB_DESC_HEAD_PTR}, 32'hDEAD_BEEF);
        apb_read ({4'h0, APB_DESC_HEAD_PTR}, rd);
        check(rd, 32'hDEAD_BEEF, "DESC_HEAD_PTR 2nd write");
        $display("  T2 ok");

        // ── T3: CTRL.start — self-clearing pulse, verify csr_start_pulse ──
        $display("[T3] CTRL.start self-clear + 2-FF sync pulse");
        apb_write({4'h0, APB_CTRL}, 32'd1);   // start=1
        apb_read ({4'h0, APB_CTRL}, rd);
        // start self-clears immediately — should read back as 0
        if ((rd & 32'd1) != 0) begin
            $display("  FAIL T3: start did not self-clear, CTRL=0x%08h", rd);
            errors=errors+1;
        end
        wait_start_pulse();
        $display("  T3 ok: start self-cleared, csr_start_pulse asserted");

        // ── T4: CTRL.soft_reset — level semantics ──
        $display("[T4] CTRL.soft_reset level");
        apb_write({4'h0, APB_CTRL}, 32'd2);   // soft_reset=1
        @(posedge PCLK);  // wait for register update
        if (soft_reset !== 1'b1) begin
            $display("  FAIL T4: soft_reset not asserted"); errors=errors+1;
        end
        apb_read({4'h0, APB_CTRL}, rd);
        check(rd & 32'd2, 32'd2, "soft_reset readback");
        apb_write({4'h0, APB_CTRL}, 32'd0);   // clear
        @(posedge PCLK);  // wait for register update
        if (soft_reset !== 1'b0) begin
            $display("  FAIL T4: soft_reset not deasserted"); errors=errors+1;
        end
        $display("  T4 ok");

        // ── T5: INT_STATUS — inference_done hw-set + W1C clear ──
        //   Software sequence: ① clear hw source → ② INT_CLEAR[0]=1
        $display("[T5] INT_STATUS inference_done set + W1C clear");
        apb_read({4'h0, APB_INT_STATUS}, rd);
        check(rd[0], 1'b0, "INT_STATUS[0] initial");
        inference_done=1;
        repeat(2) @(posedge PCLK);
        apb_read({4'h0, APB_INT_STATUS}, rd);
        check(rd[0], 1'b1, "INT_STATUS[0] after hw set");
        // Step ①: deassert hw source first (required — hw set has priority)
        inference_done=0;
        @(posedge PCLK);
        // Step ②: W1C clear
        apb_write({4'h0, APB_INT_CLEAR}, 32'd1);
        apb_read({4'h0, APB_INT_STATUS}, rd);
        check(rd[0], 1'b0, "INT_STATUS[0] after W1C clear");
        $display("  T5 ok");

        // ── T6: INT_STATUS — error_flag + error_code W1C clear ──
        $display("[T6] INT_STATUS error set + W1C clear");
        error_flag=1; error_code=4'd3;
        repeat(2) @(posedge PCLK);
        apb_read({4'h0, APB_INT_STATUS}, rd);
        check(rd[1], 1'b1, "INT_STATUS[1] error");
        check(rd[7:4], 4'd3, "INT_STATUS[7:4] error_code");
        // Clear error via INT_CLEAR[1]
        error_flag=0;
        apb_write({4'h0, APB_INT_CLEAR}, 32'd2);
        apb_read({4'h0, APB_INT_STATUS}, rd);
        check(rd[1], 1'b0, "INT_STATUS[1] after W1C clear");
        $display("  T6 ok");

        // ── T7: INT_STATUS collision — INT_CLEAR + new hw set in same cycle ──
        //   W1C clear and hw inference_done=1 on the SAME posedge.
        //   Hardware set has priority → int_inference_done must stay 1.
        $display("[T7] INT_STATUS collision: INT_CLEAR + hw set same cycle");
        // Pre-condition: int_inference_done is currently 1
        inference_done=1;
        repeat(2) @(posedge PCLK);
        apb_read({4'h0, APB_INT_STATUS}, rd);
        check(rd[0], 1'b1, "pre-condition: INT[0]=1");
        // Deassert source so W1C can attempt to clear
        inference_done=0;
        @(posedge PCLK);
        // Now: APB write INT_CLEAR[0]=1, and re-assert inference_done on same edge
        // Manual APB timing for precise collision control
        @(negedge PCLK);
        PSEL=1; PENABLE=0; PWRITE=1; PADDR={4'h0, APB_INT_CLEAR}; PWDATA=32'd1;
        inference_done=1;  // re-assert at same negedge → both true at next posedge
        @(posedge PCLK);   // setup phase
        PENABLE=1;          // → wr_int_clear fires + inference_done=1 simultaneously
        @(posedge PCLK);   // access phase
        PSEL=0; PENABLE=0;
        @(posedge PCLK);   // register update settles
        apb_read({4'h0, APB_INT_STATUS}, rd);
        check(rd[0], 1'b1, "INT[0]=1 after collision (hw-set won over W1C)");
        inference_done=0;
        $display("  T7 ok: hw set correctly won over simultaneous W1C clear");

        // ── T8: DEBUG_STATE bit-field mapping ──
        $display("[T8] DEBUG_STATE bit-field mapping");
        dma_fsm_state=D_PREFETCH; comp_fsm_state=C_WAIT_LAYER; ahb_fsm_state=A_RUN;
        bank_toggle=1; core_busy=1;
        repeat(2) @(posedge PCLK);
        apb_read({4'h0, APB_DEBUG_STATE}, rd);
        check(rd[1:0],  2'd2,  "DEBUG[1:0] dma=D_PREFETCH(2)");
        check(rd[3:2],  2'd2,  "DEBUG[3:2] comp=C_WAIT_LAYER(2)");
        check(rd[5:4],  2'd1,  "DEBUG[5:4] ahb=A_RUN(1)");
        check(rd[10:8], 3'd0,  "DEBUG[10:8] core_fsm=IDLE");
        check(rd[11],   1'b1,  "DEBUG[11] bank_toggle");
        check(rd[12],   1'b1,  "DEBUG[12] core_busy");
        // Reset for remaining tests
        dma_fsm_state=D_IDLE; comp_fsm_state=C_IDLE; ahb_fsm_state=A_IDLE; core_fsm_state=CORE_F_IDLE;
        bank_toggle=0; core_busy=0;
        $display("  T7 ok");

        // ── T8: Read reserved/unmapped address → expect 0 ──
        $display("[T9] Read unmapped address");
        apb_read(12'h100, rd);
        check(rd, 32'd0, "unmapped addr 0x100");
        $display("  T8 ok");

        // ── T9: Write to RO register (VERSION) → no effect ──
        $display("[T10] Write to RO register (VERSION)");
        apb_write({4'h0, APB_VERSION}, 32'hCAFE_F00D);
        apb_read ({4'h0, APB_VERSION}, rd);
        check(rd, 32'h0000_0100, "VERSION after write attempt");
        $display("  T9 ok");

        // ── Done ──
        if (errors==0) $display("\n==== ALL TESTS PASSED ====");
        else           $display("\n==== %0d ERRORS ====", errors);
        $finish;
    end

    initial begin #50000; $display("GLOBAL TIMEOUT"); $finish; end
endmodule

`default_nettype wire
