// ═══════════════════════════════════════════════════════════════════
// tb_gearbox — verify #6 word-order fix: the FIRST DDR word must land in
//   the LOW bits so Core's unpack (pe_array oc0@[7:0], requant ch0@[71:0])
//   sees channel 0 first.
//   weight_sram: 16 words/row, word0→[31:0], word15→[511:480]
//   requant_sram: 9 words/entry, word0→[31:0], word8→[287:256]
// ═══════════════════════════════════════════════════════════════════
`default_nettype none
`timescale 1ns / 100ps

module tb_gearbox;
    logic clk, rst_n;
    integer errors = 0;
    integer i;

    // ── weight_sram DUT ──
    logic        wt_wr_en, wt_bank_sel, wt_gb_rst;
    logic [31:0] wt_wdata;
    logic        wt_rd_req;
    logic  [7:0] wt_rd_addr;
    logic [511:0] wt_rd_data;
    logic        wt_bank_toggle;

    weight_sram u_wt (
        .clk(clk), .rst_n(rst_n), .bank_toggle(wt_bank_toggle),
        .wt_rd_req(wt_rd_req), .wt_rd_addr(wt_rd_addr),
        .wt_rd_data(wt_rd_data),
        .wt_gb_bank_sel(wt_bank_sel), .wt_gb_wr_en(wt_wr_en),
        .wt_gb_wdata(wt_wdata), .gearbox_rst(wt_gb_rst)
    );

    // ── requant_sram DUT ──
    logic        rq_wr_en, rq_bank_sel, rq_gb_rst;
    logic [31:0] rq_wdata;
    logic        rq_rd_req;
    logic  [3:0] rq_rd_addr;
    logic [287:0] rq_rd_data;
    logic        rq_bank_toggle;

    requant_sram u_rq (
        .clk(clk), .rst_n(rst_n), .bank_toggle(rq_bank_toggle),
        .rq_rd_req(rq_rd_req), .rq_rd_addr(rq_rd_addr),
        .rq_rd_data(rq_rd_data),
        .rq_gb_bank_sel(rq_bank_sel), .rq_gb_wr_en(rq_wr_en),
        .rq_gb_wdata(rq_wdata), .gearbox_rst(rq_gb_rst)
    );

    initial clk = 0; always #2.5 clk = ~clk;

    // Waveform dump
    initial begin
        $dumpfile("tb_gearbox.vcd");
        $dumpvars(0, tb_gearbox);
    end

    // push one 32-bit word into the weight gearbox (bank A).
    // Drive on negedge so the sampling posedge sees a clean, stable wr_en pulse.
    task wt_push(input [31:0] d);
        begin
            @(negedge clk); wt_wdata = d; wt_wr_en = 1; wt_bank_sel = 0;
            @(negedge clk); wt_wr_en = 0;
        end
    endtask

    task rq_push(input [31:0] d);
        begin
            @(negedge clk); rq_wdata = d; rq_wr_en = 1; rq_bank_sel = 0;
            @(negedge clk); rq_wr_en = 0;
        end
    endtask

    initial begin
        rst_n = 0;
        wt_wr_en=0; wt_bank_sel=0; wt_gb_rst=0; wt_wdata=0;
        wt_rd_req=0; wt_rd_addr=0; wt_bank_toggle=0;
        rq_wr_en=0; rq_bank_sel=0; rq_gb_rst=0; rq_wdata=0;
        rq_rd_req=0; rq_rd_addr=0; rq_bank_toggle=0;
        repeat(4) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        // ── Test 1: weight gearbox — push words 0x10..0x1F into row 0 ──
        $display("[T1] weight_sram: 16 words → row 0, check word0 @ LOW");
        for (i=0; i<16; i=i+1) wt_push(32'h10 + i);
        @(posedge clk);   // row committed; read it back
        wt_rd_req = 1; wt_rd_addr = 0; @(posedge clk);  // latch dout_A/B
        wt_rd_req = 0; @(posedge clk);                   // data valid
        if (wt_rd_data[31:0] !== 32'h10) begin
            $display("FAIL T1: [31:0]=%h expect 10 (word0 at LOW)", wt_rd_data[31:0]);
            errors=errors+1;
        end
        if (wt_rd_data[511:480] !== 32'h1F) begin
            $display("FAIL T1: [511:480]=%h expect 1F (word15 at HIGH)", wt_rd_data[511:480]);
            errors=errors+1;
        end
        if (wt_rd_data[191:160] !== 32'h15) begin
            $display("FAIL T1: [191:160]=%h expect 15 (word5)", wt_rd_data[191:160]);
            errors=errors+1;
        end
        if (errors==0) $display("  T1 ok: word0@[31:0], word5@[191:160], word15@[511:480]");

        // ── Test 2: requant gearbox — push words 0xA0..0xA8 into entry 0 ──
        $display("[T2] requant_sram: 9 words → entry 0, check word0 @ LOW");
        for (i=0; i<9; i=i+1) rq_push(32'hA0 + i);
        @(posedge clk);
        rq_rd_req = 1; rq_rd_addr = 0; @(posedge clk);  // latch dout_A/B
        rq_rd_req = 0; @(posedge clk);                   // data valid
        if (rq_rd_data[31:0] !== 32'hA0) begin
            $display("FAIL T2: [31:0]=%h expect A0 (word0 at LOW)", rq_rd_data[31:0]);
            errors=errors+1;
        end
        if (rq_rd_data[287:256] !== 32'hA8) begin
            $display("FAIL T2: [287:256]=%h expect A8 (word8 at HIGH)", rq_rd_data[287:256]);
            errors=errors+1;
        end
        if (rq_rd_data[63:32] !== 32'hA1) begin
            $display("FAIL T2: [63:32]=%h expect A1 (word1)", rq_rd_data[63:32]);
            errors=errors+1;
        end

        if (errors==0) $display("\n==== ALL TESTS PASSED ====");
        else           $display("\n==== %0d ERRORS ====", errors);
        $finish;
    end

    initial begin #50000; $display("GLOBAL TIMEOUT"); $finish; end
endmodule

`default_nettype wire
