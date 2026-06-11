// ═══════════════════════════════════════════════════════════════════════════════
// requant_sram — dual-bank requant storage (16 entries each, 288b = 4ch×72b) + S2P gearbox
//
// Write: 32-bit AHB word → shared gearbox shift-reg → 288-bit entry (every 9 words)
// Read:  synchronous dout registers + combinational MUX for BRAM inference
// v1.2 (#12): shift-reg replaces dynamic indexing; sync read replaces async assign
//
// Shared gearbox (#refactor): per-bank gb_shift/cnt/addr replaced by a single set
//   of shared registers.  DMA writes only ONE bank at a time, and gearbox_rst is
//   asserted before every transfer.  Constraint: bank_sel MUST NOT change mid-entry
//   (cnt 1..8); gearbox_rst must precede every bank_sel change.
//   gb_shift = 256-bit (8 words), self-cleaning — no gearbox_rst needed between
//   consecutive entries within a transfer.
//
// Word order (#6): 移位寄存器右移, word k 最终落入 slot k = bits[32k +: 32],
//   FIRST DDR word lands in LOW bits [31:0], 9th in [287:256]. Matches
//   requant_unit's unpack (ch0 params at rq_rd_data[71:0]). Compiler packs in
//   natural order (ch0 first).
// ═══════════════════════════════════════════════════════════════════════════════
`default_nettype none

module requant_sram #(
    parameter BANK_DEPTH = 16    // 16 entries per bank
) (
    input  wire        clk,
    input  wire        rst_n,

    // ── Bank select ──
    input  wire        bank_toggle,       // 0=Core reads A, 1=Core reads B

    // ── Core read port ──
    input  wire        rq_rd_req,
    input  wire  [3:0] rq_rd_addr,
    output wire [287:0] rq_rd_data,

    // ── DMA write port ──
    input  wire        rq_gb_bank_sel,    // 0=write Bank A, 1=write Bank B
    input  wire        rq_gb_wr_en,
    input  wire [31:0] rq_gb_wdata,

    // ── New-requant reset ──
    input  wire        gearbox_rst
);

    // ═══════════════════════════════════════════════════════════════════
    // Bank storage arrays
    // ═══════════════════════════════════════════════════════════════════
    logic [287:0] bank_A [0:BANK_DEPTH-1];
    logic [287:0] bank_B [0:BANK_DEPTH-1];

    // ═══════════════════════════════════════════════════════════════════
    // Shared gearbox — single shift/cnt/addr (DMA writes one bank at a time)
    //
    // 移位寄存器替代动态索引: 新 word 从高位推入, 旧数据向右移位。
    // 经 8 次移位后, word0 到达 [31:0]; cnt==8 时第 9 个 word 到达,
    // 完整条目 = {rq_gb_wdata, gb_shift[255:0]} = 288-bit。
    //
    // 自清洁: 移位寄存器仅 8 words (256b), 每 8 次写入即可将旧行数据
    // 完全推出, 同行间无需 gearbox_rst。
    // ═══════════════════════════════════════════════════════════════════
    logic [255:0] gb_shift;    // 仅需保存前 8 个 word (8 * 32 = 256 bit)
    logic   [3:0] gb_cnt;      // 0~8 words per 288-bit entry
    logic   [3:0] gb_addr;     // 0~15 entries per bank

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gb_shift <= 256'd0;
            gb_cnt   <= 4'd0;
            gb_addr  <= 4'd0;
        end else if (gearbox_rst) begin
            // Reset between transfers
            gb_cnt   <= 4'd0;
            gb_addr  <= 4'd0;
        end else if (rq_gb_wr_en) begin
            // 右移: 新数据从高位推入, 旧数据向低位推
            gb_shift <= {rq_gb_wdata, gb_shift[255:32]};

            if (gb_cnt == 4'd8) begin
                // Entry complete: gb_shift 含 word0..word7, rq_gb_wdata = word8
                if (rq_gb_bank_sel == 1'b0)
                    bank_A[gb_addr] <= {rq_gb_wdata, gb_shift};
                else
                    bank_B[gb_addr] <= {rq_gb_wdata, gb_shift};

                gb_addr <= gb_addr + 4'd1;
                gb_cnt  <= 4'd0;
            end else begin
                gb_cnt <= gb_cnt + 4'd1;
            end
        end
    end

    // ═══════════════════════════════════════════════════════════════════
    // Read — synchronous dout registers + combinational MUX
    //   综合工具可将 bank_A/bank_B 推断为 BRAM/SRAM 宏单元。
    // ═══════════════════════════════════════════════════════════════════
    logic [287:0] dout_A;
    logic [287:0] dout_B;

    always_ff @(posedge clk) begin
        if (rq_rd_req && !bank_toggle)
            dout_A <= bank_A[rq_rd_addr];
    end

    always_ff @(posedge clk) begin
        if (rq_rd_req && bank_toggle)
            dout_B <= bank_B[rq_rd_addr];
    end

    assign rq_rd_data = bank_toggle ? dout_B : dout_A;

endmodule

`default_nettype wire
