// ═══════════════════════════════════════════════════════════════════════════════
// weight_sram — true dual-bank weight storage (16KB each, 256×512b) + S2P gearbox
//
// Write: 32-bit AHB word → shared gearbox shift-reg → 512-bit row (every 16 words)
// Read:  bank_toggle selects active bank; Core sees logical row addr [0..255]
// DMA writes ~bank_toggle (idle bank); gearbox_rst on new-layer entry
//
// Shared gearbox (#refactor): per-bank gb_shift/cnt/row replaced by a single set
//   of shared registers.  DMA writes only ONE bank at a time, and gearbox_rst is
//   asserted before every transfer → no state retention needed across banks.
//   Constraint: bank_sel MUST NOT change mid-row (cnt 1..15); gearbox_rst must
//   precede every bank_sel change (guaranteed by dma_scheduler FSM).
//
// Word order (#6): each AHB word k is written to slot k = bits [32k +: 32], so the
//   FIRST DDR word lands in LOW bits [31:0] and the 16th in [511:480]. This matches
//   pe_array's unpack (oc0 weight at wt_rd_data[7:0]). The compiler packs weights in
//   natural order — first 32-bit word = lowest output channels (oc0..oc3).
// ═══════════════════════════════════════════════════════════════════════════════
`default_nettype none

module weight_sram #(
    parameter BANK_ROWS = 256    // 256 rows × 512-bit = 16KB per bank
) (
    input  wire        clk,
    input  wire        rst_n,

    // ── Bank select ──
    input  wire        bank_toggle,       // 0=Core reads A, 1=Core reads B

    // ── Core read port ──
    input  wire        wt_rd_req,
    input  wire  [7:0] wt_rd_addr,
    output wire [511:0] wt_rd_data,

    // ── DMA write port ──
    input  wire        wt_gb_bank_sel,    // 0=write Bank A, 1=write Bank B
    input  wire        wt_gb_wr_en,
    input  wire [31:0] wt_gb_wdata,

    // ── New-layer reset (from dma_scheduler on D_PREFILL/D_PREFETCH entry) ──
    input  wire        gearbox_rst
);

    // ═══════════════════════════════════════════════════════════════════
    // Bank storage arrays
    // ═══════════════════════════════════════════════════════════════════
    logic [511:0] bank_A [0:BANK_ROWS-1];
    logic [511:0] bank_B [0:BANK_ROWS-1];

    // ═══════════════════════════════════════════════════════════════════
    // Shared gearbox — single shift/cnt/row (DMA writes one bank at a time)
    // ═══════════════════════════════════════════════════════════════════
    logic [479:0] gb_shift;     // 仅需保存前 15 个 word (15 * 32 = 480 bit)
    logic   [3:0] gb_cnt;       // 0~15 words per 512-bit row
    logic   [7:0] gb_row;       // 0~255 rows per bank

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gb_shift <= 480'd0;
            gb_cnt   <= 4'd0;
            gb_row   <= 8'd0;
        end else if (gearbox_rst) begin
            gb_cnt   <= 4'd0;
            gb_row   <= 8'd0;
        end else if (wt_gb_wr_en) begin
            // 使用右移逻辑：新来的数据放在高 32 位，旧数据向低位推
            // 经过 15 次移位后，第 1 个字正好到达 [31:0]
            gb_shift <= {wt_gb_wdata, gb_shift[479:32]}; 
            
            if (gb_cnt == 4'd15) begin
                if (wt_gb_bank_sel == 1'b0)
                    bank_A[gb_row] <= {wt_gb_wdata, gb_shift};
                else
                    bank_B[gb_row] <= {wt_gb_wdata, gb_shift};
                
                gb_row <= gb_row + 8'd1;
                gb_cnt <= 4'd0;
            end else begin
                gb_cnt <= gb_cnt + 4'd1;
            end
        end
    end

    // ═══════════════════════════════════════════════════════════════════
    // Read MUX — Core reads active bank
    // ═══════════════════════════════════════════════════════════════════
    logic [511:0] dout_A;
    logic [511:0] dout_B;
    
    // 仅当 Core 请求读取，且 Bank A 处于激活状态时，才使能 Bank A 的读取
    always_ff @(posedge clk) begin
        if (wt_rd_req && !bank_toggle) begin 
            dout_A <= bank_A[wt_rd_addr];
        end
    end

    // 仅当 Core 请求读取，且 Bank B 处于激活状态时，才使能 Bank B 的读取
    always_ff @(posedge clk) begin
        if (wt_rd_req && bank_toggle) begin
            dout_B <= bank_B[wt_rd_addr];
        end
    end

    assign wt_rd_data = bank_toggle ? dout_B : dout_A;

endmodule

`default_nettype wire
