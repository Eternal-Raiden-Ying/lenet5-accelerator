// ═══════════════════════════════════════════════════════════════════════════════
// FIFO Gearbox — 2048b→128b 移位寄存器 (depth=1)
// ═══════════════════════════════════════════════════════════════════════════════
//
// 【v4 重构: 移位寄存器替代状态机】
//   flush_strobe → 锁存 pe_output 到 slot[2047:0]
//   下一拍起: fg_data = slot[127:0] (组合连线低位), fg_valid=1
//           每拍 slot <= slot >> 128 (右移)
//   16 拍排空 → fg_valid=0, fifo_empty=1
//
//   无 out_phase 计数器, 无 draining 状态, 无 ping-pong。
//   FSM 保证排空速度 > 生产速度, 深度=1 永不溢出。
//
// ═══════════════════════════════════════════════════════════════════════════════
`default_nettype none

module fifo_gearbox (
    input  wire        clk,
    input  wire        rst_n,

    // ── From PE Array ──
    input  wire [2047:0] pe_output,       // 组合逻辑 (= acc 展开)
    input  wire        flush_strobe,       // FSM 脉冲: 锁存 pe_output

    // ── To Requant ──
    output logic [127:0] fg_data,
    output logic        fg_valid,

    // ── Status ──
    output logic        fifo_empty
);

    logic [2047:0] slot;
    logic  [3:0]   shift_cnt;
    logic          active;

    // ── fg_data 组合连线 slot 低 128b ──
    assign fg_data   = slot[127:0];
    assign fg_valid  = active;
    assign fifo_empty = !active;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            slot      <= 2048'd0;
            shift_cnt <= 4'd0;
            active    <= 1'b0;
        end else begin
            if (flush_strobe) begin
                slot      <= pe_output;
                shift_cnt <= 4'd0;
                active    <= 1'b1;
            end else if (active) begin
                slot <= {128'd0, slot[2047:128]};   // 右移 128b
                shift_cnt <= shift_cnt + 4'd1;
                if (shift_cnt == 4'd15) begin
                    active <= 1'b0;
                end
            end
        end
    end

endmodule

`default_nettype wire
