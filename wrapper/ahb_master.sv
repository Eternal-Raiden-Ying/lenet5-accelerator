// ═══════════════════════════════════════════════════════════════════════════════
// ahb_master — AHB-Lite master @ INITEXP0, standard 2-process FSM
//
// FIX (#1): previous version cascaded two registers (next_state was clocked AND
//   state<=next_state), giving the FSM a 2-cycle latency that inserted an IDLE
//   beat into INCR16 bursts and left HBURST=SINGLE during SEQ. Now next_state is
//   pure combinational; `state` is the only FSM flop.
//
// AHB-Lite address/data pipeline modeled with TWO independent counters so the
//   "last data still pending" and "no more addresses" conditions never collide:
//     iss  = addresses remaining to present
//     dat  = data beats remaining to complete
//     pend = an address was presented last cycle → its data phase is due now
//   All datapath updates gated by HREADYM → no address/data slip on backpressure.
//
// Burst policy: ≥16 words → INCR16 (16-beat groups), tail (<16) → back-to-back
//   NONSEQ SINGLE. Writes are SINGLE (one word per dma_req, HWDATAM latched at req).
// HSIZE fixed 32b. HRESP is 1-bit (AHB-Lite). HMASTLOCKM fixed 0.
// ═══════════════════════════════════════════════════════════════════════════════
`default_nettype none

import wrapper_pkg::*;

module ahb_master (
    // ── System ──
    input  wire        HCLK,
    input  wire        HRESETn,

    // ── AHB-Lite Bus ──
    output logic [31:0] HADDRM,
    output htrans_t     HTRANSM,
    output logic        HWRITEM,
    output logic  [2:0] HSIZEM,
    output hburst_t     HBURSTM,
    output logic [31:0] HWDATAM,
    input  wire  [31:0] HRDATAM,
    input  wire         HREADYM,
    input  wire         HRESP,           // 1'b0=OKAY, 1'b1=ERROR (AHB-Lite)
    output wire         HMASTLOCKM,

    // ── dma_scheduler Request / Response Interface ──
    //   req (dma_req_t): valid, addr[31:0], word_count[15:0], dir, wdata[31:0], burst_mode
    //   rsp (dma_rsp_t): rdata[31:0], rdata_valid, done, error, active
    //   See wrapper_pkg.sv for per-field descriptions.
    input  dma_req_t    req,
    output dma_rsp_t    rsp,
    output ahb_state_t  ahb_fsm_state    // for DEBUG_STATE (#9)
);

    // ═══════════════════════════════════════════════════════════════════
    // Constants — see wrapper_pkg: ahb_state_t, htrans_t, hburst_t, dma_req_t, dma_rsp_t
    // ═══════════════════════════════════════════════════════════════════

    // ═══════════════════════════════════════════════════════════════════
    // State + datapath registers
    // ═══════════════════════════════════════════════════════════════════
    ahb_state_t state, next_state;
    logic [31:0] addr;          // next address to present
    logic [15:0] iss;           // addresses remaining to present
    logic [15:0] dat;           // data beats remaining to complete
    logic  [4:0] burst_left;    // beats left in current INCR16 group (0 ⇒ start new)
    logic        is_write;
    logic        allow_incr;
    logic        pend;          // address presented to bus (data due next cycle)
    logic        pend_d;        // pend delayed 1 cyc → qualifies registered read data

    assign HMASTLOCKM    = 1'b0;
    assign rsp.active    = (state == A_RUN);
    assign ahb_fsm_state = state;

    // last data beat will complete this cycle (pre-decrement check).
    // dat==1 means this is the final beat before it decrements to 0.
    wire all_done = (state == A_RUN) && HREADYM && pend_d && (dat == 16'd1);

    // ═══════════════════════════════════════════════════════════════════
    // Segment A — combinational next_state
    //   HRESP causes immediate return to A_IDLE (2-cycle error protocol:
    //   cycle1 HRESP=1/HREADYM=0 → HTRANSM gets IDLE in always_ff;
    //   cycle2 HRESP=1/HREADYM=1 → next_state=A_IDLE, error captured).
    // ═══════════════════════════════════════════════════════════════════
    always_comb begin
        next_state = state;
        case (state)
            A_IDLE: if (req.valid) next_state = A_RUN;
            A_RUN:  
                // 必须等待 Error 响应的第二拍 (HREADYM=1) 才能结束传输
                if (HRESP && HREADYM)  next_state = A_IDLE;
                else if (all_done)     next_state = A_IDLE;
            default: next_state = A_IDLE;
        endcase
    end

    // ═══════════════════════════════════════════════════════════════════
    // Segment B — state register + datapath (single FSM flop)
    // ═══════════════════════════════════════════════════════════════════
    always_ff @(posedge HCLK or negedge HRESETn) begin
        if (!HRESETn) begin
            state       <= A_IDLE;
            addr        <= 32'd0;
            iss         <= 16'd0;
            dat         <= 16'd0;
            burst_left  <= 5'd0;
            is_write    <= 1'b0;
            allow_incr  <= 1'b0;
            pend        <= 1'b0;
            pend_d      <= 1'b0;
            HADDRM      <= 32'd0;
            HTRANSM     <= HTRANS_IDLE;
            HWRITEM     <= 1'b0;
            HSIZEM      <= HSIZE_WORD;
            HBURSTM     <= HBURST_SINGLE;
            HWDATAM     <= 32'd0;
            rsp         <= '0;
        end else begin
            state           <= next_state;
            rsp.rdata_valid <= 1'b0;   // default pulse low
            rsp.done        <= 1'b0;
            HSIZEM          <= HSIZE_WORD; // always 32-bit Word

            case (state)
                // ── IDLE: latch request; first address presented in RUN ──
                A_IDLE: begin
                    HTRANSM   <= HTRANS_IDLE;
                    pend      <= 1'b0;
                    pend_d    <= 1'b0;
                    rsp.error <= 1'b0;
                    if (req.valid) begin
                        is_write   <= req.dir;
                        allow_incr <= (req.burst_mode != BM_SINGLE_ONLY);
                        HWRITEM    <= req.dir;
                        HWDATAM    <= req.wdata;
                        addr       <= req.addr;
                        // 仅仅将请求参数锁存到内部寄存器中，并没有驱动到总线接口上
                        // 带有 1 周期死区延迟（1-cycle latency),但是系统对于写出的要求不高，暂时未实现0周期发射
                        // 同理，没有实现burst写的操作
                        iss        <= req.word_count;
                        dat        <= req.word_count;
                        burst_left <= 5'd0;          // first addr opens a new group
                    end
                end

                // ── RUN: capture data, present next address ──────────
                A_RUN: begin
                    // (1) HRESP error: must set HTRANSM=IDLE regardless of HREADYM.
                    //     AHB-Lite 2-cycle error: T1(HRESP=1,HREADYM=0)→T2(HRESP=1,HREADYM=1).
                    //     We drive IDLE at T1 (takes effect T2); capture error at T2.
                    if (HRESP) begin
                        HTRANSM   <= HTRANS_IDLE;
                        pend      <= 1'b0;
                        pend_d    <= 1'b0;
                        if (HREADYM) rsp.error <= 1'b1;
                    end else if (HREADYM) begin
                        // (2) Data phase — valid when pend_d is high (2-cyc pipeline)
                        pend_d <= pend;
                        if (pend_d) begin
                            if (!is_write) begin
                                rsp.rdata       <= HRDATAM;
                                rsp.rdata_valid <= 1'b1;
                            end
                            dat <= dat - 16'd1;
                            if (dat == 16'd1)
                                rsp.done <= 1'b1;    // final beat
                        end

                        // (3) Address phase: present next, or close the bus
                        if (iss != 16'd0) begin
                            HADDRM <= addr;
                            addr   <= addr + 32'd4;
                            iss    <= iss - 16'd1;
                            pend   <= 1'b1;
                            if (burst_left == 5'd0) begin
                                HTRANSM <= HTRANS_NONSEQ;          // new group
                                if (iss >= 16'd16 && allow_incr) begin
                                    HBURSTM    <= HBURST_INCR16;
                                    burst_left <= 5'd15;
                                end else begin
                                    HBURSTM    <= HBURST_SINGLE;
                                    burst_left <= 5'd0;
                                end
                            end else begin
                                HTRANSM    <= HTRANS_SEQ;          // continue INCR16
                                HBURSTM    <= HBURST_INCR16;
                                burst_left <= burst_left - 5'd1;
                            end
                        end else begin
                            HTRANSM <= HTRANS_IDLE;
                            pend    <= 1'b0;
                        end
                    end
                    // HREADYM==0 (and no HRESP): hold all outputs, pulses stay low
                end

                default: HTRANSM <= HTRANS_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
