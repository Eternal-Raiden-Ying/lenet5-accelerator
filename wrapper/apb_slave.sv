// ═══════════════════════════════════════════════════════════════════════════════
// apb_slave — APB slave @ 0x4000B000, 6 CSRs, zero-wait-state, 2-FF start sync
//
// CSRs: CTRL[0]=start(self-clr) [1]=soft_reset(level); DESC_HEAD_PTR; INT_STATUS/CLEAR;
//       DEBUG_STATE ([1:0]=dma [3:2]=comp [5:4]=ahb [8]=toggle [9]=busy); VERSION
// PCLK==HCLK: 2-FF sync on CTRL.start → csr_start_pulse in HCLK domain
// APB address constants (APB_CTRL..APB_VERSION) defined in wrapper_pkg
// ═══════════════════════════════════════════════════════════════════════════════
`default_nettype none

import wrapper_pkg::*;
import core_pkg::*;

module apb_slave (
    // ── APB Bus ──
    input  wire                 PCLK,
    input  wire                 PRESETn,
    input  wire                 PSEL,
    input  wire                 PENABLE,
    input  wire                 PWRITE,
    input  wire [11:0]          PADDR,
    input  wire [31:0]          PWDATA,
    output logic [31:0]          PRDATA,
    output wire                 PREADY,

    // ── To dma_scheduler (HCLK domain) ──
    output wire                 csr_start_pulse,    // 2-FF synchronized CTRL.start pulse
    output wire [31:0]          desc_head_ptr,      // descriptor chain base address
    output wire                 soft_reset,         // level: 1=Core held in reset

    // ── Status feedback (from bus_wrapper) ──
    input  wire                 inference_done,     // all layers complete
    input  wire                 error_flag,         // 1=error detected
    input  wire  [3:0]          error_code,         // error type (0=OK)
    input  wire dma_state_t     dma_fsm_state,      // dma_scheduler state
    input  wire comp_state_t    comp_fsm_state,     // compute_mgmt state
    input  wire ahb_state_t     ahb_fsm_state,      // ahb_master state
    input  wire                 bank_toggle,        // 0=BankA active, 1=BankB active
    input  wire                 core_busy,          // 1=Core computing
    input  wire core_fsm_state_t core_fsm_state      // core FSM state (CORE_F_IDLE..CORE_F_ERROR)
);

    // ═══════════════════════════════════════════════════════════════════
    // CSR Registers
    // ═══════════════════════════════════════════════════════════════════
    logic        ctrl_start;
    logic        ctrl_soft_reset;
    logic [31:0] desc_head_ptr_reg;

    // Write strobes
    wire apb_write = PSEL && PENABLE && PWRITE;
    wire wr_ctrl         = apb_write && (PADDR[7:0] == APB_CTRL);
    wire wr_desc_head    = apb_write && (PADDR[7:0] == APB_DESC_HEAD_PTR);
    wire wr_int_clear    = apb_write && (PADDR[7:0] == APB_INT_CLEAR);

    always_ff @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn) begin
            ctrl_start      <= 1'b0;
            ctrl_soft_reset <= 1'b0;
        end else begin
            if (wr_ctrl) begin
                // Typed struct access — no manual bit slicing
                automatic reg_ctrl_t wdata_ctrl = reg_ctrl_t'(PWDATA);
                ctrl_start      <= wdata_ctrl.start;
                ctrl_soft_reset <= wdata_ctrl.soft_reset;
            end else begin
                ctrl_start <= 1'b0;  // self-clearing
            end
        end
    end

    always_ff @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn)
            desc_head_ptr_reg <= 32'd0;
        else if (wr_desc_head)
            desc_head_ptr_reg <= PWDATA;
    end

    assign desc_head_ptr = desc_head_ptr_reg;
    assign soft_reset    = ctrl_soft_reset;

    // ═══════════════════════════════════════════════════════════════════
    // INT_STATUS / INT_CLEAR
    // ═══════════════════════════════════════════════════════════════════
    logic        int_inference_done;
    logic        int_error;
    logic  [3:0] int_error_code;

    always_ff @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn) begin
            int_inference_done <= 1'b0;
            int_error          <= 1'b0;
            int_error_code     <= 4'd0;
        end else begin
            // ── 1. Inference Done (hw set has priority; sw must deassert src then W1C) ──
            if (inference_done) begin
                int_inference_done <= 1'b1;            // 硬件置位优先
            end else if (wr_int_clear && PWDATA[0]) begin
                int_inference_done <= 1'b0;            // 软件 W1C 清除（仅在 hw 源已释放后有效）
            end

            // ── 2. Error (hw set has priority; sw must deassert src then W1C) ──
            if (error_flag) begin
                int_error      <= 1'b1;                // 硬件置位优先
                int_error_code <= error_code;          // 锁定最新的错误码
            end else if (wr_int_clear && PWDATA[1]) begin
                int_error      <= 1'b0;                // 软件 W1C 清除（仅在 hw 源已释放后有效）
            end
        end
    end

    // Cast enum FSM states to logic for struct field assignment
    logic [1:0] dma_fs, comp_fs, ahb_fs;
    logic [2:0] core_fs;
    assign dma_fs  = dma_fsm_state;
    assign comp_fs = comp_fsm_state;
    assign ahb_fs  = ahb_fsm_state;
    assign core_fs = core_fsm_state;

    // ═══════════════════════════════════════════════════════════════════
    // PRDATA read MUX — typed struct composition (no raw {} concatenation)
    // ═══════════════════════════════════════════════════════════════════
    always_comb begin
        PRDATA = 32'd0;  // default: prevent latch
        if (PSEL && !PWRITE) begin
            case (PADDR[7:0])
                APB_CTRL: begin
                    reg_ctrl_t r;
                    r = '0;
                    r.start      = ctrl_start;
                    r.soft_reset = ctrl_soft_reset;
                    r.comp_fsm   = comp_fs;
                    r.dma_fsm    = dma_fs;
                    PRDATA = r;
                end
                APB_DESC_HEAD_PTR: PRDATA = desc_head_ptr_reg;
                APB_INT_STATUS: begin
                    reg_int_status_t r;
                    r = '0;
                    r.inf_done   = int_inference_done;
                    r.error      = int_error;
                    r.error_code = int_error_code;
                    PRDATA = r;
                end
                APB_INT_CLEAR:     PRDATA = 32'd0;
                APB_DEBUG_STATE: begin
                    reg_debug_state_t r;
                    r = '0;
                    r.dma_fsm     = dma_fs;
                    r.comp_fsm    = comp_fs;
                    r.ahb_fsm     = ahb_fs;
                    r.core_fsm    = core_fs;
                    r.bank_toggle = bank_toggle;
                    r.core_busy   = core_busy;
                    PRDATA = r;
                end
                APB_VERSION:       PRDATA = 32'h0000_0100;
                default:           PRDATA = 32'd0;
            endcase
        end
    end

    assign PREADY = 1'b1;

    // ═══════════════════════════════════════════════════════════════════
    // 2-FF Synchronizer: CTRL.start (PCLK → HCLK)
    // ═══════════════════════════════════════════════════════════════════
    logic sync_ff1, sync_ff2, sync_ff2_d;

    always_ff @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn)
            sync_ff1 <= 1'b0;
        else
            sync_ff1 <= ctrl_start;
    end

    always_ff @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn) begin
            sync_ff2   <= 1'b0;
            sync_ff2_d <= 1'b0;
        end else begin
            sync_ff2   <= sync_ff1;
            sync_ff2_d <= sync_ff2;
        end
    end

    assign csr_start_pulse = sync_ff2 && !sync_ff2_d;

endmodule

`default_nettype wire
