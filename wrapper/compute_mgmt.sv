// ═══════════════════════════════════════════════════════════════════════════════
// compute_mgmt — layer-iteration FSM (4 states), zero bus contact
//
// Handshake: dma_ready (in), compute_done (out), bank_toggle (shared in bus_wrapper)
// States (comp_state_t enum from wrapper_pkg):
//   C_IDLE → C_ISSUE_CFG(1-cycle cfg_valid + latch is_last) → C_WAIT_LAYER
//     Non-last: layer_done→compute_done→wait bank_toggle flip→loop to C_ISSUE_CFG
//     Last:     layer_done→compute_done→skip toggle (dma_ready=0 in D_TAIL)→C_DONE
//   C_DONE: hold compute_done (triggers D_TAIL WRITEBACK), wait inference_done→C_IDLE
// Critical: layer_done_latched bridges 1-cycle pulse → persistent flag for toggle wait
// ═══════════════════════════════════════════════════════════════════════════════
`default_nettype none

import wrapper_pkg::*;

module compute_mgmt (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        dma_ready,
    output logic       compute_done,
    input  wire        bank_toggle,
    input  wire        sr_is_last,
    output logic       cfg_valid,
    input  wire        layer_done,
    input  wire  [1:0] core_error,
    input  wire        inference_done,

    output comp_state_t comp_fsm_state,
    output logic        error_flag,
    output logic  [3:0] error_code
);

    comp_state_t state, next_state;
    logic        saved_is_last;
    logic        bank_toggle_d;
    logic        layer_done_latched;

    wire bank_toggle_changed;
    always_ff @(posedge clk) bank_toggle_d <= bank_toggle;
    assign bank_toggle_changed = (bank_toggle != bank_toggle_d);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= C_IDLE;
        else
            state <= next_state;
    end

    always_comb begin
        next_state = state;
        cfg_valid  = 1'b0;

        case (state)
            C_IDLE:       if (dma_ready) next_state = C_ISSUE_CFG;

            C_ISSUE_CFG:  begin cfg_valid = 1'b1; next_state = C_WAIT_LAYER; end

            C_WAIT_LAYER: begin
                if (saved_is_last && layer_done)
                    next_state = C_DONE;
                else if (!saved_is_last && layer_done_latched && bank_toggle_changed)
                    next_state = C_ISSUE_CFG;
            end

            C_DONE:       if (inference_done) next_state = C_IDLE;

            default:      next_state = C_IDLE;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            compute_done        <= 1'b0;
            saved_is_last       <= 1'b0;
            layer_done_latched  <= 1'b0;
            error_flag          <= 1'b0;
            error_code          <= 4'd0;
        end else begin
            if (state == C_ISSUE_CFG)
                saved_is_last <= sr_is_last;

            if (state == C_WAIT_LAYER && layer_done)
                layer_done_latched <= 1'b1;
            else if (state == C_WAIT_LAYER && next_state == C_ISSUE_CFG)
                layer_done_latched <= 1'b0;

            if (state == C_WAIT_LAYER && layer_done)
                compute_done <= 1'b1;
            else if (state == C_WAIT_LAYER && !saved_is_last && layer_done_latched && bank_toggle_changed)
                compute_done <= 1'b0;
            else if (state == C_DONE && inference_done)
                compute_done <= 1'b0;

            if (state == C_WAIT_LAYER && core_error != 2'b00) begin
                error_flag <= 1'b1;
                error_code <= {2'd0, core_error};
            end
        end
    end

    always_ff @(posedge clk) comp_fsm_state <= state;

endmodule

`default_nettype wire
