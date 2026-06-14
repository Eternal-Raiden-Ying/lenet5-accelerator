// ═══════════════════════════════════════════════════════════════════════════════
// bus_wrapper — APB+AHB+dma_scheduler+compute_mgmt+shadow_reg+weight/requant SRAMs
//
// Owns bank_toggle: flip on (compute_done && dma_ready), glitch-free register.
// cfg_* pass-through: shadow_register comb assign → Core ports, zero glue.
// PCLK==HCLK; core_rst_n = HRESETn & ~soft_reset.
//
// Internal interconnect uses packed structs (dma_req_t, dma_rsp_t, gb_wr_t,
// desc_cfg_t, desc_dma_t) from wrapper_pkg to eliminate ~90 individual wires.
// ═══════════════════════════════════════════════════════════════════════════════
`default_nettype none

import wrapper_pkg::*;
import core_pkg::*;

module bus_wrapper (
    // ── APB Slave ──
    input  wire        PCLK,
    input  wire        PRESETn,
    input  wire        PSEL,
    input  wire        PENABLE,
    input  wire        PWRITE,
    input  wire [11:0] PADDR,
    input  wire [31:0] PWDATA,
    output wire [31:0] PRDATA,
    output wire        PREADY,

    // ── AHB-Lite Master ──
    input  wire        HCLK,
    input  wire        HRESETn,
    output wire [31:0] HADDRM,
    output wire [1:0]  HTRANSM,
    output wire        HWRITEM,
    output wire [2:0]  HSIZEM,
    output wire [2:0]  HBURSTM,
    output wire [31:0] HWDATAM,
    input  wire [31:0] HRDATAM,
    input  wire        HREADYM,
    input  wire        HRESPM,        // #5: AHB-Lite error response (1'b1=ERROR)
    output wire        HMASTLOCKM,

    // ── Wrapper → Core ──
    output wire        core_clk,
    output wire        core_rst_n,
    output wire        cfg_valid,        // strobe (from compute_mgmt), not in desc_cfg_t
    output desc_cfg_t  cfg_core,         // 18 static config fields as packed struct
    output wire        im_ext_cs,
    output wire        im_ext_wr,
    output wire  [8:0] im_ext_addr,
    output wire [63:0] im_ext_wdata,
    input  wire [63:0] im_ext_rdata,
    input  wire        im_ext_ready,

    // ── Core → Wrapper ──
    input  wire        core_busy,
    input  wire        layer_done,
    input  wire  [1:0] core_error,
    input  wire core_fsm_state_t core_fsm_state,  // Core FSM 状态 (→ APB debug)
    input  wire        wt_rd_req,
    input  wire  [7:0] wt_rd_addr,
    output wire [511:0] wt_rd_data,
    input  wire        rq_rd_req,
    input  wire  [3:0] rq_rd_addr,
    output wire [287:0] rq_rd_data
);

    // ═══════════════════════════════════════════════════════════════
    // Clock + Reset
    // ═══════════════════════════════════════════════════════════════
    assign core_clk = HCLK;
    wire soft_reset;
    assign core_rst_n = HRESETn && ~soft_reset;

    // ═══════════════════════════════════════════════════════════════
    // APB Slave → internal signals
    // ═══════════════════════════════════════════════════════════════
    wire        csr_start_pulse;
    wire [31:0] desc_head_ptr;

    // ═══════════════════════════════════════════════════════════════
    // FSM status signals
    // ═══════════════════════════════════════════════════════════════
    wire            inference_done;
    wire            dma_error_flag;
    wire      [3:0] dma_error_code;
    dma_state_t     dma_fsm_state;
    comp_state_t    comp_fsm_state;
    wire            comp_error_flag;
    wire      [3:0] comp_error_code;

    // Combined error (DMA takes priority if both assert)
    wire error_flag  = dma_error_flag || comp_error_flag;
    wire [3:0] error_code = dma_error_flag ? dma_error_code : comp_error_code;

    // ═══════════════════════════════════════════════════════════════
    // AHB Master ↔ dma_scheduler (packed structs)
    // ═══════════════════════════════════════════════════════════════
    dma_req_t    dma_req;
    dma_rsp_t    dma_rsp;
    ahb_state_t  ahb_fsm_state;

    // ═══════════════════════════════════════════════════════════════
    // Shadow Register (packed structs)
    // ═══════════════════════════════════════════════════════════════
    logic        sr_wr_en;
    logic [31:0] sr_wr_data;
    desc_cfg_t   cfg_out;
    desc_dma_t   sr_dma;

    // ═══════════════════════════════════════════════════════════════
    // Handshake
    // ═══════════════════════════════════════════════════════════════
    wire dma_ready;
    wire compute_done;

    // ═══════════════════════════════════════════════════════════════
    // bank_toggle register
    // ═══════════════════════════════════════════════════════════════
    logic bank_toggle;
    logic bank_toggle_d; // 增加一个打拍寄存器，用于检测稳定态

    always_ff @(posedge HCLK or negedge HRESETn) begin
        if (!HRESETn)
            bank_toggle_d <= 1'b0;
        else
            bank_toggle_d <= bank_toggle;
    end

    always_ff @(posedge HCLK or negedge HRESETn) begin
        if (!HRESETn)
            bank_toggle <= 1'b0;
        // 核心修复：只在 bank_toggle 稳定的周期才允许翻转
        else if (compute_done && dma_ready && (bank_toggle == bank_toggle_d))
            bank_toggle <= ~bank_toggle;
    end

    // ═══════════════════════════════════════════════════════════════
    // Gearbox write bundles (packed structs)
    // ═══════════════════════════════════════════════════════════════
    gb_wr_t  wt_gb;
    gb_wr_t  rq_gb;
    wire     gearbox_rst;

    // ═══════════════════════════════════════════════════════════════
    // cfg_* pass-through: entire struct to Core (single assignment)
    // ═══════════════════════════════════════════════════════════════
    assign cfg_core = cfg_out;

    // ═══════════════════════════════════════════════════════════════
    // Sub-module instances
    // ═══════════════════════════════════════════════════════════════

    apb_slave u_apb (
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

    ahb_master u_ahb (
        .HCLK(HCLK), .HRESETn(HRESETn),
        .HADDRM(HADDRM), .HTRANSM(HTRANSM), .HWRITEM(HWRITEM),
        .HSIZEM(HSIZEM), .HBURSTM(HBURSTM), .HWDATAM(HWDATAM),
        .HRDATAM(HRDATAM), .HREADYM(HREADYM), .HRESP(HRESPM),
        .HMASTLOCKM(HMASTLOCKM),
        .req(dma_req), .rsp(dma_rsp),
        .ahb_fsm_state(ahb_fsm_state)
    );

    dma_scheduler u_dma (
        .clk(HCLK), .rst_n(HRESETn),
        .csr_start_pulse(csr_start_pulse),
        .desc_head_ptr(desc_head_ptr),
        .dma_req(dma_req), .dma_rsp(dma_rsp),
        .sr_wr_en(sr_wr_en), .sr_wr_data(sr_wr_data),
        .sr_dma(sr_dma),
        .sr_cfg(cfg_out),
        .dma_ready(dma_ready), .compute_done(compute_done),
        .bank_toggle(bank_toggle),
        .core_busy(core_busy),
        .wt_gb(wt_gb), .rq_gb(rq_gb),
        .gearbox_rst(gearbox_rst),
        .im_ext_cs(im_ext_cs), .im_ext_wr(im_ext_wr),
        .im_ext_addr(im_ext_addr), .im_ext_wdata(im_ext_wdata),
        .im_ext_rdata(im_ext_rdata), .im_ext_ready(im_ext_ready),
        .inference_done(inference_done),
        .error_flag(dma_error_flag), .error_code(dma_error_code),
        .dma_fsm_state(dma_fsm_state)
    );

    compute_mgmt u_comp (
        .clk(HCLK), .rst_n(HRESETn),
        .dma_ready(dma_ready), .compute_done(compute_done),
        .bank_toggle(bank_toggle),
        .sr_is_last(sr_dma.is_last),
        .cfg_valid(cfg_valid),
        .layer_done(layer_done), .core_error(core_error),
        .inference_done(inference_done),
        .comp_fsm_state(comp_fsm_state),
        .error_flag(comp_error_flag), .error_code(comp_error_code)
    );

    shadow_register u_shadow (
        .clk(HCLK), .rst_n(HRESETn),
        .sr_wr_en(sr_wr_en), .sr_wr_data(sr_wr_data),
        .cfg_out(cfg_out), .dma_out(sr_dma)
    );

    weight_sram u_wt_sram (
        .clk(HCLK), .rst_n(HRESETn),
        .bank_toggle(bank_toggle),
        .wt_rd_req(wt_rd_req), .wt_rd_addr(wt_rd_addr),
        .wt_rd_data(wt_rd_data),
        .wt_gb_bank_sel(wt_gb.bank_sel),
        .wt_gb_wr_en(wt_gb.wr_en), .wt_gb_wdata(wt_gb.wdata),
        .gearbox_rst(gearbox_rst)
    );

    requant_sram u_rq_sram (
        .clk(HCLK), .rst_n(HRESETn),
        .bank_toggle(bank_toggle),
        .rq_rd_req(rq_rd_req), .rq_rd_addr(rq_rd_addr),
        .rq_rd_data(rq_rd_data),
        .rq_gb_bank_sel(rq_gb.bank_sel),
        .rq_gb_wr_en(rq_gb.wr_en), .rq_gb_wdata(rq_gb.wdata),
        .gearbox_rst(gearbox_rst)
    );

endmodule

`default_nettype wire
