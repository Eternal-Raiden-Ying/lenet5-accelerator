// ═══════════════════════════════════════════════════════════════════════════════
// Lenet5v3 加速器顶层 — bus_wrapper + compute_core 纯连线集成
// ═══════════════════════════════════════════════════════════════════════════════
`default_nettype none

import wrapper_pkg::*;
import core_pkg::*;

module lenet5v3_top (
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
    input  wire        HRESPM,
    output wire        HMASTLOCKM
);

    // ── Wrapper ↔ Core interconnect ──
    logic        core_clk, core_rst_n;
    logic        cfg_valid;              // strobe, not part of desc_cfg_t
    desc_cfg_t   cfg_bus;                // 18 static config fields (packed struct)
    logic        im_ext_cs, im_ext_wr;
    logic  [8:0] im_ext_addr;
    logic [63:0] im_ext_wdata, im_ext_rdata;
    logic        im_ext_ready;
    logic        core_busy, layer_done;
    logic  [1:0] core_error;
    core_fsm_state_t core_fsm_state;
    logic        wt_rd_req;
    logic  [7:0] wt_rd_addr;
    logic [511:0] wt_rd_data;
    logic        rq_rd_req;
    logic  [3:0] rq_rd_addr;
    logic [287:0] rq_rd_data;

    // ── bus_wrapper ──
    bus_wrapper u_wrapper (
        .PCLK(PCLK), .PRESETn(PRESETn),
        .PSEL(PSEL), .PENABLE(PENABLE), .PWRITE(PWRITE),
        .PADDR(PADDR), .PWDATA(PWDATA),
        .PRDATA(PRDATA), .PREADY(PREADY),
        .HCLK(HCLK), .HRESETn(HRESETn),
        .HADDRM(HADDRM), .HTRANSM(HTRANSM), .HWRITEM(HWRITEM),
        .HSIZEM(HSIZEM), .HBURSTM(HBURSTM), .HWDATAM(HWDATAM),
        .HRDATAM(HRDATAM), .HREADYM(HREADYM), .HRESPM(HRESPM),
        .HMASTLOCKM(HMASTLOCKM),
        .core_clk(core_clk), .core_rst_n(core_rst_n),
        .cfg_valid(cfg_valid),
        .cfg_core(cfg_bus),    // packed struct: 18 config fields in one port
        .im_ext_cs(im_ext_cs), .im_ext_wr(im_ext_wr),
        .im_ext_addr(im_ext_addr), .im_ext_wdata(im_ext_wdata),
        .im_ext_rdata(im_ext_rdata), .im_ext_ready(im_ext_ready),
        .core_busy(core_busy), .layer_done(layer_done),
        .core_error(core_error),
        .core_fsm_state(core_fsm_state),
        .wt_rd_req(wt_rd_req), .wt_rd_addr(wt_rd_addr),
        .wt_rd_data(wt_rd_data),
        .rq_rd_req(rq_rd_req), .rq_rd_addr(rq_rd_addr),
        .rq_rd_data(rq_rd_data)
    );

    // ── compute_core ──
    compute_core u_core (
        .core_clk(core_clk), .core_rst_n(core_rst_n),
        .cfg_valid(cfg_valid),
        .cfg(cfg_bus),    // packed struct: 18 config fields in one port
        .im_ext_cs(im_ext_cs), .im_ext_wr(im_ext_wr),
        .im_ext_addr(im_ext_addr), .im_ext_wdata(im_ext_wdata),
        .im_ext_rdata(im_ext_rdata), .im_ext_ready(im_ext_ready),
        .core_busy(core_busy), .layer_done(layer_done),
        .core_error(core_error),
        .core_fsm_state(core_fsm_state),
        .wt_rd_req(wt_rd_req), .wt_rd_addr(wt_rd_addr),
        .wt_rd_data(wt_rd_data),
        .rq_rd_req(rq_rd_req), .rq_rd_addr(rq_rd_addr),
        .rq_rd_data(rq_rd_data)
    );

endmodule

`default_nettype wire
