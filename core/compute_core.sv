// ═══════════════════════════════════════════════════════════════════════════════
// Compute_Core — 纯计算引擎顶层 (v10: flush 打 2 拍, ocg_changed 打 2 拍)
// ═══════════════════════════════════════════════════════════════════════════════
//
// 实例化: compute_fsm | im_sram | im_agu | wt_agu | pe_array | fifo_gearbox
//         requant_unit | pingpong_pool | corner_turn
//
// 【v9 数据路径】
//   pe_array: acc → assign pe_output (组合, 无额外 reg)
//   FSM flush_strobe → FIFO 直连 (无 pipeline reg)
//   FIFO: depth=1, flush_strobe 锁存 pe_output → 16 拍串行化 → 空闲
//   寄存器总量: pe.acc(2048b) + fifo.slot(2048b) = 4096b
//
// cfg: 所有子模块统一使用 desc_cfg_t (wrapper_pkg),
//   row_stride/ch_stride/mode 由 shadow_register 派生, 无需 compute_core 映射
// ═══════════════════════════════════════════════════════════════════════════════
`default_nettype none

import wrapper_pkg::*;
import core_pkg::*;

module compute_core (
    input  wire        core_clk,
    input  wire        core_rst_n,
    input  wire        cfg_valid,          // 配置有效脉冲
    input  desc_cfg_t   cfg,                // Wrapper 下发的完整层配置
    input  wire        im_ext_cs,
    input  wire        im_ext_wr,
    input  wire  [8:0] im_ext_addr,
    input  wire [63:0] im_ext_wdata,
    output logic [63:0] im_ext_rdata,
    output logic        im_ext_ready,
    output logic        core_busy,
    output logic        layer_done,
    output logic  [1:0] core_error,
    output core_fsm_state_t core_fsm_state,  // Core FSM 状态 (→ wrapper APB debug)
    output logic        wt_rd_req,
    output logic  [7:0] wt_rd_addr,
    input  wire [511:0] wt_rd_data,
    output logic        rq_rd_req,
    output logic  [3:0] rq_rd_addr,
    input  wire [287:0] rq_rd_data
);

    // ═══════════════════════════════════════════════════════════════
    // FSM → 数据路径 控制信号
    // ═══════════════════════════════════════════════════════════════
    logic        mac_en, flush_strobe, clear_accum;
    layer_cnt_t  cnt;                  // 打包计数器 (ic,ox,oy,kx,ky,ocg)
    dda_event_t  dda;                  // DDA 事件 (ox/oy/ic/ky_changed)
    logic        layer_start;          // 层启动脉冲 (→ requant 复位 ch_group)
    desc_cfg_t   cfg_latched;          // FSM 锁存的配置 (cfg_valid 时锁存, 不受 wrapper 覆写影响)

    // ═══════════════════════════════════════════════════════════════
    // im_agu → pe_array: 坐标衍生信号 (纯组合, 无数据依赖)
    // ═══════════════════════════════════════════════════════════════
    logic  [7:0] pp_valid;

    // ═══════════════════════════════════════════════════════════════
    // 控制信号打拍 (集中在顶层, 子模块接收 _sync 信号)
    //
    //   mac_en_d2 / clear_accum_d2: 2 拍 (对齐 SRAM 1拍 + 数据 reg 1拍)
    //   flush_d2: 2 拍 (对齐 clear_accum_d2, FIFO 锁存含完整 acc)
    //   ocg_changed_d1: 1 拍 (对齐 flush_d1, requant 内部 rq_ocg 计数)
    // ═══════════════════════════════════════════════════════════════
    logic mac_en_d1, mac_en_d2;
    logic clear_accum_d1, clear_accum_d2;
    logic flush_d1, flush_d2;
    logic ocg_changed_d1;

    always_ff @(posedge core_clk or negedge core_rst_n) begin
        if (!core_rst_n) begin
            mac_en_d1 <= 0; mac_en_d2 <= 0;
            clear_accum_d1 <= 0; clear_accum_d2 <= 0;
        end else begin
            mac_en_d1       <= mac_en;
            mac_en_d2     <= mac_en_d1;
            clear_accum_d1  <= clear_accum;
            clear_accum_d2 <= clear_accum_d1;
        end
    end

    always_ff @(posedge core_clk) begin
        flush_d1 <= flush_strobe;
        flush_d2 <= flush_d1;
        ocg_changed_d1   <= dda.ocg_changed;
    end

    // ═══════════════════════════════════════════════════════════════
    // pe_array → fifo_gearbox (直连, 无 pipeline reg)
    // ═══════════════════════════════════════════════════════════════
    logic [2047:0] pe_output;

    // ═══════════════════════════════════════════════════════════════
    // fifo_gearbox → requant_unit
    // ═══════════════════════════════════════════════════════════════
    logic [127:0] fg_data;
    logic         fg_valid;
    logic         fifo_empty;

    // ═══════════════════════════════════════════════════════════════
    // requant_unit → pingpong_pool
    // ═══════════════════════════════════════════════════════════════
    logic [31:0] rq_data;
    logic        rq_valid;

    // ═══════════════════════════════════════════════════════════════
    // pingpong_pool → corner_turn
    // ═══════════════════════════════════════════════════════════════
    logic [31:0] pp_pool_data;
    logic        pp_pool_valid;

    // ═══════════════════════════════════════════════════════════════
    // corner_turn → im_sram / pingpong_pool
    // ═══════════════════════════════════════════════════════════════
    logic        ct_wr_en;
    logic  [8:0] ct_wr_addr;
    logic [63:0] ct_wr_data;
    logic [15:0] sram_write_cnt;

    // ═══════════════════════════════════════════════════════════════
    // im_sram: internal read port
    // ═══════════════════════════════════════════════════════════════
    logic        im_rd_req;
    logic  [8:0] im_rd_addr_lo, im_rd_addr_hi;
    logic [63:0] im_rd_data_lo, im_rd_data_hi;

    // ═══════════════════════════════════════════════════════════════
    // Sub-module instances
    // ═══════════════════════════════════════════════════════════════

    compute_fsm u_fsm (
        .core_clk(core_clk), .core_rst_n(core_rst_n),
        .cfg_valid(cfg_valid),
        .cfg(cfg),
        .sram_write_cnt(sram_write_cnt),
        .fifo_empty(fifo_empty),
        .core_busy(core_busy), .layer_done(layer_done), .core_error(core_error),
        .core_fsm_state(core_fsm_state),
        .layer_start(layer_start),
        .cfg_latched(cfg_latched),
        .mac_en(mac_en), .flush_strobe(flush_strobe), .clear_accum(clear_accum),
        .im_rd_req(im_rd_req), .wt_rd_req(wt_rd_req),
        .cnt(cnt),
        .dda(dda)
    );

    // 全部子模块使用 FSM 锁存的 cfg_latched (不受 wrapper 预取下一层 cfg 影响)
    im_agu u_im_agu (
        .cfg(cfg_latched),
        .cnt(cnt),
        .im_rd_addr_lo(im_rd_addr_lo), .im_rd_addr_hi(im_rd_addr_hi),
        .pp_valid(pp_valid)
    );

    wt_agu u_wt_agu (
        .cfg(cfg_latched),
        .cnt(cnt),
        .wt_rd_addr(wt_rd_addr)
    );

    pe_array u_pe (
        .clk(core_clk), .rst_n(core_rst_n),
        .cfg(cfg_latched),
        .cnt_kx(cnt.kx),
        .cnt_ox(cnt.ox),
        .im_rd_data_lo(im_rd_data_lo), .im_rd_data_hi(im_rd_data_hi),
        .pp_valid(pp_valid),
        .wt_rd_data(wt_rd_data),
        .mac_en_sync(mac_en_d2), .clear_accum_sync(clear_accum_d2),
        .pe_output(pe_output)
    );

    fifo_gearbox u_fifo (
        .clk(core_clk), .rst_n(core_rst_n),
        .pe_output(pe_output), .flush_strobe(flush_d2),
        .fg_data(fg_data), .fg_valid(fg_valid),
        .fifo_empty(fifo_empty)
    );

    requant_unit u_rq (
        .clk(core_clk), .rst_n(core_rst_n),
        .layer_start(layer_start),
        .cfg(cfg_latched),
        .fg_data(fg_data), .fg_valid(fg_valid),
        .fg_will_valid(flush_d2),
        .ocg_changed_sync(ocg_changed_d1),
        .rq_rd_data(rq_rd_data),
        .rq_rd_req(rq_rd_req), .rq_rd_addr(rq_rd_addr),
        .rq_data(rq_data), .rq_valid(rq_valid)
    );

    pingpong_pool #(.MAX_W(32)) u_pp (
        .clk(core_clk), .rst_n(core_rst_n),
        .cfg(cfg_latched),
        .rq_data(rq_data), .rq_valid(rq_valid),
        .pp_data(pp_pool_data), .pp_valid(pp_pool_valid)
    );

    corner_turn u_ct (
        .clk(core_clk), .rst_n(core_rst_n),
        .cfg(cfg_latched),
        .layer_start(layer_start),
        .pp_data(pp_pool_data), .pp_valid(pp_pool_valid),
        .im_wr_en(ct_wr_en), .im_wr_addr(ct_wr_addr), .im_wr_data(ct_wr_data),
        .sram_write_cnt(sram_write_cnt)
    );

    im_sram u_im (
        .core_clk(core_clk), .core_rst_n(core_rst_n),
        .im_ext_cs(im_ext_cs), .im_ext_wr(im_ext_wr),
        .im_ext_addr(im_ext_addr), .im_ext_wdata(im_ext_wdata),
        .im_ext_rdata(im_ext_rdata), .im_ext_ready(im_ext_ready),
        .im_rd_addr_lo(im_rd_addr_lo), .im_rd_addr_hi(im_rd_addr_hi),
        .im_rd_data_lo(im_rd_data_lo), .im_rd_data_hi(im_rd_data_hi),
        .im_rd_req(im_rd_req),
        .im_wr_en(ct_wr_en), .im_wr_addr(ct_wr_addr), .im_wr_data(ct_wr_data)
    );

endmodule

`default_nettype wire
