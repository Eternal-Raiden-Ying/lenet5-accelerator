// ═══════════════════════════════════════════════════════════════════════════════
// lenet5_pkg — Lenet5v3 加速器全局类型定义 (wrapper_pkg + core_pkg)
// ═══════════════════════════════════════════════════════════════════════════════
// 单文件双 package: 按需 import wrapper_pkg / core_pkg
//   wrapper_pkg: desc_cfg_t, desc_dma_t, CSR structs, DMA types, FSM enums
//   core_pkg:    layer_cnt_t, core_fsm_state_t, dda_event_t, HW constants
// ═══════════════════════════════════════════════════════════════════════════════
`default_nettype none

// ═══════════════════════════════════════════════════════════════════════════════
// wrapper_pkg — Wrapper/Core 共享层配置 + DMA + CSR 类型
// ═══════════════════════════════════════════════════════════════════════════════
package wrapper_pkg;

    // ────────────────────────────────────────────────────────────────
    // FSM state enums
    // ────────────────────────────────────────────────────────────────
    typedef enum logic [1:0] {
        D_IDLE     = 2'd0,
        D_PREFILL  = 2'd1,
        D_PREFETCH = 2'd2,
        D_TAIL     = 2'd3
    } dma_state_t;

    typedef enum logic [1:0] {
        C_IDLE       = 2'd0,
        C_ISSUE_CFG  = 2'd1,
        C_WAIT_LAYER = 2'd2,
        C_DONE       = 2'd3
    } comp_state_t;

    typedef enum logic [1:0] {
        A_IDLE = 2'd0,
        A_RUN  = 2'd1
    } ahb_state_t;

    typedef enum logic [2:0] {
        PH_DESC    = 3'd0,
        PH_WEIGHT  = 3'd1,
        PH_REQUANT = 3'd2,
        PH_FETCH   = 3'd3,
        PH_DONE    = 3'd4
    } dma_phase_t;

    typedef enum logic [1:0] {
        WB_READ = 2'd0,
        WB_LO   = 2'd1,
        WB_HI   = 2'd2
    } wb_sub_t;

    typedef enum logic [1:0] {
        BM_SINGLE_ONLY = 2'b00,
        BM_ALLOW_INCR  = 2'b01
    } burst_mode_t;

    // ────────────────────────────────────────────────────────────────
    // AHB-Lite protocol constants
    // ────────────────────────────────────────────────────────────────
    typedef enum logic [1:0] {
        HTRANS_IDLE   = 2'b00,
        HTRANS_NONSEQ = 2'b10,
        HTRANS_SEQ    = 2'b11
    } htrans_t;

    typedef enum logic [2:0] {
        HBURST_SINGLE = 3'b000,
        HBURST_INCR16 = 3'b011
    } hburst_t;

    localparam logic [2:0] HSIZE_WORD = 3'b010;

    // ── APB CSR register offsets ──
    localparam logic [7:0] APB_CTRL          = 8'h00;
    localparam logic [7:0] APB_DESC_HEAD_PTR = 8'h04;
    localparam logic [7:0] APB_INT_STATUS    = 8'h08;
    localparam logic [7:0] APB_INT_CLEAR     = 8'h0C;
    localparam logic [7:0] APB_DEBUG_STATE   = 8'h10;
    localparam logic [7:0] APB_VERSION       = 8'hFC;

    // ── CSR packed structs ──
    typedef struct packed {
        logic [11:0] reserved12;
        logic  [1:0] comp_fsm;
        logic  [1:0] dma_fsm;
        logic [13:0] reserved14;
        logic        soft_reset;
        logic        start;
    } reg_ctrl_t;

    typedef struct packed {
        logic [23:0] reserved24;
        logic  [3:0] error_code;
        logic  [1:0] reserved2;
        logic        error;
        logic        inf_done;
    } reg_int_status_t;

    typedef struct packed {
        logic [18:0] reserved19;
        logic        core_busy;
        logic        bank_toggle;
        logic  [2:0] core_fsm;     // core-level FSM state (CORE_F_IDLE..CORE_F_ERROR)
        logic  [1:0] reserved2;
        logic  [1:0] ahb_fsm;
        logic  [1:0] comp_fsm;
        logic  [1:0] dma_fsm;
    } reg_debug_state_t;

    // ────────────────────────────────────────────────────────────────
    // DMA request / response packed structs
    // ────────────────────────────────────────────────────────────────
    typedef struct packed {
        logic        valid;
        logic [31:0] addr;
        logic [15:0] word_count;
        logic        dir;
        logic [31:0] wdata;
        burst_mode_t burst_mode;
    } dma_req_t;

    typedef struct packed {
        logic [31:0] rdata;
        logic        rdata_valid;
        logic        done;
        logic        error;
        logic        active;
    } dma_rsp_t;

    // ── Gearbox write bundle ──
    typedef struct packed {
        logic        wr_en;
        logic [31:0] wdata;
        logic        bank_sel;
    } gb_wr_t;

    // ────────────────────────────────────────────────────────────────
    // desc_cfg_t — 编译器预填的层配置 (compiler-filled, 硬件零运算)
    //
    // 占 desc[191:0] (低 6 Word). shadow_register 以 packed struct cast 整体赋值.
    //
    // Word layout (6×32b = 192b):
    //   Word 0 [31:0]    — in/out channel + input spatial
    //   Word 1 [63:32]   — output spatial + kernel + stride
    //   Word 2 [95:64]   — padding + zp_y + im_total_writes
    //   Word 3 [127:96]  — IM 输入: row_stride + ch_stride + read_base
    //   Word 4 [159:128] — IM 输出: row_stride + ch_stride + write_base
    //   Word 5 [191:160] — WT strides(3) + flags(5) + reserved
    //
    // Stride 理论最大值 (编译器保证):
    //   im_row_stride   ≤ 32   (in_w≤255, ceil/8≤32)
    //   im_ch_stride    ≤ 511  (in_h * im_row_stride, C_CHECK bounds to IM_DEPTH)
    //   out_row_stride  ≤ 32   (out_w≤255, ceil/8≤32)
    //   out_ch_stride   ≤ 511  (out_h * out_row_stride)
    //   wt_ch_stride    ≤ 64   (kernel≤8: 8x8 → kh≤8; 1x64 → kh*kw≤64)
    //   wt_row_stride   ≤ 8    (kernel≤8: 8x8 → 1; 1x64 → kw≤8)
    //   wt_ocg_stride   ≤ 512  (1x64 多 ocg: ocg_max * in_ch * kh * kw, C_CHECK bounds ≤ 511)
    // ────────────────────────────────────────────────────────────────
    typedef struct packed {
        // Word 0  desc[31:0]    — input / output channel + input spatial dims
        logic  [7:0] in_ch;
        logic  [7:0] out_ch;
        logic  [7:0] in_w;
        logic  [7:0] in_h;

        // Word 1  desc[63:32]   — output spatial + kernel + stride (exact 32b)
        logic  [7:0] out_w;          // compiler pre-computed
        logic  [7:0] out_h;          // compiler pre-computed
        logic  [3:0] kernel_w;
        logic  [3:0] kernel_h;
        logic  [3:0] stride_w;
        logic  [3:0] stride_h;

        // Word 2  desc[95:64]   — padding + zero-point + im_total_writes
        logic  [3:0] pad_w;
        logic  [3:0] pad_h;
        logic  [7:0] zp_y;              // 输出 zero-point
        logic [15:0] im_total_writes;    // 本层 IM 写入总数 (64b words)

        // Word 3  desc[127:96]  — IM 输入 strides + read base
        logic  [7:0] im_row_stride;     // rows: 64b words / input row (≤32)
        logic  [9:0] im_ch_stride;      // ch:   64b words / input channel (≤511)
        logic  [8:0] im_read_base;      // 读起始行号 (64b word addr)
        logic  [4:0] reserved_w1;

        // Word 4  desc[159:128] — IM 输出 strides + write base
        logic  [7:0] out_row_stride;    // rows: 64b words / output row (≤32)
        logic  [9:0] out_ch_stride;     // ch:   64b words / output ch_group (≤511)
        logic  [8:0] im_write_base;     // 写起始行号 (64b word addr)
        logic  [4:0] reserved_w2;

        // Word 5  desc[191:160] — WT strides + flag
        logic  [7:0] wt_ch_stride;      // Weight rows per ic (≤64)
        logic  [7:0] wt_row_stride;     // Weight rows per ky (≤8)
        logic  [9:0] wt_ocg_stride;     // Weight rows per ocg (≤64)
        logic        mode;              // 0=MODE_8x8(Conv), 1=MODE_1x64(FC)
        logic        pool_bypass;       // 1=旁路 PingPong Pool
        logic        keep_accum;        // 1=不清 PE 累加器 (FC 跨 chunk)
        logic        disable_flush;     // 1=ic 结束不 flush FIFO (FC 中间 chunk)
        logic        ct_mode;           // 0=CT_SPATIAL(空间转置), 1=CT_GEARBOX(时间齿轮箱)
        logic        reserved_w3;
    } desc_cfg_t;

    // ────────────────────────────────────────────────────────────────
    // desc_dma_t — DMA / sequencer 字段 (占 desc[479:192], 高 9 Word = 288b)
    // ────────────────────────────────────────────────────────────────
    typedef struct packed {
        logic [31:0] weight_bytes;       // Word 6
        logic [31:0] requant_bytes;      // Word 7
        logic [31:0] weight_ddr_ptr;     // Word 8
        logic [31:0] requant_ddr_ptr;    // Word 9
        logic [31:0] feature_ddr_ptr;    // Word 10
        logic [31:0] result_ddr_ptr;     // Word 11
        logic [31:0] feature_bytes;      // Word 12
        logic [31:0] next_desc;          // Word 13
        logic        is_last;            // Word 14 bit 0
        logic [30:0] reserved_dma;       // Word 14 bits [31:1]
    } desc_dma_t;  // 9 × 32b = 288b

    // ────────────────────────────────────────────────────────────────
    // desc_full_t — 完整 512b 描述符 (cfg + dma + reserved)
    //   cfg:  Word 0-5  (192b)     dma:  Word 6-14 (288b)
    //   pad:  Word 15   (32b)      总:   16 × 32b  = 512b
    // ────────────────────────────────────────────────────────────────
    typedef struct packed {
        desc_cfg_t cfg;                // Word 0-5
        desc_dma_t dma;                // Word 6-14
        logic [31:0] reserved_desc;    // Word 15
    } desc_full_t;

endpackage


// ═══════════════════════════════════════════════════════════════════════════════
// core_pkg — Compute_Core 子系统共享类型
// ═══════════════════════════════════════════════════════════════════════════════
package core_pkg;

    // ── FSM→AGU/PE 循环计数器 (packed struct) ──
    typedef struct packed {
        logic  [7:0] ic;            // 当前输入通道 (0..in_ch-1)
        logic  [7:0] ox;            // 当前输出列 (步进 8)
        logic  [7:0] oy;            // 当前输出行
        logic  [3:0] kx;            // 当前 kernel 列
        logic  [3:0] ky;            // 当前 kernel 行
        logic  [4:0] ocg;           // 当前输出通道组 (0..ocg_max)
    } layer_cnt_t;

    // ── Core FSM 状态枚举 (5-state) ──
    typedef enum logic [2:0] {
        CORE_F_IDLE    = 3'd0,
        CORE_F_CHECK   = 3'd1,      // 参数校验 (1 拍)
        CORE_F_COMPUTE = 3'd2,
        CORE_F_DONE    = 3'd3,
        CORE_F_ERROR   = 3'd4       // 参数非法, 等待 reset/下一 cfg_valid
    } core_fsm_state_t;

    // ── 循环变化事件 (FSM→im_agu, wt_agu) ──
    typedef struct packed {
        logic ox_changed;
        logic oy_changed;
        logic ic_changed;
        logic ky_changed;
        logic kx_changed;
        logic ocg_changed;
    } dda_event_t;

    // ────────────────────────────────────────────────────────────────
    // 模式常量
    // ────────────────────────────────────────────────────────────────
    localparam logic MODE_8x8  = 1'b0;
    localparam logic MODE_1x64 = 1'b1;

    // ── corner_turn 模式 ──
    localparam logic CT_SPATIAL = 1'b0;   // 空间转置 (conv, pool 后)
    localparam logic CT_GEARBOX = 1'b1;   // 时间齿轮箱 (FC, bypass)

    // ────────────────────────────────────────────────────────────────
    // 硬件参数常量
    // ────────────────────────────────────────────────────────────────
    localparam int PE_ROWS        = 8;
    localparam int PE_COLS        = 8;
    localparam int PE_TOTAL       = 64;
    localparam int PE_ACCUM_BITS  = 32;
    localparam int PE_OUTPUT_BITS = PE_TOTAL * PE_ACCUM_BITS; // 2048b

    localparam int IM_DEPTH       = 512;
    localparam int IM_BANK_DEPTH  = 256;
    localparam int IM_ADDR_BITS   = 9;

    localparam int WT_ADDR_BITS   = 8;

    localparam int REQ_CH_PER_GROUP = 4;
    localparam int REQ_PARAM_BITS   = 72;
    localparam int REQ_MAX_GROUPS   = 16;

endpackage

`default_nettype wire
