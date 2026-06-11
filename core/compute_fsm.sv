// ═══════════════════════════════════════════════════════════════════════════════
// Compute_FSM — 5-state 层级无关循环引擎, 双模 8x8/1x64
// ═══════════════════════════════════════════════════════════════════════════════
// 状态: IDLE → CHECK → COMPUTE → DONE → IDLE (正常)
//                     ↘ ERROR → IDLE (参数非法, 等下一 cfg_valid 或 reset)
//
// 循环嵌套: ocg → oy → ox(step8) → ic → ky → kx (MODE_8x8)
//           ocg → ic                      (MODE_1x64, ox/oy/kx/ky 恒 0)
//
// DDA 事件: ox/oy/ic/ky_changed 驱动 im_agu 维护累加指针, 消除乘法
// ═══════════════════════════════════════════════════════════════════════════════
`default_nettype none

import wrapper_pkg::*;
import core_pkg::*;

module compute_fsm (
    input  wire        core_clk,
    input  wire        core_rst_n,
    input  wire        cfg_valid,          // 配置有效脉冲
    input  desc_cfg_t   cfg,                // 层配置 (wrapper_pkg, 含 row_stride/ch_stride/mode)
    input  wire [15:0] sram_write_cnt,     // Corner-Turn 写计数
    input  wire        fifo_empty,         // FIFO 排空标志
    output logic        core_busy,
    output logic        layer_done,
    output logic  [1:0] core_error,
    output core_fsm_state_t core_fsm_state,  // Core FSM 状态 (→ wrapper APB debug)
    // ── 数据路径控制 ──
    output logic        mac_en,
    output logic        flush_strobe,
    output logic        clear_accum,
    output logic        im_rd_req,
    output logic        wt_rd_req,  // 组合 assign (见末尾)
    // ── 循环计数器 (打包 struct) ──
    output layer_cnt_t  cnt,
    output logic        layer_start,        // 层启动脉冲 (C_CHECK→C_COMPUTE 过渡, 1 拍)
    output desc_cfg_t   cfg_latched,        // 锁存的层配置 (cfg_valid 时锁存, 供全部子模块)
    // ── DDA 触发事件 (→ im_agu) ──
    output dda_event_t  dda
);

    core_fsm_state_t state, next_state;

    // ── 锁存的层参数 (cfg_valid 时从 desc_cfg_t 完整锁存) ──
    desc_cfg_t  L;                 // 锁存的 config (使用 L.in_ch, L.kernel_w, L.mode 等)
    logic [4:0] L_ocg_max;         // ceil(out_ch/group_size) - 1 (派生, 不入 desc_cfg_t)
    assign cfg_latched = L;        // 组合直出锁存值, 供全部子模块 (不受 wrapper 覆写 cfg 影响)

    // ── 当前 + 前拍计数器 (packed struct, 一根线) ──
    layer_cnt_t cnt_cur, cnt_prev;

    // ── 循环终止标志 (组合) ──
    logic kx_last, ky_last, ic_last, ox_last, oy_last, ocg_last;

    // 循环终止 — kx/ky/ic/ocg 双模共用公式
    //   MODE_8x8: L.kernel_w/L.kernel_h 为卷积核尺寸
    //   MODE_1x64: L.kernel_w=in_w, L.kernel_h=in_h (编译器填入), 自然迭代
    assign kx_last  = (cnt_cur.kx == L.kernel_w - 1);
    assign ky_last  = (cnt_cur.ky == L.kernel_h - 1);
    assign ic_last  = (cnt_cur.ic == L.in_ch - 1);
    assign ocg_last = (cnt_cur.ocg == L_ocg_max);
    // ox/oy: MODE_1x64 下 out_w=out_h=1, 自然恒真; MODE_8x8 正常迭代
    assign ox_last  = (L.mode == MODE_1x64) ? 1'b1 : (cnt_cur.ox + 8 >= L.out_w);
    assign oy_last  = (L.mode == MODE_1x64) ? 1'b1 : (cnt_cur.oy == L.out_h - 1);

    // ── DDA 事件: 计数器变化检测 ──
    assign dda.ocg_changed = (cnt_cur.ocg != cnt_prev.ocg);
    assign dda.ox_changed  = (cnt_cur.ox != cnt_prev.ox);
    assign dda.oy_changed  = (cnt_cur.oy != cnt_prev.oy);
    assign dda.ic_changed  = (cnt_cur.ic != cnt_prev.ic);
    assign dda.ky_changed  = (cnt_cur.ky != cnt_prev.ky);
    assign dda.kx_changed  = (cnt_cur.kx != cnt_prev.kx);

    // ── 打包输出计数器 ──
    assign cnt = cnt_cur;

    // ═══════════════════════════════════════════════════════════════
    // 参数校验 (C_CHECK 中组合判断)
    // ═══════════════════════════════════════════════════════════════
    logic check_fail;
    logic [15:0] max_im_addr, max_wt_addr;

    always_comb begin
        // IM 读地址上限: im_read_base + (in_ch-1)*im_ch_stride + (in_h-1)*im_row_stride + ceil(in_w/8)
        max_im_addr = cfg.im_read_base
                    + (cfg.in_ch - 8'd1) * cfg.im_ch_stride
                    + (cfg.in_h  - 8'd1) * cfg.im_row_stride
                    + (cfg.in_w  + 8'd7) / 8'd8;

        // WT 读地址上限 (必须 ≤ 8'hFF, wt_rd_addr 为 8-bit)
        if (cfg.mode == MODE_1x64)
            max_wt_addr = (cfg.in_ch - 8'd1) * cfg.wt_ch_stride
                        + (cfg.kernel_h - 4'd1) * cfg.wt_row_stride
                        + (cfg.kernel_w - 4'd1);       // kx_max = kw-1
        else
            max_wt_addr = ((cfg.out_ch + 8'd7) / 8'd8 - 8'd1) * cfg.in_ch * cfg.kernel_h
                        + (cfg.in_ch - 8'd1) * cfg.kernel_h
                        + (cfg.kernel_h - 4'd1);       // ocg_max*in_ch*kh + (ic_max)*kh + ky_max

        check_fail = 1'b0;
        if (cfg.kernel_w == 0 || cfg.kernel_h == 0)     check_fail = 1'b1;
        if (cfg.stride_w == 0 || cfg.stride_h == 0)     check_fail = 1'b1;
        if (cfg.in_ch == 0 || cfg.out_ch == 0)          check_fail = 1'b1;
        if (cfg.in_w == 0 || cfg.in_h == 0)             check_fail = 1'b1;
        if (max_im_addr > (IM_DEPTH - 2))               check_fail = 1'b1;
        if (max_wt_addr > 16'd255)                      check_fail = 1'b1;  // wt_rd_addr 8b overflow
        // MODE_1x64: out_w/out_h 必须为 1×1, stride=1, pad=0
        if (cfg.mode == MODE_1x64 && (cfg.out_w != 8'd1 || cfg.out_h != 8'd1
                                   || cfg.stride_w != 4'd1 || cfg.stride_h != 4'd1
                                   || cfg.pad_w != 4'd0 || cfg.pad_h != 4'd0))
                                                        check_fail = 1'b1;
    end

    // ═══════════════════════════════════════════════════════════════
    // 状态寄存器 + next_state
    // ═══════════════════════════════════════════════════════════════
    always_ff @(posedge core_clk or negedge core_rst_n) begin
        if (!core_rst_n) state <= CORE_F_IDLE;
        else state <= next_state;
    end

    always_comb begin
        next_state = state;
        case (state)
            CORE_F_IDLE:  if (cfg_valid) next_state = CORE_F_CHECK;

            CORE_F_CHECK: if (check_fail) next_state = CORE_F_ERROR;
                          else next_state = CORE_F_COMPUTE;

            CORE_F_COMPUTE: if (ocg_last && oy_last && ox_last && ic_last && ky_last && kx_last)
                          next_state = CORE_F_DONE;

            CORE_F_DONE:  if (fifo_empty && (sram_write_cnt >= L.im_total_writes))
                          next_state = CORE_F_IDLE;

            CORE_F_ERROR: if (cfg_valid && !check_fail) next_state = CORE_F_IDLE;
                          // 否则永久卡在 ERROR, 等 reset 或合法 cfg_valid
        endcase
    end

    // ═══════════════════════════════════════════════════════════════
    // 主时序逻辑
    // ═══════════════════════════════════════════════════════════════
    always_ff @(posedge core_clk or negedge core_rst_n) begin
        if (!core_rst_n) begin
            core_busy <= 0; layer_done <= 0;
            layer_start <= 0; core_error <= 0;
            mac_en <= 0;
            flush_strobe <= 0; clear_accum <= 0;
            core_fsm_state <= CORE_F_IDLE;
            cnt_cur <= '0;
            cnt_prev <= '0;
            L <= '0;
            L_ocg_max <= 0;
        end else begin
            layer_done <= 0; flush_strobe <= 0;
            clear_accum <= 0; layer_start <= 0;

            case (state)
                CORE_F_IDLE: begin
                    core_busy <= 0;
                    if (cfg_valid) begin
                        // 锁存层配置 (desc_cfg_t 完整锁存)
                        L <= cfg;
                        // out_w/out_h 已由编译器预填在 cfg 中, 无需硬件计算
                        // ocg 分组: MODE_8x8→8ch/组, MODE_1x64→64ch/组
                        if (cfg.mode == MODE_1x64)
                            L_ocg_max <= (cfg.out_ch + 8'd63) / 8'd64 - 5'd1;
                        else
                            L_ocg_max <= (cfg.out_ch + 8'd7) / 8'd8 - 5'd1;
                    end
                end

                CORE_F_CHECK: begin
                    if (check_fail) begin
                        core_error <= 2'b10;    // FSM 参数异常
                        core_busy  <= 0;
                    end else begin
                        layer_start <= 1;       // 脉冲: 通知 im_agu 初始化 DDA 状态
                        core_error <= 2'b00;
                        // 初始化计数器
                        cnt_cur <= '0;
                        cnt_prev <= '0;
                        core_busy <= 1;
                        // keep_accum=0 时清零 PE 累加器; =1 时保留 (FC 跨 chunk)
                        if (!L.keep_accum) clear_accum <= 1;
                    end
                end

                CORE_F_COMPUTE: begin
                    mac_en <= 1;
                    // im_rd_req / wt_rd_req: 组合 assign (见末尾), 不在 always_ff 中赋值

                    // ── 统一 6 层嵌套 (ocg→oy→ox→ic→ky→kx), 双模共用 ──
                    //   MODE_8x8: L.kernel_w/L.kernel_h 为卷积核尺寸, L_ow/L_oh 为输出尺寸
                    //   MODE_1x64: L.kernel_w=in_w, L.kernel_h=in_h, L_ow=L_oh=1 (自然退化)
                    if (kx_last) begin
                        cnt_cur.kx <= 0;
                        if (ky_last) begin
                            cnt_cur.ky <= 0;
                            if (ic_last) begin
                                cnt_cur.ic <= 0;
                                if (!L.disable_flush) begin
                                    flush_strobe <= 1;
                                    clear_accum <= 1;
                                end
                                if (ox_last) begin
                                    cnt_cur.ox <= 0;
                                    if (oy_last) begin
                                        cnt_cur.oy <= 0;
                                        if (ocg_last) begin
                                            cnt_cur.ocg <= 0;
                                        end else cnt_cur.ocg <= cnt_cur.ocg + 1;
                                    end else cnt_cur.oy <= cnt_cur.oy + 1;
                                end else cnt_cur.ox <= (L.mode == MODE_8x8) ? cnt_cur.ox + 8'd8 : cnt_cur.ox + 8'd1;  // PE spatial parallelism = 8
                            end else cnt_cur.ic <= cnt_cur.ic + 1;
                        end else cnt_cur.ky <= cnt_cur.ky + 1;
                    end else cnt_cur.kx <= cnt_cur.kx + 1;

                    // 更新前拍计数器 (变化检测)
                    cnt_prev.ic  <= cnt_cur.ic;  cnt_prev.ox  <= cnt_cur.ox;
                    cnt_prev.oy  <= cnt_cur.oy;  cnt_prev.ky  <= cnt_cur.ky;
                    cnt_prev.kx  <= cnt_cur.kx;  cnt_prev.ocg <= cnt_cur.ocg;
                end

                CORE_F_DONE: begin
                    mac_en <= 0;
                    if (fifo_empty && (sram_write_cnt >= L.im_total_writes)) begin
                        layer_done <= 1;
                        core_busy <= 0;
                    end
                end

                CORE_F_ERROR: begin
                    mac_en <= 0; core_busy <= 0;
                    if (cfg_valid && !check_fail) begin
                        // 合法配置到来: 清除 error, 回到 CHECK
                        core_error <= 2'b00;
                        // next_state 会走到 CORE_F_CHECK, 这里只清 error
                    end
                end
            endcase

            core_fsm_state <= state;
        end
    end

    // ═══════════════════════════════════════════════════════════════
    // im_rd_req / wt_rd_req: 组合逻辑
    // ═══════════════════════════════════════════════════════════════
    assign im_rd_req = mac_en && (cnt.kx == 0);
    assign wt_rd_req = ((L.mode == MODE_1x64) ? 1'b1 : (cnt.kx == 0)) && mac_en;

endmodule

`default_nettype wire
