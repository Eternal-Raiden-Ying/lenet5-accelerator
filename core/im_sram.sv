// ═══════════════════════════════════════════════════════════════════════════════
// IM SRAM — 4KB 奇偶双 Bank (even:256×64b + odd:256×64b)
// ═══════════════════════════════════════════════════════════════════════════════
//
// 【设计原则: "读地址先 MUX, 读数据后 DEMUX"】
//
//   每 Bank 内部只有一处 `bank_xxx[addr]` 出现在 `always_ff` 中 → 工具推断
//   标准 1R1W BRAM(1 读地址寄存器 + 1 写地址寄存器,无多端口冲突)。
//
//   关键时序:
//     Cycle N:   组合逻辑选出 final_rd_addr_even/odd → BRAM 锁存读地址
//                同时组合控制信号(read_lo_even, ext_rd_en) 打入延迟寄存器
//     Cycle N+1: BRAM 吐出 rdata_even/odd_raw
//                延迟寄存器的控制信号(read_lo_even_q, ext_rd_en_q) 做 DEMUX
//                将 raw 数据路由到正确的输出端口
//
//   如果不用延迟寄存器,直接在 always_ff 中用组合 MUX 选数据:
//     im_rd_data_lo <= read_lo_even ? bank_even[...] : bank_odd[...];
//   则 read_lo_even 在 Cycle N 时是旧值(Cycle N-1 的地址决定),
//   在 Cycle N+1 数据到达时 read_lo_even 已更新为 Cycle N+1 的地址决定,
//   → 数据与通道错位。打拍后 read_lo_even_q 始终对齐发出读请求那一刻的地址。
//
// 【外部访问优先级】
//   当 im_ext_cs && !im_ext_wr 时,外部读覆盖内部读地址(两个 Bank 都读同一地址)。
//   spec 保证外部访问仅在 core_busy==0 时发生,因此内外读不会同时竞争。
//   写路径同理:im_ext_cs && im_ext_wr 优先于 im_wr_en。
//
// ═══════════════════════════════════════════════════════════════════════════════
`default_nettype none

import core_pkg::*;

module im_sram #(parameter DEPTH = IM_DEPTH) (   // 512 (总深度,保留以兼容)
    input  wire        core_clk,
    input  wire        core_rst_n,

    // ── External access (Wrapper) ──
    input  wire        im_ext_cs,
    input  wire        im_ext_wr,
    input  wire  [8:0] im_ext_addr,
    input  wire [63:0] im_ext_wdata,
    output logic [63:0] im_ext_rdata,
    output logic        im_ext_ready,

    // ── Internal read (im_agu / Preprocess) ──
    input  wire  [8:0] im_rd_addr_lo, im_rd_addr_hi,
    output logic [63:0] im_rd_data_lo, im_rd_data_hi,
    input  wire        im_rd_req,

    // ── Internal write (Corner-Turn) ──
    input  wire        im_wr_en,
    input  wire  [8:0] im_wr_addr,
    input  wire [63:0] im_wr_data
);

    localparam int BANK_DEPTH = DEPTH / 2;  // 256

    // ── Two independent banks (inferred BRAM, each 256×64b = 2KB) ──
    logic [63:0] bank_even [0:BANK_DEPTH-1];
    logic [63:0] bank_odd  [0:BANK_DEPTH-1];

    // ═══════════════════════════════════════════════════════════════════
    // 1. 写端口 (Write — 外部优先)
    //    外部写和内部写互斥: spec 保证 im_ext_cs 仅在 core_busy==0 时有效
    // ═══════════════════════════════════════════════════════════════════
    always_ff @(posedge core_clk) begin
        if (im_ext_cs && im_ext_wr) begin
            // Wrapper 外部写(FETCH_FEATURE)
            if (im_ext_addr[0])
                bank_odd[im_ext_addr[8:1]] <= im_ext_wdata;
            else
                bank_even[im_ext_addr[8:1]] <= im_ext_wdata;
        end else if (im_wr_en) begin
            // Corner-Turn 内部写
            if (im_wr_addr[0])
                bank_odd[im_wr_addr[8:1]] <= im_wr_data;
            else
                bank_even[im_wr_addr[8:1]] <= im_wr_data;
        end
    end

    // ═══════════════════════════════════════════════════════════════════
    // 2. 读地址仲裁 (Read Address MUX — 纯组合逻辑)
    //    外部读优先; 内部读: addr_lo[0] 决定 lo/hi 分别路由到 even/odd Bank
    // ═══════════════════════════════════════════════════════════════════
    logic        ext_rd_en;               // 外部读请求(组合)
    logic [7:0]  final_rd_addr_even;      // even Bank 最终读地址
    logic [7:0]  final_rd_addr_odd;       // odd  Bank 最终读地址

    // 内部读: 解析 addr_lo / addr_hi 的路由
    //   addr_lo[0]==0 → lo 来自 even, hi 来自 odd
    //   addr_lo[0]==1 → lo 来自 odd,  hi 来自 even
    logic        read_lo_even;
    logic [7:0]  rd_addr_even_int;
    logic [7:0]  rd_addr_odd_int;

    assign ext_rd_en = im_ext_cs && !im_ext_wr;

    always_comb begin
        // 内部读取: 按 LSB 分配 lo/hi 到 even/odd Bank
        read_lo_even = ~im_rd_addr_lo[0];
        if (read_lo_even) begin
            rd_addr_even_int = im_rd_addr_lo[8:1];
            rd_addr_odd_int  = im_rd_addr_hi[8:1];
        end else begin
            rd_addr_even_int = im_rd_addr_hi[8:1];
            rd_addr_odd_int  = im_rd_addr_lo[8:1];
        end

        // 读地址仲裁: 外部访问优先
        //   外部读时两个 Bank 都读 im_ext_addr[8:1],后续由 ext_addr_0_q 选正确的 Bank
        if (ext_rd_en) begin
            final_rd_addr_even = im_ext_addr[8:1];
            final_rd_addr_odd  = im_ext_addr[8:1];
        end else begin
            final_rd_addr_even = rd_addr_even_int;
            final_rd_addr_odd  = rd_addr_odd_int;
        end
    end

    // ═══════════════════════════════════════════════════════════════════
    // 3. 统一读端口 (Single Read Instance — 每 Bank 唯一一处读)
    //    控制信号同步打拍,对齐 BRAM 的 1-cycle 读延迟
    // ═══════════════════════════════════════════════════════════════════
    logic [63:0] rdata_even_raw;          // even Bank 读出的原始数据
    logic [63:0] rdata_odd_raw;           // odd  Bank 读出的原始数据

    // 控制信号延迟 1 拍,用于数据到达后的 DEMUX
    logic        ext_rd_en_q;
    logic        ext_addr_0_q;
    logic        read_lo_even_q;

    logic read_en;
    assign read_en = im_rd_req || ext_rd_en;

    always_ff @(posedge core_clk) begin
        if (read_en) begin
            // 每个 Bank 仅此一处读操作 → 工具推断 BRAM 的读地址/输出寄存器
            rdata_even_raw <= bank_even[final_rd_addr_even];
            rdata_odd_raw  <= bank_odd[final_rd_addr_odd];

            // 控制信号打拍,与 BRAM 数据同步到达
            ext_rd_en_q    <= ext_rd_en;
            ext_addr_0_q   <= im_ext_addr[0];
            read_lo_even_q <= read_lo_even;
        end
    end

    // ═══════════════════════════════════════════════════════════════════
    // 4. 读出数据路由 (Read Data DEMUX — 纯组合,零额外延迟)
    // ═══════════════════════════════════════════════════════════════════

    // 内部接口: 使用打拍后的控制信号,对齐 BRAM 数据到达时刻
    assign im_rd_data_lo = read_lo_even_q ? rdata_even_raw : rdata_odd_raw;
    assign im_rd_data_hi = read_lo_even_q ? rdata_odd_raw  : rdata_even_raw;

    // 外部接口: 带复位,1 拍握手延迟(标准同步 SRAM 时序)
    always_ff @(posedge core_clk or negedge core_rst_n) begin
        if (!core_rst_n) begin
            im_ext_rdata <= 64'd0;
            im_ext_ready <= 1'b0;
        end else begin
            // ext_rd_en 在 Cycle N 为 1 → Cycle N+1 ready 拉高
            im_ext_ready <= ext_rd_en;
            // ext_rd_en_q 对齐 BRAM 数据,ext_addr_0_q 选择正确 Bank
            if (ext_rd_en_q) begin
                im_ext_rdata <= ext_addr_0_q ? rdata_odd_raw : rdata_even_raw;
            end
        end
    end

endmodule

`default_nettype wire
