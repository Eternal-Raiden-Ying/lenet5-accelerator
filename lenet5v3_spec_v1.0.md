# Lenet5v3 硬件加速器规格书 v1.0

> **v1.0 重大架构变更** (自 v0.3): 引入 Bus_Wrapper / Compute_Core 物理划分; APB Slave + AHB-Lite Master 固化; 描述符 64B Shadow Register 严格字节对齐; Corner-Turn 双模 (空间转置/时间齿轮箱); Core 降为单层执行引擎; cfg_im_total_writes 编译器填入。

---

## 1. 系统总览

```
                         APB Slave                    AHB-Lite Master
                      @ 0x4000B000                    @ INITEXP0
                           │                               │
┌──────────────────────────┼───────────────────────────────┼──────────────────────┐
│                    Bus_Wrapper (总线外壳)                                          │
│                                                                                  │
│  ┌───────────┐ ┌──────────────────────┐ ┌───────────┐ ┌──────────────────────┐ │
│  │ APB CSR   │ │ dma_scheduler (4状态) │ │ AHB Trans  │ │ Weight SRAM          │ │
│  │ (6 regs)  │ │ + compute_mgmt (4状态)│ │ FSM (INCR16)│ │  Bank A 16KB         │ │
│  │           │ │                      │ │            │ │  Bank B 16KB         │ │
│  │           │ │ 握手: dma_ready      │ │            │ │  + Gearbox 32b→512b  │ │
│  │           │ │       compute_done   │ │            │ │                      │ │
│  │           │ │       bank_toggle    │ │            │ │ Requant SRAM (v1.1)  │ │
│  │           │ │                      │ │            │ │  Bank A 16×288b      │ │
│  │           │ │ shadow_register      │ │            │ │  Bank B 16×288b      │ │
│  │           │ │ (64B,组合解包)       │ │            │ │  + Gearbox 32b→288b  │ │
│  └───────────┘ └──────────┬───────────┘ └───────────┘ └──────────┬───────────┘ │
│                           │                                      │              │
│              cfg_* (18根) │ layer_done             im_ext (6根)  │ wt/rq rd     │
│              cfg_valid    │ core_busy                            │ (8根)        │
└───────────────────────────┼──────┬───────────────────────────────┼──────────────┘
                            │      │                               │
┌───────────────────────────▼──────▼───────────────────────────────▼──────────────┐
│                         Compute_Core (计算核心)                                   │
│                                                                                  │
│  ┌──────────────┐   ┌───────────────────────────────────────────────────────┐   │
│  │ Compute_FSM  │   │                    Datapath                            │   │
│  │ (单层引擎)   │   │                                                       │   │
│  │              │   │  IM SRAM (4KB, 64-bit 双口)                           │   │
│  │ cfg_valid→   │   │  ├─ Port A: Preprocess(读) + CornerTurn(写)          │   │
│  │   run 1 layer│   │  └─ Port B: Wrapper ext access (FETCH/WRITEBACK)     │   │
│  │   →layer_done│   │                                                       │   │
│  └──────────────┘   │  Preprocess(128b移位窗)→Router(8x8/1x64)→PE(64 MAC)  │   │
│                     │  →FIFO(2048b→128b)→Requant×4(TDM+zp_y)→             │   │
│                     │  PingPong(448B)→Pool(2×2,4ch)/Bypass→CornerTurn     │   │
│                     │  (双模: 空间转置/时间齿轮箱)→IM SRAM                  │   │
│                     └───────────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────────────────────────────┘
```

**关键参数**

| 参数 | 值 | 说明 |
|---|---|---|
| 目标平台 | ARM Cortex-M3 DesignStart FPGA | — |
| 目标频率 | 200 MHz | PCLK == HCLK 同频同相 |
| APB Slave | APBTARGEXP11 @ 0x4000B000 | 6 寄存器, 零等待 |
| AHB Master | INITEXP0 | 32-bit aligned, INCR16 burst, HMASTLOCK=0 |
| 计算阵列 | 64 PE | uint8 × int8 → int32, Output Stationary |
| Weight SRAM | Bank A 16KB + Bank B 16KB | 真·双缓冲, 512-bit 读 / 32-bit 写 |
| Requant SRAM | 16 条 × 2 Bank × 288-bit | 32b→288b Gearbox ×2 写入, 288-bit 直读, bank_toggle 路由 |
| IM SRAM | 4 KB | 64-bit 双口, Core 闭环 + Wrapper 外部访问 |
| 描述符 | 64 Bytes | INCR16 一次读完, Shadow Register 组合逻辑解包 |
| 控制流 | 双 FSM 拆分: dma_scheduler (4状态) + compute_mgmt (4状态) | bank_toggle 无毛刺握手, 流水线间距=1 |

---

## 2. 系统接口

### 2.1 APB Slave (APBTARGEXP11, 0x4000B000)

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `PCLK` | 1 | In | 总线时钟 (== HCLK) |
| `PRESETn` | 1 | In | 异步复位, 低有效 |
| `PSEL` | 1 | In | 片选 |
| `PENABLE` | 1 | In | APB 第二拍 |
| `PWRITE` | 1 | In | 1=写, 0=读 |
| `PADDR[11:0]` | 12 | In | 寄存器地址 |
| `PWDATA[31:0]` | 32 | In | 写数据 |
| `PRDATA[31:0]` | 32 | Out | 读数据 |
| `PREADY` | 1 | Out | 硬连线 = 1 (零等待) |

**CSR 寄存器表**

| 寄存器 | 偏移 | 位宽 | 权限 | 说明 |
|---|---|---|---|---|
| `CTRL` | 0x00 | 32 | R/W | `[0]`: start (写1启动, 自清)。`[1]`: **soft_reset (电平)** — 写 1 将 Core 摁在复位 (`core_rst_n=0`), CPU 须**显式写 0 释放**。仅复位 Core (PE/FSM/SRAM), Wrapper (AHB/APB) 正常运行。握手序列: 写 CTRL[1]=1 → 等待 `core_busy==0` → 写 CTRL[1]=0 → Core 重新就绪。`[31:16]`: FSM_STATE (RO) |
| `DESC_HEAD_PTR` | 0x04 | 32 | R/W | 描述符链表头指针 (Flash/DDR 绝对地址) |
| `INT_STATUS` | 0x08 | 32 | R | `[0]`: inference_done, `[1]`: error, `[7:4]`: error_code |
| `INT_CLEAR` | 0x0C | 32 | W | 写 1 清 `[0]` |
| `DEBUG_STATE` | 0x10 | 32 | R | `[1:0]`: dma_scheduler FSM, `[3:2]`: compute_mgmt FSM, `[5:4]`: AHB Master FSM, `[8]`: bank_toggle, `[9]`: core_busy |
| `VERSION` | 0xFC | 32 | R | `32'h0000_0100` |

> CSR 仅 6 个寄存器。`LAST_S_Y`, `LAST_ZP_Y`, `ZP_Y_CURRENT` 已删除。s_y/zp_y 是描述符中的字段，CPU 离线已知，加速器无需缓存。

### 2.2 AHB-Lite Master (INITEXP0)

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `HADDRM[31:0]` | 32 | Out | 访问绝对地址 |
| `HTRANSM[1:0]` | 2 | Out | 00=IDLE, 10=NONSEQ, 11=SEQ |
| `HWRITEM` | 1 | Out | 1=写结果, 0=读权重/描述符/特征图 |
| `HSIZEM[2:0]` | 3 | Out | 固定 3'b010 (32-bit Word) |
| `HBURSTM[2:0]` | 3 | Out | 000=SINGLE, 011=INCR16 |
| `HWDATAM[31:0]` | 32 | Out | 写数据 (结果写回) |
| `HRDATAM[31:0]` | 32 | In | 读数据 |
| `HREADYM` | 1 | In | **反压**。0=外部存储未就绪, 锁死全部输出 |
| `HRESPM` | 1 | In | **错误响应**。1=ERROR (AHB-Lite 单 bit)。捕获后置 error_code=1, 中止传输回 D_IDLE, INT_STATUS 上报 |
| `HMASTLOCKM` | 1 | Out | 固定 1'b0 |

**AHB 事务类型**:
- 描述符读: INCR16 (64B = 16 words, 恰好一次突发)
- 权重/特征图大块读: INCR16 为主, 尾部不足 16 words 用 SINGLE 或短 INCR
- 结果写回: SINGLE 或短 INCR (10 字节 ≈ 3~4 words)

---

## 3. Shadow Register 与描述符格式

### 3.1 硬件结构

Wrapper 内部唯一的 512-bit 影子寄存器 `desc_shadow_reg[511:0]`。AHB Master INCR16 读入 64 Bytes 后，全部字段通过**纯组合 assign** 解包——无状态机, 无序列解析。

```
AHB HRDATAM[31:0] → 16次 → desc_shadow_reg[511:0] → assign → cfg_* / dma_* 导线
```

### 3.2 位域映射 (最终版, 严格字节对齐)

```
┌───────────┬─────────┬─────────────────────────────────────┐
│ 字节范围   │ 位域     │ 字段                                │
├───────────┼─────────┼─────────────────────────────────────┤
│ Byte 0-1  │ [9:0]   │ cfg_in_ch (10b)                     │
│ Byte 2-3  │ [25:16] │ cfg_out_ch (10b)                    │
│ Byte 4    │ [39:32] │ cfg_in_w (8b)                       │
│ Byte 5    │ [47:40] │ cfg_in_h (8b)                       │
│ Byte 6    │ [51:48] │ cfg_kernel_w (4b)                   │
│           │ [55:52] │ cfg_kernel_h (4b)                   │
│ Byte 7    │ [59:56] │ cfg_stride_w (4b)                   │
│           │ [63:60] │ cfg_stride_h (4b)                   │
│ Byte 8    │ [67:64] │ cfg_pad_w (4b)                      │
│           │ [71:68] │ cfg_pad_h (4b)                      │
│ Byte 9    │ [73:72] │ cfg_router_mode (2b)                │
│           │ [74]    │ cfg_pool_bypass (1b)                │
│ Byte 10   │ [87:80] │ cfg_zp_y (8b)  ← 独立1字节          │
│ Byte 11   │ [95:88] │ Reserved                            │
│ Byte 12-13│ [105:96]│ cfg_mem_stride_w (10b)              │
│ Byte 14-15│ [127:112]│ cfg_im_read_base (16b) 半字对齐     │
│ Byte 16-17│ [143:128]│ cfg_im_write_base (16b) 半字对齐    │
│ Byte 18-19│ [159:144]│ cfg_im_total_writes (16b) 半字对齐  │
├───────────┼─────────┼─────────────────────────────────────┤
│           │         │ ─── Wrapper/Core 边界 ───            │
├───────────┼─────────┼─────────────────────────────────────┤
│ Byte 20-23│ [191:160]│ dma_weight_bytes (32b) Word对齐     │
│ Byte 24-27│ [223:192]│ dma_requant_bytes (32b)             │
│ Byte 28-31│ [255:224]│ dma_weight_ddr_ptr (32b)            │
│ Byte 32-35│ [287:256]│ dma_requant_ddr_ptr (32b)           │
│ Byte 36-39│ [319:288]│ dma_feature_ddr_ptr (32b) 仅desc[0]│
│ Byte 40-43│ [351:320]│ dma_result_ddr_ptr (32b) 仅desc[末] │
│ Byte 44-47│ [383:352]│ sequencer_next_desc (32b)            │
│ Byte 48   │ [384]    │ sequencer_is_last (1b)              │
│ Byte 49-51│ [415:385]│ Reserved (= 0)                      │
│ Byte 52-55│ [447:416]│ dma_feature_bytes (32b) 仅desc[0]   │
│ Byte 56-63│ [511:448]│ Reserved (= 0)                      │
└───────────┴─────────┴─────────────────────────────────────┘
```

> **编译器**: 按 Little-Endian 32-bit Word 顺序写入。`Reserved` 填 0。所有多字节字段遵循 Little-Endian 字节序 (LSB 在低地址)。
> **dma_feature_bytes** (Word 13): 输入特征图在 DDR 中的 **padded** 字节总数 (仅 desc[0] 有效)。硬件据此计算 FETCH_FEATURE 的 AHB 读字数 (`bytes>>2`) 与 IM SRAM 写次数 (`bytes>>3`)。**必须 8 字节对齐** (IM SRAM 以 64-bit word 存储; §6 已要求 mem_stride_w padding 到 8 的倍数, 故 padded 总量天然 8 对齐, 两处位移均为精确除法)。硬件禁止用逻辑尺寸 in_ch×in_w×in_h 自算——会欠取 padding 区。

---

## 4. Bus_Wrapper 模块

### 4.1 控制流 — dma_scheduler + compute_mgmt

双 FSM 已从单块 Sequencer 拆分为两个独立模块, 均运行于 HCLK 域, 通过 3 根信号握手:

| 信号 | 方向 | 含义 |
|---|---|---|
| `dma_ready` | D→C | 电平。下一层 desc+wt+rq 全部在闲置 Bank (~bank_toggle) 中就位 |
| `compute_done` | C→D | 电平。Core 完成当前层, 闲置 Bank 可被安全覆盖 |
| `bank_toggle` | 共享 | 寄存器 (在 bus_wrapper 顶层)。`(compute_done && dma_ready) → ~bank_toggle` |

**dma_scheduler (4 状态)** — DMA_FSM, 总揽全部 AHB 总线搬运:

```
D_IDLE ──(csr_start_pulse)──▶ D_PREFILL ──(core_busy)──▶ D_PREFETCH ──(is_last)──▶ D_TAIL
                                  │                          │  ▲                      │
                                  │              (bank_toggle 翻转)│                      │
                                  │                          └──┘                      │
                                  │                          (循环, 非末层)              │
                                  └──────────────────────────────────────────────────┘
                                                (is_last, 单层网络)
```

| 状态 | 子阶段 | 说明 |
|---|---|---|
| `D_IDLE` | — | 复位态, 死等 `csr_start_pulse` |
| `D_PREFILL` | phase 0→4 | 冷启动: desc[0]→shadow_reg → wt[0]→Bank A → rq[0]→Requant Bank A → FETCH_FEATURE → dma_ready=1, 等 core_busy |
| `D_PREFETCH` | phase 0→4 | 稳态循环: 等 core_busy → 检查 is_last → desc[N+1]→shadow_reg → wt[N+1]→闲置 Weight Bank → rq[N+1]→闲置 Requant Bank → dma_ready=1 → 等 bank_toggle 翻转 → 循环 |
| `D_TAIL` | phase 0→1 | 尾部: 等 compute_done → WRITEBACK_RESULT (im_ext 读→AHB 写) → inference_done → D_IDLE |

**compute_mgmt (4 状态)** — Compute_FSM, 仅管 cfg_valid 脉冲与 layer_done 判定:

```
C_IDLE ──(dma_ready)──▶ C_ISSUE_CFG ──(1拍)──▶ C_WAIT_LAYER ──(saved_is_last)──▶ C_DONE
                            ▲                        │  │                              │
                            │        (bank_toggle翻转, │  │                              │
                            │         非末层:清除       │  │                              │
                            │         compute_done)    │  │                              │
                            └──────────────────────────┘  │                              │
```

| 状态 | 说明 |
|---|---|
| `C_IDLE` | 仅冷启动。死等 `dma_ready` (D_PREFILL 完成) |
| `C_ISSUE_CFG` | 1 拍。锁存 `sr_is_last` → `saved_is_last`, 发 `cfg_valid` 脉冲 |
| `C_WAIT_LAYER` | `layer_done`→`compute_done=1`。末层(`saved_is_last`): 跳过 bank_toggle 等待 (dma_ready=0, toggle 永不会翻)→C_DONE。非末层: 等 `bank_toggle` 翻转→清除 `compute_done`→C_ISSUE_CFG |
| `C_DONE` | 保持 `compute_done=1` (供 D_TAIL 采样), 等 `inference_done`→清除→C_IDLE |

**bank_toggle 握手**:
```
翻转条件: compute_done && dma_ready
无毛刺: bank_toggle <= ~bank_toggle  (仅当上述条件满足, 在 bus_wrapper 顶层)
MUX 驱动: Weight SRAM + Requant SRAM 读端口均由 bank_toggle 寄存器直驱
DMA 写目标: wt_gb_bank_sel = rq_gb_bank_sel = ~bank_toggle (闲置侧)
并发公式: Core 算 Layer N (读 bank_toggle), DMA 预取 Layer N+1 (写 ~bank_toggle), 间距=1
```

### 4.2 存储与 Gearbox

**Weight SRAM (双 Bank)**
- 每 Bank: 16KB = 256 行 × 512-bit
- 写侧: 32-bit AHB → 32b→512b Gearbox (移位寄存器: 新 word 从高位推入, 旧数据右移; 16 个 word 后行写入)
  - gb_shift = 480-bit (15 words), 自清洁——连续行间无需 gearbox_rst
- 读侧: **同步寄存器读** `dout_A/dout_B` + 组合 MUX, 保证综合工具能将 bank_A/bank_B 推断为 BRAM/SRAM 宏单元
  - Core `wt_rd_addr[7:0]` → bank_toggle 选择 dout_A 或 dout_B → 512-bit `wt_rd_data`
- fc1 编译器切分为两个虚拟层, Wrapper 通过 bank_toggle 自动切换物理 Bank

**Requant SRAM (v1.2 双 Bank)**
- Bank A: 16 条 × 288-bit
- Bank B: 16 条 × 288-bit
- 共享 Gearbox: 32b→288b (移位寄存器: gb_shift = 256-bit, 8 words + 第 9 word 抵达写 Bank)
- 读侧: **同步寄存器读** `dout_A/dout_B` + 组合 MUX, 保证 BRAM 推断
- 写侧: DMA 在 D_PREFETCH 中与权重一同预取到闲置 Bank (~bank_toggle)
- 总容量: 32 条 × 288-bit ≈ 1.15 KB (v1 的 2×, 面积换延迟——消除 ~160 cycle requant 重载气泡)
- fc1 chunk_0/chunk_1 的 requant 在 DDR 中相同, 通过 bank_toggle 自动切换到不同物理 Bank, 每 chunk 各自预取一次 (开销可接受)

---

## 5. Compute_Core 模块

### 5.1 核心原则 (层级无关)

```
✓ cfg_valid 脉冲 → 锁存 cfg_* → 清零内部计数器 → 跑一层 → layer_done → 休眠
✗ 禁止任何硬编码层名 (conv1, fc1 等)
✗ 禁止特殊层判断 (if out_ch==10 → "这是末层")
✗ 禁止 fc1 切分感知
```

**Output Stationary 循环顺序** (kx 最内层——IM 一次读覆盖全部 kx, 仅 MUX 不重读):

```
for ocg in 0..ceil(out_ch/64)-1:      // 输出通道组 (最外层)
  for oy in 0..out_h-1:                // 输出行
    for ox in 0..out_w-1:              // 输出列
      for ic in 0..in_ch-1:            // 输入通道
        for ky in 0..kernel_h-1:       // kernel 行
          for kx in 0..kernel_w-1:     // kernel 列 (最内层)
            // 一拍 MAC
```

IM 读时机: ox/ic/ky 变化 → 2 读建 128b 窗口; kx 变化 → MUX 偏移, 不读。
每 (ox,oy) 经 in_ch×kernel_h×kernel_w 拍累加后 FLUSH 一个 2048b final sum。
典型生产周期 = 200 拍 (conv2, in_ch=8, k=5×5)。FIFO 排空 = 16 拍。16 ≪ 200 → depth=2。

### 5.2 数据路径

```
IM SRAM (4KB)
    │
    ▼
Preprocess (AGU + 128-bit 移位窗 + valid 掩码)
    │ 64-bit (8 pixels)
    ▼
Broadcast Router (MODE_8x8 / MODE_1x64)
    │
    ▼
PE Array (64 MAC, Output Stationary)
    │ 2048-bit (每 25 拍)
    ▼
FIFO Gearbox (2048b → 128b)
    │
    ▼
Requant ×4 TDM + zp_y 广播加法
    │ 公式: clamp(round(y_int32*M_fixed>>shift) + zp_y, 0, 255)
    │
    ▼
┌──────────────────────────────────┐
│ pool_bypass=0 (卷积层)           │  pool_bypass=1 (FC 层)
│   PingPong(448B) → Pool(2×2)    │   MUX 旁路直通
└──────────────┬───────────────────┘
               ▼
┌──────────────────────────────────────────────┐
│           Corner-Turn (双模)                  │
│                                              │
│  空间转置 (pool_bypass=0):                    │
│    4×8 寄存器矩阵 → Scatter Write CHW Planar │
│                                              │
│  时间齿轮箱 (pool_bypass=1):                  │
│    32b→64b 拼接寄存器 → Sequential Write     │
└──────────────────┬───────────────────────────┘
                   ▼
              IM SRAM (4KB)
```

### 5.3 Corner-Turn 双模

**空间转置模式 (pool_bypass=0)**:
- 输入: Pool 输出 (4ch × 1px/cycle, C-Major)
- 内部: 4×8 寄存器矩阵, 收集 8 个 W 空间位置
- 输出: 满 8W 后爆发 4×64-bit (每通道一个 64-bit word, 覆盖 8 像素)
- 写地址: Scatter-Write AGU (CHW Planar 跳跃散写)

**时间齿轮箱模式 (pool_bypass=1)**:
- 输入: Requant TDM 直出 (4ch × 1px/cycle), 空间永远 1×1
- 内部: 64-bit 拼接寄存器
  - Cycle 0: {C0,C1,C2,C3} → reg[31:0]
  - Cycle 1: {C4,C5,C6,C7} → reg[63:32], 写 IM SRAM, 清空
- 末组不足 8ch: 高位填 0
- 写地址: 顺序递增 (addr += 8), 无需 scatter

**写计数器**: `sram_write_cnt` 每次 Corner-Turn 写 IM SRAM 时加 1。当 `sram_write_cnt == cfg_im_total_writes` → 拉高 `layer_done`, Core 休眠。

**cfg_im_total_writes 编译器公式**:

| 模式 | 公式 |
|---|---|
| Conv (pool_bypass=0) | `out_ch × out_h × (cfg_mem_stride_w / 8)` |
| FC (pool_bypass=1) | `(out_ch + 7) / 8` |

### 5.4 Preprocess

- **128-bit 对齐移位窗**: `shift_reg = {prev_64bit_read, cur_64bit_read}`, MUX 基于 `x % 8` 选择连续 64-bit 片段
- **读请求**: 每 8 个逻辑 x 步进 (`x % 8 == 0`) 发起一次 IM SRAM 读
- **动态 Padding**: 逻辑坐标越界时 valid 掩码拉低, 输出 0
- **地址生成**: 增量累加, 无乘法器

### 5.5 Requant

- **参数**: Requant SRAM 288-bit 读 → 4 通道并发 `{M_fixed[32], shift[8], b_fused[32]}`
- **zp_y**: 来自 `cfg_zp_y[7:0]`, 由 shadow_register 组合 assign 从描述符 Byte 10 直接广播到 Core (不经过 CSR)
- **clamp(0,255)**: 同时实现 ReLU (uint8 域 min=0)
- **TDM**: 4 单元时分复用处理 64 通道

---

## 6. 执行流程

### 6.1 完整推理时序

```
1. CPU 准备描述符链 + 权重 Blob + 特征图 → DDR/Flash
2. CPU 写 DESC_HEAD_PTR, 写 CTRL.start=1 (APB)
3. dma_scheduler (DMA_FSM):
   D_PREFILL: desc[0]→shadow_reg → wt[0]→Weight Bank A → rq[0]→Requant Bank A
              → FETCH_FEATURE (im_ext 写 IM SRAM) → dma_ready=1
   D_PREFETCH 循环 {
        [并发] Core 算 Layer N (读 bank_toggle Bank, 含 Weight + Requant)
        [并发] DMA 预取 Layer N+1: desc[N+1]→shadow_reg,
               wt[N+1]→闲置 Weight Bank, rq[N+1]→闲置 Requant Bank
               (三者均在 ~bank_toggle, 无 requant 气泡)
        → dma_ready=1 → 等 bank_toggle 翻转 → 循环
     }
   D_TAIL: 等末层 compute_done → WRITEBACK_RESULT (im_ext 读→AHB 写)
          → inference_done=1
4. compute_mgmt (Compute_FSM):
   C_IDLE: 等 dma_ready → C_ISSUE_CFG: 锁存 is_last, 发 cfg_valid
   → C_WAIT_LAYER: 等 layer_done → compute_done=1
     → 非末层: 等 bank_toggle 翻转 → 回 C_ISSUE_CFG
     → 末层:   跳过 bank_toggle (dma_ready=0, 永不翻) → C_DONE → 等 inference_done
5. CPU 读 INT_STATUS, 从 RESULT_BASE 取 logits (10B uint8)
6. CPU 执行 argmax (或 + s_y/zp_y softmax)
```

### 6.2 fc1 切分示例

编译器遇到 fc1 (400×64, 25.6KB > 16KB Bank):

```
描述符 N (fc1_chunk_0):
  in_ch=400, out_ch=40
  pool_bypass=1, router_mode=MODE_1x64
  weight_ddr_ptr = &wt_fc1[0]
  requant_ddr_ptr = &rq_fc1
  im_write_base   = 0x100
  im_total_writes = (40+7)/8 = 5

描述符 N+1 (fc1_chunk_1):
  in_ch=400, out_ch=24
  (同上几何参数)
  weight_ddr_ptr = &wt_fc1[16384]
  requant_ddr_ptr = &rq_fc1           ← 完全相同!
  im_write_base   = 0x100 + 40 = 0x128
  im_total_writes = (24+7)/8 = 3

描述符 N+2 (fc2):
  in_ch=64
  im_read_base = 0x100                ← 两 chunk 结果自然拼接
```

> Core 将此视为三个普通连续层, 无任何特殊处理。

---

## 7. Wrapper ↔ Core 接口信号表

### 7.1 时钟复位

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `core_clk` | 1 | W→C | == HCLK |
| `core_rst_n` | 1 | W→C | 同步复位 |

### 7.2 层配置 (W→C)

| 信号 | 位宽 | 说明 |
|---|---|---|
| `cfg_valid` | 1 | 脉冲: 配置有效, Core 启动本层 |
| `cfg_in_ch` | 10 | — |
| `cfg_out_ch` | 10 | — |
| `cfg_in_w` | 8 | — |
| `cfg_in_h` | 8 | — |
| `cfg_kernel_w` | 4 | — |
| `cfg_kernel_h` | 4 | — |
| `cfg_stride_w` | 4 | — |
| `cfg_stride_h` | 4 | — |
| `cfg_pad_w` | 4 | — |
| `cfg_pad_h` | 4 | — |
| `cfg_router_mode` | 2 | 0=MODE_8x8, 1=MODE_1x64 |
| `cfg_pool_bypass` | 1 | 1=旁路 Pool + Corner-Turn 齿轮箱模式 |
| `cfg_zp_y` | 8 | 输出 zero-point |
| `cfg_mem_stride_w` | 10 | IM SRAM 物理行跨度 (bytes) |
| `cfg_im_read_base` | 16 | IM SRAM 读起始偏移 |
| `cfg_im_write_base` | 16 | IM SRAM 写起始偏移 |
| `cfg_im_total_writes` | 16 | 本层 64-bit 写入总次数 (编译器填入) |

### 7.3 状态握手 (C→W)

| 信号 | 位宽 | 说明 |
|---|---|---|
| `core_busy` | 1 | 电平, Core 工作中 |
| `layer_done` | 1 | 脉冲, 当前层完成 |
| `core_error` | 2 | 0=OK, 1=溢出, 2=FSM 异常 |

### 7.4 Weight / Requant SRAM 读 (C→W)

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `wt_rd_req` | 1 | C→W | — |
| `wt_rd_addr` | 8 | C→W | 层内逻辑行号 (0~255) |
| `wt_rd_data` | 512 | W→C | 64×int8 weights |
| `wt_rd_ack` | 1 | W→C | — |
| `rq_rd_req` | 1 | C→W | — |
| `rq_rd_addr` | 4 | C→W | 条目地址 (0~15) |
| `rq_rd_data` | 288 | W→C | 4ch × 72b |
| `rq_rd_ack` | 1 | W→C | — |

### 7.5 IM SRAM 外部访问 (W→C, 仅 core_busy=0 时)

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `im_ext_cs` | 1 | W→C | 片选, MUX 切换 |
| `im_ext_wr` | 1 | W→C | 1=写, 0=读 |
| `im_ext_addr` | 9 | W→C | 64-bit word 地址 (512 深) |
| `im_ext_wdata` | 64 | W→C | 写数据 |
| `im_ext_rdata` | 64 | C→W | 读数据 |
| `im_ext_ready` | 1 | C→W | 应答 |

> **1 拍读写契约 (#7)**: IM SRAM 是片上单口存储, 读写均严格 1 拍。写当拍提交 **无 ack**;
> 读当拍发起、**下一拍** `im_ext_rdata` 有效 + `im_ext_ready=1`。无反压信号——下游
> (Core Preprocess / Corner-Turn) 不得插拍。
>
> **四访问时序互斥 (#7)**: 2 写源 (FETCH `im_ext_wr` / Core Corner-Turn `im_wr_en`) +
> 2 读源 (WRITEBACK `im_ext` / Core Preprocess 双口)。关键不变式 **`im_ext_cs==1` ⇒
> `core_busy==0`**: Wrapper 仅在 Core IDLE 时接管 IM (FETCH 在 D_PREFILL、WRITEBACK 在
> D_TAIL), 与 Core 自身读写永不同拍。`im_sram.v` 写口优先级 MUX (FETCH > Corner-Turn)
> 仅为防御性兜底, 正确性由 FSM 时序保证。

---

## 8. 软件编译器契约

1. **描述符生成**: 按 §3.2 位域映射生成 64B 描述符, Little-Endian 字节序, Reserved 填 0
2. **链表组织**: 描述符通过 `sequencer_next_desc` 链接。末层 `sequencer_is_last=1`
3. **fc1 切分**: 编译器检测单层权重 > 16KB 时自动切分为多个连续虚拟层
4. **cfg_im_total_writes**: 编译器计算并填入——硬件禁止自行计算
   - Conv: `out_ch × out_h × (cfg_mem_stride_w / 8)`
   - FC: `(out_ch + 7) / 8`
5. **im_read_base / im_write_base**: 编译器静态分配 IM SRAM 的 Ping-Pong 区域, 总量 ≤ 4KB
6. **mem_stride_w**: 编译器 padding 到 8 的倍数 (Corner-Turn 空间转置正确性依赖此契约)
6b. **dma_feature_bytes** (desc[0]): 输入特征图 padded 字节总数, **必须 8 字节对齐**。硬件用 `>>2` / `>>3` 得 FETCH 的 AHB 读字数 / IM 写次数。硬件禁止用 in_ch×in_w×in_h 自算 (会欠取 padding)。
7. **权重打包 (512-bit 行 = 64 × int8):**
   - **MODE_8x8 (Conv):** 每行 = 8 个 kx 组 × 每组 8 个输出通道 weight。
     格式: `[kx=0:ocg..ocg+7][kx=1:ocg..ocg+7]...[kx=7:ocg..ocg+7]`
     行索引 = `(ocg/8) × in_ch × kernel_h + ic × kernel_h + ky`
     kernel_w < 8 时剩余 kx 组填 0。每 (ic,ky) 读 1 行, kx 拍进时 MUX 选组, 不重读 SRAM。
     每 MAC 周期 8 像素 × 8 weight = 64 MAC。
   - **MODE_1x64 (FC):** 每行 = 64 个连续输出通道 weight (同一 ic, kx=ky=0)。
     行索引 = `(ocg/64) × in_ch + ic`
     每 ic 读 1 行。每 MAC 周期 1 像素 × 64 weight = 64 MAC。
7b. **Gearbox 字序与移位寄存器实现 (#6, #12, 关键)**:
   - **移位寄存器替代动态索引**: Gearbox 使用右移寄存器 `gb_shift <= {wdata, gb_shift[N-32:0]}`,
     新 word 从高位推入, 旧数据逐次向低位移动。综合工具将此映射为简单移位链, 消除
     `gb_shift[gb_cnt*32 +: 32]` 动态切片产生的巨型地址译码器和 MUX 逻辑。
   - **自清洁**: 移位寄存器仅保留 N−1 个 word (Weight: 480-bit / 15 words;
     Requant: 256-bit / 8 words)。连续行/条目间无需 gearbox_rst——每 N 次写入将
     旧数据完全推出, 移位寄存器自然净化。
   - **字序**: 第 k 个 AHB word 最终落入目标行 slot k (`bits[32k +: 32]`),
     **第一个 word 落最低位 [31:0]**。DDR 最低地址的 word 须含 oc0/ch0 数据
     (Core 解包 oc0@`wt_rd_data[7:0]`, ch0@`rq_rd_data[71:0]`)。
   - Weight: 16 words/行 (word15→[511:480])。Requant: 9 words/条目 (word8→[287:256])。
   - **编译器按低位优先序列化上面 §7 的行布局即可, 无需额外字节翻转。**
8. **FC 权重重排**: 编译器将 FC 权重按 CHW 格式做列置换, 匹配 Conv 输出的 IM SRAM 物理布局
9. **DDR Blob 组织**: `[desc_0 64B][wt_0][rq_0][desc_1 64B][wt_1][rq_1]...` 连续存放
10. **dma_word_count 上限 (#11)**: `dma_scheduler` 用 16-bit `dma_word_count` 发起 AHB 传输, 单次
    上限 **65535 words (256KB)**。`sr_weight_bytes[17:2]` 切片隐含 "weight_bytes < 256KB"
    契约。当前 Weight Bank 仅 16KB (256×512b), 远低于上限, 无溢出风险。编译器的
    weight/requant/feature bytes 字段为 32-bit, 未来若 Bank 容量扩大, 需同步放宽
    `dma_word_count` 位宽或改用饱和截断。

---

## 9. 关键数值验证

### 9.1 IM SRAM 4KB Ping-Pong

| 层对 | 输入 (padded) | 输出 (pooled, padded) | 合计 |
|---|---|---|---|
| input → pool1 | 1×28×32=896B | 8×14×16=1792B | 2688B ✓ |
| pool1 → pool2 | 8×14×16=1792B | 16×5×8=640B | 2432B ✓ |
| pool2 → fc1 | 16×5×8=640B | 64×1×8=512B | 1152B ✓ |
| fc1 → fc2 | 64×1×8=512B | 10×1×8=80B | 592B ✓ |

### 9.2 Weight SRAM 单 Bank 16KB

| 层 | 原始权重 | 512-bit 打包后 | 单 Bank 16KB? |
|---|---|---|---|
| conv1 | 1×8×5×5=200B | ceil(200/64)×64=256B | ✓ |
| conv2 | 8×16×5×5=3200B | 3200B | ✓ |
| fc1 | 64×400=25600B | **编译器切为 16KB+9.6KB** | ✓ (切分后) |
| fc2 | 10×64=640B | 640B | ✓ |

### 9.3 Requant SRAM depth=16

| 层 | out_ch | entries = ceil(out_ch/4) | ≤16? |
|---|---|---|---|
| conv1 | 8 | 2 | ✓ |
| conv2 | 16 | 4 | ✓ |
| fc1_chunk_0 | 40 | 10 | ✓ |
| fc1_chunk_1 | 24 | 6 | ✓ |
| fc2 | 10 | 3 | ✓ |

---

## 10. 待定项

1. **INT4 SIMD 裂变**: 延后至 v1.1/v2.0
2. **低功耗门控时钟**: Core IDLE 时关闭 PE 阵列时钟树
3. **AHB Error 处理**: `HRESPM` 端口已引出并贯通 (top→bus_wrapper→ahb_master), 错误链
   `HRESPM → dma_error → error_code=1 → 中止当前传输回 D_IDLE → INT_STATUS` 已打通 (#5)。
   **仍待定**: 精确恢复策略 (重试/部分回滚/软复位重启), 当前仅做中止 + 上报, 不自动恢复。
4. **性能计数器**: 可选 debug/profiling 计数器 (cycle count, layer latency)

---

> *v1.0 基于 WRAPPER_CORE_INTERFACEv1.1.md 设计原则 + WRAPPER_CORE_INTERFACEv3.2.md 接口信号编写。为 Lenet5v3 加速器 RTL 实现的权威基准。*
