# tb_compute_core — Testbench 说明

## 概述

`tb_compute_core.sv` 是 `compute_core` 模块的完整流水线测试平台，覆盖三种数据路径模式的 bit-exact 黄金对比验证。

### 被测模块

```
compute_core
├── compute_fsm        — 5 状态循环引擎 (IDLE→CHECK→COMPUTE→DONE→IDLE)
├── im_agu             — IM SRAM 读地址生成 + pp_valid + 半窗有效标志
├── wt_agu             — Weight SRAM 读地址生成
├── pe_array           — 8×8 MAC 阵列 (Output Stationary)
├── fifo_gearbox       — 2048b→128b 串行化 FIFO (16 拍排空)
├── requant_unit       — 3 级流水 MUL→ROUND+SHR→CLAMP
├── pingpong_pool      — 双缓冲 2×2 MaxPool + Bypass
├── corner_turn        — CT_GEARBOX / CT_SPATIAL 写地址生成
└── im_sram            — 512×64b 奇偶双 Bank BRAM
```

### 行为模型

| 模型 | 说明 |
|------|------|
| **weight_sram** | 256×512b 同步读行为模型 |
| **requant_sram** | 16×288b 同步读行为模型 |
| **compute_golden** | 软件参考卷积 (pad/stride 完整实现) |
| **compute_pool_golden** | 卷积 + 2×2 MaxPool + CT_SPATIAL 散射写参考 |
| **compute_pad_golden** | 卷积 (pad=1) + gearbox 打包，覆盖全部 ox 位置 (含超界) |

---

## 测试用例

### Test #1 — Bypass + Gearbox (pad=0)

| 参数 | 值 |
|------|-----|
| 输入尺寸 | 12×10×1 |
| 卷积核 | 5×5 |
| 输出通道 | 12 (2 ocg) |
| Pad | 0 |
| Stride | 1 |
| 输出尺寸 | 8×6 |
| CT 模式 | CT_GEARBOX |
| Pool | Bypass |
| 写入数 | 96 (12 flushes × 8 gearbox 写) |

**验证点：**
- MODE_8x8 基本卷积通路
- PE 阵列 8×8 MAC 累加正确性
- FIFO gearbox 32b→64b 顺序配对写入
- Requant 恒等变换 (M=1, shift=0, b=0, zp_y=0)
- IM SRAM 奇偶 Bank 交错读取

**Golden 策略：** 软件计算 5×5 卷积 → clamp(0,255) → 按 ocg→oy→ox(step8) 顺序打包为 gearbox 64b 写

---

### Test #2 — Pool + CT_SPATIAL (pad=0)

| 参数 | 值 |
|------|-----|
| 输入尺寸 | 12×10×1 |
| 卷积核 | 5×5 |
| 输出通道 | 12 (2 ocg) |
| Pad | 0 |
| 输出尺寸 | 8×6 → Pool 后 4×3 |
| CT 模式 | CT_SPATIAL |
| Pool | 2×2 MaxPool |
| 写入数 | 24 (2 ocg × 3 pool 行 × 4 散射写) |

**验证点：**
- PingPong Pool 双缓冲读写 (4 bank × 2 行)
- rd_col 驱动流水线 (偶数列缓存 / 奇数列 max4 输出)
- CT_SPATIAL 4×8 矩阵组装 + CHW planar 散射写地址
- ocg=1 时无效通道 (ch12-15) 正确归零

**Golden 策略：** 卷积 → 2×2 maxpool → 模拟 CT_SPATIAL 矩阵-散射写顺序

---

### Test #3 — Pad=(1,1) + Bypass + Gearbox

| 参数 | 值 |
|------|-----|
| 输入尺寸 | 12×10×1 |
| 卷积核 | 5×5 |
| 输出通道 | 8 (1 ocg) |
| Pad | (1, 1) |
| Stride | 1 |
| 输出尺寸 | 10×8 (2 ox step) |
| CT 模式 | CT_GEARBOX |
| Pool | Bypass |
| 写入数 | 128 (16 flushes × 8 gearbox 写) |

**验证点：**
- **IM AGU 负地址处理** — `cur_base_x = base_x + cnt_kx`，`x_chunk = cur_base_x >>> 3` (算术右移)
- **半窗有效标志** (`im_rd_addr_lo_vld`/`hi_vld`) — 当 `x_chunk=-1` 时 lo 半窗无效清零，hi 半窗 (`x_chunk+1=0`) 仍有效
- **`safe_im_rd_data`** — compute_core 内根据 vld 信号按半窗粒度清零 pad 区域数据
- **每拍 IM read** (`im_rd_req = mac_en`) — 每个 kx 步进都有独立 SRAM 读，窗口地址随 kx 动态偏移
- **pe_array mux_shift** — `cur_base_x` 含 `cnt_kx` 再取 `[2:0]*8`，移位始终在 0~56 范围内
- **跨 ox step 累加器清零** — `pp_valid=0` 的 PE 位置也能被 `clear_accum_sync` 正确清零
- **边界像素贡献** — 左边缘 pad (xi<0) 归零、右边缘超界 (xi≥in_w) 被 pp_valid 掩码、底部窗口读超出 IM 数据区零填充

**Golden 策略：** 软件计算 pad=1 卷积 (全 16 个 ox 位置) → clamp → gearbox 打包。注意硬件每 ox step 计算全部 8 个 PE 位置，golden 需覆盖 ox=0..15 而非仅 ox=0..9。

---

## 运行方式

```batch
cd hdls\rtl\core\tb_xsim
run_core_tb.bat
```

或手动：

```powershell
xvlog --nolog -sv ..\..\top\lenet5_pkg.sv `
    ..\compute_fsm.sv ..\im_agu.sv ..\wt_agu.sv ..\pe_array.sv `
    ..\fifo_gearbox.sv ..\requant_unit.sv ..\pingpong_pool.sv `
    ..\corner_turn.sv ..\im_sram.sv ..\compute_core.sv `
    tb_compute_core.sv

xelab --nolog -timescale 1ns/100ps -top tb_compute_core -snapshot tb_compute_core_snap

xsim tb_compute_core_snap -R
```

### 环境要求

- Vivado 2024.2 (`E:\Xilinx\Vivado\2024.2\settings64.bat`)
- 所有 RTL 源文件在 `..\` (即 `hdls\rtl\core\`)

---

## 运行结果

```
==== TB_COMPUTE_CORE (GOLDEN) STARTED ====

==== TEST #1: 5x5 conv, 12x10 in, pad=0, 1->12ch, golden check ====
    out=8x6, 2 ocg, 12 flushes, 96 gearbox writes
  All 96 writes match golden ✓

==== TEST #2: Pool(2x2) + CT_SPATIAL, 5x5 conv, 12x10 in, 1->12ch (2 ocg) ====
    pool out=4x3, 24 scatter writes
  All 24 scatter writes match golden ✓

==== TEST #3: pad=(1,1), bypass+gearbox, 5x5 conv, 12x10 in, 1->8ch ====
    out=10x8, 16 flushes, 128 gearbox writes, im_rd_base=0
  All 128 writes match golden ✓

═══════════════════════════════════════
  PASSED — all golden checks match
═══════════════════════════════════════
```

| 测试 | Pad | CT 模式 | Pool | 写入数 | 结果 |
|------|:---:|------|:---:|------:|:---:|
| #1 | 0 | GEARBOX | Bypass | 96 | ✓ |
| #2 | 0 | SPATIAL | 2×2 MaxPool | 24 | ✓ |
| #3 | (1,1) | GEARBOX | Bypass | 128 | ✓ |

---

## 关键设计决策

### 每拍 IM read (`im_rd_req = mac_en`)

每个 kx 步进触发独立的 SRAM 读取，地址由 `cur_base_x = ox*stride - pad + kx` 决定。窗口数据天然随 kx 变化，消除了旧架构中"同一窗口跨 kx 复用"带来的移位超界和流水线气泡问题。

### 半窗有效标志 (`im_rd_addr_lo_vld`/`hi_vld`)

替代了 bit9 符号位检测的粗粒度方案。核心逻辑：

```verilog
x_chunk = cur_base_x >>> 3;                    // 算术右移，负数保留
lo_vld  = (x_chunk >=  0) && (x_chunk*8 < in_w) && y_ok;
hi_vld  = (x_chunk >= -1) && ((x_chunk+1)*8 < in_w) && y_ok;
```

当 `x_chunk = -1` (pad 左侧越界)：lo 窗口在负数区 → 无效清零；hi 窗口 `x_chunk+1 = 0` → 恰好是第一个有效 8 像素块。无需 `im_read_base` 补偿。

### pe_array 累加器清零

```verilog
if (pp_valid_reg[p_ox]) begin
    acc <= clear_accum_sync ? prod : acc + prod;   // 有效像素: 清零=重新开始
end else if (clear_accum_sync && !cfg.keep_accum) begin
    acc <= 0;                                       // 无效像素也清零, 防跨 ox 残留
end
```

`else if` 分支确保跨 ox step 切换时所有 PE（包括新 ox 中 pp_valid 为 0 的位置）都归零。

### Golden 模型 ox 全位置覆盖

硬件 PE 阵列每 ox step 固定计算 8 个位置（即使超出 `out_w`），golden 模型必须覆盖 `T3_OX_ALL = T3_OX_STEPS * 8` 个位置，不能按 `out_w` 截断。

---

## 待扩展场景

- [ ] MODE_1x64 (FC 层) 测试
- [ ] `keep_accum` 跨 chunk 累加
- [ ] `disable_flush` 模式
- [ ] 多 ic 通道 (in_ch > 1)
- [ ] 非单位 stride (stride > 1)
- [ ] 不同 kernel 尺寸 (3×3, 1×1)
- [ ] 多 ocg 的 pad 场景
