# tb_lenet5_top — LeNet-5 全芯片集成测试

## 测试架构

```
tb_lenet5_top
  └─ lenet5v3_top (DUT)
       ├─ bus_wrapper (APB + AHB + DMA + shadow_reg + weight/requant SRAM)
       └─ compute_core (FSM + IM_AGU + WT_AGU + PE_array + fifo_gearbox
                        + requant_unit + pingpong_pool + corner_turn + im_sram)
  + AHB memory slave (软件模型, 64KB)
  + APB master (配置接口)
  + Golden reference (软件模型)
```

## 网络结构 — 真实 LeNet-5 参数

| Layer | 类型 | in_ch | out_ch | 输入 | Kernel | Pad | Pool | 输出 |
|-------|------|-------|--------|------|--------|-----|------|------|
| conv1+pool1 | Conv+MaxPool | 1 | 8 | 28×28 | 5×5 | 2 | 2×2 | 14×14 |
| conv2+pool2 | Conv+MaxPool | 8 | 16 | 14×14 | 5×5 | 0 | 2×2 | 5×5 |
| FC_P1 | FC (keep_accum) | 10 | 64 | 5×5 | 5×5 | - | - | 1×1 |
| FC_P2 | FC | 6 | 64 | 5×5 | 5×5 | - | - | 1×1 |
| FC2 | FC (final) | 64 | 10 | 1×1 | 1×1 | - | - | 1×1 |

## 测试数据

- 输入: 28×28 全 1 灰度图
- 权重: 全 1
- Requant: identity (M=1, shift=0, b=0)

## Golden 计算

- **conv1**: pad=2, 边缘像素 kernel 覆盖 9~25 → 输出 9,12,15,...,25
- **pool1**: 2×2 max → 边缘 16, 中心 20
- **conv2**: pad=0, 输入全 20 (全 1 输入时 pool1 中心区) → 输出饱和 255
- **pool2**: 2×2 max → 全 255
- **FC**: 全 1 权重 × 全 255 输入 → 输出饱和 255

## 测试覆盖

### 描述符链
- 5 个描述符链表: DESC0 → DESC1 → FC_P1 → FC_P2 → DESC4
- 自动 DMA 取指 (INCR16 突发读)
- Shadow register 移位写入 + bit-slice 解包

### AHB Master
- 权重读取: 80 + 1280 + 4000 + 2400 + 256 = 8016 words
- Requant 读取: 9 + 36 + 9 + 36 + 36 = 126 (实际 250,含 gearbox 对齐)
- Feature 读取: 896 bytes → 112 IM writes (64-bit)

### APB Slave
- CTRL / DESC_HEAD_PTR / INT_STATUS / DEBUG_STATE 寄存器读写
- DMA FSM 状态轮询: IDLE → PREFILL → PREFETCH → TAIL → IDLE

### Dual-Bank SRAM
- Weight: Bank A/B 交替, gearbox 32b→512b 拼装
- Requant: Bank A/B 交替, gearbox 32b→288b 拼装
- Bank toggle: 层间自动翻转

### compute_core
| 模块 | 验证项 |
|------|--------|
| compute_fsm | 5 层连续调度, cfg_valid 时序 |
| im_agu | pad=2 时负坐标 pp_valid 掩码 |
| wt_agu | MODE_8x8 / MODE_1x64 双模式 |
| pe_array | 8×8 MAC 阵列, clear_accum |
| fifo_gearbox | PE→requant 流水 |
| requant_unit | M=1 全 4 通道等值输出 |
| pingpong_pool | 2×2 max pool, 行末 padding 处理 |
| corner_turn | CT_SPATIAL scatter / CT_GEARBOX sequential |
| im_sram | even/odd 双 Bank 1R1W |

### FC chunk 处理
- FC_P1: keep_accum=1, disable_flush=1 (跨 desc 保持累加器)
- FC_P2: keep_accum=0, disable_flush=0 (最终提交)
- FC2: out_ch=10, im_tw=2, 仅 2 次 CT 写

## 最终结果

```
==== TB_LENET5_TOP: ALL CHECKS PASSED ====
```

| 检查项 | 状态 |
|--------|------|
| L0 conv1+pool1: 224 CT writes, 与 golden 匹配 | ✓ |
| L1 conv2+pool2: 80 CT writes, 全 255 | ✓ |
| FC_P1: keep_accum, 无写回 | ✓ |
| FC_P2: 8 CT writes, 全 255 | ✓ |
| FC2: 2 CT writes, 全 255 | ✓ |
| WRITEBACK: 160 words, 0 X | ✓ |
| Requant: 4 通道全相等 (identity) | ✓ |
| Descriptor chain: 5 层链表 | ✓ |
| inference_done 正常拉起 | ✓ |
| 所有层 D_IDLE→PREFILL→PREFETCH→TAIL→IDLE | ✓ |

## 波形

`tb_lenet5_top.vcd` — 完整仿真波形 (VCD 格式, 可用 GTKWave 打开)

## 编译运行

```
run_top_tb.bat
```

需要 Vivado 2024.2, 自动调用 xvlog→xelab→xsim。
