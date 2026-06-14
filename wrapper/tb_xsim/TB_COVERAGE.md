# Wrapper Testbench Coverage Report

## 总览

| # | Testbench | DUT 模块 | 测试场景数 | 断言/检查点 |
|---|-----------|---------|-----------|------------|
| 1 | `tb_gearbox` | weight_sram, requant_sram | 2 | 6 |
| 2 | `tb_ahb_master` | ahb_master | 7 | ~12 |
| 3 | `tb_apb_slave` | apb_slave | 10 | ~12 |
| 4 | `tb_dma_scheduler` | dma_scheduler, shadow_register (+ AHB slave model, IM SRAM model) | 5 | ~6 |
| 5 | `tb_dma_integ` | apb_slave + compute_mgmt + dma_scheduler + ahb_master + shadow_register (+ AHB memory slave, IM SRAM model) | 8 | ~10 |

---

## 1. tb_gearbox — SRAM 字序与读写

**被测模块**: `weight_sram`, `requant_sram`

### 测试场景

| # | 场景 | 激励 | 断言 |
|---|------|------|------|
| T1 | Weight gearbox 字序 | 16 次 `wt_push(0x10+i)` → 读 `wt_rd_data` | `[31:0] == 0x10` (word0 低位), `[191:160] == 0x15` (word5 中间), `[511:480] == 0x1F` (word15 高位) |
| T2 | Requant gearbox 字序 | 9 次 `rq_push(0xA0+i)` → 读 `rq_rd_data` | `[31:0] == 0xA0` (word0 低位), `[63:32] == 0xA1` (word1), `[287:256] == 0xA8` (word8 高位) |

### 覆盖特性

- 移位寄存器 gearbox: 右移 `{wdata, shift[high:32]}`，16/9 words 后首 word 落低位
- 同步寄存器读: `wt_rd_req/rq_rd_req=1` → `dout_A/B <= bank[addr]` → 组合 MUX
- 双 Bank 存储: 只写 Bank A，读 Bank A (`bank_toggle=0`)

### 结果

```
[T1] weight_sram: 16 words → row 0, check word0 @ LOW
  T1 ok: word0@[31:0], word5@[191:160], word15@[511:480]
[T2] requant_sram: 9 words → entry 0, check word0 @ LOW
==== ALL TESTS PASSED ====
```

---

## 2. tb_ahb_master — AHB-Lite 总线协议

**被测模块**: `ahb_master`

### 测试场景

| # | 场景 | 激励 | 断言 |
|---|------|------|------|
| T1 | INCR16×16 读 | `addr=0x1000, 16 words, BM_ALLOW_INCR` | 16 字数据 = 递增地址; 16 拍全到 |
| T2 | HREADYM 反压 | (*deferred to tb_dma_integ*) | — |
| T3 | 纯 SINGLE×3 读 | `addr=0x3000, 3 words, BM_SINGLE_ONLY` | 每拍 NONSEQ SINGLE; 3 字正确 |
| T4 | INCR16 + SINGLE 尾 | `addr=0x4000, 17 words, BM_ALLOW_INCR` | 16×INCR16 + 1×SINGLE, 17 字正确 |
| T5 | SINGLE 写 | `addr=0x5000, 1 word, BM_SINGLE_ONLY, wdata=0xCAFE` | HWRITEM=1; `dma_done` 正常 |
| T6 | HRESP error 响应 | `HRESP=1`, 4-word read | `rsp.error` 捕获; FSM 回到 A_IDLE |
| T7 | 背靠背 SINGLE 写 ×3 | 3 次连续 `start_xfer` + `wait_done` | 每拍新 NONSEQ; 三次全到 |

### 协议监控器 (每拍自动检查)

| 断言 | 条件 |
|------|------|
| HSIZE 始终 32-bit | `HSIZEM === 3'b010` |
| HMASTLOCK 始终 0 | `HMASTLOCKM === 1'b0` |
| SEQ 必须带 INCR16 | `HTRANSM == SEQ → HBURST == INCR16` |
| 传输中途禁止 IDLE 插拍 | `state==A_RUN ∧ HTRANSM==IDLE ∧ pend ∧ dat>1` |

### AHB Slave 模型

`if (HREADYM && HTRANSM[1]) HRDATAM <= HADDRM` — 仅在 active transfer 时返回地址即数据，HREADYM=0 时冻结流水线。

### 结果

```
[T1] INCR16 x16 read @0x1000
  T1 ok: 16 words read, data==address verified
[T3] 3-word read (no INCR16 → SINGLEs) — ok
[T4] 17-word read (INCR16 + tail SINGLE) — ok
[T5] SINGLE write — ok
[T6] HRESP error response — ok (error captured, FSM returned to IDLE)
[T7] Back-to-back SINGLE writes (3 words) — ok
==== ALL TESTS PASSED ====
```

---

## 3. tb_apb_slave — APB CSR 寄存器

**被测模块**: `apb_slave`

### 测试场景

| # | 场景 | APB 操作 | 断言 |
|---|------|---------|------|
| T1 | 读 VERSION | `apb_read(0xFC)` | `== 0x0000_0100` |
| T2 | DESC_HEAD_PTR 读写 | 写 `0xA5A5_5A5A`/`0xDEAD_BEEF` | 读回一致 |
| T3 | CTRL.start 自清 + 同步器 | 写 `CTRL.start=1` | 读回 `start=0`; `csr_start_pulse` 经 2-FF 同步后产生 |
| T4 | CTRL.soft_reset 电平 | 写 `soft_reset=1` → 检查 → 写 `0` | 写 1 后 `soft_reset=1`; 写 0 后 `=0`; CTRL 读回 bit[1] 正确 |
| T5 | INT_STATUS W1C 清除 | `inference_done=1` → `INT_CLEAR[0]=1` | 置位后 `INT_STATUS[0]=1`; 清除后 `=0` |
| T6 | INT_STATUS error 锁定 | `error_flag=1, error_code=3` → `INT_CLEAR[1]=1` | 置位后 `INT_STATUS[1]=1, [7:4]=3`; 清除后 `[1]=0` |
| T7 | 硬件置位 vs 软件清除碰撞 | 同一 posedge: `INT_CLEAR[0]=1` + `inference_done=1` | HW 置位优先: `INT_STATUS[0]` 保持 1 |
| T8 | DEBUG_STATE bit 域 | `dma=PREFETCH/comp=WAIT_LAYER/ahb=RUN/toggle=1/busy=1` | `[1:0]=2, [3:2]=2, [5:4]=1, [11]=1, [12]=1` |
| T9 | Unmapped 地址读 | 读 `0x100` | 返回 0 |
| T10 | RO 寄存器写保护 | 写 VERSION=0xCAFE_F00D | 读回仍是 `0x0000_0100` |

### 覆盖特性

- APB 协议: setup(PSEL+PADDR) → access(PENABLE) 两拍时序
- `reg_ctrl_t`/`reg_int_status_t`/`reg_debug_state_t` 结构体读写
- CTRL.start 自清 + 2-FF 跨域同步器
- INT_STATUS W1C: HW 置位优先级 > SW 清除 (软件必须先释放 HW 源再 W1C)
- `default_nettype none` 下显式 `wire` 端口声明

### 结果

```
[T1]~[T10] ALL TESTS PASSED
```

---

## 4. tb_dma_scheduler — DMA FSM + 描述符链

**被测模块**: `dma_scheduler`, `shadow_register` (+ AHB slave behavioral model, IM SRAM model)

### 测试场景

| # | 阶段 | 硬件行为 | 断言 |
|---|------|---------|------|
| 1 | PREFILL | INCR16 读 desc[0]→shadow→解包; 读 wt[0]/rq[0]→gear_box; 读 feature(32B)→2×32b 打包→4×64b 写 IM addr 0-3 | FETCH: 4 IM words 与 DDR 源逐字比对; `feature_bytes>>3 == 4` |
| 2 | PREFETCH | `core_busy=1`→读 desc[1]→更新 `saved_is_last=1`; 读 wt[1]/rq[1]→~bank_toggle; `dma_ready=1` | `dma_ready` 拉高 |
| 3 | D_TAIL | `saved_is_last=1`→等 `compute_done`→WRITEBACK: 读 4×64b IM→拆 32b→AHB SINGLE 写 RES_BASE→`inference_done=1` | 4 个 AHB 写回结果与 IM 预载数据比对 |

### FSM 覆盖

| FSM 状态 | 触发条件 | 验证 |
|----------|---------|------|
| D_IDLE → D_PREFILL | `csr_start_pulse` | `dma_ready=1` |
| D_PREFILL → D_PREFETCH | `core_busy=1 ∧ !saved_is_last` | desc[1]+wt[1]+rq[1] 预取 |
| D_PREFETCH → D_TAIL | `core_busy=1 ∧ saved_is_last` | 跳过 `bank_toggle` 等待 |
| D_TAIL → D_IDLE | `compute_done` → WRITEBACK → `inference_done` | 结果写回完整 |

### 结果

```
[T] PREFILL dma_ready, FETCH expected 4 IM words — FETCH ok
[T] PREFETCH dma_ready (layer1 prefetched)
[T] entered D_TAIL
  WRITEBACK done (inference_done), WRITEBACK ok: 4×64b @ ×8 step
==== DMA SCHEDULER: ALL CHECKS PASSED ====
```

---

## 5. tb_dma_integ — 全 Wrapper 联合仿真

**被测模块**: `apb_slave` + `compute_mgmt` + `dma_scheduler` + `ahb_master` + `shadow_register` (+ AHB memory slave, IM SRAM model)

### 测试场景

| # | 阶段 | 操作 | 断言 |
|---|------|------|------|
| 1 | APB CSR 配置 | 写 `DESC_HEAD_PTR=D0` → 读回验证 | 读回 == D0 |
| 2 | APB 启动 | 写 `CTRL.start=1` → 等 5 拍 2-FF 同步 | `csr_start_pulse` 自动产生 |
| 3 | APB 读 VERSION | 读 `0xFC` | `== 0x0000_0100` |
| 4 | PREFILL | INCR16 读 desc[0]/wt[0]/rq[0]; 读 feature(128B)→16×64b IM 写 addr 0-15 | 16 IM words 逐字比对; `fcnt==feature_bytes>>3==16` |
| 5 | APB 读 DEBUG_STATE | 读 `0x10` | `dma_fsm == D_PREFILL(1)` |
| 6 | compute_mgmt 握手 | `core_busy=1` → `dma_ready` → `cfg_valid` 脉冲 → TB 模拟 `layer_done` → `compute_done` → `bank_toggle` 翻转 | `bank_toggle` 正确翻转 |
| 7 | PREFETCH | desc[1]+wt[1]+rq[1] 预取 → `dma_ready=1` → `saved_is_last=1` → D_TAIL | `dma_fsm` 进入 `D_TAIL` |
| 8 | D_TAIL WRITEBACK | 5×64b IM 读→拆 32b→AHB SINGLE 写 RES_BASE→`inference_done` | 5 个 AHB 结果逐字比对; `INT_STATUS[0]==1` |
| 9 | APB INT_CLEAR | 写 `INT_CLEAR[0]=1` → 读 `INT_STATUS` | `INT_STATUS[0]==0` (W1C 清除) |

### 全链路覆盖

```
APB 写 CTRL.start
  → 2-FF 同步器 → csr_start_pulse
  → dma_scheduler PREFILL → AHB INCR16 (desc/wt/rq/feature) → shadow_register 解包
  → FETCH 32b→64b 打包 → IM SRAM 写入
  → dma_ready → compute_mgmt cfg_valid 脉冲
  → (模拟 Core 计算) → layer_done → compute_done → bank_toggle 翻转
  → dma_scheduler PREFETCH → dma_ready → D_TAIL
  → WRITEBACK 64b→32b 拆解 → AHB SINGLE 写回
  → inference_done → APB INT_STATUS[0]=1
  → APB INT_CLEAR → INT_STATUS[0]=0
```

### compute_mgmt 握手协议验证

| 事件 | 行为 |
|------|------|
| `dma_ready=1` | C_IDLE → C_ISSUE_CFG, 发 `cfg_valid` 脉冲 |
| `layer_done` 脉冲 | 锁存为 `layer_done_latched` |
| `bank_toggle` 翻转 | 检测 `bank_toggle_changed` → 清 `compute_done` → 循环 C_ISSUE_CFG |
| `saved_is_last=1` + `layer_done` | 跳过 toggle 等待 → C_DONE → 保持 `compute_done=1` |
| `inference_done=1` | C_DONE → C_IDLE |

### 结果

```
[APB] Write DESC_HEAD_PTR=0x00000100
[APB] Write CTRL.start=1
[APB] VERSION=0x00000100
[T] PREFILL ready, FETCH=16 IM words
  FETCH ok: 16 words via real ahb_master INCR16
[APB] DEBUG_STATE=0x00000009 (dma_fsm=1)
[T] PREFETCH ready (layer1)
[T] entered D_TAIL
[T] WRITEBACK done (inference_done)
[APB] INT_STATUS=0x00000001 (inf_done=1)
  WRITEBACK ok: 5×64b @ ×8 step
[APB] INT_STATUS cleared ok
==== DMA+APB INTEG: ALL CHECKS PASSED ====
```

---

## 仿真环境

- **工具**: Vivado XSim 2024.2
- **编译**: `xvlog -sv lenet5_pkg.sv <DUT.sv> <tb.sv>`
- **阐述**: `xelab --timescale 1ns/100ps -top <tb> -snapshot <snap>`
- **运行**: `xsim <snap> -R`
- **波形**: `$dumpfile("tb_xxx.vcd"); $dumpvars(0, tb_xxx);` → GTKWave 兼容

## 未覆盖项

| 功能 | 状态 | 说明 |
|------|------|------|
| HREADYM 反压 (单模块级) | 未测 | 简单 slave 模型无法模拟 pipeline stall 恢复; `tb_dma_integ` 可加 HREADYM=0 场景 |
| 双 Bank 同时读写 | 部分 | gearbox TB 只写 Bank A; integ TB 验证了 `bank_toggle` 翻转但未同时读写 |
| HRESP error 传播到 INT_STATUS | 未测 | AHB TB 验证了 error 捕获但未测 `error_code→APB INT_STATUS` 链路 |
| desc_cfg_t 全字段解包 | 未测 | `put_desc` 只填了最小字段集; 未验证所有 reserved/reserved_w* 位域 |
| Core 侧 SRAM 读 (wt_rd_req/rq_rd_req) | 部分 | gearbox TB 用了 `rd_req=1`; integ TB 未使用 |
