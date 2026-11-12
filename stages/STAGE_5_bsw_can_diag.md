# Stage 5 — BSW, CAN & Diagnostics

| | |
| :--- | :--- |
| **Goal** | Build the layered firmware around the generated application: scheduler, MCAL wrappers, CAN communication driven by the DBC (with E2E protection), diagnostics and watchdog — and prove the CAN loop with the PC. |
| **Entry** | Stage 0 drivers; Stage 1 DBC skeleton and pin map; Stage 4 wrapper (`App_Step10ms`). |
| **Exit** | Integrated firmware runs the control loop on the real hardware; all CAN messages cyclic and E2E-protected; timeout, diagnostics and watchdog behave per requirements. |
| **Maps to** | Blueprint Phase 5. Requirements: CAN-001…008, DIA-001…005, SAF-001/002. |

---

## 1. Tasks

| # | Task | Output |
| :-: | :--- | :--- |
| 5.1 | Decide and document the clock source (D-005) after the CAN tolerance check (§3.1) | decision log |
| 5.2 | Scheduler (`Os_*`) with 1/10/20/50/100 ms tasks | `bsw/os.c` |
| 5.3 | MCAL wrappers finished from Stage 0 + interrupt-driven CAN RX ring buffer | `mcal/*.c` |
| 5.4 | Finalise `vehicle.dbc` (§4) and write the DBC → C generator (§5) | `05_can/` |
| 5.5 | `Com` module: pack/unpack, cyclic TX, RX monitoring | `bsw/com.c` |
| 5.6 | E2E: counter + CRC-8 (§3.2) with unit tests | `bsw/e2e.c` |
| 5.7 | Dem-lite fault memory and UDS-like read service | `bsw/dem.c`, `diag.c` |
| 5.8 | Watchdog module + reset-cause handling (§3.4) | `bsw/wdg.c` |
| 5.9 | RTE glue: `Rte_ReadInputs` / `Rte_WriteOutputs` | `bsw/rte.c` |
| 5.10 | Integration test on bench (CAN logger + joystick + motor) | logs |

## 2. Architecture and task assignment

| Task | Period | Content |
| :--- | :---: | :--- |
| `Task_1ms` | 1 ms | `Wdg_CheckpointAlive(WDG_OS)`, encoder delta accumulate, tick counters |
| `Task_10ms` | 10 ms | `Rte_ReadInputs` → `App_Step10ms` → `Rte_WriteOutputs`; `Mcal_Motor_*` update; speed estimation |
| `Task_20ms` | 20 ms | `Com_MainFunctionRx`, `Com_MainFunctionTx` (Pedal_Cmd rx, Drive_Status tx) |
| `Task_50ms` | 50 ms | `Mcal_Tof_ReadMm`, AEB_Status tx |
| `Task_100ms` | 100 ms | `Dem_Update`, Diag_Status tx, telemetry |
| ISR | event | CAN RX FIFO0 pending → ring buffer; TIM6 1 kHz tick |

```c
/* os.h */
typedef void (*Os_TaskFn)(void);
typedef struct { Os_TaskFn fn; uint16_t period_ms; uint16_t offset_ms; } Os_TaskCfg_t;

void     Os_Init(const Os_TaskCfg_t *tasks, uint8_t n);
void     Os_Tick1ms_ISR(void);       /* called from TIM6 update ISR: increments tick, sets flags */
void     Os_MainLoop(void);          /* never returns: runs due tasks, then Wdg_MainFunction()  */
uint32_t Os_GetTickMs(void);
uint32_t Os_GetTaskMaxUs(uint8_t task_idx);   /* measured WCET (DWT) */
uint32_t Os_GetOverrunCount(uint8_t task_idx);
```

Task offsets spread the load (e.g. 20 ms task at offset 3 ms, 50 ms at 7 ms) so the worst case does not occur on one tick.

## 3. Mathematics and algorithms

### 3.1 CAN bit timing and clock-tolerance check

$$f_{bit} = \frac{f_{PCLK1}}{BRP\,(1 + TS1 + TS2)}, \qquad t_q = \frac{BRP}{f_{PCLK1}}, \qquad \text{sample point} = \frac{1 + TS1}{1 + TS1 + TS2}$$

(Lab 17: $BRP=2$, $TS1=13$, $TS2=2$, $N_{bt}=16$ tq → 500 kbit/s, 87.5 %.)

**Oscillator tolerance** (from the CAN specification, resynchronisation conditions). With $N_{bt}$ time quanta per bit and phase buffer segments PS1, PS2, the maximum relative oscillator deviation $\Delta f$ between nodes must satisfy both

$$\Delta f \le \frac{\min(PS1,\ PS2)}{2\,(13\,N_{bt} - PS2)} \qquad\text{and}\qquad \Delta f \le \frac{SJW}{20\,N_{bt}}$$

| Config | $N_{bt}$ | SJW | $\min(PS)$ | Limit 1 | Limit 2 | Allowed $\Delta f$ |
| :--- | :-: | :-: | :-: | :-: | :-: | :-: |
| Lab 17 (BS1=13, BS2=2, SJW=1) | 16 | 1 | 2 | 0.485 % | 0.3125 % | **0.31 %** |
| Same, SJW=4 | 16 | 4 | 2 | 0.485 % | 1.25 % | **0.49 %** |
| BS1=11, BS2=4, SJW=4 (sample point 75 %) | 16 | 4 | 4 | 0.98 % | 1.25 % | **0.98 %** |

**What this means.** The CANable uses a crystal (error ≪ 0.1 %). The STM32 **HSI is only specified to about ±1 %** over temperature/voltage **[VERIFY in DS10693: HSI accuracy]**, which can exceed the 0.31 % margin of the Lab 17 settings. On the bench at room temperature it may well work, but it is a latent reliability problem. **Decision D-005:** use the **8 MHz ST-LINK MCO clock as HSE bypass** (default on NUCLEO-F446RE **[VERIFY UM1724: SB solder bridges]**) with the PLL for system clock, and recompute BRP/TS1/TS2 for the new PCLK1; fall back to larger SJW and TS2 if you stay on HSI. Re-run Lab 17 (loopback + bus) after the change.

### 3.2 E2E protection: counter and CRC-8 (SAE J1850)

CRC-8, polynomial $x^8 + x^4 + x^3 + x^2 + 1$ (0x1D), initial value 0xFF, no reflection, final XOR 0xFF.

```c
/* e2e.h */
typedef enum { E2E_OK, E2E_ERR_CRC, E2E_ERR_REPEAT, E2E_ERR_SEQUENCE, E2E_ERR_NO_NEW } E2E_Status_t;
typedef struct { uint8_t last_cnt; uint8_t initialised; uint16_t lost; uint16_t err_total; } E2E_State_t;

uint8_t  E2E_Crc8(const uint8_t *data, uint8_t len, uint8_t data_id);
void     E2E_Protect(uint8_t *msg, uint8_t dlc, uint8_t crc_byte_idx,
                     uint8_t cnt_byte_idx, uint8_t data_id, uint8_t *counter);
E2E_Status_t E2E_Check(const uint8_t *msg, uint8_t dlc, uint8_t crc_byte_idx,
                       uint8_t cnt_byte_idx, uint8_t data_id, E2E_State_t *st);

/* algorithm */
uint8_t E2E_Crc8(const uint8_t *d, uint8_t len, uint8_t data_id) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {            /* all bytes except the CRC byte */
    crc ^= d[i];
    for (uint8_t b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x1D) : (uint8_t)(crc << 1);
  }
  crc ^= data_id;                                /* data-ID byte, same bit loop */
  for (uint8_t b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x1D) : (uint8_t)(crc << 1);
  return (uint8_t)(crc ^ 0xFF);
}
```

**Self-check (unit test):** the *plain* CRC-8/SAE-J1850 of the ASCII string `"123456789"` (init 0xFF, xorout 0xFF) is **0x4B** (standard catalogue check value). Test the core routine without the `data_id` step against that value, then test with the ID step against the same algorithm in Python (`crccheck` or your own) to confirm both sides agree.

**Counter rules** (4-bit, wraps 0…15): $\Delta = (cnt - last)\ \&\ 0x0F$. $\Delta=1$ → OK. $\Delta=0$ → `E2E_ERR_REPEAT` (stale frame). $\Delta=2$ → accept but `lost++` (one lost frame). $\Delta \ge 3$ → `E2E_ERR_SEQUENCE`; resynchronise after one more valid frame. First frame after init is accepted. A CRC error is checked first.

### 3.3 DBC signal packing (Intel, `@1+`)

For a signal with start bit $s$, length $L$, factor $f$, offset $o$:

$$\text{raw} = \text{round}\!\left(\frac{\text{phys} - o}{f}\right)\ \ (\text{saturated to } [0,\ 2^L-1]\ \text{or the signed range}),\qquad \text{phys} = f\cdot\text{raw} + o$$

Bit $i$ of raw is stored at byte $\lfloor (s+i)/8 \rfloor$, bit $(s+i)\bmod 8$.

```c
void     Com_PackBits  (uint8_t *buf, uint8_t start_bit, uint8_t len, uint32_t raw);
uint32_t Com_UnpackBits(const uint8_t *buf, uint8_t start_bit, uint8_t len);
int32_t  Com_SignExtend(uint32_t raw, uint8_t len);       /* for signed signals */
```

Unit tests: round-trip for every signal over its range, boundary values, and a comparison against `cantools` encoding of the same values.

### 3.4 Independent watchdog timeout

$$t_{IWDG} = \frac{(RLR + 1)\cdot PR_{div}}{f_{LSI}}$$

with $f_{LSI} \approx 32$ kHz typical but only specified within a wide range (≈ 17–47 kHz **[VERIFY DS10693]**). Example: $PR_{div}=32$, $RLR=99$: nominal 100 ms; real range ≈ 68–188 ms. **Design with the worst-case minimum:** every legitimate path must kick the watchdog in less than ≈ 40 ms (half of the minimum 68 ms), and the timeout requirement (SAF-001) is stated as a range.

```c
/* wdg.h */
typedef enum { WDG_CP_OS = 0, WDG_CP_10MS, WDG_CP_20MS, WDG_CP_COUNT } Wdg_Checkpoint_t;

void    Wdg_Init(uint16_t reload, uint8_t prescaler);          /* starts IWDG */
void    Wdg_CheckpointAlive(Wdg_Checkpoint_t cp);              /* tasks report in */
void    Wdg_MainFunction(void);                                /* kick only if ALL checkpoints were reported since the last kick, then clear flags */
uint8_t Wdg_GetResetCause(void);                               /* bit0 IWDG, bit1 SW, bit2 POR; read RCC->CSR then clear flags */
```

In debug sessions call `__HAL_DBGMCU_FREEZE_IWDG()` so a breakpoint does not reset the MCU.

### 3.5 Rx timeout monitor

Per received message keep `age_ms` (incremented every 20 ms task, zeroed by a *valid* frame). Provide to the application: `can_age_ms`. The application (Stage 3) declares timeout when `age > 3 × cycle` (60 ms for `0x100`). Detection bound: $60 + 20 = 80$ ms (Stage 1 §3.4).

## 4. CAN database (`05_can/vehicle.dbc`)

Bit rate 500 kbit/s. All frames standard ID, DLC as listed, E2E data-ID = low byte of the CAN ID.

| ID | Name | DLC | Cycle | Layout (start\|len, factor, offset, unit) |
| :--- | :--- | :-: | :-: | :--- |
| `0x100` | `Pedal_Cmd` | 4 | 20 ms | `pedal_pct` 0\|16, 0.01, 0, % · `msg_counter` 16\|4 · `msg_crc` 24\|8 |
| `0x123` | `ECU_Heartbeat` | 5 | 500 ms | `hb_counter` 0\|32 (bring-up compatible with Lab 17) · `loopback_flag` 32\|8 |
| `0x200` | `Drive_Status` | 7 | 20 ms | `speed_rpm` 0\|16, 0.1, 0, rpm · `duty_pct` 16\|8, 0.5, 0, % · `mode` 24\|4 · `msg_counter` 28\|4 · `speed_req_rpm` 32\|16, 0.1, 0, rpm · `msg_crc` 48\|8 |
| `0x201` | `Diag_Status` | 5 | 100 ms | `fault_bitmap` 0\|16 · `rx_err_count` 16\|8 · `msg_counter` 24\|4 · `msg_crc` 32\|8 |
| `0x210` | `AEB_Status` | 5 | 50 ms | `distance_mm` 0\|16, 1, 0, mm · `aeb_state` 16\|4 · `tof_valid` 20\|1 · `msg_counter` 24\|4 · `msg_crc` 32\|8 |
| `0x7E0` / `0x7E8` | `Diag_Req` / `Diag_Resp` | 8 | event | single-frame UDS-like (§6) |
| `0x7F0` | `Test_Override` | 7 | event | `override_mask` 0\|8 · `ovr_pedal1_adc` 8\|16 · `ovr_pedal2_adc` 24\|16 · `ovr_distance_mm` 40\|16 — **test builds only** (`-DTEST_HOOKS`) |

DBC syntax example (verify by loading with `cantools`):

```
BO_ 256 Pedal_Cmd: 4 PC
 SG_ pedal_pct   : 0|16@1+ (0.01,0) [0|100] "%" ECU
 SG_ msg_counter : 16|4@1+ (1,0) [0|15] ""  ECU
 SG_ msg_crc     : 24|8@1+ (1,0) [0|255] "" ECU

BA_ "GenMsgCycleTime" BO_ 256 20;
```

## 5. Code generator (DBC → C configuration)

```
python 05_can/scripts/gen_com_cfg.py --dbc 05_can/vehicle.dbc --node ECU --out-dir 04_firmware/config/
```

```python
def generate(dbc_path: str, node: str, out_dir: str) -> None:
    """Parse the DBC with cantools and write com_cfg.h / com_cfg.c."""
```

Generated types (in `com_cfg.h`):

```c
typedef enum { COM_MSG_PEDAL_CMD, COM_MSG_DRIVE_STATUS, COM_MSG_DIAG_STATUS,
               COM_MSG_AEB_STATUS, COM_MSG_HEARTBEAT, COM_MSG_COUNT } Com_MsgId_t;

typedef struct {
  uint16_t can_id; uint8_t dlc; uint16_t cycle_ms; uint8_t is_tx;
  uint8_t  e2e_data_id; int8_t crc_byte; int8_t cnt_byte;   /* -1 = unprotected */
} Com_MsgCfg_t;

typedef struct {
  Com_MsgId_t msg; uint8_t start_bit; uint8_t len; uint8_t is_signed;
  float factor; float offset; float min; float max;
} Com_SignalCfg_t;

extern const Com_MsgCfg_t    Com_MsgCfg[COM_MSG_COUNT];
extern const Com_SignalCfg_t Com_SignalCfg[];
```

The generator also emits a **consistency self-test**: `python gen_com_cfg.py --selftest` encodes random values with `cantools` and with the generated C pack function (via a host-compiled test) and compares bytes.

## 6. Com, Dem and Diag interfaces

```c
/* com.h */
typedef enum { COM_OK = 0, COM_E_PARAM, COM_E_NO_DATA, COM_E_CRC, COM_E_COUNTER, COM_E_TIMEOUT } Com_Status_t;

void         Com_Init(void);
void         Com_MainFunctionRx(void);                         /* 20 ms: drain ring buffer, unpack, E2E check */
void         Com_MainFunctionTx(void);                         /* 20 ms: pack due messages, E2E protect, send */
Com_Status_t Com_GetSignal(Com_SignalId_t id, float *phys);
Com_Status_t Com_SetSignal(Com_SignalId_t id, float phys);
uint16_t     Com_GetRxAgeMs(Com_MsgId_t m);
uint16_t     Com_GetE2eErrorCount(void);

/* CAN RX path (mcal_can.h) */
int  Mcal_Can_Init(void);                                      /* 500 kbit/s, accept-all, RX interrupt on */
int  Mcal_Can_Send(uint32_t id, const uint8_t *d, uint8_t dlc);
int  Mcal_Can_RxPop(CAN_Frame_t *f);                           /* from ISR-filled ring buffer, 1 = got one */
uint32_t Mcal_Can_GetErrorFlags(void);
/* ISR: HAL_CAN_RxFifo0MsgPendingCallback() -> push into ring buffer (size >= 16) */

/* dem.h */
void     Dem_Init(void);
void     Dem_Update(uint16_t fault_bitmap_now, uint32_t t_ms);   /* edge detect, counters */
uint16_t Dem_GetActive(void);
uint16_t Dem_GetStored(void);                                   /* faults seen since last clear */
uint16_t Dem_GetOccurrenceCount(uint8_t bit);
void     Dem_ClearStored(void);

/* diag.h */
void Diag_ProcessRequest(const CAN_Frame_t *rx);               /* ID 0x7E0, ISO-TP single frame */
```

**Minimal UDS-like read** (service 0x22, single frame, ISO-TP PCI byte = payload length): request `[03 22 F1 00]` → response `[05 62 F1 00 hi lo]` for DID `0xF100` (fault bitmap); DID `0xF101` returns software version. Unknown DID → negative response `[03 7F 22 31]` (request out of range). Optional (DIA-005, *Could*).

BSW-originated fault bits `F_CAN_E2E`, `F_I2C_BUS`, `F_WDG_RESET` are OR-ed into the bitmap sent by `Diag_Status` in addition to the application's `fault_bitmap`.

## 7. Deliverables

- `04_firmware/{mcal,bsw,config}/`, the CubeIDE project with all tasks integrated, `app_generated/` from Stage 4.
- `05_can/vehicle.dbc`, `scripts/gen_com_cfg.py`, unit tests (host-compiled with Unity) for pack/unpack, CRC, E2E counter, ring buffer.
- `05_can/logs/` representative captures (`.asc`/`.csv`).
- Decision log entry D-005 (clock), D-006 (E2E scheme).

## 8. Acceptance criteria

| ID | Criterion | Evidence |
| :--- | :--- | :--- |
| A5-1 | CRC unit test: standard check value **0x4B** reproduced; Python and C implementations agree on 1000 random frames | test log |
| A5-2 | Pack/unpack round-trip passes for every signal; `cantools` and C encoders produce identical bytes | test log |
| A5-3 | All cyclic messages observed at nominal period ±3 ms over a 10-minute capture (PC timestamps); 0 lost counters, 0 E2E errors | capture + script |
| A5-4 | Rx timeout: stopping `Pedal_Cmd` raises `F_CAN_TIMEOUT` ≤ **100 ms** after the last frame (80 ms bound + one status period), recovers after ≥ 5 valid frames | log |
| A5-5 | 10 frames with wrong CRC are all discarded; `rx_err_count` increases by exactly 10; `speed_req_rpm` unchanged | log |
| A5-6 | 10 frames with a skipped counter → `E2E_ERR_SEQUENCE` handled per §3.2 | log |
| A5-7 | Watchdog: halting the 10 ms task (test hook) causes a reset within the IWDG worst-case timeout; at restart outputs are in the safe state *before* the first `Drive_Status`, `F_WDG_RESET` set in the first `Diag_Status` | log + scope |
| A5-8 | Reset cause is reported correctly for IWDG, software reset and power cycle | log |
| A5-9 | Measured task WCETs within Stage 1 budget; overrun counters all zero in a 10-minute run | `Os_GetTaskMaxUs()` dump |
| A5-10 | No single point where `HAL_Delay` or blocking printf runs inside tasks | code review checklist |
| A5-11 | Control loop works end-to-end: joystick → motor speed → `Drive_Status` on PC, with step metrics within Stage 3 criteria ×1.5 (full verification in Stage 6) | plot |

## 9. Expected issues

| ID | Issue | Why | Mitigation |
| :--- | :--- | :--- | :--- |
| ISS-5-01 | Sporadic CAN errors or lost frames, worse at temperature | HSI tolerance vs bit timing (§3.1) | Decision D-005 (HSE bypass), larger SJW/TS2, re-verify |
| ISS-5-02 | RX FIFO overflow / lost command frames | Hardware FIFO holds 3 frames; polling every 20 ms | Interrupt-driven ring buffer (≥ 16 frames) |
| ISS-5-03 | TX "timeout" returns `CAN_HAL_TIMEOUT` when PC is off | No ACK → mailboxes stay full | Treat as non-fatal; count; do not block control tasks; the error state is *error-passive*, **not** bus-off, with a lone node |
| ISS-5-04 | Bus-off never reproduces on the bench | A single node with ACK errors stays error-passive | Verify CAN-007 by analysis/code inspection (set/clear flags) or deliberate bit-error injection; mark verification method accordingly |
| ISS-5-05 | Signal values wrong by factor or sign | DBC signedness, factor, byte order mismatch | Always cross-check with `cantools` decode of captured frames (A5-2) |
| ISS-5-06 | Control loop jitter | I2C ToF read (ms) or UART print in a task; HAL tick vs TIM6 priority | I2C only in the 50 ms task with timeout; no prints; set NVIC priorities explicitly (TIM6 above SysTick) |
| ISS-5-07 | I2C bus lock-up after a reset during transfer | Slave holds SDA low | Bus recovery: toggle SCL up to 9 times, send STOP, re-init; raise `F_I2C_BUS` |
| ISS-5-08 | Watchdog resets while stepping in the debugger | IWDG keeps running when core halts | `__HAL_DBGMCU_FREEZE_IWDG()` in debug builds |
| ISS-5-09 | IWDG timeout differs from expectation | LSI spread (§3.4) | Specify and test with min/max range; do not tune to a single measurement |
| ISS-5-10 | Message corruption from Lab-style blocking `HAL_CAN_AddTxMessage` loops | Mailboxes busy | Check free level first (already in `CAN_HAL_Send`); never wait inside the 10 ms task |
| ISS-5-11 | `TEST_HOOKS` left enabled in a release build | Compile flag forgotten | Release build fails if `TEST_HOOKS` is defined (`#error`); CI/grep check in Stage 7 |
| ISS-5-12 | Heartbeat ID `0x123` from Lab 17 conflicts with new IDs | Reuse of an ID | Keep it reserved in the DBC (done), never reuse for another message |
| ISS-5-13 | Clock change (HSE/PLL) breaks UART baud, I2C timing, encoder filter, PWM frequency | Everything derives from the new clocks | Recompute all timer/UART/I2C prescalers (use the formulas in Stage 0 §3); re-run Stage 0 smoke tests T0-2…T0-9 |
