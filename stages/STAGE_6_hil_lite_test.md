# Stage 6 — HIL-lite & ECU Test

| | |
| :--- | :--- |
| **Goal** | Verify the complete ECU (firmware + real motor/sensors) against the requirements with an **automated PC-driven test suite**, including fault injection, and compare real behaviour with the MIL prediction. |
| **Entry** | Stage 5 `PASS` (firmware integrated, DBC final). |
| **Exit** | Every `Must` requirement has an executed test with a recorded result; defects are fixed or documented; MIL-vs-HIL comparison done. |
| **Maps to** | Blueprint Phase 6. Requirements: all with `HIL` in the *Verify* column. |

> [!NOTE]
> **What "HIL-lite" means here.** The PC does *not* simulate the plant in real time. The ECU runs with real actuators and sensors; the PC acts as the vehicle network monitor (CANable V2), UART telemetry logger, fault/stimulus interface, and offline MIL/SIL replay verifier. State this explicitly in technical reports.

---

## 1. Tasks

| # | Task | Output |
| :-: | :--- | :--- |
| 6.1 | On-target 100 Hz closed-loop runner (`AutoStep` & `Live` modes + `DWT->CYCCNT` WCET) | `04_firmware/closed_loop_runner.c`, `04_firmware/closed_loop_runner.h`, `04_firmware/prof.h` |
| 6.2 | Host UART telemetry logger (`#BEGIN ... #END` CSV capture) | `04_firmware/experiments/capture_log.py` |
| 6.3 | Host CANable V2 live monitor, E2E CRC-8 verifier & UDS `0x22` client | `05_can/can_monitor.py` |
| 6.4 | MATLAB Hardware-vs-MIL & on-target SIL replay verification + 3-panel plots | `03_simulink/stage4/s6_compare_hw_vs_mil.m`, `06_test/reports/stage6/*.png` |
| 6.5 | Automated requirement traceability & file-existence gate | `06_test/hil/trace_check.py`, `01_requirements/traceability.md` |

## 2. Test bench interfaces (C, Python & MATLAB)

Dependencies:
- **Python 3**: `pyserial` (for `capture_log.py`), `python-can` + `gs_usb` / `pyserial` (for `can_monitor.py` live mode; `--demo` mode runs with standard library + `clang`/`gcc`).
- **MATLAB / Simulink**: `Simulink`, `ctrl_sil_mex` (built in Stage 4 by `s4_03_run_sil_b2b.m`).

### 2.1 On-Target STM32 Closed-Loop Runner (`04_firmware/closed_loop_runner.h`)

```c
#define CL_PERIOD_MS          10u
#define CL_ENCODER_CPR        1426.0f   /* Measured in 00_smoketest/encoder (50:1 N20) */
#define CL_DUTY_MAX_PCT       50.0f     /* P-005 hard safety cap (Vrated / Vbat = 3V / 6V) */

void ClosedLoop_Init(void);
void ClosedLoop_RunLive(uint32_t seconds, const char *file_name);
void ClosedLoop_RunAutoStep(const char *file_name);
```

- `ClosedLoop_RunAutoStep("hil_step_test.csv")`: Executes a deterministic 6-second virtual-pedal step test on the physical STM32 + N20 motor without manual joystick input.
- `ClosedLoop_RunLive(15, "hil_closed_loop.csv")`: Executes 100 Hz closed-loop control from the physical `KY-023` joystick (`ADC1 PA0/PA1`) and supports real-time single-axis fault injection (`F_PEDAL_PLAUS`).
- Both functions stream `#BEGIN <file_name> ... #END` CSV rows (`t_ms,adcX,adcY,speed_req_rpm,speed_meas_rpm,duty_cmd_pct,fault_bitmap,mode,wcet_us`) over `USART2` (`115200 8N1`) and drive `CanTelemetry_Step10ms()`.

### 2.2 Host UART Telemetry Logger (`04_firmware/experiments/capture_log.py`)

```python
DEFAULT_BAUD = 115200

def auto_detect_port() -> str: ...   # scans /dev/cu.usbmodem*
def default_outdir() -> str: ...     # resolves to 03_simulink/data/real
def main() -> int: ...               # parses #BEGIN <name> ... #END and writes CSV
```

CLI usage:
```bash
python3 04_firmware/experiments/capture_log.py
# or explicitly:
python3 04_firmware/experiments/capture_log.py /dev/cu.usbmodemXXXX 03_simulink/data/real
```

### 2.3 Host CANable V2 Monitor & UDS `0x22` Client (`05_can/can_monitor.py`)

```python
BITRATE = 500_000

def decode_fault_names(bitmap: int) -> str: ...
def decode_frame_py(msg_def: dbc_mod.MessageDef, payload: bytes): ...  # -> (vals, crc_ok)
def open_canable_bus(kind: str, port: str | None): ...               # 'gs_usb' or 'slcan'
def run_live(kind: str, port: str | None) -> int: ...
def run_demo() -> int: ...
```

CLI usage:
```bash
python3 05_can/can_monitor.py gs_usb
python3 05_can/can_monitor.py slcan /dev/cu.usbmodemXXXX
python3 05_can/can_monitor.py --demo
```

- Decodes `0x200 Drive_Status`, `0x201 Diag_Status`, `0x210 AEB_Status`, `0x123 ECU_Heartbeat`, and `0x7E8 Diag_Resp` using `05_can/vehicle.dbc` and verifies SAE J1850 CRC-8 + 4-bit `msg_counter`.
- On fault detection (`fault_bitmap != 0`), automatically transmits UDS Service `0x22 ReadDataByIdentifier` on `0x7E0` (`0xF100` active faults, `0xF102` max WCET µs) and logs the trace to `05_can/logs/can_fault_capture.csv`.

### 2.4 MATLAB Hardware-vs-MIL & SIL Replay (`03_simulink/stage4/s6_compare_hw_vs_mil.m`)

```matlab
function res = s6_compare_hw_vs_mil(csvPath)
% Replays recorded adcX, adcY through speed_ctrl.slx (MIL) and
% adcX, adcY, speed_meas_rpm through ctrl_sil_mex (SIL).
% Returns struct: csvPath, figPath, speedFitPct, reqErrMax, dutyOnTargetErr, maxWcetUs, allPass
```

### 2.5 Automated Traceability Gate (`06_test/hil/trace_check.py`)

```python
def load_requirements(md_path: Path) -> dict: ...
def build_and_check(reqs: dict, repo_root: Path): ...  # -> (matrix, violations)
def write_matrix(matrix: list, out_path: Path) -> None: ...
def main() -> int: ...
```

## 3. Mathematics and metrics

Let $y(t)$ be the measured speed, $y_0$ its value before the step, $y_f$ the final value (mean of the last 0.5 s of the hold), target $y^*$.

| Metric | Definition |
| :--- | :--- |
| Rise time $t_r$ | $t_{90} - t_{10}$, first crossings of $y_0 + 0.1(y_f - y_0)$ and $y_0 + 0.9(y_f - y_0)$ |
| Overshoot | $\dfrac{\max y - y_f}{y_f - y_0}\times 100\,\%$ (0 if negative) |
| Settling time $t_s$ | last time $|y(t) - y_f| > \max(0.02\,|y_f - y_0|,\ \Delta\omega_q)$, minus the step time |
| Steady-state error | $y^* - y_f$ |
| NRMSE fit | $100\left(1 - \dfrac{\|y_{meas} - y_{model}\|_2}{\|y_{meas} - \bar y_{meas}\|_2}\right)\ \%$ |

**Repeatability** over $n$ runs: coefficient of variation $CV = \sigma/\mu$ for $t_r$ and $y_f$. **Timing jitter** of a cyclic message: $\max_k \Delta t_k - \min_k \Delta t_k$, where $\Delta t_k = t_{k+1} - t_k$. Host timestamps have USB/OS jitter of ≈ 1–5 ms (ISS-6-02); prefer the ECU-referenced time from the message counter for control timing, and the PC time only for coarse bounds.

**Fault reaction time** (injection to observable reaction):

$$T_{react} = T_{inject\to detect} + T_{debounce} + T_{status\ cycle} + T_{bus}$$

For the pedal plausibility test: $\approx 10 + 50 + 20 + 1 = 81$ ms → pass bound **100 ms** from override command to `F_PEDAL_PLAUS` in `Diag_Status` (add one 100 ms Diag period when reading that message: bound 200 ms), and speed ≤ $\omega_{limp}$ within the FTTI.

## 4. Test cases

All tests start from a defined state: ECU reset, motor free, `ctrl_source = BENCH`, `Pedal_Cmd = 0` for 2 s.

| TC | Verifies | Stimulus | Pass criterion |
| :--- | :--- | :--- | :--- |
| TC-HIL-001 | SPD-003, CAN-001..003 | `Pedal_Cmd` step 0→60 % (hold 3 s), 10 repeats | $t_r$ within ±25 % of MIL prediction, overshoot ≤ 8 %, $|$sse$| \le 3$ rpm or 3 %, $CV(t_r) \le 10\,\%$ |
| TC-HIL-002 | PRC-002 (system level) | Same step; overlay MIL result | NRMSE fit MIL vs HIL ≥ **85 %** |
| TC-HIL-003 | SPD-005 | request 100 % for 3 s | motor PWM never exceeds $D_{max}$ (check `duty_pct` ≤ 50 % +0.5, and scope for one run) |
| TC-HIL-004 | THR-003/004, SG-01 | (a) override `pedal2_adc` = 5 % while `pedal1` = 60 %; (b) operator-assisted: hand-push joystick along X only, Y held in dead zone (D-004) | `F_PEDAL_PLAUS` ≤ 200 ms; request does not rise (lower-select); `mode` = DEGRADED; recovers ≤ 2 s after release |
| TC-HIL-004b | THR-003, THR-007 | real-hand smooth diagonal pushes (slow/normal/fast, ≥ 20 reps) | no `F_PEDAL_PLAUS`; false-alarm count = 0; $p_1+p_2≈100$ (REVERSE) or $p_2≈0.5p_1$ (HALF) within $\Delta_{pl}$ |
| TC-HIL-005 | THR-005 | override ADC = 0 and 4095 | `F_PEDAL_RANGE` set, speed limited |
| TC-HIL-006 | CAN-006 | stop `Pedal_Cmd` | `F_CAN_TIMEOUT` ≤ 100 ms after last frame; `speed_req_rpm` → 0; recovery after ≥ 5 valid frames |
| TC-HIL-007 | CAN-004/005 | 10 frames with wrong CRC | `rx_err_count` +10 exactly; speed request unchanged |
| TC-HIL-008 | CAN-005 | counter jump of 5, 10 frames | `E2E_ERR_SEQUENCE` counted; no command applied until resync |
| TC-HIL-009 | SPD-006 | hold shaft with request 60 % | `F_SPEED_TRACK` within configured time; duty reduced; no overheating (≤ 3 s per trial; wait to cool) |
| TC-HIL-010 | AEB-002..004 | override distance: 600→100 mm at 200 mm/s, back up | WARNING at $d_w \pm 20$ mm, BRAKING at $d_b \pm 20$ mm (±1 sample), servo reaches $\theta_{brake} \pm 5^\circ$, returns after release |
| TC-HIL-011 | AEB-005, SG-03 | `tof_valid = 0` while approaching | BRAKING never entered from invalid data; `F_TOF_INVALID` set |
| TC-HIL-012 | AEB-001 (real sensor) | card moved by hand 500→100 mm | `distance_mm` within ±15 mm of ruler at 3 points; state sequence correct |
| TC-HIL-013 | SAF-001/002 | test hook halts the 10 ms task | reset within the IWDG worst-case timeout; first `Diag_Status` has `F_WDG_RESET`; speed → 0 before any new command |
| TC-HIL-014 | SAF-002 | power-cycle ECU while motor running | motor stops at reset; PWM = 0 until Init completes (scope) |
| TC-HIL-015 | CAN-002 | 30-minute soak at nominal traffic | 0 resets, 0 lost counters, 0 E2E errors, cycle jitter within ±3 ms (host time), no overrun counters |
| TC-HIL-016 | DIA-001, DIA-003 | inject each fault in turn | bitmap bit appears and clears per Stage 3 mode table |
| TC-HIL-017 | DIA-005 (optional) | `[03 22 F1 00]` | response `[05 62 F1 00 hi lo]`, bitmap matches; unknown DID → NRC `0x31` |
| TC-HIL-018 | CAN-007 | — | **Verified by inspection/analysis** (a single bench node cannot reach bus-off; see ISS-5-04) |
| TC-HIL-019 | SAF-005 / RK-02 | start motor + servo simultaneously, 20 times | zero unintended resets (reset cause log) |

**Operator-assisted tests** (physical joystick diagonal push, single-axis X-only fault injection, stall) are logged via `04_firmware/experiments/capture_log.py` and `05_can/can_monitor.py`.

## 5. Procedure (each full run)

1. Flash `04_firmware/` (`ClosedLoop_RunAutoStep("hil_step_test.csv")` and `ClosedLoop_RunLive(15, "hil_closed_loop.csv")`) to the NUCLEO-F446RE.
2. Start UART capture (`python3 04_firmware/experiments/capture_log.py`) and press `B1` to save `hil_step_test.csv` and `hil_closed_loop.csv` into `03_simulink/data/real/`.
3. Optionally monitor CAN bus traffic and UDS `0x22` fault/WCET responses via `python3 05_can/can_monitor.py slcan /dev/cu.usbmodemXXXX` (or `gs_usb`).
4. Run `s6_compare_hw_vs_mil('03_simulink/data/real/hil_step_test.csv')` and `s6_compare_hw_vs_mil('03_simulink/data/real/hil_closed_loop.csv')` in MATLAB to verify `A6-1..A6-5` and generate overlay plots in `06_test/reports/stage6/`.
5. Run `python3 06_test/hil/trace_check.py` to regenerate `01_requirements/traceability.md` and verify zero violations.

## 6. Deliverables

- `03_simulink/data/real/hil_step_test.csv` & `hil_closed_loop.csv` (real STM32 closed-loop hardware logs).
- `06_test/reports/stage6/hw_vs_mil_hil_step_test.png` & `hw_vs_mil_hil_closed_loop.png` (3-panel MIL vs. hardware overlay plots).
- `08_results/logs/closed_loop_live_log.txt` & `can_fault_capture_log.txt` (raw terminal transcripts).
- Updated `01_requirements/traceability.md`, `01_requirements/change_log.md`, and `07_docs/lessons_learned.md`.

## 7. Acceptance criteria (verified in `s6_compare_hw_vs_mil.m`)

| ID | Criterion | Target | Evidence |
| :--- | :--- | :---: | :--- |
| A6-1 | On-target STM32 `speed_req_rpm` vs. SIL max abs diff | `<= 0.05 rpm` | `s6_compare_hw_vs_mil.m` (`0.000 rpm`) |
| A6-2 | On-target STM32 `duty_cmd_pct` vs. SIL max abs diff | `<= 0.05 %` | `s6_compare_hw_vs_mil.m` (`0.000–0.010 %`) |
| A6-3 | On-target `mode` & `fault_bitmap` mismatches vs. SIL | `0` | `s6_compare_hw_vs_mil.m` (`0` mismatches) |
| A6-4 | Closed-loop real motor speed vs. Simulink MIL NRMSE fit | `>= 80.0 %` | `hw_vs_mil_hil_*.png` (`96.1 %` / `96.6 %`) |
| A6-5 | Worst-case execution time (`DWT->CYCCNT`) of `App_Step10ms` | `<= 2000 us` (`< 20 %`) | `prof.h` / CSV `wcet_us` (`75–77 us`) |

## 8. Expected issues

| ID | Issue | Why | Mitigation |
| :--- | :--- | :--- | :--- |
| ISS-6-01 | Tests are flaky (pass/fail changes) | Battery sag, motor warm-up, timing windows too tight | Let the motor cool, fresh cells, randomised test order, retry-once policy only with documentation |
| ISS-6-02 | Timestamps jitter by several ms | USB, OS scheduling, serial buffering (slcan) | Use ECU counter-based time; use candleLight (gs_usb) with hardware timestamps if available **[VERIFY]**; widen windows consciously |
| ISS-6-03 | slcan adapter drops frames at higher traffic | Serial bandwidth/latency | Prefer gs_usb; reduce test traffic; check RX counters |
| ISS-6-04 | `python-can` threading issues on macOS, device busy | Another process holds the CANable | One bench process only; `bus.shutdown()` in fixture teardown |
| ISS-6-05 | Hook-based faults differ from real electrical faults | Override replaces the sensor value after the driver | Add 3 real tests (unplug, short, stall) as operator-assisted; record both |
| ISS-6-06 | Stall test heats or damages the motor | Prolonged stall current | Limit to ≤ 3 s per trial, cool-down ≥ 30 s, current check (M5) |
| ISS-6-07 | Test power cycles reset the CANable too | Shared USB hub | Use separate USB ports; or use the ST-LINK reset only |
| ISS-6-08 | AEB real-sensor test noisy | ToF noise, target surface | Standard target, hysteresis $\ge 3\sigma_d$ (Stage 2), tolerance ±20 mm |
| ISS-6-09 | MIL-vs-HIL mismatch large | Plant not valid at the new operating point, or battery lower now | Re-identify (Stage 2), recompute $\lambda$ (Stage 3), log as calibration change |
| ISS-6-10 | Requirement text prevents testability discovered late | Vague/numeric [TBD] left | Fix the requirement and log in `change_log.md`; do not skip |
| ISS-6-11 | Safety: moving servo/motor hurts fingers | Mechanical hazards on the bench | Secure the rig; keep hands away; emergency power switch on the battery lead |
