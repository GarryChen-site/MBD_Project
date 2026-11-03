# Hardware Wiring, Calibration & Closed-Loop HIL Execution Guide

This guide documents the complete physical bench setup, STM32CubeMX peripheral configuration, Stage 2 open-loop calibration procedure, and Stage 6 closed-loop Hardware-in-the-Loop (HIL-lite) execution on the **STM32 NUCLEO-F446RE**.

---

## 1. Combined Physical Wiring Table

All pin assignments match the standalone bring-up projects (`smoketest/{joystick,encoder,motor}`) and [`04_firmware/closed_loop_runner.c`](closed_loop_runner.c):

| Subsystem | Module Pin / Wire | Nucleo Pin | Arduino Silk | CubeMX Peripheral / Mode |
| :--- | :--- | :---: | :--- | :--- |
| **Joystick (`KY-023`)** | `VRx` (Yellow) | **PA0** | `A0` (`CN8-1`) | `ADC1_IN0` (12-bit, `84 Cycles` sampling time) |
| **Joystick (`KY-023`)** | `VRy` (Green) | **PA1** | `A1` (`CN8-2`) | `ADC1_IN1` (12-bit, `84 Cycles` sampling time) |
| **Joystick (`KY-023`)** | `SW` (Blue) | **PA10** | `D2` (`CN9-3`) | `GPIO_Input` (`Pull-up`) |
| **Joystick (`KY-023`)** | `+5V` (VCC) / `GND` | **3.3V** / **GND** | `CN6-4` / `CN6-6` | Powered from **3.3V** (never 5V, so ADC stays within `0..3.3V`) |
| **N20 Hall Encoder** | `Green` (Phase A) | **PA6** | `D12` (`CN5-5`) | `TIM3_CH1` (`TIM_ENCODERMODE_TI12`, `Pull-up`, `Filter = 10`) |
| **N20 Hall Encoder** | `Yellow` (Phase B)| **PA7** | `D11` (`CN5-6`) | `TIM3_CH2` (`TIM_ENCODERMODE_TI12`, `Pull-up`, `Filter = 10`) |
| **N20 Hall Encoder** | `Red` (VCC) / `Black` | **3.3V** / **GND** | `CN6-4` / `CN6-6` | Hall sensor logic supply (`3.3V`, $N_{\text{rev}} = 1426\text{ counts/rev}$) |
| **TB6612FNG Driver** | `PWMA` | **PB6** | `D10` (`CN5-3`) | `TIM4_CH1` (20 kHz PWM: `PSC = 0, ARR = 799` at 16 MHz) |
| **TB6612FNG Driver** | `AIN1` | **PC7** | `D9` (`CN9-9`) | `GPIO_Output` (`HIGH` = forward) |
| **TB6612FNG Driver** | `AIN2` | **PA9** | `D8` (`CN9-8`) | `GPIO_Output` (`LOW` = forward) |
| **TB6612FNG Driver** | `STBY` | **PA8** | `D7` (`CN9-1`) | `GPIO_Output` (`HIGH` = enabled, `LOW` = standby) |
| **TB6612FNG Driver** | `AO1` / `AO2` | Motor `M+` / `M-` | — | Connected to N20 DC motor terminals |
| **TB6612FNG Driver** | `VM` / `VCC` / `GND` | **6V Bat** / **3.3V** / **GND** | — | **4×AA 6V battery** to `VM`; Nucleo `3.3V` to `VCC`; star `GND` (`SAF-005`) |
| **CAN Transceiver** | `SN65HVD230` `D` / `R` | **PB9** / **PB8** | `D14` / `D15` | `CAN1_TX` / `CAN1_RX` (`AF9`, 500 kbit/s: `BRP=2, TS1=13, TS2=2`) |
| **Nucleo Onboard** | `B1` (Blue Button)| **PC13** | — | `GPIO_Input` (triggers test sessions) |
| **Nucleo Onboard** | `LD2` (Green LED) | **PA5** | `D13` (`CN5-6`) | `GPIO_Output` (status indicator) |
| **Nucleo ST-Link** | `VCP TX / RX` | **PA2 / PA3**| — | `USART2` (`115200` baud, `8N1`) |

> [!IMPORTANT]
> **Power Isolation (`SAF-005`):** Always power `TB6612FNG VM` from the external **4×AA (6 V)** battery pack and `TB6612FNG VCC` + sensors from the Nucleo **3.3 V** rail, sharing a common ground wire. The firmware hard-clamps PWM duty at **`50.0 %`** (`CL_DUTY_MAX_PCT = 50.0f`, `CCR <= 400`) so the effective motor voltage never exceeds its `3.0 V` rating.

---

## 2. Stage 2 Open-Loop Hardware Calibration (`experiments/`)

To re-capture the 19 motor identification CSVs (`static_D05.csv`..`static_D50.csv` and `step_15_r1.csv`..`step_35_r3.csv`) and 4 joystick calibration CSVs (`joy_rest.csv`, `joy_diag_full.csv`, `joy_diag_valid.csv`, `joy_xonly.csv`):

1. **Include Experiment Loggers in STM32CubeIDE:**
   Copy [`experiments/step_logger/step_logger.c`](experiments/step_logger/step_logger.c), [`step_logger.h`](experiments/step_logger/step_logger.h), [`experiments/pedal_logger/pedal_logger.c`](experiments/pedal_logger/pedal_logger.c), and [`pedal_logger.h`](experiments/pedal_logger/pedal_logger.h) into your CubeIDE project and call `StepLogger_RunAll()` or `PedalLogger_RunSessions()` inside `main.c`.
2. **Start Host UART Capture:**
   ```bash
   python3 04_firmware/experiments/capture_log.py
   ```
   `capture_log.py` automatically detects the ST-LINK Virtual COM Port (`/dev/cu.usbmodem*`), listens at `115200` baud, and writes every `#BEGIN <filename.csv> ... #END` block directly to [`03_simulink/data/real/`](../03_simulink/data/real/).
3. **Pre-Flight Motor Check (`DEF-05`):**
   When `B1` is pressed to start `StepLogger_RunAll()`, the firmware spins the motor at `35 %` duty for `0.5 s` while blinking `LD2` so you can immediately verify that the 6 V battery pack is switched ON before the 3-minute automated sweep begins.

---

## 3. Stage 6 Closed-Loop HIL Execution (`closed_loop_runner.c`)

To run the Simulink-generated controller ([`app_generated/ctrl_codegen.c`](app_generated/ctrl_codegen.c)) on the physical STM32F446RE at `100 Hz` (`10 ms` period):

1. **Copy Firmware Files to STM32CubeIDE Project:**
   - [`app_generated/ctrl_codegen.c`](app_generated/ctrl_codegen.c), [`ctrl_codegen.h`](app_generated/ctrl_codegen.h), [`ctrl_codegen_private.h`](app_generated/ctrl_codegen_private.h), [`ctrl_codegen_types.h`](app_generated/ctrl_codegen_types.h), [`rtwtypes.h`](app_generated/rtwtypes.h)
   - [`app_wrapper.c`](app_wrapper.c), [`app_wrapper.h`](app_wrapper.h), [`prof.h`](prof.h)
   - [`closed_loop_runner.c`](closed_loop_runner.c), [`closed_loop_runner.h`](closed_loop_runner.h)
   - [`bsw/com_can_signals.c`](bsw/com_can_signals.c), [`com_can_signals.h`](bsw/com_can_signals.h), [`can_telemetry.c`](bsw/can_telemetry.c), [`can_telemetry.h`](bsw/can_telemetry.h)
2. **Execute the Two Stage 6 Verification Runs:**
   - **Run 1 — Autonomous 6-Second Step Test (`ClosedLoop_RunAutoStep("hil_step_test.csv")`):** Injects a deterministic virtual 60 % diagonal pedal step (`adcX = 3209, adcY = 3222` from `t = 0.5 s` to `3.5 s`) on the real STM32 + real N20 motor, logging `hil_step_test.csv`.
   - **Run 2 — Live 15-Second Joystick & Fault-Injection Test (`ClosedLoop_RunLive(15, "hil_closed_loop.csv")`):** Reads live physical joystick deflections on `PA0`/`PA1`:
     - Push diagonally forward (`0–9 s`) to verify normal closed-loop speed tracking (`mode = 0`).
     - Push **X-axis only** (`9.9–11.1 s`) to inject a real-time dual-channel plausibility fault (`F_PEDAL_PLAUS`, `fault_bitmap = 1`), verifying transition to `DEGRADED/LIMP` (`mode = 1`, speed request clamped to `30.0 rpm` within `50 ms`).
     - Return to valid diagonal/rest (`11.1–15 s`) to verify automatic recovery to `NORMAL` (`mode = 0`) after `100` fault-free ticks (`1.0 s`).
3. **Verify Hardware vs. Simulink MIL & SIL on Host:**
   ```bash
   /Applications/MATLAB_R2026a.app/bin/matlab -batch "addpath(genpath('03_simulink')); s6_compare_hw_vs_mil('03_simulink/data/real/hil_step_test.csv'); s6_compare_hw_vs_mil('03_simulink/data/real/hil_closed_loop.csv');"
   ```
