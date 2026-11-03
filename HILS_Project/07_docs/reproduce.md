# Clean-Checkout Reproduction Guide (`reproduce.md`)

> [!IMPORTANT]
> This guide allows any reviewer to reproduce the entire MATLAB/Simulink identification, MIL test suite, Embedded Coder C generation, SIL back-to-back verification, CAN DBC codegen self-test, Stage 6 Hardware-vs-MIL verification, and Stage 7 traceability gate in **under 15 minutes** (or **under 45 minutes** including physical STM32 flashing and live hardware capture).

---

## 1. Prerequisites

| Tool | Version Verified | Purpose |
| :--- | :--- | :--- |
| **macOS / Linux / Windows** | macOS (Apple Silicon / Intel) | Host OS |
| **MATLAB & Simulink** | R2026a (`Simulink`, `MATLAB Coder`, `Simulink Coder`, `Embedded Coder`) | Stage 2, 3, 4, 6 |
| **C Compiler (for SIL MEX)** | Apple Command Line Tools (`clang`) or GCC/MSVC | Compiles `ctrl_sil_mex` |
| **Python 3** | Python 3.9+ (`pyserial` optional for live UART capture) | Stage 5 DBC codegen & Stage 7 traceability gate |
| **STM32CubeIDE** *(for live HW)* | 1.15+ (with NUCLEO-F446RE connected via USB) | Compiles & flashes `04_firmware/` |

---

## 2. Fast Offline Reproduction (No Hardware Needed — ~5 Minutes)

All 25 real hardware CSV logs (`03_simulink/data/real/*.csv`) captured from the physical NUCLEO-F446RE + N20 motor + KY-023 joystick are committed in the repository. You can reproduce every model, C file, plot, and test report from scratch with three commands:

### Step 2.1 — Rebuild Stage 2, Stage 3 & Stage 4 from Real Hardware Data (~2 min)
Run from the repository root in terminal (or inside the MATLAB Command Window):

```bash
/Applications/MATLAB_R2026a.app/bin/matlab -batch "addpath(genpath('03_simulink')); rebuild_from_real_data();"
```

**Expected Output:**
- `[1/4] Stage 2 Identification on Real Hardware Data`: `K = 3.074 rpm/%`, `tau = 0.0273 s`, `dz = 1.93 %`, `val fit = 95.2 %`, `Simulink plant vs measured: fit = 89.7 % -> PASS`
- `[2/4] Stage 3 Rebuild Controller & MIL Test Suite`: `19 / 19 PASS — Stage 3 MIL: ALL PASS`
- `[3/4] Stage 4 Regenerate Embedded C Code & Run SIL Back-to-Back`: `5 / 5 PASS — Stage 4 Codegen & SIL: ALL PASS`

### Step 2.2 — Run Stage 6 Hardware-vs-MIL & SIL Verification (~1 min)

```bash
/Applications/MATLAB_R2026a.app/bin/matlab -batch "addpath(genpath('03_simulink')); s6_compare_hw_vs_mil('03_simulink/data/real/hil_step_test.csv'); s6_compare_hw_vs_mil('03_simulink/data/real/hil_closed_loop.csv');"
```

**Expected Output:**
- `hil_step_test.csv`: `A6-1..A6-5 PASS` (`0.000 rpm` diff, `96.1 %` fit, `75 us` WCET)
- `hil_closed_loop.csv`: `A6-1..A6-5 PASS` (`0.000 rpm` diff, `96.6 %` fit, `77 us` WCET)
- Saves plots to `06_test/reports/stage6/hw_vs_mil_hil_step_test.png` and `hw_vs_mil_hil_closed_loop.png`.

### Step 2.3 — Run Stage 5 CAN DBC Codegen + E2E C/Python Self-Test (~10 sec)

```bash
python3 05_can/scripts/dbc_codegen.py --selftest
```

**Expected Output:**
- Generates `04_firmware/bsw/com_can_signals.h` and `04_firmware/bsw/com_can_signals.c` from `05_can/vehicle.dbc`.
- Compiles the C pack/unpack + CRC-8 SAE J1850 + E2E alive-counter test binary and verifies `CRC8("123456789") == 0x4B` and `1000 / 1000` random frames (`ALL PASS`).

### Step 2.4 — Run Stage 7 Automated Traceability Gate (~5 sec)

```bash
python3 06_test/hil/trace_check.py
```

**Expected Output:**
- `Must coverage : 27/27 (100.0%)`, `Total coverage: 47/47 (100.0%)`, `PASS: 0 violations found`.

---

## 3. Full Physical Hardware Reproduction (~30 Minutes)

If you have the physical NUCLEO-F446RE + N20 motor + TB6612FNG + KY-023 bench wired per [`04_firmware/HARDWARE_CALIBRATION_AND_HIL_GUIDE.md`](../04_firmware/HARDWARE_CALIBRATION_AND_HIL_GUIDE.md):

1. **Copy Firmware Sources to STM32CubeIDE Project:**
   ```bash
   cp 04_firmware/app_generated/ctrl_codegen.c 04_firmware/app_generated/ctrl_codegen.h 04_firmware/app_generated/rtwtypes.h /Users/church/Documents/github/STM32_Projects/MBD_ECU_HAL/Core/Src/
   cp 04_firmware/app_wrapper.c 04_firmware/app_wrapper.h 04_firmware/prof.h 04_firmware/closed_loop_runner.c 04_firmware/closed_loop_runner.h /Users/church/Documents/github/STM32_Projects/MBD_ECU_HAL/Core/Src/
   cp 04_firmware/bsw/com_can_signals.c 04_firmware/bsw/com_can_signals.h 04_firmware/bsw/can_telemetry.c 04_firmware/bsw/can_telemetry.h /Users/church/Documents/github/STM32_Projects/MBD_ECU_HAL/Core/Src/
   ```
2. **Build & Flash in STM32CubeIDE:** Call `ClosedLoop_RunAutoStep();` and `ClosedLoop_RunLive();` inside `main.c` (`USER CODE BEGIN 2`).
3. **Capture Live Telemetry:**
   ```bash
   python3 04_firmware/experiments/capture_log.py
   ```
   Press the Blue Button (`B1`) on the Nucleo board to run the 6-second autonomous step test (`hil_step_test.csv`), then press `B1` again to run the 15-second live joystick + fault-injection test (`hil_closed_loop.csv`).
4. **Re-run Step 2.2** to generate fresh Hardware-vs-MIL comparison plots.
