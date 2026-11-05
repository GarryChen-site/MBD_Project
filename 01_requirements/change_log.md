# Change Log & Decision Log

Newest first. Log any change to scope, requirements, parameters or architecture. One line is enough.

## Decisions

| Date | ID | Decision | Reason | Revisit when |
| :--- | :--- | :--- | :--- | :--- |
| 2026-10-24 | D-009 | Set discrete lambda-tuning stability floor $\lambda \ge \max(3\,T_{\text{eff}}, 4\,T_s + \tau, 0.25\,\tau) = 0.060\text{ s}$ | Real 50:1 N20 motor has fast mechanical time constant ($\tau = 27.3\text{ ms}$); $6\,T_s = 60\text{ ms}$ ensures overdamped discrete response (0 % overshoot) at $T_s = 10\text{ ms}$ | If sample rate is increased to 1 kHz |
| 2026-10-24 | D-008 | Calibrated encoder resolution $N_{\text{rev}} = 1426\text{ counts/rev}$ (50.9:1 gearbox $\times 7\text{ PPR} \times 4$) and aligned motor PWM to `TIM4_CH1 (PB6)` with `AIN1=PC7, AIN2=PA9, STBY=PA8` | Matched physical hardware bring-up verified in `MBD_Project/smoketest/{encoder,motor}` | Fixed for hardware v1.0 |
| 2026-10-12 | D-004 | **Decided: diagonal emulation with the KY-023.** X→`ADC1_IN0 (PA0)`, Y→`ADC1_IN1 (PA1)`. `PedalCond` maps X to forward pedal 0–100 % ($s_1$) and Y to sensor 2 ($s_2$) in REVERSE or HALF mode. Normal driving = diagonal push; fault demo = X-only push | Exercises dual-channel plausibility (`D_pl = 15.72 %`, `sep = 5.82`) and limp-home recovery without extra hardware | Verified in Stage 2 & Stage 6 (`hil_closed_loop.csv`) |
| 2026-10-05 | D-007 | `float32` (`single`) only in generated code; call `App_Step10ms()` at 10 ms base rate | STM32F446RE Cortex-M4F FPU is single-precision hardware (`0` `double` verified in `ctrl_codegen.c`) | Verified in Stage 4 |
| 2026-10-05 | D-006 | E2E: 4-bit alive counter + CRC-8 SAE J1850 (`0x1D`, init `0xFF`, xorout `0xFF`), data-ID = CAN ID low byte | Standard automotive E2E profile check (`"123456789"` $\to$ `0x4B`), deterministic pack/unpack | Verified in Stage 5 |
| 2026-10-05 | D-005 | CAN clock: 16 MHz PCLK1 ($BRP=2, TS1=13, TS2=2 \to 500\text{ kbit/s}$, 87.5 % sample point); recommend 8 MHz ST-LINK MCO HSE-bypass for thermal margin | HSI is $\pm 1\text{ \%}$ over temperature vs 0.31 % SJW=1 tolerance; HSE-bypass or $SJW=4, TS2=4$ removes drift risk | Documented in Stage 5 & `report.md` |
| 2026-10-05 | D-003 | BSW is AUTOSAR-inspired (`App_Step10ms` / `Com` / `E2E` / `Dem`), handwritten + Python DBC codegen | Demonstrates architectural layering and tooling without proprietary Vector DaVinci license | — |
| 2026-10-05 | D-002 | AEB (UC2) implemented in `speed_ctrl.slx` and `ctrl_codegen.c` and verified in MIL/SIL (`TC-AEB-001`…`004`); physical HIL focuses on UC1 + fault injection | Keeps bench wiring clean while preserving 100 % model and C code coverage of AEB logic | Verified in Stage 3 & Stage 4 |
| 2026-10-05 | D-001 | CAN 2.0A, 500 kbit/s | Standard automotive high-speed CAN baud rate | — |

## Changes

| Date | Version | Change | Affected IDs |
| :--- | :---: | :--- | :--- |
| 2026-10-31 | 1.1 | **Documentation & Traceability Audit:** Synchronized all documentation (`PRODUCT_SPEC.md`, `requirements.md`, `hara_lite.md`, `report.md`, `README.md`, subdirectory `README.md`s) with the exact compiled constants in `init_params.m` / `ctrl_codegen.c` (`Dmax=50%`, `omega_limp=30 rpm`, `Nclear=100 ticks`, `7,555` SIL B2B ticks across `S1..S5`), removed duplicate stub files, fixed relative guide paths, and added on-disk file verification to `trace_check.py`. | All docs & `trace_check.py` |
| 2026-10-24 | 1.0 | **Pass 2 & Pass 3 Baseline Release:** Completed real hardware identification (`K=3.074 rpm/%`, `tau=0.0273 s`, `dz=1.93 %`, `Dpl=15.72 %`, `sep=5.82`), regenerated Simulink models & Embedded Coder C code (`lambda=0.060 s`, `Kp=0.1478`, `Ki=5.4218`), verified Stage 6 Closed-Loop HIL vs MIL (`96.1 %` step fit, `96.6 %` live joystick + fault-injection fit, `0.000 rpm` on-target vs SIL diff, `75–77 us` WCET), generated Stage 5 CAN DBC + E2E C code, and verified 100 % traceability via `trace_check.py`. | All (`THR-*`, `SPD-*`, `AEB-*`, `CAN-*`, `DIA-*`, `SAF-*`, `PRC-*`) |
| 2026-10-19 | 0.2 | Verified Stage 2 & Stage 3 on MATLAB R2026a (`ALL PASS`): aligned discrete window-averaging in `stage2_lib.m`/`s2_02_identify_steps.m`, added 5 % safety margin in `choose_dpl.m`, fixed `UnitDelay` `single` init & dead-zone feedforward in `build_controller_subsystem.m`, and fixed Dataset/timestamp alignment in `run_mil.m`. | SPD-001, SPD-004, SPD-007, THR-002, THR-008 |
| 2026-10-05 | 0.1 | Initial draft of requirements | all |
