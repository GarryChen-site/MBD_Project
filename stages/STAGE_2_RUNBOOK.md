# Stage 2 Runbook — executable steps

> Companion to [STAGE_2_plant_sensor_models.md](STAGE_2_plant_sensor_models.md) (theory, criteria).
> **Status: all code below was written but NOT run or compiled** (MATLAB run was declined, no hardware attached). First run = your verification; fix and log anything that breaks.

All MATLAB paths are relative to `MBD_HILS_Project/03_simulink/`.

## Part A — Prove the scripts first (no hardware, ~1 min)

```matlab
cd MBD_HILS_Project/03_simulink
setup_paths
results = run_stage2('synthetic');     % generates data/synthetic, runs 01..04, prints A2-x table
s2_05_build_plant                      % builds plant/motor_plant.slx programmatically
s2_06_validate_plant                   % simulates it vs the held-out run, fit >= 85 %
```

Expected: the A2-1…A2-8 table prints `PASS`. The synthetic truth is in `data/synthetic/truth.mat` (K=4.6, tau=0.115, dead zone 7.5 %, nd=1); identified values should land within a few % — **that comparison is your check that the identification code is right.**
Synthetic data proves the scripts, not the motor.

| Script | Does | Key formula (Stage 2 doc §) |
| :--- | :--- | :--- |
| `s2_00_make_synthetic_data` | writes CSVs in the *same format* as the firmware loggers | 3.1, 3.4 |
| `s2_01_static_map` | duty→rpm_ss, fits K and dz | 3.3 |
| `s2_02_identify_steps` | fits K, tau, nd on 8 runs; validates on `step_25_r2` | 3.5 |
| `s2_03_joystick_calibration` | centre, dead band, diagonal maxima, Dpl, false alarms, X-only separation (both modes) | Stage 3 §3.1 |
| `run_stage2` | runs all, writes `data/params_identified.mat` | — |
| `init_params` | defaults overlaid by `params_identified.mat` (`par.src`) | — |

**Hand check (do once):** open `s2_02_identify_steps` output; confirm tau ≈ time to 63 % of the step (graphical estimate, §3.5) on one CSV.
**A2-4 note:** `dN` is a differenced floor, so residual lag-1 autocorrelation is ≈ −0.5×(quantisation share) by construction. The check judges lags 2–10 and prints lag 1 separately.

## Part B — Real data

### B1. CubeMX configuration (NUCLEO-F446RE)

| Peripheral | Setting |
| :--- | :--- |
| TIM1 CH1 (PA8) | PWM, prescaler 0, ARR = timer_clock/20 kHz − 1. With the project's HSI 16 MHz / no PLL (P-002) the timer clock is 16 MHz → **ARR = 799**. If you change the clock tree, recompute. **[VERIFY timer clock in the CubeMX clock tree]** |
| TIM3 (PA6/PA7) | Encoder mode TI1&TI2, X4 (both edges), input filter 5–10, 16-bit |
| GPIO | PB0, PB1, PC0 outputs (TB6612 IN1/IN2/STBY) |
| ADC1 | IN0 (PA0), 12 bit, software trigger, single conversion, sampling ≥ 56 cycles |
| ADC2 | IN1 (PA1), same settings |
| USART2 | 115200 8N1 (ST-LINK VCP) |

Add to the project: [`step_logger.c/.h`](../04_firmware/experiments/step_logger/), [`pedal_logger.c/.h`](../04_firmware/experiments/pedal_logger/). Adjust the `extern` handle names if CubeMX named them differently.

### B2. Joystick sessions (needs T0-5 passed)

`main.c` after init:
```c
#include "pedal_logger.h"
/* USER CODE BEGIN 2 */
PedalLogger_RunSessions();      /* prompts over UART, ~4.5 min total */
/* USER CODE END 2 */
```
On the Mac:
```bash
pip install pyserial
python3 MBD_HILS_Project/04_firmware/experiments/capture_log.py /dev/cu.usbmodemXXXX MBD_HILS_Project/03_simulink/data/real
```
Technique: *full* pushes go to the corner of the gate, smooth (0.5–2 s); *valid* pushes vary peak and speed; *X-only* pushes keep Y at rest. Do not touch the stick during `joy_rest`.

### B3. Motor sessions (needs T0-6, T0-7 passed, fresh batteries)

```c
#include "step_logger.h"
/* USER CODE BEGIN 2 */
StepLogger_Init();
StepLogger_RunAll();            /* 10 static + 9 step runs, 60 s rest between: ~20 min */
/* USER CODE END 2 */
```
Capture with the same `capture_log.py` command. **Safety:** duty is hard-capped at 500 ‰ in firmware (`STEP_DUTY_MAX_PERMILLE`); recompute it if `V_bat` is not 6 V (P-005). Motor must be free-running.

### B4. Analyse real data

Edit nothing; just:
```matlab
results = run_stage2('real');   % reads data/real/*.csv
s2_05_build_plant; s2_06_validate_plant(fullfile(pwd,'data','real'))
```
Then set `P-006` (`par.Nrev`) from T0-6 in `init_params.m` **before** this step, and record `V_bat` (M4).

## Verification checklist

| # | Check | Pass |
| :-: | :--- | :--- |
| 1 | `run_stage2('synthetic')` all rows PASS | ☐ |
| 2 | Identified K, tau, dz within 5 % of `truth.mat` | ☐ |
| 3 | `motor_plant.slx` opens; `s2_06` fit ≥ 85 % | ☐ |
| 4 | Real: `data/real` has 19 motor CSVs + 4 joy CSVs | ☐ |
| 5 | Real: `run_stage2('real')` table; investigate each FAIL (see ISS-2-xx) | ☐ |
| 6 | `params_identified.mat` exists and `init_params().src == 'identified'` | ☐ |
