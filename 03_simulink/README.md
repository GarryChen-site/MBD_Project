# 03 Simulink — Plant Identification, Control Design, Codegen & SIL/HIL Verification

This directory contains all MATLAB scripts, Simulink models (`.slx`), reusable library functions, and experimental datasets for **Stages 2, 3, 4, and 6**.

---

## 1. Directory Layout

```text
03_simulink/
├── setup_paths.m                  # Adds lib/, plant/, controller/, stage2..4/, tests/ to MATLAB path
├── init_params.m                  # Single source of truth for all P-xxx parameters (loads params_identified.mat)
├── data/
│   ├── params_identified.mat      # Final calibrated plant & sensor parameter struct `ident`
│   ├── real/                      # 25 physical STM32F446RE + N20 + KY-023 CSV logs & validation plots
│   └── synthetic/                 # Synthetic ground-truth CSVs (`truth.mat`) for Pass 1 verification
├── lib/                           # Reusable algorithmic & mathematical library
│   ├── stage2_lib.m               # FOPDT window-averaged step ID, static map OLS, ARX, NRMSE fit
│   ├── pedal_cond_core.m          # Codegen-compatible single-precision D-004 pedal conditioning
│   ├── pedal_cond_ref.m           # Vectorized reference model for PedalCond
│   ├── choose_dpl.m               # Statistical plausibility threshold & separation ratio calculator
│   ├── loop_delay.m               # Effective discrete loop delay Td = (1.5 + (M-1)/2)*Ts
│   ├── choose_lambda.m            # IMC lambda selector with discrete ZOH stability floor (D-009)
│   ├── tune_pi_lambda.m           # Pole-cancellation PI + back-calculation anti-windup gain calculator
│   ├── debounce_step.m            # Up/down fault debounce counter reference
│   ├── step_metrics.m             # Rise time, settling time, overshoot & steady-state error evaluator
│   ├── back_to_back_compare.m     # MIL vs SIL signal equivalence verifier
│   ├── cal_to_vec.m               # Packs pedal calibration struct into single-precision vector `P_cal`
│   └── load_params_to_base.m      # Exports `P_*` (controller) and `PL_*` (plant) to base workspace
├── plant/
│   ├── build_plant_subsystem.m    # Programmatic builder for the DC motor + encoder quantization subsystem
│   └── motor_plant.slx            # Standalone Simulink plant model (Stage 2)
├── controller/
│   ├── build_controller_subsystem.m # Programmatic builder for the complete Stage 3/4 Controller
│   ├── ert_stm32.m                # Embedded Coder (`ert.tlc`, C99, single-precision) configuration
│   ├── speed_ctrl.slx             # Closed-loop MIL model (Controller + Plant + FbDelay)
│   └── ctrl_codegen.slx           # Standalone discrete controller model for C code generation
├── stage2/                        # Stage 2: Empirical Plant & Sensor Identification
│   ├── run_stage2.m               # Master runner: `run_stage2('real')` or `run_stage2('synthetic')`
│   ├── s2_00_make_synthetic_data.m
│   ├── s2_01_static_map.m         # Fits steady-state gain K_stat and Coulomb dead-zone D_dz
│   ├── s2_02_identify_steps.m     # Fits window-averaged FOPDT K, tau, nd & cross-validates
│   ├── s2_03_joystick_calibration.m # Calibrates KY-023 centers, deadbands, D_pl & separation ratio
│   ├── s2_05_build_plant.m        # Builds `plant/motor_plant.slx`
│   └── s2_06_validate_plant.m     # Simulates `motor_plant.slx` vs held-out hardware step log
├── stage3/                        # Stage 3: Control Design & MIL Simulation
│   ├── run_stage3.m               # Master runner for Stage 3
│   ├── s3_00_design_pi.m          # Designs Kp, Ki, Kb and verifies phase margin >= 60 deg
│   ├── s3_01_build_speed_ctrl.m   # Programmatically builds `controller/speed_ctrl.slx`
│   ├── s3_02_check_interface.m    # Audits port types (`single`/`uint16`/`uint8`) and sample times
│   ├── run_mil.m                  # Executes closed-loop MIL simulation for a given joystick trajectory
│   ├── scn_pedal.m                # Generates `diag`, `xonly`, `yonly`, `ygnd` test trajectories
│   └── aeb_ref.m                  # Golden reference state machine for AEB (`TC-AEB-001..003`)
├── stage4/                        # Stage 4 & Stage 6: Codegen, SIL B2B & Hardware-vs-MIL Overlay
│   ├── run_stage4.m               # Master runner for Stage 4 (`s4_01` -> `s4_02` -> `s4_03`)
│   ├── s4_01_build_codegen_model.m # Builds `controller/ctrl_codegen.slx`
│   ├── s4_02_generate_code.m      # Generates `04_firmware/app_generated/ctrl_codegen.{c,h}` & audits hygiene
│   ├── s4_03_run_sil_b2b.m        # Compiles `ctrl_sil_mex` and runs 5 MIL-vs-SIL B2B scenarios (7,555 ticks)
│   ├── rebuild_from_real_data.m   # One-command Pass 2 rebuild of Stages 2, 3 & 4 from `data/real/*.csv`
│   └── s6_compare_hw_vs_mil.m     # Replays STM32 HIL CSVs through MIL & SIL and generates overlay plots
├── tests/
│   └── run_stage3_tests.m         # 19-test automated MIL requirement verification suite
└── codegen_work/                  # Working directory for `ctrl_sil_mex.c` and Embedded Coder build artifacts
```

---

## 2. Key Commands

Run from the repository root:

```bash
# Rebuild all models, parameters, C code, MIL tests, and SIL B2B reports from real hardware data:
/Applications/MATLAB_R2026a.app/bin/matlab -batch "addpath(genpath('03_simulink')); rebuild_from_real_data();"

# Replay real STM32 closed-loop logs against Simulink MIL and C MEX SIL:
/Applications/MATLAB_R2026a.app/bin/matlab -batch "addpath(genpath('03_simulink')); s6_compare_hw_vs_mil('03_simulink/data/real/hil_step_test.csv'); s6_compare_hw_vs_mil('03_simulink/data/real/hil_closed_loop.csv');"
```
