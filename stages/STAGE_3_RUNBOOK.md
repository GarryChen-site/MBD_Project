# Stage 3 Runbook — executable steps

> Companion to [STAGE_3_control_design_mil.md](STAGE_3_control_design_mil.md) (theory, criteria).
> **Status: written but NOT run** (MATLAB/Simulink run was declined). Simulink build scripts in particular may need small fixes on first run (block parameter names differ across releases). Treat the first run as debugging, and log fixes in `01_requirements/change_log.md`.

Prerequisite: `run_stage2('synthetic')` (or `'real'`) so `data/params_identified.mat` exists.

## One command

```matlab
cd MBD_HILS_Project/03_simulink
setup_paths
run_stage3        % design PI -> build model -> interface check -> 18 MIL tests
```

## Step by step

| Step | Command | What happens / what to verify |
| :-: | :--- | :--- |
| 1 | `s3_00_design_pi` | Prints Kp, Ki, Kb, lambda and (with Control System Toolbox) phase margin ≥ 60° (A3-2). Hand-check: Kp = tau/(K·lambda). |
| 2 | `s3_01_build_speed_ctrl` | Builds `controller/speed_ctrl.slx` programmatically (below). Open it and compare with the diagram. |
| 3 | `s3_02_check_interface` | A3-5 port list, A3-6 no `double` inside Controller |
| 4 | `run_stage3_tests` | Table of PASS/FAIL with measured numbers |

## What the builder creates (block connection tables)

Parameters live in the base workspace (`load_params_to_base`): controller `P_*`, plant `PL_*` (separate so tests can perturb the plant only). Fixed-step `ode4`, step `Ts/10`; controller blocks run at `P_Ts`.

### Top level (`speed_ctrl`)

| From | To |
| :--- | :--- |
| `adcX_in` (From Workspace `scn_adcX`) | `Controller/1` pedal1_adc |
| `adcY_in` (`scn_adcY`) | `Controller/2` pedal2_adc |
| `FbDelay` (Unit Delay) | `Controller/3` speed_meas_rpm |
| `Controller/1` duty_cmd_pct | `Plant/1` |
| `Plant/1` | `FbDelay/1` and Outport `speed_meas_rpm` |
| `Controller/2…8` | Outports `speed_req_rpm, fault_bitmap, mode, s1_pct, p2_pct, e_pct, p_cmd_pct` |

`FbDelay` breaks the algebraic loop (ISS-3-04). Outports log as a Dataset (`out.yout`).

### Controller subsystem (`controller/build_controller_subsystem.m`)

| Block | Type | Inputs ← | Notes |
| :--- | :--- | :--- | :--- |
| X_single, Y_single, W_single | Data Type Conversion → single | pedal1/2, speed | single-precision contract |
| cal, fp | Constant (`P_cal`, `P_fm`) single | — | `[cX cY dbX dbY maxX maxY modeId alo ahi]`, `[Dpl Npl Nrange Nclear ω_max ω_limp]` |
| **PedalCond** | MATLAB Function → `pedal_cond_core` | X_single, Y_single, cal | out s1,s2,p2,e,rangeBad |
| **FaultMode** | MATLAB Function, persistent debounce counters | e, rangeBad, fp | out fault_bitmap, mode, ω_lim |
| PCmd | MinMax min | s1, p2 | lower-select (THR-007) |
| PedalLPF | Discrete Filter `[1-α]/[1 -α]` | PCmd | α = e^(−Ts/τf) |
| PedalRateLim | Rate Limiter ±R | PedalLPF | |
| ReqScale | Gain ω_max/100 | PedalRateLim | |
| ReqLimit | MinMax min | ReqScale, ω_lim | → `speed_req_rpm` |
| SpeedErr | Sum +− | ReqLimit, W_single | |
| FF_gain/FF_sum/FF_switch | Gain 1/K, +dz, Switch (req>0) | ReqLimit | `D_ff = req/K + dz` |
| Kp_gain, KiTs_gain | Gain | SpeedErr | |
| I_prev | Unit Delay (state I[k−1]) | I_acc | |
| U_raw | Sum ++++ | FF, Kp·e, I_prev, KiTs·e | u_raw (§3.4) |
| Sat | Saturation [0, Dmax] | U_raw | → `duty_cmd_pct` |
| SatDiff, Kb_gain, Ki_gain, I_rate, Ts_gain, I_acc | back-calculation | Sat−U_raw, e | `I[k]=I[k−1]+Ts(Ki e + Kb(u−u_raw))` |

### Plant subsystem (`plant/build_plant_subsystem.m`)

`duty → ToDouble → DeadZone ±PL_dz → TF PL_K/(PL_tau s+1) → TransportDelay PL_td → Gain Nrev/60 → Integrator → ZOH(Ts) → floor → (−UnitDelay) = dN → Gain 60/(Nrev·Ts)`.

## Test inventory (`tests/run_stage3_tests.m`)

| Test | What it does | Pass criterion |
| :--- | :--- | :--- |
| TC-THR-001 | MATLAB-only ramp, both modes | s1 error ≤ 0.5; s1+s2=100 (REVERSE) / s2=0.5·s1 (HALF) ±0.5; dead-band edge → 0 |
| A3-9 | Simulink vs `pedal_cond_ref` on a ramp | max diff < 1e-3 |
| TC-THR-002 | X-only 60 % | e ≈ 60 ± 1, fault after Npl ± 1 ticks, DEGRADED |
| TC-THR-003a/b | lower-select; s1=90, p2=60 | request never rises; ω_req ≤ ω_limp 2 ticks after fault |
| TC-THR-004 | Y=0, X=4095 raw | raw range fault bit set |
| TC-THR-005 | pedal step to 100 % | slope ≤ R_up·ω_max/100 |
| TC-THR-006 | replay `joy_diag_valid.csv` (50 pushes) | 0 plausibility events |
| TC-THR-007 | REVERSE and HALF in the model | no fault, s1=p2=90 |
| TC-THR-008 | Y stuck at max | p2=100, s1=0, e=100, p_cmd=0, fault |
| TC-SPD-001 | 0→60 % step (pedal shaping off) | rise = 2.2λ ± 20 %, overshoot ≤ 5 %, ts ≤ 5λ+0.05, sse small |
| TC-SPD-002 | plant K×0.6, request 100 %→50 % | recovers ≤ 1 s, no undershoot < 0.9 (anti-windup) |
| TC-SPD-003 | request max | duty ≤ Dmax |
| TC-SPD-004 | K ±10 %, τ ±20 % corners | overshoot ≤ 15 %, ts ≤ 2× nominal |
| TC-DIA-001 | 1-tick glitch / 10-tick fault | no fault / set after Npl |
| TC-SAF-001 | fault then release | NORMAL→DEGRADED→NORMAL |
| TC-AEB-001/2/3 | **reference model** `aeb_ref.m` (MATLAB) | warn/brake points, no chatter (also with 8 mm noise), no braking on invalid data |

## Known gaps (be honest in reviews)

- **AEB is a MATLAB reference model, not yet a Stateflow chart** and not in `speed_ctrl.slx`; TC-AEB-004 (Stateflow coverage) is not covered. Next step: build the chart by hand (states NORMAL/WARNING/BRAKING, tick counters as in `aeb_ref.m`) and compare it against `aeb_ref` on the same vectors.
- The CAN interface, `ctrl_source` and `reset_cause` ports of the full contract (§2) are not in this milestone; `s3_02_check_interface` checks the reduced contract.
- Mode manager is the two-mode (NORMAL/DEGRADED) subset; SAFE and WDG transitions come with Stage 5.
- Controller input is converted straight to `single` (uint16 contract is respected by the sources, not enforced by the model).

## If something fails

| Symptom | Likely cause |
| :--- | :--- |
| `Parameter 'X' does not exist` in build | block parameter renamed in your Simulink release: open the block, hover the field, fix in the builder |
| `Algebraic loop` | feedback path lost its Unit Delay |
| MATLAB Function block compile error | `pedal_cond_core` not on path (`setup_paths`) or type mismatch (single/double) |
| TC-SPD-001 rise off by > 20 % | real loop delay larger than `loop_delay` assumes (ISS-3-03); re-check `Td` and `Mavg` |
| TC-THR-006 events > 0 | `Dpl` too tight for the measured X–Y skew: re-run `s2_03`, enlarge data set or Npl |
