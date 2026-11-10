# Stage 3 — Control Design & MIL Verification

| | |
| :--- | :--- |
| **Goal** | Design, tune and verify (in Simulink, Model-in-the-Loop) the controller logic: pedal processing, speed PI(D), AEB state machine, fault manager and mode manager — all against the plant from Stage 2. |
| **Entry** | Stage 1 baseline; Stage 2 parameters ($K,\tau,t_d,d_{dz},\sigma$'s). |
| **Exit** | Every MIL test in §6 passes; the controller model has the exact I/O contract of `PRODUCT_SPEC.md` §5. |
| **Maps to** | Blueprint Phase 3. Requirements: THR-*, SPD-002…004, AEB-001…006, DIA-002, SAF-003. **Executable steps and code: [STAGE_3_RUNBOOK.md](STAGE_3_RUNBOOK.md).** |

---

## 1. Tasks

| # | Task | Output |
| :-: | :--- | :--- |
| 3.1 | Write `modelling_guideline.md` (naming, ports, data types, sample times) | file |
| 3.2 | Pedal processing subsystem (§3.1) | model + MIL test |
| 3.3 | Speed controller (§3.2–3.4) tuned with the lambda method | model + step metrics |
| 3.4 | Fault manager with debounce (§3.5) and mode manager (§3.6) | Stateflow/Simulink |
| 3.5 | AEB Stateflow chart (§3.7) | chart + coverage |
| 3.6 | Assemble `speed_ctrl.slx` (top-level, closed-loop with plant) | model |
| 3.7 | MIL test suite (§6) with pass/fail assertions | `tests/` |
| 3.8 | Model checks (Model Advisor, MISRA-oriented settings) | report |
| 3.9 | Archive the verified reference signals for Stage 4 back-to-back | `.mat` |

## 2. Top-level model interface (fixed contract)

Model name: `speed_ctrl`. Top level = **atomic subsystem `Controller`** (this is what Stage 4 generates code from), plus `Plant` and `Scenario` blocks outside.

| Direction | Port | Data type | Sample time | Notes |
| :--- | :--- | :--- | :---: | :--- |
| In | `pedal1_adc`, `pedal2_adc` | `uint16` | 0.01 | |
| In | `speed_meas_rpm` | `single` | 0.01 | |
| In | `speed_valid` | `boolean` | 0.01 | |
| In | `distance_mm` | `uint16` | 0.05 | rate transition to 0.01 in model |
| In | `distance_valid` | `boolean` | 0.05 | |
| In | `can_pedal_cmd_pct` | `single` | 0.02 | |
| In | `can_cmd_valid`, `can_age_ms` | `boolean`, `uint16` | 0.02 | |
| In | `ctrl_source`, `reset_cause` | `uint8` | 0.1 | |
| Out | `duty_cmd_pct` | `single` | 0.01 | |
| Out | `servo_cmd_deg` | `single` | 0.05 | |
| Out | `speed_req_rpm` | `single` | 0.01 | |
| Out | `mode`, `aeb_state` | `uint8` (enum-backed) | 0.01 / 0.05 | |
| Out | `fault_bitmap` | `uint16` | 0.01 | |

Parameters are `Simulink.Parameter` objects created by `init_params()`:

```matlab
Kp = Simulink.Parameter(single(par.Kp));
Kp.CoderInfo.StorageClass = 'ExportedGlobal';   % calibratable global in generated C
```

## 3. Mathematics and algorithms

Let $T_s = 0.01$ s and $k$ the tick index.

### 3.1 Pedal processing (decision D-004: KY-023 diagonal emulation)

The joystick axes are **not** physically redundant. Redundancy is *emulated*: the operator pushes the stick smoothly along the **diagonal** (X and Y rise together) for normal driving; pushing along **X only** (Y stays in its dead zone) is the fault demonstration. X is wired to **ADC1** (PA0), Y to **ADC2** (PA1). A **signal-conditioning subsystem** (`PedalCond`) sits between the raw ADC inputs and the control logic and presents the two channels as if they were two pedal sensors.

#### 3.1.1 Signal-conditioning subsystem `PedalCond`

Inputs: `pedal1_adc` (X), `pedal2_adc` (Y). Calibration (Stage 2, THR-008): centre $c_X, c_Y$; dead band $db_X, db_Y$ (counts); **diagonal maxima** $a_{max,X}, a_{max,Y}$; mode `PEDAL2_MODE` $\in \{\text{REVERSE},\text{HALF}\}$.

**Step 1 — deflection with dead band** (forward direction only; $i \in \{X, Y\}$, $d_i = a_i - c_i$):

$$n_i = \begin{cases} 0 & d_i \le db_i \\ \text{clamp}\!\left(\dfrac{d_i - db_i}{a_{max,i} - c_i - db_i},\ 0,\ 1\right) & d_i > db_i \end{cases}$$

Use the **diagonal** full-push readings for $a_{max,i}$: a round/square gate limits each axis when the stick is pushed diagonally, so the single-axis maximum would never be reached in normal use and the other channel would saturate low.

**Step 2 — sensor values** (what each "pedal sensor" would report):

$$s_1 = 100\,n_X \qquad\qquad s_2 = \begin{cases} 100\,(1 - n_Y) & \text{REVERSE (100\% to 0\%)} \\ 50\,n_Y & \text{HALF (0\% to 50\%)} \end{cases}$$

**Step 3 — expected relationship** (verifiable by hand):

$$\text{REVERSE: } s_1 + s_2 \approx 100 \qquad\qquad \text{HALF: } s_2 \approx 0.5\,s_1$$

**Step 4 — back-conversion to the pedal axis** and plausibility residual:

$$p_2 = \begin{cases} 100 - s_2 & \text{REVERSE} \\ \min(2\,s_2,\ 100) & \text{HALF} \end{cases}\qquad e[k] = |s_1[k] - p_2[k]|$$

Both modes reduce to $p_2 = 100\,n_Y$; the mode only changes the *signal domain* in which the check is expressed. In HALF mode the Y noise is amplified ×2 by the back-conversion, so REVERSE gives the better detection margin; keep HALF for demonstrating the "half-gain" scheme used in real pedals.

Outputs of `PedalCond`: $s_1,\ s_2,\ p_2,\ e$.

**Honest limitation:** the inversion/half-scaling is done in software on two pots that share supply and ground, so *electrical* common-cause faults (shared supply wires between channels) cannot be represented. The demonstration covers per-channel faults (stuck, floating, open, mechanical disagreement) and exercises the same detection logic a real redundant pedal would use.

#### 3.1.2 Fault detection and request generation

**Range check (on the raw ADC value, *before* clamping):** `F_PEDAL_RANGE` if $a_i < a_{lo}$ or $a_i > a_{hi}$ (about 2.5 % margin inside 0–4095) for $N_{range}$ ticks. This catches a wire break to GND/3.3 V; clamping first would hide it (raw 0 would look like "stick at rest").

**Plausibility:** $e[k] > \Delta_{pl}$ for $N_{pl}$ consecutive ticks (debounce, §3.5) → `F_PEDAL_PLAUS`. $\Delta_{pl}$ comes from **measured diagonal pushes** (Stage 2): $\Delta_{pl} = \mu_e + 3\sigma_e$ (or the 99.9th percentile) plus margin, and must give a **detection separation** $e_{fault}/\Delta_{pl} \ge 2$ for the X-only push at 60 % pedal.

**Request (fail-safe lower select, THR-007):** $p_{cmd}[k] = \min(s_1[k],\ p_2[k])$. For any single-channel fault in the demonstration, one of the two values is 0 or small, so the request *cannot rise* even during the debounce time. The limp-home cap (THR-004) matters when the channels disagree but both are non-zero (e.g. $s_1 = 90\,\%$, $p_2 = 60\,\%$): while `F_PEDAL_PLAUS` is active, $\omega_{req} \leftarrow \min(\omega_{req},\ \omega_{limp})$.

| Case (REVERSE mode) | $n_X$ | $n_Y$ | $s_1$ | $s_2$ | $p_2$ | $e$ | Result |
| :--- | :-: | :-: | :-: | :-: | :-: | :-: | :--- |
| Rest | 0 | 0 | 0 | 100 | 0 | 0 | request 0 |
| Diagonal push | 0.6 | 0.6 | 60 | 40 | 60 | 0 | request 60 % ($s_1+s_2=100$ ✓) |
| X-only push (fault demo) | 0.6 | 0 | 60 | 100 | 0 | 60 | `F_PEDAL_PLAUS`, request = min = 0, mode DEGRADED |
| Y-only push | 0 | 0.6 | 0 | 40 | 60 | 60 | `F_PEDAL_PLAUS`, request 0 |
| Y wire to GND (raw ≈ 0) | 0.6 | — | 60 | — | — | — | `F_PEDAL_RANGE` (raw check) |
| Y wire floating (reads ≈ mid-scale) | 0.6 | 0 | 60 | 100 | 0 | 60 | `F_PEDAL_PLAUS` (a floating input often looks like "stick at centre") |

HALF mode: same rows with $s_2 = 50\,n_Y$, e.g. diagonal 0.6 → $s_2 = 30 = 0.5\cdot 60$ ✓; X-only → $s_2 = 0$ vs expected 30.

**Transient mismatch:** during a push X and Y do not rise at exactly the same instant, and the dead bands matter at low deflection. $N_{pl}$ and $\Delta_{pl}$ must give **0 false positives in 50 valid diagonal pushes** (slow, normal, fast) — acceptance A3-8.

**Low-pass filter** (first-order, discrete): $\ p_f[k] = \alpha\,p_f[k-1] + (1-\alpha)\,p_{cmd}[k],\quad \alpha = e^{-T_s/\tau_f}$

**Rate limiter:** $\ p_r[k] = p_r[k-1] + \text{clamp}\big(p_f[k] - p_r[k-1],\ -R_{dn}T_s,\ R_{up}T_s\big)$

**Speed request:** $\ \omega_{req}[k] = \dfrac{p_r[k]}{100}\,\omega_{max}$, then $\ \omega_{req} \leftarrow \min(\omega_{req},\ \omega_{lim,mode},\ \omega_{lim,AEB})$.

**Source selection:** if `ctrl_source = BENCH` and `can_cmd_valid`, bypass `PedalCond` and use `can_pedal_cmd_pct` as $s_1 = p_2$ (so $e = 0$); if the CAN command is invalid use 0 and let the fault manager react. Both sources go through the same filter and limiter. PC-injected pedal faults in HIL-lite use the `TEST_HOOKS` override of the raw ADC values, which *does* pass through `PedalCond`.

```matlab
function [s1, s2, p2, e] = pedal_cond_ref(adcX, adcY, cal)
% Reference (golden) implementation of PedalCond for MIL tests and hand-checks.
% cal: .cX .cY .dbX .dbY .maxX .maxY .mode ('REVERSE'|'HALF')

function [Dpl, sep] = choose_dpl(e_valid, e_fault)
% Dpl = mean(e_valid) + 3*std(e_valid) (or 99.9th percentile); sep = e_fault / Dpl (must be >= 2).
```

### 3.2 Plant, delay and the design target

Plant (Stage 2): $G(s) = \dfrac{K}{\tau s + 1}$. Include the loop delay of a sampled implementation:

$$T_d \approx \frac{T_s}{2}\ (\text{ZOH}) + T_s\ (\text{computation/actuation delay}) + \frac{(M-1)T_s}{2}\ (\text{speed averaging, if } M \text{ samples})$$

For $M = 1$: $T_d = 1.5\,T_s = 15$ ms.

### 3.3 PI design by the lambda (IMC) method

For $G(s) = K/(\tau s + 1)$ and a PI controller $C(s) = K_p\big(1 + \frac{1}{T_i s}\big)$, choose $T_i = \tau$ (cancel the plant pole) and

$$K_p = \frac{\tau}{K\lambda}, \qquad K_i = \frac{K_p}{T_i} = \frac{1}{K\lambda}$$

Then $L(s) = C(s)G(s) = \dfrac{1}{\lambda s}$ and the closed loop is exactly first order:

$$T(s) = \frac{1}{\lambda s + 1}, \qquad t_r(10\text{–}90\,\%) = 2.2\lambda, \qquad t_s(2\,\%) \approx 3.9\lambda,\quad \text{overshoot} = 0$$

**Choosing $\lambda$ with delay.** The loop crossover is $\omega_c = 1/\lambda$ and the delay costs $\omega_c T_d$ radians of phase:

$$PM \approx 90^\circ - \omega_c T_d\cdot\frac{180}{\pi}$$

Require $PM \ge 60^\circ$: $\ \lambda \ge \dfrac{6}{\pi}T_d \approx 1.9\,T_d$ → with $T_d = 15$ ms, $\lambda \ge 29$ ms. Rule: **$\lambda = \max(1.9\,T_d,\ 0.5\,\tau)$**, then check duty saturation in simulation (very small $\lambda$ demands more than $D_{max}$).

**Feed-forward** (steady-state inverse, reduces the demand on the integrator):

$$D_{ff} = \frac{\omega_{req}}{K} + d_{dz}\,\text{sign}(\omega_{req})\ \ (\omega_{req}>0)$$

**Verification in MATLAB** (do this by hand to learn it):

```matlab
Ts = 0.01;  Td = 1.5*Ts;
G  = tf(K, [tau 1], 'InputDelay', Td);
C  = pid(Kp, Ki);                  % parallel form: Kp + Ki/s
L  = C*G;   T = feedback(L, 1);
[Gm, Pm, ~, Wcp] = margin(L);      % expect Pm >= 60 deg
S  = stepinfo(T);                  % RiseTime ~ 2.2*lambda, Overshoot ~ 0
```

### 3.4 Discrete PI with anti-windup (what the code does each tick)

Error and unsaturated output (backward-Euler integral):

$$e[k] = \omega_{req}[k] - \omega_{meas}[k]$$
$$u_{raw}[k] = D_{ff}[k] + K_p\,e[k] + I[k-1] + K_iT_s\,e[k]$$
$$u[k] = \text{sat}(u_{raw}[k],\ 0,\ D_{max})$$

**Back-calculation anti-windup** (Åström): feed the saturation difference back to the integrator

$$I[k] = I[k-1] + T_s\Big(K_i\,e[k] + K_b\big(u[k] - u_{raw}[k]\big)\Big), \qquad K_b = \frac{1}{T_t}$$

Start with $T_t = T_i = \tau$ (tune between $0.1\,T_i$ and $T_i$). **Why:** while the duty is clamped, the integral term would otherwise keep growing and cause overshoot after the clamp releases (SPD-004).

*(If you use the Simulink PID Controller block, set "Anti-windup method: back-calculation" and "Back-calculation coefficient" $K_b$; check that generated code and block behave equivalently to the equations above — that is a good MIL-vs-hand-model exercise.)*

**Derivative:** not needed for a first-order plant; omit unless a test shows ringing from a second pole.

### 3.5 Fault debounce (up/down counter)

For each fault condition `cond[k]` (boolean):

$$c[k] = \begin{cases}\min(c[k-1] + 1,\ N_{set}) & \text{cond} \\ \max(c[k-1] - 1,\ 0) & \text{not cond}\end{cases}$$

Fault becomes **active** when $c \ge N_{set}$ and **clears** when $c = 0$ (symmetric time; use different up/down steps for asymmetric behaviour). Detection time: $N_{set}\,T_{task}$ (e.g. $N_{pl}=5$, $T_s=10$ ms → 50 ms).

### 3.6 Mode manager

| From | To | Condition |
| :--- | :--- | :--- |
| INIT | NORMAL | self-test OK and $t > 0.5$ s |
| NORMAL | DEGRADED | any non-critical fault active |
| DEGRADED | NORMAL | all faults cleared for ≥ 1 s |
| any | SAFE | critical fault: `F_SPEED_TRACK` persistent > $T_{crit}$, or repeated WDG resets (≥ 3) |
| SAFE | INIT | reset only |

Mode effects: NORMAL → $\omega_{lim} = \omega_{max}$. DEGRADED → $\omega_{lim} = \omega_{limp}$. SAFE → $\omega_{lim} = 0$, servo = safe angle.

### 3.7 AEB (Stateflow)

**Signal conditioning:** $d_f[k] = \text{LPF}\big(\text{median}_3(d[k])\big)$ at the 50 ms rate. Closing speed and time-to-collision over $M$ samples:

$$v_c[k] = -\frac{d_f[k] - d_f[k-M]}{M\,T_{tof}}, \qquad TTC[k] = \begin{cases} d_f[k]/v_c[k] & v_c[k] > v_{min} \\ \infty & \text{otherwise}\end{cases}$$

**Braking distance sanity (choose $d_b$):**

$$d_b \ \ge\ v_{c,max}\,t_{react} + \frac{v_{c,max}^2}{2\,a_{max}}$$

with $t_{react} = T_{tof} + N_{deb}T_{tof} + t_{servo}$ (sensor delay + debounce + servo travel; SG90 ≈ 0.1 s per 60° **[VERIFY]**).

**States and transitions:**

| State | Entry actions | → Next | Condition |
| :--- | :--- | :--- | :--- |
| `NORMAL` | `servo = 0`, `ω_lim = ω_max` | `WARNING` | valid and ($d_f < d_w$ or $TTC < TTC_w$) for $N_{deb}$ samples |
| `WARNING` | `servo = 0`, flag on CAN | `BRAKING` | valid and ($d_f < d_b$ or $TTC < TTC_b$) for $N_{deb}$ samples |
| `WARNING` | | `NORMAL` | $d_f > d_w + d_h$ for $N_{rel}$ samples |
| `BRAKING` | `ω_lim = 0`, `servo = θ_brake` | `NORMAL` (via release) | $d_f > d_w + d_h$ for $N_{rel}$ samples **and** $\omega_{meas} < \omega_{stop}$ |
| any | | **no braking transition** | `distance_valid = 0` for longer than $T_{inv}$ → raise `F_TOF_INVALID`, stay in current state or go to `WARNING` only (AEB-005, SG-03) |

Design constraints: $d_b < d_w$; $d_h \ge 3\sigma_d$ (Stage 2); $N_{deb} \ge 2$.

## 4. Function interfaces (MATLAB support code)

```matlab
function [Kp, Ki, Kb, info] = tune_pi_lambda(K, tau, lambda, Ts, TtFactor)
% Kp = tau/(K*lambda), Ki = 1/(K*lambda), Kb = 1/(TtFactor*tau). info: Ti, wc, expected tr, ts.

function Td = loop_delay(Ts, Mavg)
% Td = 1.5*Ts + (Mavg-1)*Ts/2.

function lambda = choose_lambda(tau, Td)
% lambda = max(6/pi*Td, 0.5*tau).

function m = step_metrics(t, y, yref, tol)
% m.rise (10-90%), m.settle (tol band, default 2%), m.overshoot (%), m.sse.

function c = debounce_step(c, cond, Nset)       % reference for Stateflow/Simulink logic

function [t, sig] = run_mil(modelName, scenarioFcn, stopTime)
% Runs closed-loop, returns logged signals as timetable.
```

## 5. Simulink/Stateflow modelling rules (typical stumbling blocks)

- Break every algebraic loop explicitly with a **Unit Delay** at the encoder feedback.
- Discrete blocks only in the controller; sample times set explicitly; **Rate Transition** blocks where 0.05 s → 0.01 s data cross.
- Data types set on ports (`single` for floats); turn on "Inherit via back propagation" **off**.
- Stateflow: use *enumerated types* for `mode`/`aeb_state`; use temporal logic in ticks (`after(N, tick)` if the chart is discrete) or counters; no continuous-time chart.
- Saturation for $D_{max}$ appears **twice**: in the controller (for anti-windup math) and in BSW (hard cap).

## 6. MIL test suite

| TC | Verifies | Setup | Pass criterion |
| :--- | :--- | :--- | :--- |
| TC-SPD-001 | SPD-003 | pedal step 0 → 60 % at 1 s | 10–90 % rise within $2.2\lambda \pm 20\,\%$; overshoot ≤ 5 %; $t_s(2\,\%) \le 5\lambda$; steady-state error ≤ max(1 rpm, 1 %) |
| TC-SPD-002 | SPD-004 | request above achievable speed for 3 s, then drop to 50 % | overshoot after unsaturation ≤ 5 %; integrator $I$ stays bounded |
| TC-SPD-003 | SPD-005 | request max | $u[k] \le D_{max}$ always |
| TC-SPD-004 | SPD-007 | plant model with ±10 % $K$ and ±20 % $\tau$ | still stable; overshoot ≤ 15 %; $t_s \le 2\times$ nominal |
| TC-SPD-005 | robustness | add speed noise $\sigma_q$ and quantisation | duty noise within ±3 % of span; no limit cycle (amplitude < 1 rpm) |
| TC-THR-001 | THR-002 | ramp raw X and raw Y from centre to diagonal maximum together | $s_1$ rises 0→100 ±0.5; REVERSE: $s_1 + s_2 = 100 \pm 0.5$; HALF: $s_2 = 0.5\,s_1 \pm 0.5$; outputs 0 inside the dead band |
| TC-THR-002 | THR-003 | X-only: raw X ramps to 60 %, raw Y stays at centre (inside dead band) | $e = 60 \pm 1$; `F_PEDAL_PLAUS` set after exactly $N_{pl}$ ticks (±1) |
| TC-THR-003 | THR-004, THR-007 | continue TC-THR-002; then a case with $s_1 = 90\,\%$, $p_2 = 60\,\%$ | $p_{cmd} = \min$ (request never rises during debounce); with `F_PEDAL_PLAUS` active $\omega_{req} \le \omega_{limp}$ within 2 ticks; mode = DEGRADED |
| TC-THR-004 | THR-005 | raw ADC (Y, then X) = 0 / 4095 for 100 ms | `F_PEDAL_RANGE` set from the **raw** value (not hidden by clamping); speed limited |
| TC-THR-005 | THR-006 | pedal step | slope never exceeds $R_{up}$ / $R_{dn}$ |
| TC-THR-006 | THR-003 (no false alarm) | 50 recorded or synthetic valid diagonal pushes (slow/normal/fast, with measured X–Y skew) | **0** `F_PEDAL_PLAUS` events |
| TC-THR-007 | THR-002 | repeat TC-THR-001 with `PEDAL2_MODE` = REVERSE and HALF | both relationships hold; switching mode needs no other change |
| TC-THR-008 | THR-007 | Y stuck at max, X at rest | $p_2 = 100$, $s_1 = 0$, $e = 100$, request 0, fault flagged |
| TC-AEB-001 | AEB-002/3 | $d$ ramps 600 → 100 mm at 200 mm/s | WARNING at $d \approx d_w$, BRAKING at $d \approx d_b$ (each ±1 sample of debounce) |
| TC-AEB-002 | AEB-004 | $d$ back up to 400 mm | returns to NORMAL after $N_{rel}$ and $\omega_{meas}<\omega_{stop}$; no chattering (≤ 1 transition per event) |
| TC-AEB-003 | AEB-005 | `distance_valid = 0` while approaching | BRAKING never entered from invalid data; `F_TOF_INVALID` set |
| TC-AEB-004 | AEB-006 | all of the above | Stateflow transition coverage = 100 %, state coverage = 100 % |
| TC-DIA-001 | DIA-002 | cond toggles 1 tick on/off | no fault set (debounce works); sustained → set after $N_{set}$ |
| TC-SAF-001 | SAF-003 | inject each fault | mode transitions exactly as table 3.6 |
| TC-CAN-001 | CAN-006 | `can_age_ms` > 60 in BENCH mode | `F_CAN_TIMEOUT`, $p = 0$ |

All tests are automated (assessments in Simulink Test, or a MATLAB script with `assert`). **Use identical test vectors in Stage 4** (save them with `Simulink.SimulationData.Dataset`).

## 7. Deliverables

- `03_simulink/controller/speed_ctrl.slx` (+ referenced models), `plant/` from Stage 2.
- `03_simulink/tests/` (Simulink Test file or scripts), results and coverage report.
- `03_simulink/modelling_guideline.md`, Model Advisor report.
- `03_simulink/data/mil_reference/*.mat` (reference signals for Stage 4).
- Updated traceability rows (design + MIL test columns).

## 8. Acceptance criteria

| ID | Criterion | Evidence |
| :--- | :--- | :--- |
| A3-1 | All TCs in §6 `PASS` | test report |
| A3-2 | Analytical check: loop phase margin ≥ 60° (with $T_d$) and closed-loop $t_r$ within ±20 % of $2.2\lambda$ in simulation | MATLAB output |
| A3-3 | Stateflow transition coverage 100 % for AEB and mode manager | coverage report |
| A3-4 | Model Advisor: no *failed* checks in the selected set (naming, data types, no algebraic loops, no implicit casts) | report |
| A3-5 | Controller interface exactly matches §2 (checked by script comparing port list to `signal_list.md`) | script output |
| A3-6 | No `double` data types in the `Controller` subsystem (script: `find_system` + `get_param(...,'CompiledPortDataTypes')`) | script output |
| A3-7 | Maximum duty $\le D_{max}$ and no integrator wind-up in any test | logs |
| A3-8 | `PedalCond`: 0 false plausibility faults in 50 valid diagonal pushes, and detection separation $e_{fault}/\Delta_{pl} \ge 2$ for the X-only push at 60 % (both modes) | TC-THR-002/006/007 output |
| A3-9 | `pedal_cond_ref` (MATLAB reference) and the Simulink `PedalCond` subsystem agree exactly on all test vectors | comparison script |

## 9. Expected issues

| ID | Issue | Why | Mitigation |
| :--- | :--- | :--- | :--- |
| ISS-3-01 | Controller works in MIL but demands unrealistic duty (saturates constantly) | $\lambda$ too small for available voltage | Increase $\lambda$; check $\omega_{max}$ reachable ($K\,D_{max}$) |
| ISS-3-02 | Limit-cycle chatter at low speed | Dead zone + encoder quantisation + integrator | Add small deadband on error (≈ $\Delta\omega_q$); filter speed; feed-forward with $d_{dz}$ |
| ISS-3-03 | Measured delay larger than assumed | Averaging window, ToF latency, CAN latency | Recompute $T_d$; re-tune $\lambda$ (rule in §3.3) |
| ISS-3-04 | Algebraic loop error | Direct feedthrough of speed into command | Unit delay in the feedback path |
| ISS-3-05 | Stateflow chart gets *stuck* or chatters at thresholds | No hysteresis, noise | Use $d_h$ and debounce counters; test with noisy $d(t)$ |
| ISS-3-06 | Different results with fixed-step vs. variable-step | Solver config | Always use fixed-step, set `Ts` from `par.Ts` |
| ISS-3-07 | `double` slipped into model (constants, Gain blocks) | Default data type in Simulink is `double` | Set "Output data type" explicitly; run A3-6 script; set model-level default to `single` where available |
| ISS-3-08 | Stateflow temporal logic uses time, not ticks, and misbehaves in code | Absolute-time temporal logic needs a timer | Use tick-count logic or explicit counters |
| ISS-3-09 | "Simulink Test" or "Model Advisor" licence feature missing or slow | Optional add-ons | Fall back to MATLAB scripts with `assert` and manual coverage via counters |
| ISS-3-10 | MIL passes but real motor differs | Plant model error | Stage 6 will show it; keep $K$, $\tau$ as calibratable and re-tune (that is what calibration is for) |
