# Stage 2 — Plant & Sensor Models

| | |
| :--- | :--- |
| **Goal** | Obtain a *measured*, validated model of the N20 motor + TB6612 + encoder chain, and numeric characterisation of every sensor, so that Stage 3 can be designed on data rather than guesses. |
| **Entry** | Stage 0 `PASS`; Stage 1 signal list exists (names/units). |
| **Exit** | Plant model validated (fit ≥ 90 % on held-out data), parameters in `init_params.m`, sensor statistics recorded. |
| **Maps to** | Blueprint Phase 2. Requirements: SPD-007, SPD-001, THR-002, AEB-001. **Executable steps and code: [STAGE_2_RUNBOOK.md](STAGE_2_RUNBOOK.md).** |

---

## 1. Tasks

| # | Task | Output |
| :-: | :--- | :--- |
| 2.1 | Write the experiment firmware (logger) | `04_firmware/experiments/step_logger/` |
| 2.2 | Static map: duty → steady-state rpm (M1, M3, M4) | `data/static_map.csv` |
| 2.3 | Dynamic data: step responses at 3 amplitudes × 3 repeats (M2) | `data/step_*.csv` |
| 2.4 | Estimate $K$, $\tau$, dead zone, delay (§3) | `init_params.m` entries |
| 2.5 | Build `plant/motor_plant.slx` including encoder quantisation | model |
| 2.6 | Validate on a held-out experiment (§3.5) | validation report |
| 2.7 | Characterise sensors: joystick (D-004 calibration: centre, dead band, diagonal maxima, 20–50 diagonal pushes slow/normal/fast → residual $e=\lvert s_1-p_2\rvert$, $\Delta_{pl}=\mu+3\sigma$; plus X-only pushes → fault residual and X→Y leakage), ToF, (opt.) temp, accelerometer | `data/sensor_*.csv`, stats |
| 2.8 | Build the vehicle/obstacle scenario generator for AEB MIL | `plant/aeb_scenario.m` |

## 2. Experiment protocol

1. Fully charged/fresh batteries. Record $V_{bat}$ unloaded and under load (M4).
2. Motor free (no load), same temperature each time (let it rest 1 min between runs; the motor warms and changes $R$).
3. Log at $T_s = 10$ ms (100 Hz) over UART: line format `t_ms,duty_permille,dN\n` (≈ 20 bytes → 2 kB/s, well below the ≈ 11.5 kB/s limit of 115200 baud).
4. Step sequence per run: 0 % for 0.5 s → $D_{step}$ for 2 s → 0 % for 1 s. Amplitudes: 15 %, 25 %, 35 % (all ≤ $D_{max}$, and above the dead zone found in 2.2).
5. Three repeats per amplitude. Use run 2 of the 25 % set as **validation only**, never for fitting.

Logger interface:

```c
/* step_logger.h */
typedef struct {
  uint16_t duty_start_permille;
  uint16_t duty_step_permille;
  uint32_t t_pre_ms, t_step_ms, t_post_ms;
  uint32_t period_ms;           /* 10 */
} StepExp_t;

void StepLogger_Run(const StepExp_t *cfg);   /* blocking; one CSV line per period */
```

Timestamps must come from a hardware timer (`HAL_GetTick()` is 1 ms resolution; fine for 10 ms). Do not call `printf` inside the timing-critical section; format after the run if buffers allow, or use a ring buffer drained by the main loop.

## 3. Mathematics

### 3.1 Motor equations (standard brushed DC model)

$$L\frac{di}{dt} = v - R i - K_e \omega, \qquad J\frac{d\omega}{dt} = K_t i - b\,\omega - \tau_c\,\text{sign}(\omega) - \tau_L$$

with armature resistance $R$, inductance $L$, back-EMF constant $K_e$, torque constant $K_t$ ($K_t = K_e$ in SI), inertia $J$, viscous friction $b$, Coulomb friction $\tau_c$, load torque $\tau_L$.

Neglecting $\tau_c$ and $\tau_L$, the transfer function from voltage to speed is

$$\frac{\Omega(s)}{V(s)} = \frac{K_t}{LJ\,s^2 + (Lb + RJ)\,s + (Rb + K_t K_e)}$$

The electrical time constant $\tau_e = L/R$ is typically a few ms or less for an N20, much faster than the mechanical one (tens to hundreds of ms). Dropping $L$ gives the first-order form used in the project:

$$\frac{\Omega(s)}{V(s)} = \frac{K_v}{\tau_m s + 1}, \qquad K_v = \frac{K_t}{Rb + K_tK_e}, \qquad \tau_m = \frac{RJ}{Rb + K_tK_e}$$

**Input scaling.** The ECU commands duty $D$ (%). The average motor voltage is $v \approx V_{bat}\,D/100$ (minus driver drop, see 3.3). With speed in rpm:

$$G(s) = \frac{\omega_{rpm}(s)}{D(s)} = \frac{K}{\tau s + 1}, \qquad K = \frac{60}{2\pi}\cdot\frac{V_{bat}}{100}\,K_v\ \ [\text{rpm}/\%],\quad \tau = \tau_m$$

### 3.2 Electrical/mechanical parameters from simple bench tests (optional cross-check)

| Parameter | Formula | Measurement |
| :--- | :--- | :--- |
| $R$ | $R = V_{test}/I_{stall}$ | M5, stall for < 1 s at low voltage |
| $K_e$ | $K_e = \dfrac{V - R I_0}{\omega_0}$ | no-load current $I_0$ and speed $\omega_0$ at voltage $V$ |
| $b$ | $b = \dfrac{K_t I_0}{\omega_0}$ | same no-load test |
| $J$ | $J = \dfrac{\tau_m (Rb + K_tK_e)}{R}$ | from the fitted $\tau_m$ |

These are cross-checks only; the **fitted $K$ and $\tau$ from step data are what the controller uses.** A large mismatch (> 30 %) means the model has a missing effect (e.g. gearbox friction), not that the fit is wrong.

### 3.3 Static map with dead zone and driver drop

Measured steady-state speed vs. duty typically shows a dead zone (static friction) and a nearly linear region:

$$\omega_{ss}(D) = K\,\text{sign}(D)\,\max\big(|D| - d_{dz},\ 0\big)$$

Fit $K$ and $d_{dz}$ by linear regression of $\omega_{ss}$ on $D$ for $D > d_{dz}$: $\omega_{ss} = K D - K d_{dz}$; the intercept on the $D$ axis is $d_{dz}$. The TB6612FNG's on-resistance ($\approx 0.5\ \Omega$ total, **[VERIFY datasheet]**) causes a voltage drop that scales with current; this is lumped into $K$ and $d_{dz}$ and is one reason the map flattens at higher duty.

### 3.4 Speed measurement from the encoder

For $\Delta N$ counts in a window $T_w$:

$$\omega_{rpm} = \frac{60\,\Delta N}{N_{rev}\,T_w}, \qquad \Delta\omega_{q} = \frac{60}{N_{rev}\,T_w}\ \text{(quantisation step)}$$

Quantisation noise (uniform): $\sigma_q = \Delta\omega_q/\sqrt{12}$.

Example: $N_{rev} = 4200$, $T_w = 10$ ms → $\Delta\omega_q = 1.43$ rpm, $\sigma_q = 0.41$ rpm. A moving average over $M$ samples gives $T_w = M T_s$ and divides the step by $M$, but adds about $(M-1)T_s/2$ of delay, which Stage 3 must include in its margin. **Use the same estimator in the plant model and in firmware.**

### 3.5 System identification from a step

**Continuous step model** (step of size $\Delta u$ at $t_0$, dead time $t_d$):

$$y(t) = y_0 + K\,\Delta u\,\Big(1 - e^{-(t - t_0 - t_d)/\tau}\Big)\ \ \text{for } t \ge t_0 + t_d$$

Quick estimates (graphical, for sanity): $K = (y_{ss}-y_0)/\Delta u$; $\tau$ = time from $t_0 + t_d$ to reach $63.2\,\%$ of the final change. Rise time (10–90 %) $t_r = 2.2\,\tau$.

**Least-squares fit** (preferred): find $\theta = [K,\tau,t_d]$ minimising

$$J(\theta) = \sum_k \big(y_k - \hat y_k(\theta)\big)^2$$

(`lsqcurvefit` or `fminsearch`; initial guess from the graphical estimate).

**Discrete ARX alternative** (zero-order hold, sample time $T_s$, delay of $n_d$ samples):

$$y[k+1] = a\,y[k] + b\,u[k - n_d], \qquad a = e^{-T_s/\tau},\quad b = K(1 - a)$$

Stack data into $Y = \Phi\theta$ with $\theta = [a,\ b]^T$, $\Phi = [\,y[k]\ \ u[k-n_d]\,]$, then

$$\hat\theta = (\Phi^T\Phi)^{-1}\Phi^T Y, \qquad \hat\tau = -\frac{T_s}{\ln \hat a}, \qquad \hat K = \frac{\hat b}{1 - \hat a}$$

Choose $n_d$ by trying 0, 1, 2 and keeping the best fit. (Subtract the dead-zone offset from $u$ first: $u_{eff} = D - d_{dz}$.)

**Fit quality** (as in MATLAB `compare`/`goodnessOfFit` NRMSE):

$$\text{fit}\,[\%] = 100\left(1 - \frac{\|y - \hat y\|_2}{\|y - \bar y\|_2}\right)$$

**Residual check:** the autocorrelation of $e = y - \hat y$ should stay inside $\pm 2.58/\sqrt{N}$ (99 % band) for lags ≥ 1. Strong structure means a missing dynamic (e.g. second pole or friction).

**Linearity check:** $K$ from 15 %, 25 % and 35 % steps should agree within ±15 %. If not, add the static nonlinearity (3.3) before the linear block, or schedule gains.

### 3.6 Sensor models

**ADC / joystick.** $x_n = \dfrac{\text{raw} - c}{x_{max} - c}$ (clamped to [0,1]) for the pedal direction; noise $\sigma_{adc}$ from the resting data; minimum plausibility threshold should satisfy $\Delta_{pl} \ge 5\,\sigma_{adc}/\text{span}\cdot 100\,\%$ plus joystick mechanical asymmetry (measure it).

**VL53L0X.** Model as delayed, noisy, quantised measurement:

$$d_{meas}[k] = \text{round}\Big(d_{true}(t_k - t_{tof}) + n[k]\Big), \qquad n \sim \mathcal N(0, \sigma_d^2)$$

with timing budget $t_{tof}$ (≈ 20–33 ms **[VERIFY configuration]**). $\sigma_d$ depends on surface and distance (M8). Design rule used in Stage 3: hysteresis $d_h \ge 3\,\sigma_d$ so that noise alone cannot toggle the state. Filter option: median of 3, then first-order low-pass.

**Grove temperature (UC4, optional).** The module is an NTC divider; with the common Beta model ($B \approx 4275$, $R_0 = 100\ \text{k}\Omega$ at $T_0 = 298.15$ K):

$$R = R_0\left(\frac{4095}{\text{raw}} - 1\right), \qquad T[^\circ\text{C}] = \frac{1}{\dfrac{\ln(R/R_0)}{B} + \dfrac{1}{T_0}} - 273.15$$

**[VERIFY against the Grove temperature module schematic: which leg the NTC is on and the reference resistor.]**

**MMA7660FC (optional).** 6-bit signed output, ±1.5 g range, ≈ 21.3 LSB/g **[VERIFY datasheet]**. Tilt: $\theta = \text{atan2}\big(a_x,\ \sqrt{a_y^2 + a_z^2}\big)$.

### 3.7 Virtual vehicle and obstacle (for AEB in MIL)

Wheel speed to vehicle speed: $v = \dfrac{2\pi}{60}\,\omega_{rpm}\,r_w$ (choose $r_w$ as a model constant, e.g. 0.03 m). Relative distance:

$$\dot d = v_{obs} - v, \qquad TTC = \frac{d}{v - v_{obs}}\ \ (\text{only if } v > v_{obs}),\qquad a_{req} = \frac{(v - v_{obs})^2}{2d}$$

On the bench the real card movement replaces $d(t)$; in MIL, `aeb_scenario.m` generates $d(t)$ profiles (constant approach, accelerating approach, sensor dropout).

## 4. Simulink model structure (`plant/motor_plant.slx`)

```
duty_pct ──► [Gain V_bat/100] ──► [Dead zone d_dz] ──► [TF K/(τs+1)] ──► [Transport delay t_d]
        ──► [Integrator: counts = ω·N_rev/60] ──► [Round + Δ per Ts] ──► [Encoder estimator (same as firmware)] ──► speed_meas_rpm
```

- The **encoder estimator block is a copy of the firmware algorithm** (counts → rpm over window), so MIL includes the same quantisation.
- Discretise the plant at $T_s/10$ (sub-stepping) or use a continuous block with a fixed-step solver and a rate transition. Document the choice.

## 5. MATLAB interfaces (create these functions)

```matlab
function params = init_params()
% Returns struct of all P-xxx parameters (single-precision where used in code-gen).

function [t, duty_pct, dN] = load_step_log(csvPath, Nrev, Ts)
% Reads CSV (t_ms,duty_permille,dN). Returns SI-time vector and duty in %.

function rpm = counts_to_rpm(dN, Nrev, Tw)
% rpm = 60*dN/(Nrev*Tw).

function [K, tau, td, fitPct, info] = identify_motor_step(t, y_rpm, duty_pct, opts)
% Nonlinear LS fit of the first-order + delay model (3.5). opts.dz = dead zone (%).
% info: residuals, autocorrelation, parameter covariance estimate.

function [a, b, tauHat, Khat] = identify_arx_first_order(y, u, Ts, nd)
% Discrete LS fit (3.5). nd = delay in samples.

function [K, dz, R2] = fit_static_map(duty_pct, rpm_ss)
% Linear regression above the dead zone (3.3).

function s = sensor_stats(x, label)
% mean, std, min, max, histogram, normality hint (for ADC, ToF, temperature).
```

## 6. Deliverables

- `03_simulink/data/` raw CSV files (never edit raw data), `README` describing each run (date, $V_{bat}$, temperature).
- `03_simulink/data/init_params.m` with $K$, $\tau$, $t_d$, $d_{dz}$, $N_{rev}$, $V_{bat}$, sensor σ's.
- `03_simulink/plant/motor_plant.slx`, `aeb_scenario.m`.
- `07_docs/lessons_learned.md` entry for Stage 2.
- Updated parameter registry (`PRODUCT_SPEC.md` §4) and change log.

## 7. Acceptance criteria

| ID | Criterion | Evidence |
| :--- | :--- | :--- |
| A2-1 | NRMSE fit ≥ **90 %** on the held-out validation step (not used for fitting) | plot + number |
| A2-2 | $K$ and $\tau$ repeatable across 3 runs within ±10 % | table |
| A2-3 | $K$ across 3 amplitudes within ±15 % (or nonlinearity modelled) | table |
| A2-4 | Residual autocorrelation within 99 % band for lags ≥ 1, or the structure is explained | plot |
| A2-5 | Peak speed error model vs. measurement ≤ 10 % of step height (SPD-007) | plot |
| A2-6 | M1–M10 recorded (those relevant to hardware present) | log |
| A2-7 | Joystick, ToF noise σ and ranges recorded; joystick diagonal-push residual $e$ and X-only fault residual measured, $\Delta_{pl}$ chosen so fault residual / $\Delta_{pl}$ ≥ 2 and false-alarm rate on diagonal pushes ≈ 0; $d_h$ candidates proposed | `sensor_stats` output |
| A2-8 | `init_params()` runs with no [TBD] left for $K$, $\tau$, $d_{dz}$, $N_{rev}$ | script |

## 8. Expected issues

| ID | Issue | Why | Mitigation |
| :--- | :--- | :--- | :--- |
| ISS-2-01 | Step response is not a clean exponential (starts slowly, "S" shape) | Static friction / dead zone; backlash in the gearbox | Step *above* the dead zone; include $t_d$; add the static map |
| ISS-2-02 | $K$ drifts between runs | Battery sag, motor warm-up | Fresh cells, rest between runs, log $V_{bat}$, normalise by measured voltage |
| ISS-2-03 | Speed trace very noisy | Encoder quantisation at 10 ms (§3.4) | Longer window / moving average; **account for its delay** |
| ISS-2-04 | Encoder miscounts at high speed | Missed edges (noise), TIM filter too low | Set input capture filter (ICxF), shorten wires, check power |
| ISS-2-05 | UART logging causes timing jitter or lost samples | Blocking transmit inside the sample loop | Ring buffer + drain in the main loop; log at 100 Hz max |
| ISS-2-06 | Dead time seems negative | Sampling alignment between command and encoder window | Define consistently: command at start of period $k$, count delta over period $k$; check $n_d$ ≥ 0 |
| ISS-2-07 | Fit looks great but validation fails | Over-fitting or non-stationary hardware | Use validation set strictly; reduce parameters |
| ISS-2-08 | ToF reads wrong on shiny/dark/angled targets | Reflectivity, multipath, ambient IR | Standardise target (white card), record per-surface σ |
| ISS-2-09 | ToF distance has an offset | Sensor offset calibration | Calibrate with a ruler at known distances (e.g. 100/200/300 mm); store offset |
| ISS-2-10 | Joystick centre drifts, axes not equal, diagonal gate limits each axis | Cheap potentiometer, square/round gate | Calibrate centre and dead band at start; take $a_{max}$ from *diagonal* full pushes (not single-axis); set $\Delta_{pl}$ from measured data (D-004) |
| ISS-2-11 | Plant parameters differ for reverse direction | Asymmetric friction | Model forward only for this project (single-direction throttle); document |
