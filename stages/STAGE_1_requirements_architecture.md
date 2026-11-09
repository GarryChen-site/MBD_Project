# Stage 1 — Requirements & Architecture

| | |
| :--- | :--- |
| **Goal** | Turn the draft requirements into a reviewed baseline, and fix the architecture, signal list, pin map and timing/bus budgets that the later stages depend on. |
| **Entry** | Stage 0 `PASS` (or at least T0-5…T0-9 so pin choices are proven). |
| **Exit** | Requirements `Baselined`, signal list + pin map complete, all budgets in §3 close with margin. |
| **Maps to** | Blueprint Phase 1. |

---

## 1. Tasks

| # | Task | Output |
| :-: | :--- | :--- |
| 1.1 | Answer the open questions in `requirements.md` and fill the [TBD]s that do not need measurement | updated requirements |
| 1.2 | Review each requirement against the *requirement quality checklist* (§2) | review log |
| 1.3 | Baseline requirements (v1.0), tag `req-v1.0` | git tag |
| 1.4 | Create `02_architecture/signal_list.md` (from `PRODUCT_SPEC.md` §5) | file |
| 1.5 | Create `02_architecture/pin_map.md` (§4) after verifying every AF in DS10693 | file |
| 1.6 | Create `sw_architecture.md`: layers, tasks, mode manager, data flow | file |
| 1.7 | Compute budgets: task utilisation, CAN load, FTTI vs detection time (§3) | `budgets.md` |
| 1.8 | Finish HARA-lite (ASIL as discussion, FTTI numeric) | `hara_lite.md` |
| 1.9 | Draft the DBC skeleton so Stage 5 and 6 have a fixed contract | `05_can/vehicle.dbc` |

## 2. Requirement quality checklist (use for 1.2)

Each requirement must be: **Unambiguous** (one reading), **Testable** (numeric criterion + method), **Atomic** (one "shall"), **Traceable** (ID, parent), **Feasible** on this hardware, **Necessary** (links to a use case or safety goal). Reject words like "fast", "robust", "appropriate". Typical fixes:

| Weak | Better |
| :--- | :--- |
| "The ECU shall respond quickly to the pedal." | "When the pedal request steps, the speed shall reach 90 % of the target within 0.35 s." |
| "The ECU shall handle sensor faults." | "When the pedal channel difference exceeds Δ for N ticks, the ECU shall set `F_PEDAL_PLAUS` and limit speed to $\omega_{limp}$." |

## 3. Budgets and the formulas behind them

### 3.1 CPU utilisation

For periodic tasks with worst-case execution time $C_i$ and period $T_i$:

$$U = \sum_i \frac{C_i}{T_i}$$

Cooperative scheduling with tasks that run to completion: require $U \le 0.5$ initially, and the longest task $\max C_i <$ the shortest period (here the 1 ms task). The rate-monotonic bound $n(2^{1/n}-1)$ (for $n=5$: 0.743) is the theoretical limit for preemptive scheduling; staying at 0.5 leaves margin for jitter.

**Starting estimates [TBD, measured in Stage 4]:**

| Task | $T_i$ | $C_i$ (guess) | $C_i/T_i$ |
| :--- | :---: | :---: | :---: |
| 1 ms | 1 ms | 0.03 ms | 0.030 |
| 10 ms | 10 ms | 0.8 ms | 0.080 |
| 20 ms | 20 ms | 0.4 ms | 0.020 |
| 50 ms (I2C) | 50 ms | 2.5 ms | 0.050 |
| 100 ms | 100 ms | 0.5 ms | 0.005 |
| | | **U** | **0.185** |

### 3.2 CAN bus load

Worst-case bits of a standard-ID data frame with $n$ data bytes (including inter-frame space and worst-case bit stuffing):

$$B(n) = 8n + 47 + \left\lfloor \frac{34 + 8n - 1}{4} \right\rfloor$$

For $n=8$: $64 + 47 + 24 = 135$ bits. At 500 kbit/s each bit is 2 µs, so a frame takes at most 270 µs.

$$\text{Load} = \frac{\sum_m B(n_m)\,/\,T_m}{\text{bitrate}}$$

| Message | $n$ | $B(n)$ | $T_m$ | bits/s |
| :--- | :-: | :-: | :-: | ---: |
| `0x100` Pedal_Cmd | 4 | 47+32+16 = 95 | 20 ms | 4750 |
| `0x200` Drive_Status | 7 | 47+56+22 = 125 | 20 ms | 6250 |
| `0x201` Diag_Status | 5 | 47+40+18 = 105 | 100 ms | 1050 |
| `0x210` AEB_Status | 5 | 105 | 50 ms | 2100 |
| `0x123` Heartbeat | 5 | 105 | 500 ms | 210 |
| **Total** | | | | **14 360** |

Load $= 14\,360 / 500\,000 = 2.9\,\%$. Requirement: ≤ 30 % (leaves room for diagnostics and test traffic).

(Check the floor term: $n=4$: $\lfloor(34+32-1)/4\rfloor=\lfloor 16.25 \rfloor = 16$ ✓; $n=7$: $\lfloor 89/4 \rfloor = 22$ ✓; $n=5$: $\lfloor 73/4\rfloor = 18$ ✓.)

### 3.3 Fault tolerant time interval (FTTI) check

For each safety goal, the chain must satisfy:

$$T_{detect} + T_{debounce} + T_{react} \le \text{FTTI}$$

Example, pedal plausibility (SG-01): $T_{detect} = T_s = 10$ ms, debounce $= N_{pl}T_s = 50$ ms, reaction (PID brings speed to $\omega_{limp}$) $\approx 4\lambda$. Choose FTTI [TBD, e.g. 500 ms] and verify in Stage 6.

### 3.4 CAN timeout detection time

Timeout $T_{to} = 3 T_{cycle}$ with the monitor running every $T_{mon}$:

$$T_{detect,max} = T_{to} + T_{mon}$$

For `0x100`: $60 + 20 = 80$ ms (requirement CAN-006). Used as the pass bound in Stage 6.

## 4. Pin map (draft — you must verify each alternate function in DS10693)

| Function | Pin | Peripheral / AF | Nucleo header note | Used in |
| :--- | :--- | :--- | :--- | :---: |
| Joystick X | PA0 | ADC1_IN0 | A0 | 0, 3 |
| Joystick Y | PA1 | ADC2_IN1 | A1 | 0, 3 |
| Grove temp (UC4) | PA4 | ADC1_IN4 | A2 | opt |
| Encoder A / B | PA6 / PA7 | TIM3_CH1 / CH2 (AF2) | D12 / D11 | 0, 2 |
| Motor PWM | PA8 | TIM1_CH1 (AF1) | D7 | 0, 3 |
| Motor IN1 / IN2 | PB0 / PB1 | GPIO out | A3 / — | 0 |
| Motor STBY | PC0 | GPIO out | A5 | 0 |
| Servo PWM | PB6 | TIM4_CH1 (AF2) | D10 | 0 |
| I2C SCL / SDA | PB8 / PB9 | I2C1 (AF4) | D15 / D14 | 0 |
| CAN TX / RX | PA12 / PA11 | CAN1 (AF9) | CN10 pins 12 / 14 | 0, 5 |
| Log UART | PA2 / PA3 | USART2 (ST-LINK VCP) | — | all |
| ESP32 UART (opt) | PA9 / PA10 | USART1 | D8 / D2 | opt |
| LD2 | PA5 | GPIO out | — | all |
| Scheduler tick | — | TIM6 (1 kHz) or SysTick | — | 4, 5 |

**Check list for pin conflicts:** no pin used twice; PA13/PA14 (SWD) untouched; PB3 (SWO) free; timer channels do not share a counter with conflicting frequency (TIM1 = 20 kHz, TIM4 = 50 Hz, TIM3 = encoder, separate counters ✓). Some header labels (D-numbers) are from memory **[VERIFY against UM1724]**.

## 5. Software architecture deliverable

`sw_architecture.md` must contain:

1. **Layer diagram** (App / RTE / BSW / MCAL).
2. **Task table**: name, period, priority/order, inputs, outputs, WCET budget.
3. **Mode manager state machine** (Init → Normal → Degraded → Safe), transitions with conditions and requirement IDs.
4. **Interface list**: every function signature of `Rte_*`, `Com_*`, `Dem_*`, `Mcal_*` (start from the prototypes in `PRODUCT_SPEC.md` §5.4 and Stage 0 §4).
5. **Data ownership**: who writes each signal (single writer rule).

## 6. Deliverables

- `01_requirements/requirements.md` v1.0 (baselined), `hara_lite.md` v1.0, `traceability.md` skeleton.
- `02_architecture/{signal_list,pin_map,sw_architecture,budgets}.md`.
- `05_can/vehicle.dbc` (skeleton, all Must messages).

## 7. Acceptance criteria

| ID | Criterion | Evidence |
| :--- | :--- | :--- |
| A1-1 | 100 % of requirements pass the quality checklist; no [TBD] remains in a `Must` requirement without a stated owner stage | review log |
| A1-2 | Every `Must` requirement has a verification method and a target stage | `requirements.md` |
| A1-3 | Pin map has zero conflicts (checked in CubeMX by importing it) | CubeMX screenshot |
| A1-4 | CPU utilisation estimate $U \le 0.5$; CAN load $\le 30\,\%$ | `budgets.md` |
| A1-5 | Each safety goal has FTTI value and a closed time chain | `hara_lite.md` |
| A1-6 | DBC loads with `cantools` without errors and lists all messages in `PRODUCT_SPEC` | command output |
| A1-7 | Single-writer rule: no signal has two producers | signal list review |

## 8. Expected issues

| ID | Issue | Why it happens | Mitigation |
| :--- | :--- | :--- | :--- |
| ISS-1-01 | Requirements keep changing | Plant behaviour unknown until Stage 2 | Baseline only the structure; keep numeric [TBD] under change control (log) |
| ISS-1-02 | KY-023 is not a true redundant pedal | Two axes of one stick | **Decided (D-004, 2026-10-12): diagonal emulation.** X→ADC1, Y→ADC2, `PedalCond` maps Y to REVERSE (100→0 %) or HALF (0–50 %). Normal = diagonal push; fault demo = X-only push. Limitation: cannot represent electrical common-cause faults (shared supply/GND). Dual-gang pot remains a later upgrade |
| ISS-1-03 | Pin mapping looks fine in the table but CubeMX shows conflicts | AF not available on that pin | Let CubeMX place the peripheral, then update the table |
| ISS-1-04 | ASIL ratings feel arbitrary | Subjective S/E/C | Treat as discussion; record your rationale in one sentence each |
| ISS-1-05 | Over-engineering the architecture | Tempting to model full AUTOSAR | Limit to the layers in §5 |
