# Stage 4 — Code Generation, SIL & PIL

| | |
| :--- | :--- |
| **Goal** | Generate C code from the verified `Controller` model with Embedded Coder, prove it behaves like the model (back-to-back MIL = SIL = PIL), and prove it fits the MCU (time, memory, no `double`). |
| **Entry** | Stage 3 `PASS`, reference signals archived. Stage 0 drivers exist. |
| **Exit** | Equivalence within tolerance on all reference vectors; WCET and memory within budget; generated code integrated in the CubeIDE project and called from the 10 ms task. |
| **Maps to** | Blueprint Phase 4. Requirements: PRC-002, PRC-003, PRC-004, SAF-004, SPD-001. |

---

## 1. Tasks

| # | Task | Output |
| :-: | :--- | :--- |
| 4.1 | Configure the model for code generation (§2) via a script, not by clicking | `ert_stm32.m` |
| 4.2 | Generate code; inspect: types, names, entry points, no `double`, no dynamic memory | code + review notes |
| 4.3 | SIL: run the generated code on the PC against the same test vectors | SIL logs |
| 4.4 | Integrate into CubeIDE project via the wrapper (§4) | firmware builds |
| 4.5 | PIL on the NUCLEO (support-package PIL, or the manual UART/CAN PIL fallback, §5) | PIL logs |
| 4.6 | Back-to-back comparison and report (§3) | `reports/b2b_report.md` |
| 4.7 | Execution time, stack, flash/RAM measurement (§6) | `reports/resources.md` |
| 4.8 | Static analysis / MISRA subset on generated + handwritten code | report with deviations |

## 2. Code-generation configuration (scripted)

```matlab
% ert_stm32.m — apply once per model; keep in version control
function ert_stm32(model)
cs = getActiveConfigSet(model);
set_param(cs, 'SystemTargetFile', 'ert.tlc');            % Embedded Coder
set_param(cs, 'TargetLang', 'C');
set_param(cs, 'SolverType', 'Fixed-step');
set_param(cs, 'Solver', 'FixedStepDiscrete');
set_param(cs, 'FixedStep', 'par.Ts');                    % 0.01 s
set_param(cs, 'GenCodeOnly', 'on');                      % we build in CubeIDE
set_param(cs, 'MatFileLogging', 'off');
set_param(cs, 'SupportNonFinite', 'off');                % no rt_nonfinite.c
set_param(cs, 'CodeInterfacePackaging', 'Nonreusable function');  % single instance
set_param(cs, 'GenerateReport', 'on');
set_param(cs, 'ProdHWDeviceType', 'ARM Compatible->ARM Cortex');  % [VERIFY exact string in your release]
set_param(cs, 'ProdIntDelivery', 'off');                 % illustrative; verify names with get_param
end
```

> [!NOTE]
> Parameter *names* above are the ones I know; **verify each with `get_param(cs,'ObjectParameters')` or by changing the option in the GUI and reading the generated script** (`Configuration Parameters → save as script`). Release differences are the most common source of `set_param` errors.

**Settings to check in the GUI (equivalents):** Solver = fixed-step, discrete; Hardware Implementation = ARM Cortex-M (or the Nucleo board entry if the support package provides one); Code Generation → Interface: single-instance, no MAT-file logging; Code Generation → Code Style: fixed-width types; "Remove code that protects against division arithmetic exceptions" **off** until reviewed; "Default parameter behavior" = Inlined, with tunables as `Simulink.Parameter` (`ExportedGlobal`).

**Generated entry points** (names depend on model name and release; typical for `speed_ctrl`):

```c
void speed_ctrl_initialize(void);
void speed_ctrl_step(void);              /* one base-rate tick (10 ms); slower rates run inside */
extern ExtU_speed_ctrl_T speed_ctrl_U;   /* inputs  (older releases: rtU) */
extern ExtY_speed_ctrl_T speed_ctrl_Y;   /* outputs (older releases: rtY) */
```

Because the base sample time is 10 ms and the 20/50/100 ms rates are multiples, **call `speed_ctrl_step()` once every 10 ms**; Embedded Coder generates the sub-rate counters internally (single-tasking). Do not call it from the 1 ms task.

## 3. Back-to-back equivalence (the mathematics)

Let $y_{ref}[k]$ be the MIL output and $y_{test}[k]$ the SIL or PIL output for the **same input vector** over $N$ ticks (open-loop test: feed recorded plant outputs, not a re-simulated plant, so errors cannot be amplified by feedback).

$$e_\infty = \max_k |y_{ref}[k] - y_{test}[k]|, \qquad e_{rms} = \sqrt{\tfrac{1}{N}\sum_k (y_{ref}[k]-y_{test}[k])^2}, \qquad e_{rel} = \frac{e_\infty}{\max_k |y_{ref}[k]|}$$

**Rules per signal class**

| Signal | Criterion |
| :--- | :--- |
| `boolean`, integer, enum (`mode`, `aeb_state`, `fault_bitmap`) | **exactly equal** at every tick (any difference is a defect) |
| `single` signals | $|y_{ref}-y_{test}| \le \max(\epsilon_{abs},\ \epsilon_{rel}|y_{ref}|)$ with $\epsilon_{abs}=10^{-4}$ (signal unit), $\epsilon_{rel}=10^{-4}$ |

**Why $10^{-4}$:** single precision has unit roundoff $u = 2^{-24} \approx 6\times10^{-8}$. Worst-case accumulation in an integrator over $N$ steps is $\approx N u$ (for $N=2000$: $1.2\times10^{-4}$); the random-walk estimate is $\sqrt N\,u \approx 2.7\times10^{-6}$. Differences between compilers (fused multiply-add, math library) are typically 1–few ULP. A tolerance of $10^{-4}$ is therefore conservative; investigate anything larger than $10^{-5}$ even if it passes.

```matlab
function report = back_to_back_compare(y_ref, y_test, absTol, relTol)
% Inputs: timetables or matrices (N x nSignals) on the same time base.
% report.max_abs, report.rms, report.max_rel (per signal), report.pass (logical),
% report.first_violation (tick index or []), report.table (printable summary).
```

**Test vectors:** all MIL tests of Stage 3 §6 plus a 60 s random scenario (pedal, distance, CAN age) with fixed random seed. For coverage, optionally compare code coverage of the generated code (Embedded Coder / `gcov` for SIL) against model coverage: uncovered code is either dead code or a missing test.

## 4. Firmware integration: the wrapper (interface)

The application never touches HAL; the wrapper translates between `Rte_*` structs (PRODUCT_SPEC §5.4) and the generated structs.

```c
/* app_wrapper.h */
#include "rte.h"

void App_Init(void);                                   /* once at start, after BSW init     */
void App_Step10ms(const Rte_AppIn_t *in,               /* called from Task_10ms             */
                  Rte_AppOut_t *out);

/* app_wrapper.c (essence) */
void App_Init(void) { speed_ctrl_initialize(); }

void App_Step10ms(const Rte_AppIn_t *in, Rte_AppOut_t *out) {
  speed_ctrl_U.pedal1_adc        = in->pedal1_adc;
  speed_ctrl_U.pedal2_adc        = in->pedal2_adc;
  speed_ctrl_U.speed_meas_rpm    = in->speed_meas_rpm;
  /* ... remaining inputs ... */
  speed_ctrl_step();
  out->duty_cmd_pct  = speed_ctrl_Y.duty_cmd_pct;
  out->servo_cmd_deg = speed_ctrl_Y.servo_cmd_deg;
  /* ... remaining outputs ... */
}
```

Rules: (1) the wrapper has **no logic**, only copying and type casts; (2) it is the only file that includes the generated headers; (3) generated files are never edited — regenerate instead.

## 5. PIL options

**Option A — support-package PIL** (preferred): set the model's simulation mode to *Processor-in-the-Loop* (or use a PIL model block) and run the same test vectors. Embedded Coder compiles, downloads and communicates (typically over the ST-LINK virtual COM port). **[VERIFY that your support-package version provides PIL for the Nucleo-F446RE and which COM port it uses.]**

**Option B — manual PIL over UART** (fallback; also very instructive). Protocol (little-endian, one frame per tick):

```
Request  PC→ECU:  0xA5 | len | payload = serialized Rte_AppIn_t | crc8
Response ECU→PC:  0x5A | len | payload = serialized Rte_AppOut_t | cycles(uint32) | crc8
```

```c
/* pil_link.h — build only with -DPIL_BUILD */
void PilLink_Init(UART_HandleTypeDef *huart);
void PilLink_Poll(void);   /* blocks for one request; runs App_Step10ms; sends response */
```

```matlab
function Y = pil_manual_run(port, baud, U)
% U: N x nIn matrix (one row per tick) -> Y: N x nOut plus cycles column.
% Uses serialport(); same CRC-8 as Stage 5. Resets ECU state before the run.
```

Use `uint32` cycle count from the DWT counter in the response to get per-tick execution time.

## 6. Resource and timing measurement

**Execution time** (DWT cycle counter, independent of the toolchain):

```c
static inline void Prof_Init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}
static inline uint32_t Prof_Now(void) { return DWT->CYCCNT; }

/* t_us = cycles / (SystemCoreClock / 1e6)  ->  16 cycles = 1 us at 16 MHz */
```

Measure at least 10 000 ticks; record min / mean / max. The **maximum** occurs on the tick where the 10 ms, 20 ms, 50 ms and 100 ms sub-rates coincide (every 100 ms) — make sure the test includes it.

**Stack:** paint the stack with a pattern (`0xCAFEBABE`) at start and read the high-water mark after the test. **Flash/RAM:** `arm-none-eabi-size firmware.elf`, and compare with the F446RE (512 kB flash, 128 kB RAM). **No soft-double:** `arm-none-eabi-objdump -d firmware.elf | grep -E "__aeabi_(dadd|dmul|ddiv|dsub|f2d|d2f)"` must find nothing in generated code (these are the software `double` helpers).

## 7. Interfaces added in this stage

```c
/* prof.h */
void     Prof_Init(void);
uint32_t Prof_Now(void);
uint32_t Prof_CyclesToUs(uint32_t cycles);

/* app_wrapper.h */  (see §4)
/* pil_link.h */     (see §5, optional)
```

```matlab
function ert_stm32(model)                              % §2
function report = back_to_back_compare(y_ref, y_test, absTol, relTol)   % §3
function Y = pil_manual_run(port, baud, U)             % §5 (optional)
function export_reference(modelName, outMat)           % saves inputs/outputs of MIL runs as .mat
```

## 8. Deliverables

- `03_simulink/controller/ert_stm32.m`, generated code folder (checked in as a snapshot `04_firmware/app_generated/`).
- `04_firmware/app_wrapper.{c,h}`, `prof.{c,h}`, optional `pil_link.{c,h}`.
- `06_test/sil_pil/` scripts, logs, `b2b_report.md`, `resources.md`.
- Static-analysis report, with each MISRA deviation justified.

## 9. Acceptance criteria

| ID | Criterion | Evidence |
| :--- | :--- | :--- |
| A4-1 | SIL vs MIL: integer/enum signals identical; `single` within $\epsilon$ on **all** reference vectors | `b2b_report.md` |
| A4-2 | PIL vs MIL: same criteria | `b2b_report.md` |
| A4-3 | WCET of `App_Step10ms` ≤ **2 ms** (20 % of $T_s$); mean ≤ 1 ms (PRC-003) | `resources.md` |
| A4-4 | Total CPU utilisation (all tasks, measured) $U \le 0.5$ (Stage 1 budget) | `resources.md` |
| A4-5 | Stack high-water mark ≤ 50 % of the configured stack | `resources.md` |
| A4-6 | No soft-double calls in generated code (command in §6 returns empty) | command output |
| A4-7 | No handwritten edits in `app_generated/` (git diff vs. fresh generation is empty) | CI script / manual |
| A4-8 | MISRA subset: zero *required* violations in handwritten code; deviations in generated code documented (PRC-004) | report |
| A4-9 | Build is warning-free at `-Wall -Wextra` for handwritten files | build log |

## 10. Expected issues

| ID | Issue | Why | Mitigation |
| :--- | :--- | :--- | :--- |
| ISS-4-01 | `set_param` errors or missing options | Release-dependent parameter names | Generate a script from the GUI; guard with `try/catch` and report the option name |
| ISS-4-02 | Support package generates and builds an *entire* project that conflicts with the CubeIDE one | Two toolchains competing | Use **GenCodeOnly** and integrate the sources yourself (reproducible, matches job practice) |
| ISS-4-03 | Generated code is slow (ms instead of µs) | Hidden `double` ops (constants, Math blocks, `sqrt`), soft float | Enforce `single`, use `f` suffix constants; check §6 `objdump` command; enable FPU flags `-mfpu=fpv4-sp-d16 -mfloat-abi=hard` in CubeIDE (default for F4 with FPU **[VERIFY]**) |
| ISS-4-04 | PIL mismatch only in a few samples near thresholds | Compiler differences (FMA), different rounding in comparisons | Keep thresholds away from exact equality in tests; accept ULP-level differences in `single`; never in integers/enums |
| ISS-4-05 | Name clash: generated `rtwtypes.h` vs HAL typedefs | Both define fixed-width aliases | Keep generated types in the generated headers only; wrapper maps to `stdint` |
| ISS-4-06 | PIL cannot connect (port busy / wrong baud) | ST-LINK VCP used by log UART | Free the VCP (disable logging in PIL builds) or use a second USB-UART on USART1 |
| ISS-4-07 | Sub-rate tasks fire on the wrong ticks | `step()` called from an irregular context | Call exactly once per 10 ms from the scheduler; check with GPIO toggle on a scope/logic analyser |
| ISS-4-08 | PIL and SIL timing mixed up | PIL measures MCU cycles; SIL does not | Report timing only from on-target (DWT) measurements |
| ISS-4-09 | Stateflow-generated code uses large enum/`int` sizes or uncommon constructs | Default enum storage `int` | Set enum storage to `uint8` where permitted; check MISRA deviations |
| ISS-4-10 | Tunable parameters not visible in the debugger | Inlined by default | Use `Simulink.Parameter` with `ExportedGlobal`; gives calibratable globals (preparation for XCP/calibration demo) |
| ISS-4-11 | `printf("%f")` prints nothing | newlib-nano without float printf | Add linker flag `-u _printf_float` (costs ≈ 10 kB flash) or print scaled integers |
