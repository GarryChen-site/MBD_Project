# Stage 0 — Setup & Peripheral Bring-up

| | |
| :--- | :--- |
| **Goal** | Prove that the toolchain and every piece of hardware works *individually*, before any integration. |
| **Entry** | Hardware on the bench, MATLAB add-ons installed (checked in `BLUEPRINT.md` §5b). |
| **Exit** | All tests T0-1 … T0-9 `PASS`, evidence stored in `06_test/reports/stage0/`. |
| **Maps to** | Blueprint Phase 0. Requirements: SAF-005, CAN-001 (partial). |

---

## 1. Tasks

| # | Task | Output |
| :-: | :--- | :--- |
| 0.1 | Verify MATLAB: `ver`, support package installed, toolchain configured | `ver.txt` |
| 0.2 | Simulink → NUCLEO blinky (Embedded Coder + STM32 support package) | model + photo/video |
| 0.3 | CAN loopback + real bus (follow `Lab17_CAN/HAL/README.md`) | log files |
| 0.4 | Bring up each peripheral with a **tiny standalone CubeIDE test** (reuse earlier Labs where they exist) | one folder per test |
| 0.5 | Wire the power domains and ground (see `data_control_hardware_flow.md` §6) | photo + notes |
| 0.6 | Create the repo hygiene: `.gitignore`, tags, folder layout | commit |

## 2. Peripheral bring-up tests (the "smoke tests")

| Test | What | Pass criterion | Reuse |
| :--- | :--- | :--- | :--- |
| **T0-1** | Simulink blinky | LD2 toggles at 1 Hz ±5 % (scope or stopwatch over 60 s) | — |
| **T0-2** | UART log via ST-LINK VCP | 115200 8N1, 1000 lines without garbled characters | Lab 11/16 |
| **T0-3** | CAN loopback | `SELFTEST PASS`, 100 frames sent/received, 0 mismatches | Lab 17 |
| **T0-4** | CAN real bus | CANable shows ≥ 120 heartbeat frames in 60 s (`500 ms` ± 2 ms jitter by PC timestamps), 0 error frames | Lab 17 |
| **T0-5** | ADC joystick (X→ADC1 PA0, Y→ADC2 PA1) | Both read 0–4095 range; a smooth diagonal push makes X and Y rise together; centre noise $\sigma \le 10$ counts (≈ 8 mV) at rest; record centres and diagonal-push maxima | Lab 14 |
| **T0-6** | Encoder (TIM3 encoder mode) | Rotate output shaft exactly 1 turn by hand → count $= N_{rev} \pm 2\,\%$ (gives P-006); direction sign correct | — |
| **T0-7** | Motor + TB6612 | 20 kHz PWM visible on scope (or audible silence); duty 0 → 50 % spins in the expected direction; STBY low stops motor | — |
| **T0-8** | I2C scan | `0x29` (VL53L0X) and `0x4C` (MMA7660) ACK; VL53L0X reads 3 distances within ±10 % of a ruler | Lab 16 I2C |
| **T0-9** | Servo sweep | SG90 moves 0 → 90 → 0° without stalling/buzzing; no MCU reset when starting the motor and servo together | — |

### TB6612FNG truth table (needed for T0-7)

| IN1 | IN2 | PWM | STBY | Result |
| :-: | :-: | :-: | :-: | :--- |
| H | L | H/PWM | H | forward (CW) |
| L | H | H/PWM | H | reverse (CCW) |
| H | H | x | H | short brake |
| L | L | H | H | stop (coast) |
| x | x | x | L | standby |

(Confirm the CW/CCW direction against your wiring of motor leads A01/A02; swap leads rather than code if it is backwards.)

## 3. Formulas used in this stage

**Timer PWM frequency** (TIM clocked at $f_{TIM}$, auto-reload ARR, prescaler PSC):

$$f_{PWM} = \frac{f_{TIM}}{(PSC+1)(ARR+1)}, \qquad \text{duty} = \frac{CCR}{ARR+1}$$

Example for P-015: $f_{TIM}=16$ MHz, PSC = 0, ARR = 799 → $f_{PWM}=20$ kHz, 800 duty steps (≈ 9.6 bits).

**Servo pulse** (50 Hz frame, tick = 1 µs → PSC = 15, ARR = 19999):

$$t_{pulse}[\mu s] = t_0 + \frac{\theta}{180}(t_{180}-t_0), \quad \text{nominally } t_0 = 1000,\ t_{180}=2000$$

SG90 units vary (some need 500–2400 µs). **Measure the end positions and store $t_0$, $t_{180}$ as calibration.** CCR = $t_{pulse}$ in µs.

**ADC conversion** (12-bit, $V_{ref}=3.3$ V): $V = \dfrac{\text{raw}}{4096}V_{ref}$, 1 LSB ≈ 0.806 mV.

**Encoder resolution** (X4 quadrature mode): $N_{rev} = 4 \cdot PPR_{motor} \cdot i_{gear}$. For 7 PPR and 1:150 → 4200 **[VERIFY with T0-6]**.

**Hardware limits** (for T0-7/T0-9): do not exceed $D_{max}=V_{rated}/V_{bat}$ on the motor (P-005).

## 4. Interfaces (introduced here, reused later)

Minimal driver prototypes created during bring-up. They become the MCAL in Stage 5, so keep these signatures.

```c
/* mcal_adc.h */
int  Mcal_Adc_Init(void);
int  Mcal_Adc_ReadPedals(uint16_t *ch0, uint16_t *ch1);      /* 0 = OK */

/* mcal_encoder.h */
int  Mcal_Enc_Init(void);
int32_t Mcal_Enc_ReadDelta(void);  /* signed counts since last call, wrap-safe */

/* mcal_pwm.h */
int  Mcal_Motor_Init(void);
void Mcal_Motor_SetDutyPermille(uint16_t duty_permille);     /* 0..1000, driver clamps to cap */
void Mcal_Motor_SetDir(int8_t dir);                          /* +1 fwd, -1 rev, 0 coast */
void Mcal_Motor_Brake(void);
void Mcal_Motor_Standby(uint8_t on);

/* mcal_servo.h */
int  Mcal_Servo_Init(void);
void Mcal_Servo_SetPulseUs(uint16_t us);

/* mcal_tof.h  (VL53L0X) */
int  Mcal_Tof_Init(void);
int  Mcal_Tof_ReadMm(uint16_t *mm);                          /* 0 = OK, <0 = bus/timeout */
```

Wrap-safe encoder delta (16-bit timer):

```c
int32_t Mcal_Enc_ReadDelta(void) {
  static uint16_t prev;
  uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  int16_t  d   = (int16_t)(now - prev);   /* two's complement handles wrap */
  prev = now;
  return d;
}
```

Valid if $|\Delta N| < 32768$ per call: at 150 rpm and 4200 cnt/rev, 10 ms → 105 counts ✓.

## 5. Deliverables

- `06_test/reports/stage0/` : `ver.txt`, T0-1 … T0-9 evidence (photos, logs).
- `04_firmware/mcal/` : the minimal drivers above (even if rough).
- Updated `BLUEPRINT.md` §5b checklist.

## 6. Acceptance criteria

| ID | Criterion | Evidence |
| :--- | :--- | :--- |
| A0-1 | T0-1 … T0-9 all `PASS` | report folder |
| A0-2 | $N_{rev}$ measured and written to P-006 | param registry |
| A0-3 | Joystick min / centre / max and noise σ recorded | CSV |
| A0-4 | Servo end positions $t_0$, $t_{180}$ recorded | calibration note |
| A0-5 | Starting motor + servo together does not reset the MCU (check reset flags) | log |
| A0-6 | CANH–CANL resistance ≈ 60 Ω (power off) | photo |

## 7. Expected issues (read before starting)

| ID | Symptom | Likely cause | What to do |
| :--- | :--- | :--- | :--- |
| ISS-0-01 | Support package installs but "Build" fails or the toolchain can't be found | STM32CubeMX/ARM GCC path not configured, or MATLAB release incompatible with the package version | Open the package's hardware setup screen, point it to CubeMX and the GNU Arm toolchain **[VERIFY exact menu]**. Fallback: set the model to *Generate code only*, copy the generated sources into a CubeIDE project |
| ISS-0-02 | Board not found by the support package on macOS | ST-LINK firmware outdated; USB cable is charge-only | Update ST-LINK via STM32CubeProgrammer; try another cable |
| ISS-0-03 | Simulink blinky works but pin differs | Package maps LED as D13/PA5 | Check the block's pin number vs. board silkscreen |
| ISS-0-04 | `gs_usb` device not found on macOS | libusb missing, or CANable has slcan firmware | `brew install libusb`; or use slcan serial port (see Lab 17 README) |
| ISS-0-05 | MCU resets when motor starts | Supply sag (RK-02), or motor spikes through shared ground | Bulk capacitor ≥ 470 µF at TB6612 VM, 100 nF across motor terminals, separate supplies |
| ISS-0-06 | Servo jitters or buzzes | Shared PWM timer, weak supply, mechanical stop | Dedicated timer channel, power from battery, limit angle range |
| ISS-0-07 | VL53L0X not found | Wrong address, SDA/SCL swapped, 5 V on a 3.3 V part, missing pull-ups | Power from 3.3 V; check module pull-ups; scan the bus |
| ISS-0-08 | Encoder counts only one direction or half the expected counts | Channel A/B swapped, wrong encoder mode (X1/X2 vs X4), motor encoder powered from wrong voltage | Swap A/B; set TI1&TI2 mode; encoder VCC per motor datasheet (3.3 V recommended) |
| ISS-0-09 | UART log corrupts when SWO/debugger attached | Wrong baud due to clock mismatch | Use the same clock config as Lab 16 (HSI 16 MHz) |
