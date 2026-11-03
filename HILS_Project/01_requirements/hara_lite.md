# HARA-lite (Hazard Analysis & Risk Assessment, simplified)

> [!NOTE]
> Conceptual exercise in the style of ISO 26262 Part 3. It is **not** a certified assessment. Its purpose is to
> show the engineering reasoning: **Hazard → Safety Goal → Safe State + FTTI → Derived Technical Requirements → Verification**.

## Method

Rate each hazardous event with **S**everity (`S0`–`S3`), **E**xposure (`E0`–`E4`), **C**ontrollability (`C0`–`C3`), then look up the illustrative ASIL (`QM`, `A`, `B`, `C`, `D`) per ISO 26262-3 Table 4.

## Hazardous events (Baseline v1.0)

| ID | Malfunction | Situation | Hazardous event | S | E | C | ASIL (illustrative) | Safety goal | Rationale |
| :--- | :--- | :--- | :--- | :-: | :-: | :-: | :---: | :--- | :--- |
| H-01 | Unintended acceleration (pedal sensor single-axis fault or software error) | Urban / highway driving | Vehicle accelerates without driver request | S3 | E4 | C2 | **ASIL C** | SG-01 | High exposure in normal driving; controllable by braking (`C2`) but potentially severe (`S3`). |
| H-02 | Loss of drive torque or stalled motor | Overtaking / junction | Unexpected loss of propulsion or thermal stall | S2 | E3 | C2 | **ASIL A** | SG-02 | Moderate exposure and controllable by steering to shoulder, or thermal risk on stalled DC motor. |
| H-03 | Unwanted AEB braking on invalid ToF sensor | Highway | Sudden deceleration, rear-end collision risk | S3 | E3 | C2 | **ASIL B** | SG-03 | False-positive emergency braking at speed creates rear-impact hazard. |
| H-04 | AEB does not brake when obstacle is within 150 mm | Approaching obstacle | Forward collision | S3 | E3 | C3 | **ASIL C** | SG-04 | Driver may not react in time (`C3`) when obstacle is below braking distance. |
| H-05 | Stale or corrupted CAN pedal command accepted | Remote command mode | Wrong speed request applied | S2 | E3 | C2 | **ASIL A** | SG-01 | Corrupted/frozen bus frame could hold throttle open without E2E/timeout check. |

## Safety goals & FTTI Time-Chain Verification

$$\text{FTTI} \ge T_{\text{detect}} + T_{\text{debounce}} + T_{\text{react}}$$

| SG | Goal | Safe state | FTTI | $T_{\text{detect}} + T_{\text{debounce}}$ | $T_{\text{react}}$ (Measured) | Derived requirements | Verification |
| :--- | :--- | :--- | :---: | :---: | :---: | :--- | :--- |
| **SG-01** | Prevent unintended acceleration | `DEGRADED/LIMP` ($\omega_{\text{req}} \le 30\text{ rpm}$) or `0 rpm` | **500 ms** | $10\text{ ms} + 50\text{ ms} = 60\text{ ms}$ (`F_PEDAL_PLAUS`)<br>$10\text{ ms} + 50\text{ ms} = 60\text{ ms}$ (`F_PEDAL_RANGE`)<br>$60\text{ ms} + 10\text{ ms} = 70\text{ ms}$ (`F_CAN_TIMEOUT`) | $\approx 4\tau = 110\text{ ms}$ | THR-003, THR-004, THR-005, THR-007, CAN-004, CAN-005, CAN-006, SAF-003 | `TC-THR-002`, `TC-THR-003a/b`, `TC-THR-004`, `TC-SAF-001`, `S2_XOnly_Fault_Recovery`, `hil_closed_loop.csv` |
| **SG-02** | Prevent thermal damage / uncontrolled stall | Safe duty clamp ($\text{PWM} \le 50\text{ \%}$ / $0\text{ \%}$) + fault flag | **600 ms** | $10\text{ ms} + 500\text{ ms} = 510\text{ ms}$ (`F_SPD_TRACK`) | $10\text{ ms}$ (immediate PWM cut) | SPD-005, SPD-006, SAF-001, SAF-002, SAF-003 | `TC-SPD-003`, `A6-2` |
| **SG-03** | Prevent false AEB braking on invalid ToF sensor | `DEGRADED` (`F_TOF_INVALID`, AEB `BRAKING` inhibited) | **100 ms** | $10\text{ ms} + 40\text{ ms} = 50\text{ ms}$ (`F_TOF_INVALID`) | $0\text{ ms}$ (inhibited same tick) | AEB-005, SAF-003 | `TC-AEB-003` |
| **SG-04** | Brake when collision is imminent ($d < 150\text{ mm}$) | `BRAKING` ($\omega_{\text{req}} = 0\text{ rpm}$, $\theta_{\text{servo}} = 60^\circ$) | **150 ms** | $30\text{ ms}$ (`N_deb = 3` steps) | $\approx 110\text{ ms}$ (motor decel) | AEB-002, AEB-003, AEB-004, AEB-006 | `TC-AEB-001`, `TC-AEB-002` |

## Architectural Limitations (Transparent Engineering Notes)

1. **Single MCU without lockstep core:** The STM32F446RE is a single Cortex-M4F MCU without a lockstep checker core or external safety PMIC. Hardware diagnostic coverage is therefore limited to software plausibility checks, DWT execution-time monitoring, and the independent watchdog (IWDG).
2. **KY-023 shared power/ground:** Because the KY-023 joystick shares a single `3.3V` and `GND` header across its X and Y potentiometers, it emulates independent mechanical wipers (`PA0` and `PA1`) and catches single-wire open/short and mechanical axis mismatch, but cannot physically isolate a common-cause `VCC` droop without a dual-supply automotive pedal sensor.
