# Requirements (Baseline v1.0)

> [!NOTE]
> Baselined at `v1.0` after Pass 2 real-hardware calibration and Stage 6 HIL verification.
> All changes and architectural decisions are logged in [`change_log.md`](change_log.md) (ASPICE-style traceability).

## Conventions

- **Format:** EARS — *"When `<trigger>`, the `<system>` shall `<response>`."*
- **ID:** `<AREA>-<NNN>`. IDs are never reused. If a requirement is dropped, mark it `Dropped`, don't delete it.
- **Level:** `SYS` = system (ECU + plant), `SW` = software/ECU function.
- **Verify:** `MIL` (Simulink), `SIL`, `PIL`, `HIL` (HIL-lite on bench), `INSP` (inspection/review), `ANA` (analysis).
- **Priority:** `M` = must (core demo), `S` = should, `C` = could (cut first if time is short).
- **Status:** `Draft` → `Reviewed` → `Baselined` → (`Verified` / `Dropped`).
- **Phase:** which phase of [`BLUEPRINT.md`](../BLUEPRINT.md) §5 first needs it, so you can scope work per phase.

Use cases: **UC1** electronic throttle → motor speed, **UC2** AEB/FCW, **UC3** CAN + diagnostics, **UC4** cabin temp (optional).

---

## 1. Throttle input (UC1)

| ID | Requirement | Level | Verify | Pri | Phase | Status |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| THR-001 | The ECU shall read the pedal emulation channels X (`ADC1_IN0`, `PA0`) and Y (`ADC1_IN1`, `PA1`) of the KY-023 at a period of **10 ms**. | SW | HIL | M | 3 | Verified |
| THR-002 | A signal-conditioning stage shall convert raw X to sensor-1 value $s_1$ (`0–100 %`, forward) and raw Y to sensor-2 value $s_2$ using calibratable centre ($c_X=2044, c_Y=2071$), dead band ($db_X=16, db_Y=22$) and diagonal-maximum values ($X_{\max}=3975, Y_{\max}=3975$); $s_2$ shall be `REVERSE` (`100→0 %`) or `HALF`-scale (`0→50 %`) as selected by a calibration parameter. | SW | MIL | M | 3 | Verified |
| THR-003 | When the sensor-2 value deviates from its expected relationship to $s_1$ (`REVERSE`: $s_2 = 100 - s_1$; `HALF`: $s_2 = 0.5\,s_1$) by more than **15.72 %** (in pedal-axis units) for longer than **50 ms** (5 ticks), the ECU shall flag a pedal plausibility fault (`F_PEDAL_PLAUS`). | SW | MIL, HIL | M | 3 | Verified |
| THR-004 | When a pedal plausibility fault is active, the ECU shall limit the speed request to a limp-home value of **30 rpm**. | SW | MIL, HIL | M | 3 | Verified |
| THR-005 | When a pedal ADC value is outside the valid range ($< 100$ or $> 3995$ counts, open or short circuit) for **50 ms** (5 ticks), the ECU shall flag a pedal range fault (`F_PEDAL_RANGE`). | SW | MIL, SIL | S | 6 | Verified |
| THR-006 | The ECU shall apply a configurable rate limit ($+400\text{ \%/s}$ rise, $-600\text{ \%/s}$ fall) and first-order low-pass filter ($\tau_f = 50\text{ ms}$) to the pedal request. | SW | MIL | S | 3 | Verified |
| THR-007 | The ECU shall derive the pedal request as the lower of $s_1$ and the back-converted sensor-2 value ($p_2$), so that a single-channel fault can never increase the request. | SW | MIL, HIL | M | 3 | Verified |
| THR-008 | The pedal calibration (centre, dead band, diagonal-maximum per axis, mode) shall be determined by a documented procedure (`s2_03_joystick_calibration.m`) and stored as calibration parameters (`params_identified.mat`). | SYS | INSP | S | 2 | Verified |

## 2. Speed control (UC1)

| ID | Requirement | Level | Verify | Pri | Phase | Status |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| SPD-001 | The ECU shall measure motor speed from the Hall encoder (`TIM3` X4 quadrature mode, $N_{\text{rev}} = 1426\text{ counts/rev}$) and compute rpm every **10 ms**. | SW | HIL | M | 4 | Verified |
| SPD-002 | The ECU shall control motor speed with a discrete PI controller ($K_p = 0.1478, K_i = 5.4218\text{ s}^{-1}$) plus dead-zone feedforward ($D_{dz} = 1.93\text{ \%}$) driving the TB6612FNG PWM duty (**20 kHz** on `TIM4_CH1 PB6`). | SW | MIL, HIL | M | 3 | Verified |
| SPD-003 | When the pedal request changes in a step, the speed shall settle within **$\pm 2\text{ \%}$** (plus 1 encoder LSB quantization) of the target in less than **0.50 s**, with overshoot below **5 %** and steady-state error below **2 rpm**. | SYS | MIL, HIL | M | 3 | Verified |
| SPD-004 | The controller shall include back-calculation anti-windup ($K_b = 1/\tau = 36.68\text{ s}^{-1}$) so that the integrator does not accumulate while the output is saturated. | SW | MIL, SIL | M | 3 | Verified |
| SPD-005 | The ECU shall limit the PWM duty to a configurable maximum (**50 %**) to stay within the 3 V motor rating on a 4×AA (6 V) battery supply. | SW | MIL, HIL | M | 4 | Verified |
| SPD-006 | When the measured speed deviates from the request by more than **30 rpm** for longer than **0.50 s** (50 ticks), the ECU shall flag a speed-tracking fault (`F_SPD_TRACK`, e.g. stalled motor). | SW | MIL | S | 6 | Verified |
| SPD-007 | The plant model ($K = 3.074\text{ rpm/\%}, \tau = 0.0273\text{ s}, D_{dz} = 1.93\text{ \%}$) shall reproduce the measured step response of the real motor with NRMSE fit $\ge 90\text{ \%}$ (achieved **95.2 %** analytically, **89.7 %** full Simulink plant with quantization, **96.6 %** closed-loop HIL). | SYS | ANA, HIL | M | 2 | Verified |

## 3. AEB / FCW (UC2)

| ID | Requirement | Level | Verify | Pri | Phase | Status |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| AEB-001 | The ECU shall process the target distance measurement ($d_{\text{mm}}$) at a period of **10 ms** (with CAN `AEB_Status` broadcast every **50 ms**). | SW | MIL | S | 3 | Verified |
| AEB-002 | When the distance falls below the warning threshold **300 mm**, the ECU shall enter state `WARNING` (`aeb_state = 1`) and send a warning signal on CAN. | SW | MIL | S | 3 | Verified |
| AEB-003 | When the distance falls below the braking threshold **150 mm**, the ECU shall enter state `BRAKING` (`aeb_state = 2`), override the speed request to **0 rpm** and command the brake actuator to **60°**. | SW | MIL | S | 3 | Verified |
| AEB-004 | When the distance stays above the warning threshold plus hysteresis (**350 mm**) for at least **5 cycles**, the ECU shall return to state `NORMAL` (`aeb_state = 0`). | SW | MIL | S | 3 | Verified |
| AEB-005 | When the distance sensor reports invalid data (`tof_ok = 0`) for **4 cycles**, the ECU shall flag a sensor fault (`F_TOF_INVALID`) and shall not enter `BRAKING` based on that data. | SW | MIL | S | 6 | Verified |
| AEB-006 | The AEB state machine shall be implemented with deterministic state transitions (`NORMAL`, `WARNING`, `BRAKING`) covered 100 % by automated MIL tests (`TC-AEB-001`…`003`). | SW | MIL | S | 3 | Verified |
| AEB-007 | The AEB function may be verified via MIL reference simulation (`aeb_ref.m`) if physical VL53L0X hardware is omitted on the bench (`D-002`). | SYS | INSP | C | 5 | Verified |

## 4. CAN communication (UC3)

| ID | Requirement | Level | Verify | Pri | Phase | Status |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| CAN-001 | The ECU shall communicate on CAN 2.0A at 500 kbit/s (sample point 87.5 %, $BRP=2, TS1=13, TS2=2$ at 16 MHz PCLK1). | SYS | INSP, HIL | M | 5 | Verified |
| CAN-002 | The ECU shall transmit status frames (`0x200 Drive_Status` at 20 ms, `0x201 Diag_Status` at 100 ms, `0x210 AEB_Status` at 50 ms, `0x123 ECU_Heartbeat` at 500 ms) cyclically with the periods defined in `05_can/vehicle.dbc`. | SW | SIL, HIL | M | 5 | Verified |
| CAN-003 | The ECU shall receive command frames (`0x100 Pedal_Cmd` at 20 ms) defined in the DBC and unpack them via generated BSW pack/unpack functions. | SW | SIL, HIL | M | 5 | Verified |
| CAN-004 | Every cyclic E2E-protected frame shall carry a 4-bit rolling counter (`msg_counter`) and a **CRC-8 SAE J1850** checksum (`msg_crc`, poly `0x1D`, init `0xFF`, xorout `0xFF`, data-ID = low byte of CAN ID). | SW | SIL | S | 5 | Verified |
| CAN-005 | When a received frame has an invalid counter or checksum, the ECU shall discard it and increment `rx_err_count`. | SW | SIL | S | 6 | Verified |
| CAN-006 | When a required command frame is not received within $3\times$ its period (**60 ms**), the ECU shall flag a CAN timeout condition (`can_age_ms > 60 ms`) within **70 ms** and transition to degraded/limp-home mode. | SW | MIL, SIL | M | 5 | Verified |
| CAN-007 | When the CAN controller enters bus-off, the ECU shall support automatic bus-off recovery (`ABOM = ENABLE`). | SW | INSP | C | 6 | Verified |
| CAN-008 | The signal definitions (ID, start bit, length, scaling, units, cycle time) shall be maintained in one `.dbc` file (`05_can/vehicle.dbc`), which is the single source of truth for code generation (`05_can/scripts/dbc_codegen.py`). | SYS | INSP | M | 5 | Verified |

## 5. Diagnostics (UC3)

| ID | Requirement | Level | Verify | Pri | Phase | Status |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| DIA-001 | The ECU shall maintain a 16-bit fault bitmap (`fault_bitmap`) with bit 0 `F_PEDAL_PLAUS` and bit 1 `F_PEDAL_RANGE` in the generated ASW, plus diagnostic supervision for CAN timeout, ToF validity, and speed tracking. | SW | MIL, SIL, HIL | M | 5 | Verified |
| DIA-002 | Each fault shall have a configurable set/clear debounce counter before it latches active or recovers (`N_pl=5`, `N_range=5`, `N_clear=100` ticks in `init_params.m`). | SW | MIL, HIL | M | 5 | Verified |
| DIA-003 | The ECU shall report the active `fault_bitmap` and `mode` (`0=NORMAL, 1=DEGRADED/LIMP`) in UART CSV telemetry and in `0x201 Diag_Status`. | SW | MIL, SIL, HIL | M | 5 | Verified |
| DIA-004 | The ECU shall store latched fault history in RAM (`stored_fault_bitmap` in `can_telemetry.c`) until reset. | SW | SIL, INSP | C | 6 | Verified |
| DIA-005 | The ECU shall support diagnostic status reporting over UART CSV telemetry, CAN `0x201 Diag_Status`, and UDS `0x22 ReadDataByIdentifier` (`0x7E0 -> 0x7E8`). | SW | SIL, HIL | C | 6 | Verified |

## 6. Safety & robustness (conceptual, ISO 26262-inspired)

| ID | Requirement | Level | Verify | Pri | Phase | Status |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| SAF-001 | The ECU shall monitor periodic 10 ms task execution (`HAL_GetTick`) and support independent watchdog supervision. | SW | INSP, HIL | M | 5 | Verified |
| SAF-002 | After a reset or before control starts, the ECU shall initialize outputs in a safe state (`duty_cmd_pct = 0 %`, `App_Init()` zeroes all states). | SW | MIL, HIL | M | 5 | Verified |
| SAF-003 | When any pedal plausibility or range fault is active, the ECU shall transition to `DEGRADED/LIMP` (`mode = 1`) within 50 ms and clamp `speed_req_rpm` to **30 rpm**, returning to `NORMAL` (`mode = 0`) only after **100 consecutive fault-free ticks** (`1.0 s`). | SW | MIL, SIL, HIL | M | 5 | Verified |
| SAF-004 | The ECU shall measure the worst-case execution time (WCET) of every 10 ms step using the Cortex-M4 hardware cycle counter (`DWT->CYCCNT` in `prof.h`). | SW | HIL | S | 4 | Verified |
| SAF-005 | The hardware design shall keep the 6 V motor battery supply (`VM`) isolated from the 3.3 V logic supply (`VCC`), sharing common ground only. | SYS | INSP, HIL | M | 1 | Verified |

## 7. Process & non-functional

| ID | Requirement | Level | Verify | Pri | Phase | Status |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| PRC-001 | Every requirement shall be traced to at least one design element and one test in `01_requirements/traceability.md`, verified automatically by `06_test/hil/trace_check.py`. | SYS | INSP | M | 7 | Verified |
| PRC-002 | The generated code (`ctrl_codegen.c`) shall pass back-to-back equivalence (MIL vs SIL vs on-target HIL) within **$1\times 10^{-4}$** tolerance on float outputs and **0** difference on integer/enum outputs across all 5 verification scenarios (`7,555` ticks total). | SW | SIL, HIL | M | 4 | Verified |
| PRC-003 | The step execution time of the generated code shall be below **20 %** of the 10 ms task period ($\le 2000\text{ }\mu\text{s}$), profiled on the STM32F446RE (achieved **75–77 µs**, or **0.77 %** CPU load). | SW | HIL | M | 4 | Verified |
| PRC-004 | The generated code and handwritten wrapper shall comply with embedded static hygiene rules (`0` `double`, `0` dynamic allocation `malloc`/`free`, single-precision `float` literals). | SW | INSP | S | 4 | Verified |
| PRC-005 | The Simulink models (`motor_plant.slx`, `speed_ctrl.slx`, `ctrl_codegen.slx`) shall be built reproducibly from version-controlled MATLAB scripts (`s2_05_build_plant.m`, `s3_01_build_speed_ctrl.m`, `s4_01_build_codegen_model.m`) with explicit `single`/`uint16`/`uint8` port types and 10 ms discrete sample time. | SW | INSP | S | 3 | Verified |
| PRC-006 | The project shall be reproducible by a reviewer from `07_docs/reproduce.md` in under 1 hour. | SYS | INSP | M | 7 | Verified |
| TLM-001 | The UART/CAN telemetry path shall be read-only from the control loop's perspective and shall not block the 10 ms step budget. | SYS | HIL | C | — | Verified |

---

## Summary

| Area | Count | Must | Should | Could | Verified |
| :--- | :---: | :---: | :---: | :---: | :---: |
| THR | 8 | 5 | 3 | 0 | 8 / 8 |
| SPD | 7 | 6 | 1 | 0 | 7 / 7 |
| AEB | 7 | 0 | 6 | 1 | 7 / 7 |
| CAN | 8 | 5 | 2 | 1 | 8 / 8 |
| DIA | 5 | 3 | 0 | 2 | 5 / 5 |
| SAF | 5 | 4 | 1 | 0 | 5 / 5 |
| PRC/TLM | 7 | 4 | 2 | 1 | 7 / 7 |
| **Total** | **47** | **27** | **15** | **5** | **47 / 47 (100 %)** |

## Resolved Open Questions

1. **Target speed unit:** Output shaft speed in rpm ($N_{\text{rev}} = 1426\text{ counts/rev}$ for the 50.9:1 N20 gearmotor, $\omega_{\max} = 147.75\text{ rpm}$ at 50 % duty cap).
2. **Pedal channel semantics (D-004, 2026-10-12):** KY-023 diagonal emulation (`PA0 = ADC1_IN0`, `PA1 = ADC1_IN1`), calibrated threshold $D_{\text{pl}} = 15.72\text{ \%}$ with $\text{sep} = 5.82$.
3. **Final CAN checksum (D-006, 2026-10-05):** CRC-8 SAE J1850 (`0x1D`, init `0xFF`, xorout `0xFF`) + 4-bit alive counter.
4. **AEB scope (D-002):** Full AEB state machine verified in MIL (`aeb_ref.m`, `TC-AEB-001`…`003`) and CAN `0x210 AEB_Status` packer verified in Stage 5 (`com_can_signals.c`).
