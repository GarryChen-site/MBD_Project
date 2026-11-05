# 08 Results & Media — Real Hardware Execution Logs, Bench Photos & Videos

This directory stores the **raw physical bench execution evidence** captured from the **STM32F446RE (`NUCLEO-F446RE`)** + **N20 50:1 DC Gearmotor + Quadrature Hall Encoder** + **KY-023 Dual-Axis Analog Pedal Emulator** + **SN65HVD230 / CANable V2** test bench.

> [!TIP]
> For the MATLAB-generated verification plots (`validation.png`, `plant_validation.png`, `hw_vs_mil_hil_step_test.png`, `hw_vs_mil_hil_closed_loop.png`) and the Stage 4 SIL back-to-back report (`b2b_report.md`), see [`06_test/reports/`](../06_test/reports/) and [`03_simulink/data/real/`](../03_simulink/data/real/).

---

## 1. Directory Layout

```text
08_results/
├── README.md                                      # Index of hardware execution logs, bench photos & videos
├── logs/                                          # Raw terminal transcripts from real STM32 hardware sessions
│   ├── joystick_motor_calibration_log.txt         # Stage 2: 4 joystick calibration runs + 19 motor static/step sweeps
│   ├── closed_loop_live_log.txt                   # Stage 6: Closed-loop 6 s auto-step (75 µs WCET) & 15 s live joystick (77 µs WCET)
│   └── can_fault_capture_log.txt                  # Stage 5/6: Live CANable 500 kbit/s slcan trace + UDS 0x22 fault & WCET readout
├── photos/                                        # Physical hardware bench & wiring photos
└── videos/                                        # Closed-loop motor control & fault-injection demonstration videos
```

---

## 2. Real Hardware Terminal Logs (`logs/`)

| Log File | V-Model Stage | Interface | What It Records |
| :--- | :---: | :---: | :--- |
| **[`joystick_motor_calibration_log.txt`](logs/joystick_motor_calibration_log.txt)** | **Stage 2: System ID** | `USART2` (`115200` baud) via [`capture_log.py`](../04_firmware/experiments/capture_log.py) | Complete on-target capture session recording all **23 real-hardware identification datasets** into [`03_simulink/data/real/`](../03_simulink/data/real/):<br>• 4 dual-channel pedal sessions (`joy_rest.csv`, `joy_diag_full.csv`, `joy_diag_valid.csv`, `joy_xonly.csv`)<br>• 10 static duty-sweep runs (`static_D05.csv` … `static_D50.csv`, 271 rows each)<br>• 9 dynamic step-response runs (`step_15_r1..r3`, `step_25_r1..r3`, `step_35_r1..r3`, 351 rows each) |
| **[`closed_loop_live_log.txt`](logs/closed_loop_live_log.txt)** | **Stage 6: Closed-Loop HIL** | `USART2` (`115200` baud) via [`closed_loop_runner.c`](../04_firmware/closed_loop_runner.c) | Real-time 100 Hz closed-loop execution of Embedded Coder generated [`ctrl_codegen_step()`](../04_firmware/app_generated/ctrl_codegen.c) on the STM32F446RE:<br>• `hil_step_test.csv` (601 rows, 6.0 s autonomous 60 % step test, max WCET = **`75 µs`**, **96.1 %** NRMSE fit vs. Simulink MIL)<br>• `hil_closed_loop.csv` (1,501 rows, 15.0 s live joystick driving + single-axis `F_PEDAL_PLAUS` fault injection + `30 rpm` limp-home clamp + 1.0 s auto-recovery, max WCET = **`77 µs`**, **96.6 %** NRMSE fit vs. Simulink MIL) |
| **[`can_fault_capture_log.txt`](logs/can_fault_capture_log.txt)** | **Stage 5 & 6: CAN BSW & UDS Diagnostics** | `bxCAN1` (`500 kbit/s`) ↔ `CANable V2` via [`can_monitor.py`](../05_can/can_monitor.py) | Live 500 kbit/s CAN 2.0A bus trace verifying:<br>• Cyclic `0x200 Drive_Status` reception with valid SAE J1850 CRC-8 (`CRC:OK`) and rolling 4-bit `msg_counter`<br>• Automatic transition from `mode=NORMAL` to `mode=DEGRADED` (`req = 30.0 rpm` limp-home clamp) upon pedal fault injection<br>• Event-driven `0x201 Diag_Status` fault detection (`fault_bitmap = 0x0002`) triggering a live **UDS `0x22 ReadDataByIdentifier`** request (`0x7E0`) and ECU response (`0x7E8`) returning `DID 0xF100 (Active Faults) = 0x0002` and `DID 0xF102 (Max Step WCET) = 75 us` |

---

## 3. Bench Photos (`photos/`) & Demo Videos (`videos/`)

Place physical test-bench photographs in [`photos/`](photos/) and closed-loop demonstration recordings in [`videos/`](videos/).
