# 04 Firmware — STM32F446RE Generated ASW, RTE Wrapper, BSW & HIL Runners

This directory contains all C99 source and header files deployed to the **STM32 NUCLEO-F446RE** microcontroller across **Stages 2, 4, 5, and 6**, as well as the host UART log capture utility.

---

## 1. Directory Layout

```text
04_firmware/
├── README.md                              # Firmware architecture & integration overview
├── HARDWARE_CALIBRATION_AND_HIL_GUIDE.md  # Complete wiring table, CubeMX setup & bench execution guide
├── app_generated/                         # Embedded Coder (`ert.tlc`) C99 output from `ctrl_codegen.slx`
│   ├── ctrl_codegen.c                     # Single-precision step function `ctrl_codegen_step()` (0 `double`, 0 `malloc`)
│   ├── ctrl_codegen.h                     # State (`DW_ctrl_codegen_T`), input (`ExtU`), and output (`ExtY`) structs
│   ├── ctrl_codegen_private.h
│   ├── ctrl_codegen_types.h
│   ├── rtwtypes.h                         # Fixed-width & single-precision (`real32_T`) typedefs
│   └── ert_main.c                         # Reference standalone entry point (excluded from MCU/MEX builds)
├── app_wrapper.h / app_wrapper.c          # Clean RTE-like boundary (`App_Init`, `App_Step10ms`)
├── prof.h                                 # ARM Cortex-M4 hardware cycle counter (`DWT->CYCCNT`) WCET profiler
├── closed_loop_runner.h / .c              # Stage 6 100 Hz closed-loop hardware runner (`ClosedLoop_RunAutoStep`, `ClosedLoop_RunLive`)
├── bsw/                                   # Stage 5 AUTOSAR-inspired Basic Software (CAN, E2E, Dem, UDS)
│   ├── com_can_signals.h / .c             # Auto-generated from `05_can/vehicle.dbc` by `dbc_codegen.py`
│   └── can_telemetry.h / .c               # 10 ms CAN scheduler, E2E supervision, timeout monitor & UDS `0x22`
└── experiments/                           # Stage 2 open-loop hardware identification loggers & host capture tool
    ├── capture_log.py                     # Host Python script that parses `#BEGIN <name> ... #END` UART streams into CSVs
    ├── step_logger/
    │   └── step_logger.h / .c             # Automated 19-run motor static & step-response sweep logger (`StepLogger_RunAll`)
    └── pedal_logger/
        └── pedal_logger.h / .c            # Interactive 4-session KY-023 joystick calibration logger (`PedalLogger_RunSessions`)
```

---

## 2. Software Layering (AUTOSAR-Inspired)

| Layer | Module / File | Key Functions / Structs | Notes |
| :--- | :--- | :--- | :--- |
| **Application (ASW)** | [`app_generated/ctrl_codegen.c`](app_generated/ctrl_codegen.c) | `ctrl_codegen_initialize()`, `ctrl_codegen_step()` | Auto-generated from Simulink (`s4_02_generate_code.m`). Uses 32 B of static state RAM (`ctrl_codegen_DW`). |
| **RTE Wrapper** | [`app_wrapper.c`](app_wrapper.c), [`app_wrapper.h`](app_wrapper.h) | `App_Init()`, `App_Step10ms()`, `Rte_AppIn_t`, `Rte_AppOut_t` | Isolates `ctrl_codegen.h` (`rtwtypes.h`) from both STM32 HAL headers and MATLAB `mex.h` (`DEF-04`). |
| **BSW (Com / E2E / Diag)** | [`bsw/com_can_signals.c`](bsw/com_can_signals.c), [`bsw/can_telemetry.c`](bsw/can_telemetry.c) | `CanTelemetry_Init()`, `CanTelemetry_Step10ms()`, `CanTelemetry_OnRxFrame()`, `E2E_Crc8()` | Packs/unpacks DBC frames with SAE J1850 CRC-8 + 4-bit alive counter; tracks `can_age_ms` and `stored_fault_bitmap`; answers UDS `0x22`. |
| **MCAL / Runner** | [`closed_loop_runner.c`](closed_loop_runner.c), [`prof.h`](prof.h) | `ClosedLoop_Init()`, `ClosedLoop_RunAutoStep()`, `ClosedLoop_RunLive()` | Reads `ADC1` (`PA0`/`PA1`) and `TIM3` encoder (`PA6`/`PA7`), profiles `App_Step10ms` with `DWT->CYCCNT` (**75–77 µs**), drives `TIM4_CH1` PWM (`PB6`), and streams 100 Hz CSV telemetry over `USART2`. |
