# Data · Control · Hardware Flow

How a physical quantity becomes a signal, passes through control logic, and returns as a physical action.
Companion to [`README.md`](README.md) and [`sandbox_simulation.md`](sandbox_simulation.md).

> [!NOTE]
> Reading guide: **Hardware** (physical) → **MCAL/BSW** (drivers, scaling) → **Application** (Simulink-generated logic) → back out to **Hardware**.

---

## 1. Big picture: closed loops across three layers

```mermaid
flowchart LR
  subgraph HW_IN["Hardware: inputs"]
    J["KY-023 X/Y"]
    E["N20 Hall encoder"]
    T["VL53L0X"]
    CANIN["CANable -> SN65HVD230"]
  end

  subgraph SW["ECU software (STM32F446RE)"]
    direction TB
    subgraph DRV["MCAL / BSW: raw -> engineering units"]
      ADC["ADC driver\nraw 0..4095"]
      ENC["Encoder driver\ncounts -> rpm"]
      I2C["I2C driver\nmm"]
      COMRX["Com Rx\nunpack + counter/CRC check"]
    end
    subgraph APP["Application (Simulink / Embedded Coder)"]
      PED["Pedal processing\nscale, plausibility, rate limit"]
      PID["Speed PID\n+ anti-windup"]
      AEB["AEB state machine\n(Stateflow)"]
      DIAG["Fault manager\ndebounce, degraded mode"]
    end
    subgraph DRVOUT["MCAL / BSW: engineering units -> raw"]
      PWM["PWM driver\nduty -> TB6612"]
      SRVD["Servo driver\nangle -> pulse"]
      COMTX["Com Tx\npack + counter/CRC"]
    end
  end

  subgraph HW_OUT["Hardware: outputs"]
    DR["TB6612FNG"] --> M["N20 motor"]
    S["SG90 brake"]
    CANOUT["SN65HVD230 -> CANable -> PC"]
  end

  J --> ADC --> PED
  E --> ENC --> PID
  T --> I2C --> AEB
  CANIN --> COMRX --> PED
  PED -->|speed_req| PID
  AEB -->|"brake_req / speed_limit"| PID
  PED -- "pedal faults" --> DIAG
  AEB -- "sensor faults" --> DIAG
  COMRX -- "timeout / bad CRC" --> DIAG
  DIAG -- "mode, limp-home limit" --> PID
  PID -->|duty_cmd| PWM --> DR
  M -. "shaft rotation (physics)" .-> E
  AEB -->|servo_deg| SRVD --> S
  PID --> COMTX
  AEB --> COMTX
  DIAG --> COMTX --> CANOUT
```

**Three closed loops**

| Loop | Path | Rate | Requirements |
| :--- | :--- | :---: | :--- |
| **L1 Speed control** | encoder → rpm → PID → duty → TB6612 → motor → encoder | 10 ms | SPD-001..005 |
| **L2 AEB** | VL53L0X → distance → state machine → speed limit + servo → (vehicle slows, distance changes) | 50 ms | AEB-001..006 |
| **L3 Supervision** | all fault sources → fault manager → mode → limits on L1/L2 + CAN report | 10–100 ms | DIA-*, SAF-* |

L1 runs fully on the ECU. The PC is **not** in any control loop; it is only a stimulus and logger (R8 in the sandbox).

---

## 2. Signal-by-signal flow

Follow one signal from the physical world to the output. `Unit` and `Rate` are drafts.

| # | Physical → raw | Driver | Signal in app | Used by | Output signal | Physical result | Rate |
| :-: | :--- | :--- | :--- | :--- | :--- | :--- | :---: |
| 1 | Joystick X angle → voltage 0–3.3 V | ADC1 CH0 (timer-triggered) | `pedal1_pct` | Pedal processing | `speed_req_rpm` | motor setpoint | 10 ms |
| 2 | Joystick Y angle → voltage | ADC2 CH1 | `pedal2_pct` (via PedalCond, REVERSE/HALF) | Plausibility check | `pedal_fault` | limp-home limit | 10 ms |
| 3 | Shaft rotation → Hall A/B pulses | TIM encoder mode | `speed_meas_rpm` | PID | `duty_cmd_pct` | PWM duty | 10 ms |
| 4 | Duty → PWM + IN1/IN2/STBY | TIM PWM + GPIO | — | TB6612 → motor | motor torque/speed | shaft speed | 10 ms |
| 5 | Target distance → ToF timing | I2C1 (VL53L0X) | `distance_mm` | AEB state machine | `aeb_state`, `speed_limit_rpm`, `servo_deg` | slow down + brake servo | 50 ms |
| 6 | PC frame `0x100` → CAN bits | bxCAN + FIFO0 | `pedal_cmd_pct` (alt. source) | Pedal processing | same as #1 | same | 20 ms |
| 7 | Faults → debounced flags | (software) | `fault_bitmap`, `mode` | PID limit, CAN report | `0x201` Diag_Status | PC sees faults | 100 ms |
| 8 | Status → CAN frames | bxCAN TX | `speed_meas_rpm`, `duty`, `mode`, `distance`, `aeb_state` | PC / ESP32 | `0x200`, `0x210` | logs, dashboard | 20/50 ms |

### Pedal source arbitration (draft rule)

```mermaid
flowchart LR
  A["Joystick ADC\n(pedal1, pedal2)"] --> SEL{"ctrl_source"}
  B["CAN 0x100\npedal_cmd"] --> SEL
  SEL -->|"LOCAL (default)"| P["Pedal processing"]
  SEL -->|"BENCH (test mode)"| P
  P --> O["speed_req_rpm"]
```

Both sources go through the same plausibility and limiting logic. This lets the PC run **HIL-lite** tests by injecting a pedal profile via CAN without touching the joystick. Select the source with a calibration flag or a CAN command (**TBD**).

---

## 3. Control data path (the 10 ms tick, in order)

```mermaid
sequenceDiagram
  autonumber
  participant HW as Hardware
  participant BSW as MCAL/BSW
  participant APP as App (generated)
  Note over HW,APP: every 10 ms
  HW->>BSW: ADC samples (DMA), encoder count
  BSW->>APP: pedal1_pct, pedal2_pct, speed_meas_rpm, ctrl inputs (via RTE-like wrappers)
  APP->>APP: plausibility -> speed_req
  APP->>APP: apply limits (mode, AEB speed_limit)
  APP->>APP: PID(speed_req, speed_meas) -> duty_cmd
  APP->>BSW: duty_cmd_pct, dir
  BSW->>HW: TIM CCR update (clamped by duty cap)
  Note over BSW: BSW enforces the hard duty cap even if the app asks for more (SPD-005)
```

**Design rules this encodes**
- The application never touches registers or HAL. It only reads/writes signals through RTE-like wrappers.
- Scaling to engineering units happens in BSW; limits that protect hardware (duty cap, servo angle limits) live in BSW as the last barrier.
- Order inside a tick is fixed: **read → process → decide → write**, so the data is consistent in one cycle.

---

## 4. Communication data flow (CAN)

```mermaid
flowchart TB
  subgraph PC
    PY["Python / Simulink bench"] --> DBC1["DBC (cantools)\nencode"]
    DBC2["DBC (cantools)\ndecode"] --> LOGS["Logs / plots / test verdict"]
  end
  DBC1 -->|"0x100 Pedal_Cmd (20 ms)"| BUS(("CAN bus\n500 kbit/s"))
  BUS -->|"0x200, 0x201, 0x210, 0x123"| DBC2
  subgraph ECU
    RX["Com Rx: unpack\ncounter + CRC check\nrx timeout monitor"] --> APPIN["App inputs"]
    APPOUT["App outputs"] --> TX["Com Tx: pack\ncounter + CRC"]
  end
  BUS --> RX
  TX --> BUS
  GEN["DBC -> C generator (script)"] -. "generates pack/unpack code" .-> RX
  GEN -.-> TX
```

**One source of truth:** `05_can/vehicle.dbc` is used by (a) the PC scripts (cantools), (b) the C code generator for BSW Com, and (c) the test cases. If a signal changes, change the DBC first.

---

## 5. Fault and mode flow

```mermaid
flowchart LR
  F1["Pedal mismatch / range"] --> D
  F2["CAN timeout / bad CRC"] --> D
  F3["ToF invalid"] --> D
  F4["Motor stall"] --> D
  F5["Watchdog reset flag"] --> D
  D["Debounce\n(set/clear counters)"] --> FM["Fault table\nfault_bitmap"]
  FM --> MODE{"Mode manager"}
  MODE -->|none| N["Normal"]
  MODE -->|non-critical| DG["Degraded:\nspeed limit = limp-home"]
  MODE -->|critical| SS["SafeState:\nduty 0, servo safe"]
  N --> LIM["Limits to PID / AEB"]
  DG --> LIM
  SS --> LIM
  FM --> CANST["CAN 0x201 Diag_Status"]
  MODE --> CANST
```

| Fault | Reaction | Mode | Safety goal |
| :--- | :--- | :---: | :---: |
| Pedal mismatch | speed limit = limp-home | Degraded | SG-01 |
| CAN timeout | substitute pedal (0) | Degraded | SG-01 |
| ToF invalid | no braking from that data, warn | Degraded | SG-03 |
| Motor stall | reduce duty | Degraded/Safe | SG-02 |
| Watchdog reset | start in safe outputs, report reset cause | Init → Normal | SG-02 |

---

## 6. Power and ground flow (physical)

```mermaid
flowchart LR
  USB["PC USB"] --> NUC["Nucleo (ST-Link, 3.3 V logic)"]
  NUC -- "3.3 V" --> SENS["VL53L0X, Grove, CAN transceiver, joystick"]
  BAT["4xAA 6 V"] --> TB["TB6612 VM"] --> MOT["N20 motor"]
  BAT --> SRV["SG90"]
  NUC -- "3.3 V -> VCC" --> TB2["TB6612 VCC (logic)"]
  GND(("Common GND\n(star point)"))
  NUC --- GND
  BAT --- GND
  TB --- GND
```

Rules: motor and servo power come **only** from the battery. Logic is powered from the Nucleo (USB). Only GND is shared. Add a bulk capacitor near the driver and a 100 nF capacitor across the motor terminals (R1/R2 in the sandbox).

---

## 7. Development data flow (MBD V-model)

How the same model becomes tests at each level (MIL → SIL → PIL → HIL-lite).

```mermaid
flowchart LR
  REQ["Requirements\nREQ-IDs"] --> MODEL["Simulink models:\ncontroller + plant"]
  MODEL --> MIL["MIL\nSimulink Test\n(scenarios S1-S6)"]
  MODEL --> CG["Embedded Coder\ngenerated C"]
  CG --> SIL["SIL: compiled on PC\nback-to-back vs MIL"]
  CG --> PIL["PIL: runs on F446RE\nback-to-back vs MIL"]
  CG --> FW["CubeIDE firmware\n(app + BSW + MCAL)"]
  FW --> HIL["HIL-lite: real motor/sensors\nPC injects CAN, logs"]
  MIL --> REP["Test report +\ntraceability"]
  SIL --> REP
  PIL --> REP
  HIL --> REP
  REP --> REQ
```

| Level | What is real | What is simulated | Signals compared |
| :--- | :--- | :--- | :--- |
| MIL | nothing | controller + plant in Simulink | reference |
| SIL | generated code (PC) | plant in Simulink | duty, state vs MIL |
| PIL | generated code on STM32 | plant in Simulink | duty, state vs MIL, execution time |
| HIL-lite | ECU + N20 + sensors | environment (PC stimulus, CAN) | speed, CAN signals, faults vs requirements |

---

## 8. Quick reference: where is each concern handled?

| Concern | Layer | Where |
| :--- | :--- | :--- |
| ADC scaling, encoder → rpm, ToF read | MCAL/BSW | `04_firmware/mcal/`, `bsw/` |
| Pedal logic, PID, AEB, fault logic | Application | `03_simulink/controller/` |
| Hard limits (duty cap, servo angle) | BSW | `04_firmware/bsw/` |
| Signal pack/unpack, CAN timeout | BSW Com | generated from `05_can/vehicle.dbc` |
| Plant physics | Simulink (MIL) / real hardware (HIL) | `03_simulink/plant/` |
| Stimulus, fault injection, logging | PC | `06_test/hil/` |
