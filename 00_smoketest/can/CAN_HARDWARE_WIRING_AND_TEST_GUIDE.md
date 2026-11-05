# CAN Real-Bus Bring-up Guide: STM32F446RE + SN65HVD230 + Makerbase CANable 2.0 + Mac

This guide documents the complete physical wiring, firmware configuration, and automated test procedure to achieve the real-bus milestone:

> **Target Goal:**  
> **CANable receives >= 120 heartbeat frames in 60 s (500 ms ± 2 ms jitter), 0 error frames.**

---

## 1. System Overview & Architecture

```
+-------------------------------------------------------------------------------------------------------+
|                                        CAN REAL-BUS TOPOLOGY                                          |
|                                                                                                       |
|  [ Mac PC ]                                                                                           |
|      |                                                                                                |
|     USB                                                                                               |
|      v                                                                                                |
|  [ Makerbase CANable 2.0 ]                                                                            |
|      |  * Switch 4 (120 Ohm): ON                                                                      |
|      |  * Terminal: GND, CANH, CANL                                                                   |
|      |                                                                                                |
|      +================= PHYSICAL TWISTED PAIR CAN BUS =================+                              |
|      |   CAN_H (Differential High)                                     |                              |
|      |   CAN_L (Differential Low)                                      |                              |
|      |   GND   (Common Reference)                                      |                              |
|      +=================================================================+                              |
|                                                                        |                              |
|                                                     [ SN65HVD230 Transceiver ]                        |
|                                                         |  * Yellow Jumper (120 Ohm): INSTALLED       |
|                                                         |  * Header: 3.3V, GND, RX, TX                |
|                                                         |                                             |
|                                                         +--- 3.3V <--- 3V3                            |
|                                                         +--- GND  <--- GND                            |
|                                                         +--- RX   ---> PA11 (CAN1_RX)                 |
|                                                         +--- TX   <--- PA12 (CAN1_TX)                 |
|                                                         |                                             |
|                                                         v                                             |
|                                             [ NUCLEO-F446RE Board ]                                   |
+-------------------------------------------------------------------------------------------------------+
```

---

## 2. Hardware Components & Pin Definitions

### A. SN65HVD230 (VP230) 3.3V Transceiver Module
Based on your module schematic and layout:
- **Left 4-Pin Header (P1)**:
  - `3.3V` : DC 3.0V - 3.6V power supply (draws from STM32 3V3).
  - `GND`  : Common Ground.
  - `RX`   : VP230 pin 4 (R, Receiver Output) -> Connects to STM32 **PA11 (CAN1_RX)**.
  - `TX`   : VP230 pin 1 (D, Driver Input) -> Connects to STM32 **PA12 (CAN1_TX)**.
- **Right 2-Pin Screw Terminal**:
  - `CANH` : Differential high line.
  - `CANL` : Differential low line.
- **Onboard 120 Ohm Jumper**:
  - Yellow 2-pin jumper across `R2` (120 Ohm).
  - **Status: MUST BE INSTALLED (ON)**.

---

### B. Makerbase MKS CANable V2.0 USB-CAN Adapter
- **USB Plug**: Plugs into Mac PC (or USB-C hub).
- **Side Slide Switch (Switch 4)**:
  - Labeled `120Ω OFF / ON`.
  - **Status: SLIDE TO ON** (enables the adapter's internal 120 Ohm termination).
- **Right 3-Pin Screw Terminal**:
  - `GND`  : Signal ground reference.
  - `CANH` : Differential high line.
  - `CANL` : Differential low line.
- **Status LEDs**:
  - 1. **PWR** (Red): Always ON when powered via USB.
  - 2. **State** (Blue): Flashes rapidly during CAN traffic transmission/reception.
  - 3. **Work** (Green): Always ON when the CAN controller is active.

---

### C. NUCLEO-F446RE Board Pinout
On the STM32F446RE, CAN1 is configured on **GPIOA** pins with Alternate Function **AF9**:
- **CAN1_RX**: **`PA11`** -> Located on Morpho connector **`CN10 Pin 14`**.
- **CAN1_TX**: **`PA12`** -> Located on Morpho connector **`CN10 Pin 12`**.
- **Power**:
  - **`3V3`** -> Arduino Power header **`CN6 Pin 4`** (or Morpho **`CN7 Pin 16`**).
  - **`GND`** -> Arduino Power header **`CN6 Pin 6`** (or Morpho **`CN7 Pin 20`**).

> [!TIP]
> **Locating CN10 on Nucleo-64**: CN10 is the double-row male header on the **right side** of the board. The outer row contains the even-numbered pins. Pin 12 (`PA12`) and Pin 14 (`PA11`) are adjacent on this outer row.

---

## 3. Detailed Hardware Wiring Table

### Table 1: STM32F446RE <-> SN65HVD230 (Logic Side)
| SN65HVD230 Pin | Wire Color (Suggested) | NUCLEO-F446RE Header / Pin | Description |
| :--- | :--- | :--- | :--- |
| **`3.3V`** | Red | **CN6 Pin 4** (`3V3`) | 3.3V Power from Nucleo |
| **`GND`** | Black | **CN6 Pin 6** (`GND`) | Ground Reference |
| **`RX`** | Yellow | **CN10 Pin 14** (`PA11`) | CAN1 Receive line (AF9) |
| **`TX`** | Blue | **CN10 Pin 12** (`PA12`) | CAN1 Transmit line (AF9) |

> [!IMPORTANT]
> **Straight connection**: Transceiver **`RX`** connects to MCU **`PA11 (CAN1_RX)`**, and Transceiver **`TX`** connects to MCU **`PA12 (CAN1_TX)`**.  
> *(Do NOT cross RX/TX here - the SN65HVD230 module labels `TX` as Driver Input from MCU, and `RX` as Receiver Output to MCU).*

---

### Table 2: SN65HVD230 <-> Makerbase CANable 2.0 (CAN Bus Side)
You can use standard **Male-to-Female jumper wires** for desktop testing (no special twisted pair required):

| SN65HVD230 (Yellow Pin Header) | Jumper Wire | Makerbase CANable 2.0 (Screw Terminal) |
| :--- | :--- | :--- |
| **`CANH`** (Top yellow pin) | Female end -> Male pin | Insert male pin into **`CANH`** slot & tighten screw |
| **`CANL`** (Bottom yellow pin) | Female end -> Male pin | Insert male pin into **`CANL`** slot & tighten screw |
| **`GND`** (from STM32/Transceiver) | Male end -> Male pin | Insert male pin into **`GND`** slot & tighten screw |

> [!TIP]
> Optional: Gently twist the two CANH and CANL jumper wires around each other 3-5 times by hand for noise immunity.

---

### Table 3: Bus Termination Summary
ISO 11898 requires standard high-speed CAN buses to be terminated at **both ends** with 120 Ohm resistors (60 Ohm equivalent):

| Device | Termination Control | Required Setting |
| :--- | :--- | :--- |
| **Makerbase CANable 2.0** | Slide Switch 4 (`120Ω`) | **`ON`** |
| **SN65HVD230 Board** | Yellow 2-pin Jumper / Onboard R2 | **`INSTALLED` / ACTIVE** |

*(You can verify with a multimeter between CANH and CANL when powered off: it should measure approximately 60 Ohm).*

---

## 4. STM32 Firmware Configuration

To transition from the internal loopback smoke test to the physical bus, update [gen.c](file:///Users/church/Documents/github/MBD_Project/smoketest/gen.c):

### 1. Disable Loopback Mode Define
In [gen.c line 40](file:///Users/church/Documents/github/MBD_Project/smoketest/gen.c#L40):
```c
// Change from 1 to 0:
#define CAN_LOOPBACK_TEST 0
```

### 2. Set CAN Peripheral to Normal Mode
In `MX_CAN1_Init()` in [gen.c line 255](file:///Users/church/Documents/github/MBD_Project/smoketest/gen.c#L255):
```c
// Change from CAN_MODE_LOOPBACK to CAN_MODE_NORMAL:
hcan1.Init.Mode = CAN_MODE_NORMAL;
```

### 3. CAN1 GPIO MSP Initialization (PA11 / PA12)
In your `stm32f4xx_hal_msp.c` (or `HAL_CAN_MspInit`):
```c
void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan) {
  if (hcan->Instance == CAN1) {
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}
```

---

## 5. Automated 60-Second Acceptance Test

A dedicated verification script is provided at [smoketest/can_verify_60s.py](file:///Users/church/Documents/github/MBD_Project/smoketest/can_verify_60s.py).

### Running the Test
Activate your virtual environment and run the benchmark script:

```bash
# 1. Activate venv
source .venv/bin/activate

# 2. Run the 60-second verification benchmark
python smoketest/can_verify_60s.py slcan /dev/cu.usbmodem2089335D38451
```

### How the Script Works
1. Connects to the Makerbase CANable 2.0 in `slcan` mode at **500 kbit/s**.
2. Synchronizes to the arrival of the first heartbeat frame (`ID 0x123`).
3. Logs high-resolution timestamps (`time.perf_counter`) of all incoming frames for exactly 60.0 seconds.
4. Computes intervals (Delta t), jitter relative to 500.0 ms, standard deviation, and error frame count.
5. Prints a breakdown and overall **PASS / FAIL** verdict.

### Expected Output
```text
=================================================================
  CAN Real-Bus 60s Acceptance Test
  Target: >= 120 frames in 60s (500 ms +- 2 ms), 0 errors
=================================================================
[*] Connected via slcan at 500000 bit/s.
[*] Waiting for first heartbeat (ID 0x123) to start timing...
[SYNC] First heartbeat received: counter=0, loopback_flag=0
[*] Benchmark running for 60.0 seconds. Please wait...
  -> Elapsed: 10s / 60s | Frames: 21 | Errors: 0
  -> Elapsed: 20s / 60s | Frames: 41 | Errors: 0
  -> Elapsed: 30s / 60s | Frames: 61 | Errors: 0
  -> Elapsed: 40s / 60s | Frames: 81 | Errors: 0
  -> Elapsed: 50s / 60s | Frames: 101 | Errors: 0

=================================================================
  TEST RESULTS & METRICS
=================================================================
Total Heartbeat Frames : 121 (Requirement: >= 120)
Total Error Frames     : 0 (Requirement: 0)
-----------------------------------------------------------------
Nominal Period         : 500.0 ms
Mean Measured Period   : 500.01 ms
Min / Max Period       : 499.12 ms / 500.94 ms
Standard Deviation     : 0.35 ms
Mean Jitter            : 0.28 ms
Max Jitter             : 0.94 ms
Frames within +-2ms    : 120 / 120 (100.0%)
-----------------------------------------------------------------
Criteria Breakdown:
 1. Frame Count (>= 120 in 60s)       : PASS (121)
 2. Error Frames (0 error frames)      : PASS (0)
 3. Average Jitter (<= 2 ms)           : PASS (0.28 ms)

>>> OVERALL RESULT: [PASS] <<<
```

---

## 6. Visual Indicators & Hardware Sanity Check

| Device | Indicator | Healthy State |
| :--- | :--- | :--- |
| **CANable 2.0** | **PWR LED (Red)** | Solid ON |
| **CANable 2.0** | **Work LED (Green)** | Solid ON |
| **CANable 2.0** | **State LED (Blue)** | Flashes once every 500 ms (or rapidly when receiving traffic) |
| **NUCLEO-F446RE** | **LD2 (Green)** | Toggles when command `0x321` is sent by PC |

---

## 7. Troubleshooting Checklist

| Symptom | Probable Cause | Fix |
| :--- | :--- | :--- |
| `0 frames received`, timeout | SN65HVD230 power missing | Check multimeter on VP230 pin 1 & 2: must read 3.3V. |
| `0 frames received` | Reversed CAN wires | Swap `CANH` and `CANL` at the screw terminal. |
| STM32 reports `TX ERROR 0x00000004` (NACK) | Bus not acknowledged | 1. Ensure CANable is connected and initialized via script.<br>2. Check common GND wire between CANable and STM32.<br>3. Verify both 120 Ohm terminations are active. |
| Large timing jitter (> 10 ms) | Mac USB hub buffering | Connect CANable directly to a Mac USB port or powered hub; avoid high-load hubs sharing webcams/disks. |
