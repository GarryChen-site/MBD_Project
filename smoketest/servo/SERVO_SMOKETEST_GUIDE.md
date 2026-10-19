# SG90 Micro Servo & Concurrent Motor Smoketest Guide

Brings up and validates the **TowerPro SG90 9g Micro Servo Motor** (simulating physical ADAS interventions like automatic brake pedal actuation or steering angle adjustments) using **TIM3 hardware PWM at 50 Hz (20 ms period)** on the STM32F446RE (Nucleo-64).

> **Target Milestone:**  
> 1. **Accurate Angular Motion:** Servo moves **0° → 90° → 0°** without mechanical stalling or gear buzzing.  
> 2. **Silent Neutral Hold:** Soft PWM release prevents holding jitter and audible coil buzz.  
> 3. **Dual Actuator Concurrency:** DC Motor (TB6612FNG) and SG90 Servo start and run **simultaneously without triggering an MCU brownout or reset**.

---

## 1. SG90 Hardware Specifications & Wire Pinout

Based on the official SG90 datasheet specifications:

| Parameter | Specification | Details / Operational Notes |
| :--- | :--- | :--- |
| **Dimensions & Weight** | 23 × 12.2 × 29 mm, 9g | Ultra-compact micro servo |
| **Operating Voltage** | **4.8 V – 6.0 V DC** | **DO NOT run on 3.3V!** Connect to Nucleo 5V (CN6 pin 5) or external 6V battery |
| **No-load Speed** | 0.09 sec / 60° | ~0.27 sec for full 180° sweep |
| **No-load Current** | 90 mA | Idle running current |
| **Stall Current** | **750 mA (±10%)** | Peak surge current during sudden starts or stall |
| **Stall Torque** | 1.3 – 1.6 kg·cm | Peak mechanical intervention force |
| **PWM Frequency** | **50 Hz (20.0 ms period)** | Standard analog servo frame rate |
| **Pulse Width Range** | **500 µs – 2500 µs** | Standard 180° travel range |
| **Dead Band Width** | 8 µs | Minimum control dead zone |

### Wire Color Mapping

| Wire Color | Function | Target Connection | Pin Location on Nucleo-64 |
| :---: | :--- | :--- | :--- |
| **Brown** | **Ground (GND)** | **Common Ground** | CN6 pin 6 / Breadboard Blue Rail |
| **Red** | **Power (VCC)** | **5.0 V DC (or Battery 6V)** | CN6 pin 5 (5V Power Header) |
| **Orange / Yellow** | **PWM Signal** | **STM32 PA6 (`TIM3_CH1`)** | CN5 pin 5 (Arduino connector "D12") |

---

## 2. 180° Angle to PWM Pulse Width Mapping

The SG90 position is determined strictly by the active HIGH pulse width delivered every 20.0 ms:

```
+-----------------------------------------------------------------------------------------+
|                                50 Hz PWM TIMING (PERIOD = 20.0 ms)                      |
|                                                                                         |
|   0° (CCW Limit)     : [ 0.5 ms / 500 µs ]======================> (19.5 ms LOW)         |
|   45° Position       : [ 1.0 ms / 1000 µs ]=====================> (19.0 ms LOW)         |
|   90° (Neutral Rest) : [ 1.5 ms / 1500 µs ]=====================> (18.5 ms LOW)         |
|   135° Position      : [ 2.0 ms / 2000 µs ]=====================> (18.0 ms LOW)         |
|   180° (CW Limit)    : [ 2.5 ms / 2500 µs ]=====================> (17.5 ms LOW)         |
+-----------------------------------------------------------------------------------------+
```

Formula for any target angle θ (in degrees, 0° to 180°):
```
Pulse Width (µs) = 500 + (θ / 180.0) × 2000
```

---

## 3. Power Isolation Architecture (Preventing MCU Reset)

> [!CAUTION]
> **Why do Microcontrollers Reset when Motor and Servo Start Together?**  
> * SG90 stall/surge current: **750 mA**  
> * N20 DC motor stall/surge current: **300 – 500 mA**  
> * Combined instantaneous inrush current: **1.15 A – 1.25 A**!  
> 
> If both actuators are powered directly from the Nucleo board's internal voltage regulator or a computer USB port (limited to 500 mA), the voltage rail collapses below 2.7 V. The STM32's internal Brownout Reset (BOR) detects the dip and immediately reboots the MCU.

### Proper Multi-Power Domain Layout:

```
  +-----------------------------------------------------------------------------------+
  |                                  POWER DISTRIBUTION                               |
  |                                                                                   |
  |  [4xAA Battery Box (~6.0 V)]                                                      |
  |         |                                                                         |
  |         +---------> TB6612 H-Bridge "VM" Pin (Drives DC Motor coils)              |
  |         |                                                                         |
  |  [Nucleo 5V / External 5V]                                                        |
  |         |                                                                         |
  |         +---------> SG90 Servo "VCC" (Red wire)                                   |
  |         |             [Optional 100µF – 470µF decoupling capacitor to GND]        |
  |         |                                                                         |
  |  [Nucleo 3.3V (CN6-4)]                                                            |
  |         |                                                                         |
  |         +---------> TB6612 "VCC" (Logic power only, < 5 mA)                       |
  |                                                                                   |
  |  [COMMON GND BUS]                                                                 |
  |         |                                                                         |
  |         +======== Battery (-) + Servo Brown + TB6612 GND + Nucleo CN6-6 (GND)     |
  +-----------------------------------------------------------------------------------+
```

---

## 4. STM32 Timer Calculations (TIM3 for 50 Hz PWM)

The STM32F446RE internal HSI clock runs at 16 MHz:
* **Timer Clock Source (`TIM3CLK`)** = 16,000,000 Hz
* **Target PWM Frequency** = 50.0 Hz (Period = 20.0 ms = 20,000 µs)
* **Prescaler (`PSC`) Calculation:**  
  Set Prescaler so each counter tick equals **exactly 1.0 µs**:  
  `PSC = (16,000,000 / 1,000,000) - 1 = 16 - 1 = 15`
* **Auto-Reload Register (`ARR`) Calculation:**  
  `ARR = (20,000 µs / 1.0 µs) - 1 = 20,000 - 1 = 19,999`
* **Duty Cycle / Pulse Width Register (`CCR1`):**  
  Because 1 tick = 1.0 µs, **`CCR1` directly equals the desired pulse width in microseconds**:
  - `CCR1 = 500`  → 0° position
  - `CCR1 = 1500` → 90° neutral position
  - `CCR1 = 2500` → 180° position

---

## 5. Full-Size Breadboard Layout & Wiring Guide

Wiring all three components on a **single full-size solderless breadboard** keeps wiring clean, prevents loose jumper disconnects, and cleanly isolates high-current motor power from delicate MCU logic.

### Breadboard Component Placement Map

```
  ============================= TOP POWER RAILS =============================
  (+) Red Rail  : [====== 4xAA Battery Box (+) Rail (~6.0 V VM) ======]
  (-) Blue Rail : [====== COMMON GROUND BUS (All GNDs Tied Here) ======]
  ===========================================================================

       [Columns A - E]               [Center Trench]              [Columns F - J]
  ---------------------------------------------------------------------------------
  Row 10: [ PWMA  ] <--- Nucleo PB6 (D10)       [ VM   ] <--- Jumper to Top (+) 6V Rail
  Row 11: [ AIN2  ] <--- Nucleo PA9 (D8)        [ VCC  ] <--- Nucleo 3.3V (CN6 pin 4)
  Row 12: [ AIN1  ] <--- Nucleo PC7 (D9)        [ GND  ] <--- Jumper to Blue GND Rail
  Row 13: [ STBY  ] <--- Nucleo PA8 (D7)        [ A01  ] <--- N20 Motor Lead (Red)
  Row 14: [ BIN1  ] (Unused - leave open)       [ A02  ] <--- N20 Motor Lead (White)
  Row 15: [ BIN2  ] (Unused - leave open)       [ B02  ] (Unused - leave open)
  Row 16: [ PWMB  ] (Unused - leave open)       [ B01  ] (Unused - leave open)
  Row 17: [ GND   ] <--- Jumper to Blue Rail    [ GND  ] <--- Jumper to Blue GND Rail
  ---------------------------------------------------------------------------------
  Row 18-21: (Free Isolation Gap)
  ---------------------------------------------------------------------------------
  Row 22: [ Servo PWM (Orange/Yellow) ] <--- Nucleo PA6 (Arduino D12 / CN5 pin 5)
  Row 23: [ Servo VCC (Red)           ] <--- Nucleo 5V (CN6 pin 5)
  Row 24: [ Servo GND (Brown)         ] <--- Jumper to Blue GND Rail
  ---------------------------------------------------------------------------------

  ============================ BOTTOM POWER RAILS ============================
  (-) Blue Rail : [====== Tied to Top Blue Rail (Common GND) ======]
  (+) Red Rail  : [====== Nucleo 5.0 V Distribution Rail ======]
  ============================================================================
```

---

### Step-by-Step Assembly Sequence

#### Step 1: Establish the Common Ground Bus
1. Run a black jumper wire connecting the **Top Blue Rail (-)** to the **Bottom Blue Rail (-)**.
2. Connect **Nucleo CN6 pin 6 (GND)** to the Blue Rail.
3. Connect the **4xAA Battery Box Black (-) wire** to the Blue Rail.
> **Rule:** All grounds (Battery, Nucleo, TB6612, SG90) share this common reference!

#### Step 2: Establish the Isolated Power Rails
1. Connect the **4xAA Battery Box Red (+) wire (~6.0 V)** to the **Top Red Rail (+)**.  
   *(This rail supplies high current exclusively to the TB6612 motor `VM` pin. NEVER jumper this to the Nucleo!)*
2. Connect **Nucleo CN6 pin 5 (5V)** to the **Bottom Red Rail (+)**.  
   *(This rail powers the SG90 Servo).*
3. Connect **Nucleo CN6 pin 4 (3.3V)** directly to **Row 11 Col J (TB6612 VCC)**.  
   *(Powers the low-power internal logic gates of the TB6612).*

#### Step 3: Insert the TB6612 Driver & Wire N20 Motor
1. Insert the TB6612 across the center divider between **Row 10 and Row 17**:
   - Pins 1 to 8 (Left header) sit in Columns E (PWMA down to GND).
   - Pins 1 to 8 (Right header) sit in Columns F (VM down to GND).
2. Wire the power jumpers:
   - **Row 10, Col J (`VM`)** → Jumper to **Top Red Rail (+) [Battery ~6V]**.
   - **Row 11, Col J (`VCC`)** → Jumper to **Nucleo 3.3V (CN6 pin 4)**.
   - **Row 12, Col J (`GND`)** and **Row 17, Col A (`GND`)** → Jumpers to **Blue Rail (GND)**.
3. Connect the N20 Motor:
   - Red lead → **Row 13, Col J (`A01`)**.
   - White lead → **Row 14, Col J (`A02`)**.
4. Connect the MCU control jumpers from Nucleo to TB6612:
   - **Row 10, Col A (`PWMA`)** → Nucleo **PB6** (CN5 pin 3 / Arduino "D10", `TIM4_CH1` 20 kHz PWM).
   - **Row 11, Col A (`AIN2`)** → Nucleo **PA9** (CN9 pin 8 / Arduino "D8").
   - **Row 12, Col A (`AIN1`)** → Nucleo **PC7** (CN9 pin 9 / Arduino "D9").
   - **Row 13, Col A (`STBY`)** → Nucleo **PA8** (CN9 pin 1 / Arduino "D7").

#### Step 4: Plug in the SG90 Micro Servo
1. Insert a 3-pin male header (or 3 male-to-male jumper wires) into **Rows 22, 23, and 24** (e.g. Column B):
   - **Row 22**: Servo **Orange / Yellow wire** (PWM Signal).
   - **Row 23**: Servo **Red wire** (VCC Power).
   - **Row 24**: Servo **Brown wire** (GND).
2. Wire the connections:
   - **Row 22** → Jumper to Nucleo **PA6** (CN5 pin 5 / Arduino "D12", `TIM3_CH1` 50 Hz PWM).
   - **Row 23** → Jumper to **Bottom Red Rail (+) [Nucleo 5V]**.
   - **Row 24** → Jumper to **Blue Rail (GND)**.
3. *(Optional Tip)*: Place a 100 µF to 470 µF electrolytic capacitor across the Bottom Red Rail (5V) and Blue Rail (GND) right next to the servo to absorb inrush current spikes.

---

### Master Connection Summary Table

| Hardware Pin | Module Label | Breadboard Location | Target Source / Nucleo Pin | Notes |
| :--- | :---: | :---: | :--- | :--- |
| **TB6612 VM** | VM (Pin 1 Right) | Row 10, Col J | **4xAA Battery (+)** (~6.0V) | **Dedicated motor power!** |
| **TB6612 VCC** | VCC (Pin 2 Right) | Row 11, Col J | **Nucleo 3.3V** (CN6 pin 4) | Logic power (< 5 mA) |
| **TB6612 GND** | GND (Pins 3 & 8) | Row 12, Col J / Row 17 | **Common GND Rail** | Ground reference |
| **TB6612 A01** | A01 (Pin 4 Right) | Row 13, Col J | **N20 Red wire** | H-Bridge Motor Output A |
| **TB6612 A02** | A02 (Pin 5 Right) | Row 14, Col J | **N20 White wire** | H-Bridge Motor Output B |
| **TB6612 PWMA** | PWMA (Pin 1 Left) | Row 10, Col A | **Nucleo PB6** (CN5-3 / D10) | TIM4_CH1 (20 kHz PWM) |
| **TB6612 AIN2** | AIN2 (Pin 2 Left) | Row 11, Col A | **Nucleo PA9** (CN9-8 / D8) | Motor Direction 2 |
| **TB6612 AIN1** | AIN1 (Pin 3 Left) | Row 12, Col A | **Nucleo PC7** (CN9-9 / D9) | Motor Direction 1 |
| **TB6612 STBY** | STBY (Pin 4 Left) | Row 13, Col A | **Nucleo PA8** (CN9-1 / D7) | Standby Enable (Active HIGH) |
| **SG90 PWM** | Signal (Orange/Yellow) | Row 22, Col B | **Nucleo PA6** (CN5-5 / D12) | TIM3_CH1 (50 Hz PWM) |
| **SG90 VCC** | Power (Red) | Row 23, Col B | **Nucleo 5V** (CN6-5) | 4.8V – 6.0V Servo Supply |
| **SG90 GND** | Ground (Brown) | Row 24, Col B | **Common GND Rail** | Ground reference |
| **Battery (-)** | Black wire | Blue GND Rail | **Common GND Rail** | Tied to Nucleo CN6-6 (GND) |

---

## 6. STM32CubeIDE IOC Configuration

### Step 1 — Timer 3 (SG90 Servo PWM)
1. In CubeMX Left Panel: **Timers → TIM3**
2. In Mode settings:
   - **Clock Source**: `Internal Clock`
   - **Channel 1**: `PWM Generation CH1`
3. In Configuration → **Parameter Settings**:
   - **Prescaler (PSC)**: `15`
   - **Counter Period (ARR)**: `19999`
   - **Auto-Reload Preload**: `Enable`
   - **Pulse (CCR1)**: `1500` (starts at neutral 90°)
   - **CH Polarity**: `High`
4. Confirm in Pinout View that **PA6** is assigned to `TIM3_CH1`.

### Step 2 — Timer 4 (DC Motor 20 kHz Ultrasonic PWM)
1. In CubeMX Left Panel: **Timers → TIM4**
2. In Mode settings:
   - **Clock Source**: `Internal Clock`
   - **Channel 1**: `PWM Generation CH1`
3. In Configuration → **Parameter Settings**:
   - **Prescaler (PSC)**: `0`
   - **Counter Period (ARR)**: `799` (gives 20.0 kHz)
   - **Auto-Reload Preload**: `Enable`
   - **Pulse (CCR1)**: `0`
4. Confirm in Pinout View that **PB6** is assigned to `TIM4_CH1`.

---

## 7. Running the Automated Acceptance Test

1. Connect the STM32 board via USB.
2. In your terminal, run the monitor:
   ```bash
   python smoketest/servo/servo_monitor.py
   ```
3. Observe the automated 3-stage sequence:
   * **Stage 1 (0° → 90° → 0° Transition):**  
     Moves to 0° (500 µs), holds for 1.2s. Moves to 90° (1500 µs), holds for 1.2s. Returns to 0° (500 µs). Verifies no gear chatter or stalling.
   * **Stage 2 (Smooth 180° Slew Sweep):**  
     Sweeps 0° → 180° → 0° at 10 ms/deg slew-rate. De-energizes PWM compare to release holding torque and confirm silent operation.
   * **Stage 3 (Concurrent Motor + Servo Load Test):**  
     Click the Blue Button (**B1**). The DC motor spins at 50% PWM while the SG90 executes an immediate full-stroke intervention.  
     **Pass Criteria:** System uptime continues uninterrupted with **zero MCU resets or brownouts**.
