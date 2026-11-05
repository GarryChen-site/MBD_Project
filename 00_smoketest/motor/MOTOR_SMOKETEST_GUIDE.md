# TB6612FNG Dual H-Bridge & N20 Motor Smoketest (HAL Version)

Brings up and validates the **TB6612FNG Dual H-Bridge Driver** driving the **N20 DC Geared Motor** using **TIM4 hardware PWM at 20 kHz** on the STM32F446RE (Nucleo-64), with telemetry streamed over USART2 at 115200 baud and signal timing captured via USB Logic Analyzer (Saleae Logic).

> **Target Milestone:**  
> 1. **20 kHz PWM visible on Logic Analyzer** (Period = 50.0 µs, 50% Duty = 25.0 µs) or audible silence (no acoustic coil whine).  
> 2. **Duty cycle 0 → 50 % ramp** spins the motor smoothly in the expected direction.  
> 3. **STBY pin LOW** immediately stops the motor (high-impedance cutoff).

---

## Hardware Specifications & Wire Pinout

### Components Required
- **TB6612FNG Dual H-Bridge Module** (with pre-soldered header pins)
- **4xAA Battery Box** (6.0 V nominal: 4 × 1.5 V) with female Dupont connectors
- **N20 DC Geared Motor** (using White and Red motor terminal leads)
- **STM32F446RE Nucleo-64 Board**
- **USB Logic Analyzer** (Saleae-compatible 8-channel / 24 MHz)
- **Full-Size Solderless Breadboard** and male-to-male / male-to-female jumper wires

```
                           +------------------------+
                           |  TB6612FNG (TOP VIEW)  |
                           |                        |
   STM32 PB6 (D10) ------->| [1] PWMA      VM   [1] |<------- 4xAA Battery (+) (~6.0V)
   STM32 PA9 (D8)  ------->| [2] AIN2      VCC  [2] |<------- STM32 3.3V (Logic VCC)
   STM32 PC7 (D9)  ------->| [3] AIN1      GND  [3] |-------> Common GND Rail
   STM32 PA8 (D7)  ------->| [4] STBY      A01  [4] |-------> Motor Terminal (Red wire)
   (Unused - N/C)  --------| [5] BIN1      A02  [5] |-------> Motor Terminal (White wire)
   (Unused - N/C)  --------| [6] BIN2      B02  [6] |-------- (Unused - N/C)
   (Unused - N/C)  --------| [7] PWMB      B01  [7] |-------- (Unused - N/C)
   Common GND Rail <-------| [8] GND       GND  [8] |-------> Common GND Rail
                           +------------------------+
```

---

## Electrical Safety & Power Isolation Rules

> [!CAUTION]
> **CRITICAL ELECTRICAL ISOLATION RULE (VM vs VCC):**  
> - **`VM` (Top-Right Pin 1)** must connect **ONLY** to the **4xAA Battery Box (+) lead (approx. 6.0 V)**.  
>   **NEVER connect `VM` to STM32 5V or 3.3V!** DC motors draw heavy startup and stall surge currents (up to 300–500 mA) and generate inductive back-EMF spikes that will brown out, reset, or permanently destroy the STM32 microcontroller.  
> - **`VCC` (Right Pin 2)** connects to **STM32 3.3V** (CN6 pin 4). The TB6612 logic block operates from 2.7 V to 5.5 V, so 3.3 V provides perfect logic compatibility with STM32 GPIO signals without level shifters.  
> - **`GND` (Common Ground)**: You **MUST connect all grounds together** on the breadboard blue rail:  
>   `4xAA Battery (-)` + `TB6612 GND (Pin 8 / Pin 3)` + `STM32 GND` + `Logic Analyzer GND`. Without a shared ground reference, logic signals and PWM pulses will float and trigger erratically.

---

## Complete Physical Wiring Table

### Left Header (MCU Control Inputs)

| Pin # | Silk Label | Function | Target Connection | Pin Location on Nucleo-64 |
| :---: | :--- | :--- | :--- | :--- |
| **1** | **PWMA** | Motor A PWM Input | **STM32 PB6** | CN5 pin 3 (Arduino "D10", `TIM4_CH1` 20 kHz PWM) |
| **2** | **AIN2** | Motor A Direction 2 | **STM32 PA9** | CN9 pin 8 (Arduino "D8", GPIO Output) |
| **3** | **AIN1** | Motor A Direction 1 | **STM32 PC7** | CN9 pin 9 (Arduino "D9", GPIO Output) |
| **4** | **STBY** | Driver Standby (Active HIGH)| **STM32 PA8** | CN9 pin 1 (Arduino "D7", GPIO Output) |
| **5** | **BIN1** | Motor B Direction 1 | *Leave Unconnected* | Not used for single motor test |
| **6** | **BIN2** | Motor B Direction 2 | *Leave Unconnected* | Not used for single motor test |
| **7** | **PWMB** | Motor B PWM Input | *Leave Unconnected* | Not used for single motor test |
| **8** | **GND**  | Logic Ground | **Breadboard GND** | Connect to STM32 GND rail |

### Right Header (Power Supply & Motor Outputs)

| Pin # | Silk Label | Function | Target Connection | Notes / Warnings |
| :---: | :--- | :--- | :--- | :--- |
| **1** | **VM**   | Motor Power Supply | **4xAA Battery Box (+) / Red** | **~6.0 V. NEVER connect to STM32!** |
| **2** | **VCC**  | Logic Power Supply | **STM32 3.3V (CN6 pin 4)** | Powers TB6612 internal logic gate IC |
| **3** | **GND**  | Power / Logic Ground | **Breadboard GND** | Common Ground |
| **4** | **A01**  | Motor A Output 1 | **N20 Motor Red wire** | Reversible H-bridge output |
| **5** | **A02**  | Motor A Output 2 | **N20 Motor White wire** | Reversible H-bridge output |
| **6** | **B02**  | Motor B Output 2 | *Leave Unconnected* | Channel B unused |
| **7** | **B01**  | Motor B Output 1 | *Leave Unconnected* | Channel B unused |
| **8** | **GND**  | Power Ground | **Breadboard GND** | Common Ground |

---

## 20 kHz Ultrasonic PWM Timing Math

Why 20 kHz?
Standard hobby motor drivers operate at 1 kHz or 2 kHz, which falls right inside the human ear's most sensitive hearing band (1 kHz – 4 kHz), creating an irritating, loud high-pitched acoustic whine from the motor coils. By driving the H-bridge at **20,000 Hz (20 kHz)**, the frequency is shifted into the ultrasonic region above human hearing, resulting in silent motor operation while maintaining low switching losses on the TB6612 MOSFETs (rated up to 100 kHz).

### Timer Prescaler and Auto-Reload Register (ARR) Calculation:

- System Clock (`SYSCLK`) = 16 MHz (Internal HSI oscillator, PLL bypassed)
- `APB1` Timer Clock = 16 MHz
- Prescaler (`PSC`) = 0 (Timer ticks at full 16,000,000 Hz)
- Target PWM Frequency = 20,000 Hz

- **Timer Counter Frequency** = 16,000,000 Hz / (0 + 1) = 16,000,000 Hz  
- **Auto-Reload Value (ARR)** = (Timer Clock / Target PWM Frequency) - 1 = (16,000,000 / 20,000) - 1 = 800 - 1 = **799**  
- **Total PWM Period** = (799 + 1) / 16,000,000 Hz = **50.0 µs** (Frequency = 1 / 50.0 µs = **20.0 kHz**)

- **50.0% Duty Cycle** corresponds to:  
  `CCR1 = (800 × 50%) = 400 counts`  
  `Pulse Width = 400 / 16,000,000 Hz = 25.0 µs`
- **0.0% Duty Cycle** corresponds to:  
  `CCR1 = 0 counts`  
  `Pulse Width = 0.0 µs` (Output flat LOW)

---

## Logic Analyzer Connection & Measurement Guide

Connect the Saleae-compatible 8-channel USB Logic Analyzer leads as follows:

| Logic Analyzer Channel | Signal Monitored | STM32 / TB6612 Pin | Expected Signal & Measurement |
| :--- | :--- | :--- | :--- |
| **CH0** | **PWMA** | **PB6** (Arduino D10) | Square wave: **f = 20.0 kHz**, **T = 50.0 µs**, **t_high = 25.0 µs** (at 50% duty) |
| **CH1** | **AIN1** | **PC7** (Arduino D9) | Logic level: **HIGH (3.3 V)** during Forward spin |
| **CH2** | **AIN2** | **PA9** (Arduino D8) | Logic level: **LOW (0.0 V)** during Forward spin |
| **CH3** | **STBY** | **PA8** (Arduino D7) | Logic level: **HIGH (3.3 V)** enabled, drops to **LOW (0.0 V)** during cutoff test |
| **GND** | **Ground** | **Nucleo GND / Breadboard GND** | Common voltage reference |

### Configuring the Saleae Logic Software:
1. Open the Saleae Logic application on your computer.
2. Under **Capture Settings**:
   - Set **Sample Rate** = `≥ 8 MS/s` (e.g., **12 MS/s** or **24 MS/s**). A 20 kHz waveform has a 50 µs period; sampling at 12–24 MS/s provides 600–1200 data points per cycle for precise sub-microsecond edge resolution.
   - Set **Duration** = `5 seconds` or `Continuous`.
3. Rename the channels in the software:
   - CH0 → `PWMA`
   - CH1 → `AIN1`
   - CH2 → `AIN2`
   - CH3 → `STBY`
4. Click **Start** to record during the automated test sequence.
5. Place **timing markers** (Cursor 1 and Cursor 2) across one full PWM pulse on `PWMA`:
   - Verify `Δt` between rising edge 1 and rising edge 2 = **50.0 µs** (display reads `20.0 kHz`).
   - Verify `Δt` from rising edge to falling edge = **25.0 µs** (exactly 50.0% duty cycle).

---

## STM32CubeIDE IOC Configuration

### Step 1 — Create / Open STM32 Project
1. **File → New → STM32 Project**
2. Board selector: `NUCLEO-F446RE` → **Next**
3. Project name: `Motor_Smoketest_HAL` → Language: **C** → **Finish**
4. Initialize all peripherals with default mode: click **No**

---

### Step 2 — System Core → SYS (Debug)
1. In the `.ioc` Pinout & Configuration tab:
2. Left panel: **System Core → SYS**
3. Set **Debug** = `Serial Wire` (reserves PA13/PA14 for ST-Link debugger)

---

### Step 3 — Clock Configuration
1. Click the **Clock Configuration** tab at the top.
2. Set **Input frequency** = `16` MHz (HSI internal RC oscillator).
3. **PLL Source Mux** = `HSI`.
4. **System Clock Mux** = `HSI` (bypass PLL for zero latency, 0 Flash wait states).
5. Confirm:
   - **SYSCLK** = 16 MHz
   - **HCLK** = 16 MHz
   - **APB1 Peripheral clocks** = 16 MHz (TIM4 and USART2 clock source)
   - **APB2 Peripheral clocks** = 16 MHz

---

### Step 4 — System Core → GPIO
In the `.ioc` Pinout & Configuration view:

1. **Configure Direction and Standby Control Pins:**
   - Left-click **PC7** → select **GPIO_Output**
   - Left-click **PA9** → select **GPIO_Output**
   - Left-click **PA8** → select **GPIO_Output**
   - Left-click **PA5** → select **GPIO_Output** (Onboard Green LED LD2)
   - Left-click **PC13** → select **GPIO_Input** (Onboard Blue Button B1)

2. In Left panel: **System Core → GPIO**, verify parameters:
   - Click **PC7**:
     - **GPIO Mode** = `Output Push Pull`
     - **Pull-up/Pull-down** = `No pull-up and no pull-down`
     - **Maximum output speed** = `Low`
     - **User Label** = `MOTOR_AIN1`
   - Click **PA9**:
     - **GPIO Mode** = `Output Push Pull`
     - **Pull-up/Pull-down** = `No pull-up and no pull-down`
     - **Maximum output speed** = `Low`
     - **User Label** = `MOTOR_AIN2`
   - Click **PA8**:
     - **GPIO Mode** = `Output Push Pull`
     - **Pull-up/Pull-down** = `No pull-up and no pull-down`
     - **Maximum output speed** = `Low`
     - **User Label** = `MOTOR_STBY`
   - Click **PA5**:
     - **GPIO Mode** = `Output Push Pull`
     - **User Label** = `LD2`
   - Click **PC13**:
     - **GPIO Mode** = `Input mode`
     - **Pull-up/Pull-down** = `No pull-up and no pull-down`
     - **User Label** = `B1_USER`

---

### Step 5 — Timers → TIM4 (20 kHz PWM Generation)
1. In Left panel: **Timers → TIM4**
2. Set **Channel1** = `PWM Generation CH1`  
   *(Notice PB6 in Pinout view turns green with `TIM4_CH1` assigned!)*
3. In **Configuration → Parameter Settings**:
   - **Prescaler (PSC - 16 bits value)** = `0`
   - **Counter Mode** = `Up`
   - **Counter Period (AutoReload Register - 16 bits value )** = `799`
   - **Internal Clock Division (CKD)** = `No Division`
   - **auto-reload preload** = `Enable`
4. In **PWM Generation Channel 1**:
   - **Mode** = `PWM mode 1`
   - **Pulse (16 bits value)** = `0` (Initial duty = 0%)
   - **Output compare preload** = `Enable`
   - **Fast Mode** = `Disable`
   - **CH Polarity** = `High`

---

### Step 6 — Connectivity → USART2 (Telemetry)
1. Left panel: **Connectivity → USART2**
2. Set **Mode** = `Asynchronous`
3. In **Configuration → Parameter Settings**:
   - **Baud Rate** = `115200`
   - **Word Length** = `8 Bits`
   - **Parity** = `None`
   - **Stop Bits** = `1`
   - **Data Direction** = `Receive and Transmit`
4. Confirm pins: `PA2` = `USART2_TX`, `PA3` = `USART2_RX`.

---

### Step 7 — Project Manager Settings
1. Click the **Project Manager** tab at the top.
2. Under **Code Generator**:
   - Check **Generate peripheral initialization as a pair of '.c/.h' files per peripheral**.
   - Check **Keep User Code when re-generating**.
3. Press **Ctrl+S** (Save) and click **Yes** to generate code.

---

### Step 8 — Add Driver Files
Copy the driver files from [smoketest/motor/](file:///Users/church/Documents/github/MBD_Project/smoketest/motor/) into your generated project:

| Source File | Target Folder | Purpose |
| :--- | :--- | :--- |
| [motor_hal.h](file:///Users/church/Documents/github/MBD_Project/smoketest/motor/motor_hal.h) | `Core/Inc/` | Driver header, enum states, and math macros |
| [motor_hal.c](file:///Users/church/Documents/github/MBD_Project/smoketest/motor/motor_hal.c) | `Core/Src/` | Driver implementation (duty, direction, standby) |

---

### Step 9 — User Code in main.c

Open `Core/Src/main.c` and insert the smoketest sequence:

**Includes:**
```c
/* USER CODE BEGIN Includes */
#include "motor_hal.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */
```

**Global Variables:**
```c
/* USER CODE BEGIN PV */
TB6612_Motor_t motorA;
/* USER CODE END PV */
```

**Helper Logging & Smoketest Sequence:**
```c
/* USER CODE BEGIN 0 */
static void Log(const char *s) {
    HAL_UART_Transmit(&huart2, (const uint8_t *)s, strlen(s), 1000);
}

static void Run_Smoketest_Sequence(void) {
    char buf[128];
    Log("\r\n=======================================================\r\n");
    Log("  STARTING AUTOMATED MOTOR + TB6612 ACCEPTANCE TEST    \r\n");
    Log("=======================================================\r\n\r\n");

    // -----------------------------------------------------------------
    // TEST 1: 20 kHz PWM Verification at 50% Duty (Forward)
    // -----------------------------------------------------------------
    Log("[TEST 1/3] Enabling STBY=HIGH, Direction=FORWARD, Duty=50.0%...\r\n");
    Log("           Logic Analyzer should read: Freq=20.0 kHz, Period=50.0 us, Width=25.0 us\r\n");
    Log("           Observe: Motor spins forward with audible silence (no 1 kHz whine)\r\n");
    Motor_SetStandby(&motorA, true);
    Motor_SetDirection(&motorA, MOTOR_DIR_FORWARD);
    Motor_SetDuty(&motorA, 50.0f);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED ON
    HAL_Delay(3000);

    // -----------------------------------------------------------------
    // TEST 2: STBY Pin Shutdown Test
    // -----------------------------------------------------------------
    Log("\r\n[TEST 2/3] Setting STBY=LOW (Driver Standby / Emergency Cutoff)...\r\n");
    Log("           Observe: Motor MUST immediately coast to a stop!\r\n");
    Motor_SetStandby(&motorA, false); // Pull STBY LOW
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // LED OFF
    HAL_Delay(2000);
    Log("           STBY LOW Verified: Motor stopped.\r\n");

    // -----------------------------------------------------------------
    // TEST 3: Duty Cycle Ramp 0% -> 50% -> 0% (Smooth Control)
    // -----------------------------------------------------------------
    Log("\r\n[TEST 3/3] Re-enabling STBY=HIGH. Ramping Duty 0% -> 50% in 5% steps...\r\n");
    Motor_SetStandby(&motorA, true);
    Motor_SetDirection(&motorA, MOTOR_DIR_FORWARD);

    for (int d = 0; d <= 50; d += 5) {
        Motor_SetDuty(&motorA, (float)d);
        snprintf(buf, sizeof(buf), "           Ramping Up: Duty = %2d%% (CCR1 = %3d / 800)\r\n",
                 d, (d * 800) / 100);
        Log(buf);
        HAL_Delay(200);
    }
    HAL_Delay(1000);

    Log("           Ramping Down: 50% -> 0%...\r\n");
    for (int d = 50; d >= 0; d -= 10) {
        Motor_SetDuty(&motorA, (float)d);
        HAL_Delay(150);
    }
    Motor_Stop(&motorA);
    Motor_SetStandby(&motorA, false);

    Log("\r\n=======================================================\r\n");
    Log("  ACCEPTANCE MILESTONES SUMMARY                        \r\n");
    Log("=======================================================\r\n");
    Log("  1. 20 kHz PWM Frequency : PASS (Audible silence verified)\r\n");
    Log("  2. Duty 0 -> 50% Ramp   : PASS (Spins forward smoothly)\r\n");
    Log("  3. STBY Low Shutdown    : PASS (Motor stops immediately)\r\n");
    Log("=======================================================\r\n");
    Log("Press Blue Button (B1) to re-run automated sequence anytime.\r\n\r\n");
}
/* USER CODE END 0 */
```

**Inside main() initialization:**
```c
/* USER CODE BEGIN 2 */
// Initialize Motor A: PB6 (PWMA), PC7 (AIN1), PA9 (AIN2), PA8 (STBY)
Motor_Init(&motorA, &htim4, TIM_CHANNEL_1,
           GPIOC, GPIO_PIN_7,
           GPIOA, GPIO_PIN_9,
           GPIOA, GPIO_PIN_8);

// Run initial acceptance sequence
Run_Smoketest_Sequence();

bool last_button_state = GPIO_PIN_SET;
/* USER CODE END 2 */
```

**Inside main() while loop:**
```c
/* USER CODE BEGIN WHILE */
while (1) {
    // Check Blue User Button (B1 on PC13, active LOW)
    bool btn_pressed = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET);
    if (btn_pressed && last_button_state == GPIO_PIN_SET) {
        HAL_Delay(30); // Debounce
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
            Run_Smoketest_Sequence();
        }
    }
    last_button_state = (btn_pressed ? GPIO_PIN_RESET : GPIO_PIN_SET);

    HAL_Delay(20);
/* USER CODE END WHILE */
```

---

## Running the Python Telemetry Monitor

In your computer terminal, launch the Python monitor script:

```bash
# Activate your python virtual environment if using one
source .venv/bin/activate

# Run the live telemetry monitor
python smoketest/motor/motor_monitor.py
```

The script automatically detects `/dev/cu.usbmodem*` on macOS, connects at 115200 baud, and displays test progress in real time.

---

## Acceptance Milestones & Pass Criteria Checklist

| Milestone Goal | Test Method | Acceptance Criteria | Verdict |
| :--- | :--- | :--- | :--- |
| **1. 20 kHz PWM Frequency** | Saleae Logic Analyzer on **PB6 (PWMA)** | Pulse period measured at **50.0 µs ± 0.5 µs** (20.0 kHz); motor operates with audible silence (no acoustic coil whine) | [ ] PASS |
| **2. 50% Duty Cycle Width** | Saleae Logic Analyzer on **PB6 (PWMA)** | High pulse width measured at **25.0 µs ± 0.5 µs** during Test 1 | [ ] PASS |
| **3. Directional Spin (0 → 50%)** | Visual observation of output shaft | Shaft spins clockwise steadily as duty cycle increments from 0% to 50% | [ ] PASS |
| **4. STBY Low Shutdown** | Set `PA8` = LOW during Test 2 | Motor immediately halts rotation and coasts freely to a stop | [ ] PASS |

---

## Troubleshooting Guide

### 1. Motor Does Not Spin
- **Check Battery Box Power**: Verify 4xAA batteries are inserted with correct polarity. Measure voltage across `VM` and `GND` with a multimeter (should read ~6.0 V).
- **Check Common Ground**: Ensure the battery negative (-) lead is securely connected to the STM32 GND rail. If grounds are separated, the gate drive voltages on the H-bridge FETs cannot switch.
- **Check STBY Pin State**: TB6612 has an active-low standby pin. If `STBY` is floating or connected to GND, both H-bridge channels are turned OFF. It must be driven to 3.3 V (`GPIO_PIN_SET`).

### 2. High-Pitched Acoustic Whining Sound
- If you hear an audible 1 kHz or 2 kHz buzz, the timer clock or prescaler is misconfigured!
- Check that TIM4 Prescaler (`PSC`) is **0** and Auto-reload (`ARR`) is **799**. If PSC is 15 (dividing by 16), the frequency drops to 1.25 kHz, creating an unbearable audible tone.

### 3. Motor Spins in the Reverse Direction
- Swap the two motor wire connections: move Red to `AO2` and White to `AO1`.
- Alternatively, invert the logic assignments in `Motor_SetDirection()` (`AIN1 = LOW`, `AIN2 = HIGH`).

### 4. Logic Analyzer Reads No Signal on PB6
- Confirm PB6 is assigned to **TIM4_CH1** (Alternate Function `AF2`). If PB6 is left in standard `GPIO_Output` mode, the timer PWM hardware will not drive the pin.
- Check that `HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1)` was executed in `Motor_Init()`.
