# KY-023 Dual-Axis Joystick Smoketest (HAL Version)

Measures dual-axis analog position (VRx on PA0, VRy on PA1) and pushbutton switch (SW on PA10) 
using ADC1 (12-bit) and displays real-time telemetry over USART2 at 115200 baud.

> **Target Milestone:**  
> X and Y read 0–4095 over full travel; centre noise σ ≤ 10 counts (≈ 8 mV) at rest.

---

## Hardware Wiring

| Component Pin | Wire Color (Suggested) | Nucleo location | Description |
| :--- | :--- | :--- | :--- |
| **GND** | Black | Any **GND** (CN6 pin 6 or CN8 pin 5) | System Ground |
| **+5V / +V** | Red | **3.3V** (CN6 pin 4) | **CRITICAL: Connect to 3.3V, NOT 5V!** |
| **VRx** | Yellow | **PA0** (CN8 pin 1, Arduino "A0") | ADC1_IN0 (X-axis pot wiper) |
| **VRy** | Green | **PA1** (CN8 pin 2, Arduino "A1") | ADC1_IN1 (Y-axis pot wiper) |
| **SW** | Blue | **PA10** (CN9 pin 3, Arduino "D2") | Digital input (pushbutton, active LOW) |

> **Critical Rule on Power Voltage:**  
> The PCB silkscreen says `+5V`, but **you MUST connect it to STM32 3.3V (3V3)**.  
> STM32 12-bit ADC full-scale reference is V_REF+ = 3.3V (4095 counts). Powering from 3.3V maps the physical mechanical motion cleanly to **0–4095**. If powered from 5V, values saturate at 4095 early, and voltages above 3.6V risk damaging the non-5V-tolerant ADC pins.

---

## STM32CubeIDE IOC Configuration

### Step 1 — Create a new STM32 project

1. **File → New → STM32 Project**
2. Board selector: type `NUCLEO-F446RE` → select it → **Next**
3. Project name: `Joystick_Smoketest_HAL`
4. Targeted language: **C**
5. Click **Finish** → when asked "Initialize all peripherals with their default mode?", click **No**

---

### Step 2 — System Core → SYS (Debug)

In the `.ioc` Pinout & Configuration tab:

1. Left panel: **System Core → SYS**
2. Set **Debug** = `Serial Wire`

> This reserves PA13/PA14 for the ST-Link debugger (already done by default on Nucleo).

---

### Step 3 — Clock Configuration

1. Click the **Clock Configuration** tab at the top
2. Set **Input frequency** = `16` MHz (HSI internal RC oscillator)
3. **PLL Source Mux** = `HSI`
4. **System Clock Mux** = `HSI` (bypass the PLL — keeps it simple and robust for this smoketest)
5. Confirm:
   - **HCLK** = 16 MHz
   - **APB1 Peripheral clocks** = 16 MHz  (USART2 clock)
   - **APB2 Peripheral clocks** = 16 MHz  (ADC1 clock source)

> **Smoketest Note:** Bypassing the PLL and running directly from HSI 16 MHz requires 0 Flash wait states, has zero startup delay, and avoids any external crystal dependencies. When integrating with CAN and other subsystems later, the unified system clock can be configured then.

---

### Step 4 — System Core → GPIO (Button & LED)

In the `.ioc` Pinout & Configuration tab:

1. In the **Pinout view**:
   - Left-click **PA10** → select **GPIO_Input**
   - Left-click **PA5** → select **GPIO_Output**
2. Left panel: **System Core → GPIO**:
   - Click **PA10**:
     - **GPIO Mode** = `Input mode`
     - **GPIO Pull-up/Pull-down** = `Pull-up`  ← **Required!** Switch pulls to GND when clicked
     - **User Label** = `JOY_SW`
   - Click **PA5**:
     - **GPIO Mode** = `Output Push Pull`
     - **GPIO Pull-up/Pull-down** = `No pull-up and no pull-down`
     - **Maximum output speed** = `Low`
     - **User Label** = `LD2`

---

### Step 5 — Connectivity → USART2 (Debug Telemetry)

1. Left panel: **Connectivity → USART2**
2. Set **Mode** = `Asynchronous`
3. In **Configuration → Parameter Settings**:
   - **Baud Rate** = `115200`
   - **Word Length** = `8 Bits`
   - **Parity** = `None`
   - **Stop Bits** = `1`
   - **Data Direction** = `Receive and Transmit`
   - **Over Sampling** = `16 Samples`
4. Confirm pins in Pinout view:
   - `PA2` = `USART2_TX` (connected to ST-Link VCP)
   - `PA3` = `USART2_RX` (connected to ST-Link VCP)

---

### Step 6 — Analog → ADC1 (Dual-Axis Joystick)

1. Left panel: **Analog → ADC1**
2. Set **IN0** = `IN0 (single-ended)`  *(PA0 turns green in Pinout view)*
3. Set **IN1** = `IN1 (single-ended)`  *(PA1 turns green in Pinout view)*
4. In **Configuration → Parameter Settings**:
   - **Clock Prescaler** = `PCLK2 divided by 4`  (= 4 MHz ADC clock) or `divided by 2` (= 8 MHz)
   - **Resolution** = `12 bits (15 ADC Clock cycles)`
   - **Data Alignment** = `Right alignment`
   - **Scan Conversion Mode** = `Disabled`
   - **Continuous Conversion Mode** = `Disabled`
   - **Discontinuous Conversion Mode** = `Disabled`
   - **DMA Continuous Requests** = `Disabled`
   - **End Of Conversion selection** = `End of single conversion`
   - **Number of Conversion** = `1`
   - **External Trigger Conversion Source** = `Regular Conversion launched by software`
5. In **Rank 1 → Sampling Time** = `84 Cycles` *(or 144 Cycles / 480 Cycles)*

> **Why 84 Cycles (or longer) is essential:**  
> The KY-023 joystick uses 10 kΩ potentiometers. A longer sampling time allows the internal sample-and-hold capacitor to charge fully, eliminating source-impedance settling error and guaranteeing center noise **σ ≤ 10 counts (≈ 8 mV)**.

---

### Step 7 — Project Manager Settings

1. Click the **Project Manager** tab at the top
2. Under **Project**:
   - **Toolchain / IDE** = `STM32CubeIDE`
3. Under **Code Generator**:
   - Check **Generate peripheral initialization as a pair of '.c/.h' files per peripheral**
   - Check **Keep User Code when re-generating**
4. Click **Save** (Ctrl+S) → CubeIDE will ask "Do you want to generate code?" → click **Yes**

---

### Step 8 — Add Project Driver Files

After code generation, your project structure looks like:

```text
Joystick_Smoketest_HAL/
├── Core/
│   ├── Inc/  main.h
│   └── Src/  main.c   gpio.c   adc.c   usart.c   stm32f4xx_hal_msp.c
├── Drivers/
│   └── STM32F4xx_HAL_Driver/
```

**Copy these files from `smoketest/joystick/` into your project:**

| Source File | Destination Folder | Description |
| :--- | :--- | :--- |
| `joystick_hal.h` | `Core/Inc/` | Driver header and data structures |
| `joystick_hal.c` | `Core/Src/` | Driver source, ADC channel polling, and noise σ math |

Right-click the `Core/` folder in CubeIDE → **Refresh** so the IDE indexes the new files.

---

### Step 9 — Fill in USER CODE sections in main.c

Open **`Core/Src/main.c`** and add the code into the designated matching comment blocks:

**Includes:**
```c
/* USER CODE BEGIN Includes */
#include "joystick_hal.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */
```

**Before main() — UART Log helper:**
```c
/* USER CODE BEGIN 0 */
static void Log(const char *s) {
    HAL_UART_Transmit(&huart2, (const uint8_t *)s, strlen(s), 1000);
}
/* USER CODE END 0 */
```

**After peripheral init in main():**
```c
/* USER CODE BEGIN 2 */
Log("\r\n==========================================\r\n");
Log("  KY-023 Joystick Smoketest (12-bit ADC)  \r\n");
Log("  Target: X/Y 0-4095, Centre Noise sigma <= 10\r\n");
Log("==========================================\r\n");

// Initialize joystick driver (ADC1, PA10 as SW button)
Joystick_State_t joy;
Joystick_Init(&hadc1, GPIOA, GPIO_PIN_10);
Joystick_ResetTravel(&joy);

// Benchmark centre noise at rest (100 samples)
Log("[TEST 1] Measuring centre noise at rest (keep stick untouched)...\r\n");
if (Joystick_ComputeNoise(&joy, 100) == JOYSTICK_OK) {
    char buf[128];
    snprintf(buf, sizeof(buf),
             "[CENTRE] Mean: X=%.1f (%.3fV), Y=%.1f (%.3fV)\r\n"
             "[NOISE ] Sigma: X=%.2f counts (%.2f mV), Y=%.2f counts (%.2f mV)\r\n",
             joy.mean_x, (joy.mean_x * 3.3f / 4095.0f),
             joy.mean_y, (joy.mean_y * 3.3f / 4095.0f),
             joy.std_x,  (joy.std_x  * 3300.0f / 4095.0f),
             joy.std_y,  (joy.std_y  * 3300.0f / 4095.0f));
    Log(buf);

    if (joy.std_x <= 10.0f && joy.std_y <= 10.0f) {
        Log("[RESULT] Centre Noise: [PASS] (sigma <= 10 counts / 8 mV)\r\n\r\n");
    } else {
        Log("[RESULT] Centre Noise: [FAIL] Check ground / wiring!\r\n\r\n");
    }
}

Log("[TEST 2] Live Monitor & Full Travel Test\r\n");
Log("         Move joystick to all 4 edges to verify range 0 -> 4095.\r\n");
Log("         Press down on joystick to verify SW button.\r\n\r\n");

uint32_t last_print = HAL_GetTick();
/* USER CODE END 2 */
```

**Main while loop:**
```c
/* USER CODE BEGIN WHILE */
while (1) {
    Joystick_Read(&joy);

    // Toggle onboard LD2 LED when button pressed (active LOW)
    if (joy.btn_sw == 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    }

    // Stream telemetry every 100 ms
    if ((HAL_GetTick() - last_print) >= 100) {
        last_print = HAL_GetTick();

        char line[128];
        snprintf(line, sizeof(line),
                 "X=%4u Y=%4u | SW=%s | Travel X:[%4u..%4u] Y:[%4u..%4u]\r\n",
                 joy.raw_x, joy.raw_y,
                 (joy.btn_sw == 0) ? "PRESSED" : "UP     ",
                 joy.min_x, joy.max_x,
                 joy.min_y, joy.max_y);
        Log(line);
    }
/* USER CODE END WHILE */
```

---

## Build and Flash

1. **Project → Build All** (Ctrl+B)
2. Zero errors expected
3. **Run → Debug** (F11) → **Resume** (F8) to run

---

## Acceptance Test Procedure

### Method A: Using PC Python Monitor
With your virtual environment activated, run:

```bash
source .venv/bin/activate
python smoketest/joystick/joystick_monitor.py
```

### Method B: Using Serial Terminal
Open any terminal (minicom, screen, PuTTY) at **115200 baud, 8N1**:

```bash
screen /dev/cu.usbmodem* 115200
```

### Verification Criteria Checklist

1. **Centre Noise at Rest:**
   - Keep hands off the stick.
   - Verify `Sigma X <= 10.0 counts` and `Sigma Y <= 10.0 counts` (typical reading is 2 to 4 counts).
2. **Full Travel Range:**
   - Push stick fully **LEFT** → X drops to ≤ 100.
   - Push stick fully **RIGHT** → X rises to ≥ 3950.
   - Push stick fully **DOWN** → Y drops to ≤ 100.
   - Push stick fully **UP** → Y rises to ≥ 3950.
3. **Pushbutton Switch (SW):**
   - Click the stick down → monitor shows `SW=PRESSED` and green LED **LD2** lights up.
   - Release the stick → monitor shows `SW=UP` and **LD2** turns off.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
| :--- | :--- | :--- |
| **Noise σ > 10 counts** | Sampling time too short or loose GND | Set ADC Sampling Time to `84 Cycles` or `144 Cycles`; verify firm ground connection. |
| **Max value clips at 4095 early** | Powered from 5V instead of 3.3V | Move power jumper wire from Nucleo `5V` to **`3V3`**. |
| **ADC reads 0 always** | Pins not assigned in IOC | Verify PA0=IN0, PA1=IN1 in IOC; check wiring on CN8 pins 1 & 2. |
| **SW always reads PRESSED (0)** | Missing pull-up resistor | Configure PA10 with `GPIO_PULLUP` in IOC GPIO tab. |
| **User code erased on re-generate** | Code placed outside user blocks | Keep all manual code strictly inside `/* USER CODE BEGIN/END */`. |
