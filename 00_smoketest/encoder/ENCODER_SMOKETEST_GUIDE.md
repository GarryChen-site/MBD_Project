# N20 Geared Motor TIM3 Encoder Smoketest (HAL Version)

Measures shaft position, direction, and revolution counts for the **N20 DC Geared Motor with Magnetic Hall Encoder (XY-N20)** 
using **TIM3 in hardware Encoder Mode (TI1 and TI2, ×4 count)** on the STM32F446RE, streaming real-time telemetry over USART2 at 115200 baud.

> **Target Milestone:**  
> Rotate output shaft exactly 1 turn by hand → count = N_rev ± 2% ; direction sign correct (CW = +, CCW = -).

---

## Hardware Specifications & Wire Pinout

Based on the **XY-N20 Geared Motor Encoder Datasheet**:

- **Motor Type**: N20 DC Geared Motor with integrated magnetic Hall encoder
- **Base Resolution**: **7 PPR** (pulses per motor rotor revolution)
- **Signal Output**: Quadrature square wave AB phase (Green and Yellow wires)
- **Encoder Operating Voltage**: 3.3V – 5V (Supports 3.3V directly from STM32!)
- **Hall Frequency Response**: 100 kHz

```
       +-------------------------------------------------------------+
       |             N20 MOTOR WITH INTEGRATED ENCODER               |
       |                                                             |
       |   [ DC Motor ] === [ 7 PPR Hall Disc ] === [ Gearbox (G) ] === [ Output Shaft ]
       +-------------------------------------------------------------+
```

---

### Wiring Table: N20 Encoder Cable to NUCLEO-F446RE

| Wire Color | Japanese Datasheet Label | Function | Nucleo-F446RE Pin | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Blue (青)** | エンコーダ電源 - | Encoder GND | **GND** (CN6 pin 6) | Ground reference |
| **Black (黒)** | エンコーダ電源 + | Encoder VCC (3.3V - 5V) | **3.3V** (CN6 pin 4) | **CRITICAL: Black is VCC (+)** |
| **Green (緑)** | 信号フィードバック (Phase A) | Hall Phase A | **PA6** (CN5 pin 5 / "D12") | TIM3_CH1 (TI1 Input) |
| **Yellow (黄)** | 信号フィードバック (Phase B) | Hall Phase B | **PA7** (CN5 pin 6 / "D11") | TIM3_CH2 (TI2 Input) |
| **White (白)** | モーター出力 - | Motor Terminal (-) | *Leave Disconnected* | Not needed for hand-turn test |
| **Red (赤)** | モーター出力 + | Motor Terminal (+) | *Leave Disconnected* | Not needed for hand-turn test |

> [!CAUTION]
> **CRITICAL WIRING ALERT (Power Colors):**  
> On this specific XY-N20 cable harness:  
> - **Blue = GND (-)**  
> - **Black = VCC (+)**  
> Do **NOT** assume Black is Ground! Connecting Black to GND and Blue to 3.3V will reverse-bias the Hall sensor IC.

---

## Gear Ratio & Encoder Counting Math

1. **Motor Rotor Resolution (Before Gearbox)**:
   - The magnetic disc on the motor shaft has **7 PPR**.
   - With TIM3 in **TI1 and TI2 mode (×4 quadrature decoding)**, the timer counts on both edges of both phases:
     **Motor Counts / Rotor Turn = 7 × 4 = 28 counts**
2. **Output Shaft Resolution (After Gearbox)**:
   - The output shaft spins through gear ratio **G** (e.g. 1:30, 1:50, 1:100, 1:150, 1:210, 1:298):
     **N_rev (counts per output shaft turn) = 28 × G**
   - *Example Estimates*:
     - If G = 30:1 → N_rev = 28 × 30 = **840 counts**
     - If G = 50:1 → N_rev = 28 × 50 = **1400 counts**
     - If G = 100:1 → N_rev = 28 × 100 = **2800 counts**
     - If G = 150:1 → N_rev = 28 × 150 = **4200 counts**
3. **Why the 5-Step Tape Procedure is Required**:
   - Because different N20 motor variants have different exact gear ratios, turning the output shaft by hand with a tape mark establishes your motor's exact **N_rev** and verifies encoder repeatability within **±2%**.

---

## The 5-Step Smoketest Procedure

1. **Configure TIM3 in Encoder mode, TI1 and TI2 (the ×4 count).**
2. **Print the counter value every 100 ms to the serial terminal.**
3. **Put the tape mark at a known position on the output shaft (after the gearbox), and zero the counter.**
4. **Turn the shaft by hand exactly one full turn and read the count.**
5. **Repeat 3–5 times and take the average.**

---

## STM32CubeIDE IOC Configuration

### Step 1 — Create / Open STM32 Project

1. **File → New → STM32 Project**
2. Board selector: `NUCLEO-F446RE` → **Next**
3. Project name: `Encoder_Smoketest_HAL` → Language: **C** → **Finish**
4. Initialize peripherals with default mode: click **No**

---

### Step 2 — System Core → SYS (Debug)

In the `.ioc` Pinout & Configuration tab:
1. Left panel: **System Core → SYS**
2. Set **Debug** = `Serial Wire` (reserves PA13/PA14 for ST-Link)

---

### Step 3 — Clock Configuration

1. Click the **Clock Configuration** tab
2. Set **Input frequency** = `16` MHz (HSI internal oscillator)
3. **PLL Source Mux** = `HSI`
4. **System Clock Mux** = `HSI` (bypass PLL — keeps smoketest simple and standalone)
5. Confirm:
   - **HCLK** = 16 MHz
   - **APB1 Peripheral clocks** = 16 MHz  (TIM3 and USART2 clock source)
   - **APB2 Peripheral clocks** = 16 MHz

---

### Step 4 — System Core → GPIO (Button & LED)

1. In the **Pinout view**:
   - Left-click **PC13** → `GPIO_Input` (Blue button B1)
   - Left-click **PA5** → `GPIO_Output` (Green LED LD2)
2. Left panel: **System Core → GPIO**:
   - **PC13**: Mode = `Input mode`, Label = `B1_Pin`
   - **PA5**: Mode = `Output Push Pull`, Speed = `Low`, Label = `LD2_Pin`

---

### Step 5 — Connectivity → USART2 (Telemetry)

1. Left panel: **Connectivity → USART2**
2. Set **Mode** = `Asynchronous`
3. Parameter Settings:
   - Baud Rate = `115200`
   - Word Length = `8 Bits`, Parity = `None`, Stop Bits = `1`
4. Confirm pins: `PA2` = `USART2_TX`, `PA3` = `USART2_RX`

---

### Step 6 — Timers → TIM3 (Encoder Mode TI1 and TI2)

1. Left panel: **Timers → TIM3**
2. Set **Combined Channels** = `Encoder Mode`
3. In **Configuration → Parameter Settings**:
   - **Counter Settings**:
     - **Prescaler** = `0`
     - **Counter Mode** = `Up`
     - **Counter Period (ARR)** = `65535`  (16-bit auto-reload: 0xFFFF)
     - **Internal Clock Division** = `No division`
   - **Encoder Mode**:
     - **Encoder Mode** = `Encoder Mode TI1 and TI2`  ← **4x Quadrature Decoding**
   - **Input Capture 1 (TI1)**:
     - **Polarity** = `Rising Edge`
     - **Input Filter** = `10`  ← **Digital noise filter for clean edges**
   - **Input Capture 2 (TI2)**:
     - **Polarity** = `Rising Edge`
     - **Input Filter** = `10`
4. In Pinout view, confirm:
   - `PA6` = `TIM3_CH1`
   - `PA7` = `TIM3_CH2`
5. Under **System Core → GPIO**, click on **PA6** and **PA7**:
   - Set **GPIO Pull-up/Pull-down** = **`Pull-up`**

---

### Step 7 — Project Manager Settings

1. Click **Project Manager** tab
2. Toolchain / IDE = `STM32CubeIDE`
3. Code Generator:
   - Check **Generate peripheral initialization as a pair of '.c/.h' files per peripheral**
   - Check **Keep User Code when re-generating**
4. Click **Save** (Ctrl+S) → generate code.

---

### Step 8 — Add Driver Files to Project

Copy the files from `smoketest/encoder/` into your project:

| Source File | Destination Folder | Description |
| :--- | :--- | :--- |
| `encoder_hal.h` | `Core/Inc/` | Driver header, N20 7 PPR specs, and trial math |
| `encoder_hal.c` | `Core/Src/` | 16-bit wrap-around handling and 3-5 trial averaging |

---

### Step 9 — User Code in main.c

Open `Core/Src/main.c` and insert the code from `smoketest/encoder/main.c`:

**Includes:**
```c
/* USER CODE BEGIN Includes */
#include "encoder_hal.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */
```

**Before main():**
```c
/* USER CODE BEGIN 0 */
static void Log(const char *s) {
    HAL_UART_Transmit(&huart2, (const uint8_t *)s, strlen(s), 1000);
}
/* USER CODE END 0 */
```

**Inside main() initialization:**
```c
/* USER CODE BEGIN 2 */
Log("\r\n=======================================================\r\n");
Log("  N20 DC Geared Motor Encoder Smoketest (TIM3 TI1/TI2) \r\n");
Log("=======================================================\r\n");
Log("Spec: N20 Hall Encoder = 7 PPR (28 counts / motor rotor rev)\r\n");
Log("Goal: 1 turn of output shaft -> count = N_rev +- 2%\r\n\r\n");

Encoder_State_t enc = {0};
Encoder_Init(&htim3);
Encoder_Zero(&enc);

uint32_t last_print = HAL_GetTick();
bool last_button_state = GPIO_PIN_SET;
bool waiting_for_turn = false;
/* USER CODE END 2 */
```

**Inside main() while loop:**
```c
/* USER CODE BEGIN WHILE */
while (1) {
    Encoder_Update(&enc);

    // Blue Button (B1) handles Zeroing (step 3) and Recording (step 4 & 5)
    bool btn_pressed = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET);
    if (btn_pressed && last_button_state == GPIO_PIN_SET) {
        HAL_Delay(30);
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
            if (!waiting_for_turn) {
                // Step 3: Zero the counter at tape mark
                Encoder_Zero(&enc);
                waiting_for_turn = true;
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
                Log("\r\n>>> [TRIAL START] Counter ZEROED. Turn shaft 1 full turn, then press B1... <<<\r\n");
            } else {
                // Step 4 & 5: Record trial
                uint8_t count = Encoder_RecordTrial(&enc);
                waiting_for_turn = false;
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

                char msg[128];
                snprintf(msg, sizeof(msg), "\r\n[TRIAL %u RECORDED] Counts = %+ld (Dir: %s)\r\n",
                         (unsigned)count, (long)enc.total_counts,
                         (enc.total_counts >= 0) ? "CW (+)" : "CCW (-)");
                Log(msg);

                if (count >= 3) {
                    float avg_n_rev = 0.0f;
                    float max_err = 0.0f;
                    bool pass = Encoder_ComputeAverage(&enc, &avg_n_rev, &max_err);

                    char line_buf[128];
                    Log("\r\n=======================================================\r\n");
                    Log("  3-5 TRIAL CALIBRATION & ACCEPTANCE SUMMARY\r\n");
                    Log("=======================================================\r\n");

                    for (uint8_t i = 0; i < enc.trial_count; i++) {
                        snprintf(line_buf, sizeof(line_buf), "  Trial %u            : %+ld counts\r\n",
                                 (unsigned)(i + 1), (long)enc.trial_counts[i]);
                        Log(line_buf);
                    }

                    int32_t avg_int = (int32_t)avg_n_rev;
                    int32_t avg_frac = (int32_t)labs((int32_t)(avg_n_rev * 10.0f)) % 10;
                    snprintf(line_buf, sizeof(line_buf),
                             "  Average N_rev       : %+ld.%01ld counts / turn\r\n", (long)avg_int, (long)avg_frac);
                    Log(line_buf);

                    int32_t ratio_int = (int32_t)enc.estimated_ratio;
                    int32_t ratio_frac = (int32_t)labs((int32_t)(enc.estimated_ratio * 100.0f)) % 100;
                    snprintf(line_buf, sizeof(line_buf),
                             "  Estimated Gear Ratio: %ld.%02ld : 1 (G = N_rev / 28)\r\n", (long)ratio_int, (long)ratio_frac);
                    Log(line_buf);

                    int32_t err_int = (int32_t)max_err;
                    int32_t err_frac = (int32_t)labs((int32_t)(max_err * 100.0f)) % 100;
                    snprintf(line_buf, sizeof(line_buf),
                             "  Max Deviation       : %ld.%02ld%% (Milestone <= 2.0%%)\r\n", (long)err_int, (long)err_frac);
                    Log(line_buf);

                    snprintf(line_buf, sizeof(line_buf), "  Direction Sign      : %s\r\n",
                             (avg_n_rev >= 0) ? "CW (+, PASS)" : "CCW (-, PASS)");
                    Log(line_buf);

                    snprintf(line_buf, sizeof(line_buf), "  VERDICT             : %s\r\n",
                             pass ? ">>> [PASS] <<<" : ">>> [FAIL] <<<");
                    Log(line_buf);

                    Log("=======================================================\r\n");
                    Log("Press B1 to begin another trial if desired.\r\n\r\n");
                }
            }
        }
    }
    last_button_state = (btn_pressed ? GPIO_PIN_RESET : GPIO_PIN_SET);

    // Requirement 2: Print counter value every 100 ms to the serial terminal
    if ((HAL_GetTick() - last_print) >= 100) {
        last_print = HAL_GetTick();

        char line[128];
        snprintf(line, sizeof(line), "TIM3_CNT=%5u | Total=%+6ld | Dir=%s\r\n",
                 enc.raw_cnt, (long)enc.total_counts,
                 (enc.direction == ENCODER_DIR_FORWARD) ? "CW (+)" : "CCW (-)");
        Log(line);
    }
/* USER CODE END WHILE */
```

---

## Executing the Test in Terminal

With your virtual environment activated, run:

```bash
source .venv/bin/activate
python smoketest/encoder/encoder_monitor.py
```

### Performing the 5 Steps:
1. Terminal shows live telemetry updating every **100 ms**:
   ```text
   [100ms] TIM3_CNT:   124  |  Total Counts:   +1496  |  Dir: CW (+)
   ```
2. Put a piece of tape or marker dot on the output shaft edge. Align it with a fixed reference mark on the gearbox.
3. **Press Blue Button (B1)**:
   - Green LED LD2 turns ON.
   - Counter is **ZEROED**.
4. **Turn the shaft by hand exactly one full turn (360°)** until the tape mark aligns again with your reference mark.
5. **Press Blue Button (B1)** again:
   - Records Trial 1 (e.g. `Counts = +1502`).
6. Repeat steps 3–5 for 3 to 5 trials.
7. Upon completing trial 3, 4, or 5, the terminal prints:
   ```text
   =======================================================
     CALIBRATION & ACCEPTANCE SUMMARY (Trials: 3)
   =======================================================
     Average N_rev       : +1501.3 counts / turn
     Estimated Gear Ratio: 53.62 : 1 (G = N_rev / 28)
     Max Deviation       : 0.22% (Milestone <= 2.0%)
     Direction Sign      : CW (+, PASS)
     VERDICT             : >>> [PASS] <<<
   =======================================================
   ```
8. Turn CCW for 1 trial to confirm negative sign (`CCW (-, PASS)`).
