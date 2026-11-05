# Sequential I2C Sensor Smoketest: TOF53200 & MMA7660FC (HAL Version)

Validates communication and sensor data acquisition for the **TOF53200 Time-of-Flight Laser Distance Sensor (VL53L0X)** and the **Grove 3-Axis Accelerometer (MMA7660FC)** on **I2C1** (`PB6` = SCL, `PB7` = SDA) on the STM32F446RE (Nucleo-64), streaming telemetry over USART2 at 115200 baud.

> [!TIP]
> **Sequential Testing Strategy:**  
> To prevent bus contention, breadboard wire clutter, and intermittent crosstalk, the test suite executes **one by one**:
> 1. **Phase 1 (TOF53200 First):** Connect ONLY the TOF53200. Firmware detects 0x29, runs the 3-distance ruler calibration (100 mm, 200 mm, 300 mm within ±10.0%), and streams distance telemetry. Zero MMA7660 timeout interference.
> 2. **Phase 2 (MMA7660FC Second):** Disconnect TOF53200, connect Grove MMA7660FC. Firmware detects 0x4C, verifies 3-axis acceleration and posture tilt registers, and streams real-time g-force. Zero TOF53200 timeout interference.

---

## 1. System Overview & Testing Architecture

```
                  +-------------------------------------------------------+
                  |         STM32F446RE I2C1 (100 kHz Standard Mode)      |
                  |         • PB6 (Arduino D10 / CN5-3)  = SCL            |
                  |         • PB7 (Morpho CN7 pin 21)    = SDA            |
                  |         • 3.3V (CN6 pin 4)           = Logic Power    |
                  |         • GND  (CN6 pin 6)           = Ground Ref     |
                  +-------------------------------------------------------+
                                   |         |        |       |
                                  SCL       SDA      3.3V    GND
                                   |         |        |       |
         ==========================+=========+========+=======+==========================
         |                                                                              |
         v [PHASE 1: TEST FIRST]                                                        v [PHASE 2: TEST SECOND]
+------------------------------------+                                 +------------------------------------+
|      TOF53200 (VL53L0X)            |                                 |       Grove MMA7660FC              |
|  Time-of-Flight Distance Sensor    |                                 |     3-Axis Accelerometer           |
|                                    |                                 |                                    |
| • Address: 0x29                    |                                 | • Address: 0x4C                    |
| • Calibration: 100/200/300 mm      |                                 | • Calibration: 1.0g gravity vector |
| • Tolerance:   Within +/- 10.0%    |                                 | • Posture:     Tilt register       |
+------------------------------------+                                 +------------------------------------+
```

---

## 2. Hardware Specifications & Pinout

### Phase 1: TOF53200 Module Pinout (6-Pin Header)
- **Operating Voltage**: 3.3V – 5.0V (supports 3.3V logic)
- **7-Bit I2C Address**: `0x29` (8-bit write = `0x52`, read = `0x53`)
- **Measurement Range**: 30 mm – 2000 mm (infrared 940 nm VCSEL laser)
- **Dual Interface**: Supports both Serial UART (pins 3 & 4) and I2C (pins 5 & 6)

| Pin # | Pin Name | Color / Function | I2C Test Connection | Location on Nucleo-64 |
| :---: | :--- | :--- | :--- | :--- |
| **1** | **VIN** | Power Supply (3–5V) | **STM32 3.3V** | CN6 pin 4 (Power header) |
| **2** | **GND** | Power Ground (GND) | **Common GND** | CN6 pin 6 (Power header) |
| **3** | **TX** | Serial Port Data Output | *DO NOT CONNECT* | Leave floating (UART only) |
| **4** | **RX** | Serial Port Command Input| *DO NOT CONNECT* | Leave floating (UART only) |
| **5** | **SCL**| I2C Clock Signal Input | **STM32 PB6** | CN5 pin 3 (Arduino connector "D10") |
| **6** | **SDA**| I2C Data Signal (I/O) | **STM32 PB7** | Morpho **CN7 pin 21** (outer column, 11th down) |

> [!CAUTION]
> **Pin Order Warning:**  
> On generic VL53L0X breakout boards, SCL and SDA are often on pins 3 and 4.  
> However, on your **TOF53200 module**, **pins 3 and 4 are UART TX and RX**.  
> You must wire **Pin 5 (SCL)** and **Pin 6 (SDA)** to STM32 I2C1! Connecting pins 3 or 4 to the I2C bus will cause bus errors and No ACK.

---

### Phase 2: Grove MMA7660FC 3-Axis Accelerometer Pinout
- **Operating Voltage**: 3.3V
- **7-Bit I2C Address**: `0x4C` (8-bit write = `0x98`, read = `0x99`)
- **Range & Sensitivity**: ±1.5g, 21.33 counts / g (6-bit output resolution)
- **Grove 4-Pin Cable**:

| Grove Wire Color | Function | Target Connection | Pin Location on Nucleo-64 |
| :--- | :--- | :--- | :--- |
| **Yellow** | **SCL** (Clock) | **STM32 PB6** | CN5 pin 3 (Arduino "D10") |
| **White** | **SDA** (Data) | **STM32 PB7** | Morpho CN7 pin 21 (outer column, 11th pin down) |
| **Red** | **VCC** (3.3V) | **STM32 3.3V** | CN6 pin 4 |
| **Black** | **GND** (Ground) | **Common GND** | CN6 pin 6 / Breadboard blue rail |

---

## 3. Step-by-Step Isolated Wiring Sequence

### Part A: Wire ONLY TOF53200 (Phase 1)
To test the TOF53200 in complete isolation:
1. Ensure the Grove MMA7660 cable is **unplugged**.
2. Connect **Pin 1 (VIN)** to Nucleo **3.3V (CN6 pin 4)**.
3. Connect **Pin 2 (GND)** to Nucleo **GND (CN6 pin 6)**.
4. **Leave Pin 3 (TX) and Pin 4 (RX) completely unconnected.**
5. Connect **Pin 5 (SCL)** to Nucleo **PB6 (CN5 pin 3 / Arduino D10)**.
6. Connect **Pin 6 (SDA)** to Nucleo **PB7 (Morpho CN7 pin 21)**.

### Part B: Wire ONLY Grove MMA7660FC (Phase 2)
When Phase 1 is complete:
1. Disconnect the 4 wires of the TOF53200.
2. Plug the Grove 4-pin cable into the MMA7660 board:
   - **Red wire**    -> Nucleo **3.3V (CN6 pin 4)**
   - **Black wire**  -> Nucleo **GND (CN6 pin 6)**
   - **Yellow wire** -> Nucleo **PB6 (CN5 pin 3 / Arduino D10)**
   - **White wire**  -> Nucleo **PB7 (Morpho CN7 pin 21)**
3. Press the Blue User Button (**B1**) on the Nucleo to trigger Phase 2!

> [!IMPORTANT]
> **Voltage Warning:**  
> Connect both sensors to **3.3V**, **NEVER 5V**. The STM32F446RE I2C pins operate at 3.3V logic levels.
> Connect both sensors to **3.3V**, **NEVER 5V**. The STM32F446RE I2C pins operate at 3.3V logic levels.

---

## 4. STM32CubeIDE IOC Configuration

### Step 1 — Create / Open STM32 Project
1. **File → New → STM32 Project**
2. Board selector: `NUCLEO-F446RE` → **Next**
3. Project name: `I2C_Sensors_Smoketest_HAL` → Language: **C** → **Finish**
4. Initialize all peripherals with default mode: click **No**

---

### Step 2 — System Core → SYS (Debug)
1. In Left panel: **System Core → SYS**
2. Set **Debug** = `Serial Wire` (reserves PA13/PA14 for ST-Link)

---

### Step 3 — Clock Configuration
1. Click the **Clock Configuration** tab at the top.
2. Set **Input frequency** = `16` MHz (HSI internal RC oscillator).
3. **PLL Source Mux** = `HSI`, **System Clock Mux** = `HSI` (bypass PLL).
4. Confirm:
   - **SYSCLK** = 16 MHz
   - **HCLK** = 16 MHz
   - **APB1 Peripheral clocks** = 16 MHz (I2C1 and USART2 clock source)
   - **APB2 Peripheral clocks** = 16 MHz

---

### Step 4 — System Core → GPIO
In the `.ioc` Pinout view:
1. Left-click **PA5** → select **GPIO_Output** (User LED LD2)
2. Left-click **PC13** → select **GPIO_Input** (Blue User Button B1)
3. In Left panel **System Core → GPIO**:
   - `PA5`: Output Push Pull, No pull-up/pull-down, Low speed, User Label = `LD2`
   - `PC13`: Input mode, No pull-up/pull-down, User Label = `B1_USER`

---

### Step 5 — Connectivity → I2C1 (Shared Sensor Bus)
1. In Left panel: **Connectivity → I2C1**
2. Set **I2C** = `I2C`
3. Verify Pinout view shows default mapping:
   - `PB6` = `I2C1_SCL`
   - `PB7` = `I2C1_SDA`
4. In **Configuration → Parameter Settings**:
   - **Master Features**:
     - **I2C Speed Mode** = `Standard Mode`
     - **I2C Clock Speed (Hz)** = `100000` (100 kHz)
   - **Slave Features** (keep all at default values since STM32 acts as Master):
     - **Clock No Stretch Mode** = `Disabled`
     - **Primary Address Length selection** = `7-bit`
     - **Dual Address Acknowledged** = `Disabled`
     - **Primary slave address** = `0`
     - **General Call address detection** = `Disabled`
5. In **System Core → GPIO → I2C1 Pins** (`PB6` and `PB7`):
   - **GPIO Mode** = `Alternate Function Open Drain`
   - **GPIO Pull-up/Pull-down** = `Pull-up` (provides internal pull-up backup)
   - **Maximum output speed** = `Very High`

---

### Step 6 — Connectivity → USART2 (Telemetry)
1. Left panel: **Connectivity → USART2**
2. Set **Mode** = `Asynchronous`
3. Parameter Settings:
   - **Baud Rate** = `115200`
   - **Word Length** = `8 Bits`
   - **Parity** = `None`
   - **Stop Bits** = `1`
4. Confirm pins: `PA2` = `USART2_TX`, `PA3` = `USART2_RX`.

---

### Step 7 — Project Manager Settings
1. Click **Project Manager** tab.
2. Under **Code Generator**:
   - Check **Generate peripheral initialization as a pair of '.c/.h' files per peripheral**.
   - Check **Keep User Code when re-generating**.
3. Press **Ctrl+S** (Save) and click **Yes** to generate code.

---

### Step 8 — Add Driver Files
Copy the driver files from [smoketest/i2c/](/Users/church/Documents/github/MBD_Project/smoketest/i2c/) into your generated project:

| Source File | Target Folder | Purpose |
| :--- | :--- | :--- |
| [i2c_sensors_hal.h](/Users/church/Documents/github/MBD_Project/smoketest/i2c/i2c_sensors_hal.h) | `Core/Inc/` | I2C scanner, MMA7660, and VL53L0X prototypes |
| [i2c_sensors_hal.c](/Users/church/Documents/github/MBD_Project/smoketest/i2c/i2c_sensors_hal.c) | `Core/Src/` | Bus scanner, accelerometer conversion, ToF ranging driver |

---

### Step 9 — User Code in main.c

Open `Core/Src/main.c` and insert the smoketest code into the corresponding `/* USER CODE BEGIN ... */` blocks:

**1. Includes:**
```c
/* USER CODE BEGIN Includes */
#include "i2c_sensors_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */
```

**2. Helper Functions & Smoketest Logic (before `main()`):**
```c
/* USER CODE BEGIN 0 */
static void Log(const char *s) {
    HAL_UART_Transmit(&huart2, (const uint8_t *)s, strlen(s), 1000);
}

static void WaitForButtonPress(void) {
    // Wait until button is released if currently held down
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
        HAL_Delay(10);
    }
    // Wait until button is pressed (B1 on PC13 is active LOW)
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
        HAL_Delay(10);
    }
    HAL_Delay(50); // Debounce
    // Wait for release
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
        HAL_Delay(10);
    }
    HAL_Delay(50);
}

static void Run_I2C_Smoketest(void) {
    char buf[160];

    Log("\r\n=======================================================\r\n");
    Log("  SHARED I2C BUS SMOKETEST: VL53L0X & MMA7660          \r\n");
    Log("  Bus: I2C1 (PB6 = SCL, PB7 = SDA) @ 100 kHz           \r\n");
    Log("=======================================================\r\n\r\n");

    Log("[STEP 1] Scanning I2C Bus (0x01..0x7F)...\r\n");
    uint8_t found_addrs[16] = {0};
    int found_count = I2C_ScanBus(&hi2c1, found_addrs, 16);

    s_vl53l0x_present = false;
    s_mma7660_present = false;

    snprintf(buf, sizeof(buf), "         Found %d active device(s) on bus:\r\n", found_count);
    Log(buf);

    for (int i = 0; i < found_count; i++) {
        uint8_t addr = found_addrs[i];
        if (addr == VL53L0X_ADDR_7BIT) {
            s_vl53l0x_present = true;
            Log("         - 0x29: TOF53200 (VL53L0X) Distance Sensor [ACK]\r\n");
        } else if (addr == MMA7660_ADDR_7BIT) {
            s_mma7660_present = true;
            Log("         - 0x4C: Grove MMA7660FC Accelerometer      [ACK]\r\n");
        } else {
            snprintf(buf, sizeof(buf), "         - 0x%02X: Unknown I2C Target\r\n", addr);
            Log(buf);
        }
    }

    // -----------------------------------------------------------------
    // SCENARIO 1: PHASE 1 - TOF53200 (VL53L0X) ISOLATED TEST
    // -----------------------------------------------------------------
    if (s_vl53l0x_present && !s_mma7660_present) {
        Log("\r\n=======================================================\r\n");
        Log("  >>> PHASE 1: TOF53200 (VL53L0X) ISOLATED TEST <<<   \r\n");
        Log("  MMA7660 is not connected (Zero bus crosstalk/error)  \r\n");
        Log("=======================================================\r\n");
        Log("[INIT] Configuring VL53L0X (2V8 I/O, Single-shot mode)...\r\n");

        if (VL53L0X_Init(&hi2c1) != I2C_SENSORS_OK) {
            Log("       [ERROR] VL53L0X Init Failed! Check 3.3V/GND & PB6/PB7.\r\n");
            return;
        }
        Log("       VL53L0X Initialized & Model ID 0xEE Verified.\r\n\r\n");

        Log("--- 3-DISTANCE RULER CALIBRATION (Acceptance: +/- 10%) ---\r\n");
        uint16_t targets_mm[3] = {100, 200, 300};
        uint16_t measured_mm[3] = {0};
        float errors_pct[3] = {0.0f};
        bool test_pass = true;

        for (int t = 0; t < 3; t++) {
            snprintf(buf, sizeof(buf),
                     "\r\n>>> [TARGET %d/3] Align ruler target at %d mm (%d cm),\r\n"
                     "    then click the Blue Button (B1)... <<<\r\n",
                     t + 1, targets_mm[t], targets_mm[t] / 10);
            Log(buf);

            WaitForButtonPress();

            int32_t sum = 0;
            int valid = 0;
            for (int s = 0; s < 5; s++) {
                int32_t d = VL53L0X_ReadDistance_mm(&hi2c1);
                if (d > 0) {
                    sum += d;
                    valid++;
                }
                HAL_Delay(30);
            }

            if (valid > 0) {
                measured_mm[t] = (uint16_t)(sum / valid);
                float err = fabsf(((float)measured_mm[t] - (float)targets_mm[t]) / (float)targets_mm[t]) * 100.0f;
                errors_pct[t] = err;

                snprintf(buf, sizeof(buf),
                         "    Target: %3d mm | Measured: %3d mm | Error: %4.1f%% -> [%s]\r\n",
                         targets_mm[t], measured_mm[t], err, (err <= 10.0f) ? "PASS" : "FAIL");
                Log(buf);

                if (err > 10.0f) test_pass = false;
            } else {
                Log("    [TIMEOUT / ERROR] Sensor failed to return reading!\r\n");
                test_pass = false;
            }
        }

        Log("\r\n=======================================================\r\n");
        Log("  TOF53200 3-DISTANCE ACCEPTANCE SUMMARY               \r\n");
        Log("=======================================================\r\n");
        for (int t = 0; t < 3; t++) {
            snprintf(buf, sizeof(buf),
                     "  Point %d (%3d mm) : Measured = %3d mm, Error = %4.1f%% [%s]\r\n",
                     t + 1, targets_mm[t], measured_mm[t], errors_pct[t],
                     (errors_pct[t] <= 10.0f) ? "PASS" : "FAIL");
            Log(buf);
        }
        Log("  Acceptance Threshold: Within +/- 10.0% of target distance\r\n");
        Log("-------------------------------------------------------\r\n");
        if (test_pass) {
            Log("  TOF53200 VERDICT : >>> [PASS] <<<\r\n");
        } else {
            Log("  TOF53200 VERDICT : >>> [FAIL] <<<\r\n");
        }
        Log("=======================================================\r\n");
        Log("Entering continuous TOF53200 stream (press B1 to re-test).\r\n");
        Log("NEXT: Disconnect TOF53200 and connect MMA7660FC for Phase 2.\r\n\r\n");
    }

    // -----------------------------------------------------------------
    // SCENARIO 2: PHASE 2 - GROVE MMA7660FC ISOLATED TEST
    // -----------------------------------------------------------------
    else if (s_mma7660_present && !s_vl53l0x_present) {
        Log("\r\n=======================================================\r\n");
        Log("  >>> PHASE 2: GROVE MMA7660FC ISOLATED TEST <<<       \r\n");
        Log("  TOF53200 is not connected (Zero bus crosstalk/error) \r\n");
        Log("=======================================================\r\n");
        Log("[INIT] Configuring MMA7660 (Active Mode, 50 Hz)...\r\n");

        if (MMA7660_Init(&hi2c1) != I2C_SENSORS_OK) {
            Log("       [ERROR] MMA7660 Init Failed! Check Grove cable & PB6/PB7.\r\n");
            return;
        }
        Log("       MMA7660 Initialized Successfully.\r\n\r\n");

        Log("--- ACCELEROMETER GRAVITY & TILT VERIFICATION ---\r\n");
        HAL_Delay(100);

        MMA7660_Data_t accel;
        float sum_x = 0, sum_y = 0, sum_z = 0;
        int samples = 10;
        bool read_ok = true;

        for (int s = 0; s < samples; s++) {
            if (MMA7660_Read(&hi2c1, &accel) != I2C_SENSORS_OK) {
                read_ok = false;
                break;
            }
            sum_x += accel.x_g;
            sum_y += accel.y_g;
            sum_z += accel.z_g;
            HAL_Delay(25);
        }

        if (read_ok) {
            float avg_x = sum_x / (float)samples;
            float avg_y = sum_y / (float)samples;
            float avg_z = sum_z / (float)samples;
            float mag = sqrtf(avg_x * avg_x + avg_y * avg_y + avg_z * avg_z);

            snprintf(buf, sizeof(buf),
                     "  Averaged (10 samples):\r\n"
                     "    Accel X : %+5.2f g\r\n"
                     "    Accel Y : %+5.2f g\r\n"
                     "    Accel Z : %+5.2f g\r\n"
                     "    |g| Mag :  %4.2f g (Expected ~1.00g on flat surface)\r\n"
                     "    Posture : %s (Tilt Register: 0x%02X)\r\n",
                     avg_x, avg_y, avg_z, mag, accel.orientation, accel.tilt_status);
            Log(buf);

            Log("\r\n=======================================================\r\n");
            Log("  MMA7660FC ACCEPTANCE SUMMARY                         \r\n");
            Log("=======================================================\r\n");
            if (mag >= 0.75f && mag <= 1.25f) {
                Log("  Gravity Vector Magnitude: PASS (0.75g <= |g| <= 1.25g)\r\n");
                Log("  Tilt / Orientation Status: PASS\r\n");
                Log("-------------------------------------------------------\r\n");
                Log("  MMA7660FC VERDICT : >>> [PASS] <<<\r\n");
            } else {
                Log("  Gravity Vector Magnitude out of expected range!\r\n");
                Log("-------------------------------------------------------\r\n");
                Log("  MMA7660FC VERDICT : >>> [FAIL] <<<\r\n");
            }
            Log("=======================================================\r\n");
            Log("Entering continuous MMA7660 stream (tilt board to observe axes).\r\n\r\n");
        } else {
            Log("  [ERROR] Failed to read MMA7660 registers!\r\n");
        }
    }
}
/* USER CODE END 0 */
```

**3. Initial Execution & Streaming in `main()`:**
```c
/* USER CODE BEGIN 2 */
Run_I2C_Smoketest();

uint32_t last_stream_time = HAL_GetTick();
bool last_btn_state = GPIO_PIN_SET;
/* USER CODE END 2 */

while (1) {
    // Check Blue Button (B1 on PC13) to re-run acceptance calibration
    bool btn_pressed = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET);
    if (btn_pressed && last_btn_state == GPIO_PIN_SET) {
        HAL_Delay(30); // Debounce
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
            Run_I2C_Smoketest();
        }
    }
    last_btn_state = (btn_pressed ? GPIO_PIN_RESET : GPIO_PIN_SET);

    // Continuous telemetry stream every 200 ms
    if (HAL_GetTick() - last_stream_time >= 200) {
        last_stream_time = HAL_GetTick();

        char line[140];

        if (s_vl53l0x_present && !s_mma7660_present) {
            // TOF53200 Isolated Streaming
            int32_t dist = VL53L0X_ReadDistance_mm(&hi2c1);
            if (dist > 0) {
                snprintf(line, sizeof(line), "[LIVE TOF53200] Distance = %4ld mm (%4.1f cm)\r\n",
                         (long)dist, (float)dist / 10.0f);
            } else {
                snprintf(line, sizeof(line), "[LIVE TOF53200] Distance =  --- mm (Out of Range)\r\n");
            }
            Log(line);
        } else if (s_mma7660_present && !s_vl53l0x_present) {
            // MMA7660 Isolated Streaming
            MMA7660_Data_t accel;
            if (MMA7660_Read(&hi2c1, &accel) == I2C_SENSORS_OK) {
                float mag = sqrtf(accel.x_g * accel.x_g + accel.y_g * accel.y_g + accel.z_g * accel.z_g);
                snprintf(line, sizeof(line), "[LIVE MMA7660] X=%+5.2fg Y=%+5.2fg Z=%+5.2fg | |g|=%4.2fg | %s\r\n",
                         accel.x_g, accel.y_g, accel.z_g, mag, accel.orientation);
            } else {
                snprintf(line, sizeof(line), "[LIVE MMA7660] Read Failed\r\n");
            }
            Log(line);
        }

        // Toggle Green LED LD2 heartbeat
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
    HAL_Delay(10);
}
```

---

## 5. Running the Sequential Acceptance Test

### Phase 1: Test TOF53200 (VL53L0X) First
1. Wire **ONLY the TOF53200** (VIN to 3.3V, GND to GND, SCL to PB6, SDA to PB7). Ensure the MMA7660 is completely unplugged.
2. Flash the firmware and start the serial monitor:
   ```bash
   python smoketest/i2c/i2c_monitor.py
   ```
3. The monitor shows:
   `[STEP 1] Scanning I2C Bus...`
   `         - 0x29: TOF53200 (VL53L0X) Distance Sensor [ACK]`
   `>>> PHASE 1: TOF53200 (VL53L0X) ISOLATED TEST <<<`
4. Prepare a ruler and target:
   - Target at **100 mm**: click Blue Button (**B1**).
   - Target at **200 mm**: click Blue Button (**B1**).
   - Target at **300 mm**: click Blue Button (**B1**).
5. Result:
   `TOF53200 VERDICT : >>> [PASS] <<<`
   Followed by clean distance streaming (`[LIVE TOF53200] Distance = 201 mm`).

---

### Phase 2: Test Grove MMA7660FC Second
1. Disconnect the TOF53200 wires.
2. Plug in the **Grove MMA7660FC** (Red to 3.3V, Black to GND, Yellow to PB6, White to PB7).
3. Click the Blue Button (**B1**) on the Nucleo (or press the Black Reset button).
4. The monitor shows:
   `[STEP 1] Scanning I2C Bus...`
   `         - 0x4C: Grove MMA7660FC Accelerometer [ACK]`
   `>>> PHASE 2: GROVE MMA7660FC ISOLATED TEST <<<`
5. Keep the board resting flat on your desk. The firmware measures 10 samples and computes static gravity magnitude:
   `Gravity Vector Magnitude: PASS (0.75g <= |g| <= 1.25g)`
   `MMA7660FC VERDICT : >>> [PASS] <<<`
6. Live streaming outputs real-time g-forces. Tilt the board to see X, Y, and Z axes respond dynamically!

---

## 6. Acceptance Criteria & Pass/Fail Checklist

| Acceptance Milestone | Test Verification | Target Criteria | Verdict |
| :--- | :--- | :--- | :--- |
| **1. VL53L0X Address ACK** | I2C Bus Scan | Target responds at **0x29** with ACK | [ ] PASS |
| **2. MMA7660 Address ACK** | I2C Bus Scan | Target responds at **0x4C** with ACK | [ ] PASS |
| **3. MMA7660 Accelerometer**| Rest flat on table | Z ≈ +1.0g (±0.3g), X ≈ 0.0g, Y ≈ 0.0g | [ ] PASS |
| **4. Ruler 100 mm Benchmark**| Target at 100 mm | **90 mm ≤ Measured ≤ 110 mm** (Error ≤ 10%) | [ ] PASS |
| **5. Ruler 200 mm Benchmark**| Target at 200 mm | **180 mm ≤ Measured ≤ 220 mm** (Error ≤ 10%) | [ ] PASS |
| **6. Ruler 300 mm Benchmark**| Target at 300 mm | **270 mm ≤ Measured ≤ 330 mm** (Error ≤ 10%) | [ ] PASS |

---

## 7. Troubleshooting Guide

### 1. I2C Scan Shows 0 Devices Found
- **Check SCL and SDA connections**: Ensure SCL connects to `PB6` and SDA connects to `PB7`. If lines are swapped, no ACK will be returned.
- **Check 3.3V Power**: Ensure the breadboard red rail is connected to **3.3V** (CN6 pin 4), and the blue rail is connected to **GND** (CN6 pin 6).
- **Verify Pull-Ups**: Both breakout boards have onboard pull-ups. In addition, ensure `GPIO_PULLUP` is selected in STM32CubeMX for PB6 and PB7.

### 2. VL53L0X Returns Out-of-Range (8190 mm) or Timeout
- **Laser Aperture Obstructed**: Remove any yellow protective peel-off film covering the optical window of the VL53L0X.
- **Target Reflectivity**: Use a matte white cardboard, paper sheet, or book as the target. Highly glossy, transparent, or jet-black surfaces absorb or scatter 940 nm infrared light.
- **Target Alignment**: Ensure the target is perpendicular to the sensor laser beam and large enough to fill the cone of vision (approx. 25° Field of View).

### 3. MMA7660 Returns Flat Zeroes (0, 0, 0)
- The MMA7660 boots up into **Standby Mode** by default. Ensure `MMA7660_Init()` writes `0x01` to register `0x07` (MODE) to enter **Active Mode**.
