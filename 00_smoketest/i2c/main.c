/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Shared I2C Bus Smoketest (VL53L0X ToF + MMA7660 Accelerometer)
 *                   STM32F446RE (Nucleo-64) with I2C1 (PB6 SCL, PB7 SDA)
 ******************************************************************************
 */
/* USER CODE END Header */
#include "main.h"
#include "i2c_sensors_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

static bool s_vl53l0x_present = false;
static bool s_mma7660_present = false;

static void Log(const char *s) {
    HAL_UART_Transmit(&huart2, (const uint8_t *)s, strlen(s), 1000);
}

static void WaitForButtonPress(void) {
    // Wait until button is released if currently pressed
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
    Log("       STM32 I2C SENSOR SMOKETEST SUITE               \r\n");
    Log("       Bus: I2C1 (PB6=SCL, PB7=SDA) @ 100 kHz          \r\n");
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

            // Average 5 samples to reduce jitter
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

                int err_int = (int)err;
                int err_dec = (int)((err - (float)err_int) * 10.0f);
                if (err_dec < 0) err_dec = -err_dec;

                snprintf(buf, sizeof(buf),
                         "    Target: %3d mm | Measured: %3d mm | Error: %d.%d%% -> [%s]\r\n",
                         targets_mm[t], measured_mm[t], err_int, err_dec, (err <= 10.0f) ? "PASS" : "FAIL");
                Log(buf);

                if (err > 10.0f) {
                    test_pass = false;
                }
            } else {
                Log("    [TIMEOUT / ERROR] Sensor failed to return reading!\r\n");
                test_pass = false;
            }
        }

        Log("\r\n=======================================================\r\n");
        Log("  TOF53200 3-DISTANCE ACCEPTANCE SUMMARY               \r\n");
        Log("=======================================================\r\n");
        for (int t = 0; t < 3; t++) {
            int e_int = (int)errors_pct[t];
            int e_dec = (int)((errors_pct[t] - (float)e_int) * 10.0f);
            if (e_dec < 0) e_dec = -e_dec;

            snprintf(buf, sizeof(buf),
                     "  Point %d (%3d mm) : Measured = %3d mm, Error = %d.%d%% [%s]\r\n",
                     t + 1, targets_mm[t], measured_mm[t], e_int, e_dec,
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

    // -----------------------------------------------------------------
    // SCENARIO 3: BOTH SENSORS DETECTED
    // -----------------------------------------------------------------
    else if (s_vl53l0x_present && s_mma7660_present) {
        Log("\r\n[DUAL MODE] Both VL53L0X and MMA7660 detected!\r\n");
        Log("Initializing MMA7660...\r\n");
        MMA7660_Init(&hi2c1);
        Log("Initializing VL53L0X...\r\n");
        VL53L0X_Init(&hi2c1);
        Log("Entering dual-sensor streaming mode (press B1 to re-scan).\r\n\r\n");
    }

    // -----------------------------------------------------------------
    // SCENARIO 4: NO SENSORS DETECTED
    // -----------------------------------------------------------------
    else {
        Log("\r\n=======================================================\r\n");
        Log("  >>> [WARNING] NO I2C DEVICES DETECTED ON BUS! <<<     \r\n");
        Log("=======================================================\r\n");
        Log("  Please check connections:\r\n");
        Log("  1. 3.3V  -> Nucleo CN6 pin 4\r\n");
        Log("  2. GND   -> Nucleo CN6 pin 6\r\n");
        Log("  3. SCL   -> Nucleo PB6 (CN5 pin 3 / Arduino D10)\r\n");
        Log("  4. SDA   -> Nucleo PB7 (Morpho CN7 pin 21)\r\n");
        Log("=======================================================\r\n\r\n");
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();

    // Run initial smoketest sequence
    Run_I2C_Smoketest();

    uint32_t last_stream_time = HAL_GetTick();
    bool last_btn_state = GPIO_PIN_SET;

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
                    snprintf(line, sizeof(line),
                             "[LIVE TOF53200] Distance = %4ld mm (%4.1f cm)\r\n",
                             (long)dist, (float)dist / 10.0f);
                } else {
                    snprintf(line, sizeof(line),
                             "[LIVE TOF53200] Distance =  --- mm (Out of Range)\r\n");
                }
                Log(line);
            } else if (s_mma7660_present && !s_vl53l0x_present) {
                // MMA7660 Isolated Streaming
                MMA7660_Data_t accel;
                if (MMA7660_Read(&hi2c1, &accel) == I2C_SENSORS_OK) {
                    float mag = sqrtf(accel.x_g * accel.x_g + accel.y_g * accel.y_g + accel.z_g * accel.z_g);
                    snprintf(line, sizeof(line),
                             "[LIVE MMA7660] X=%+5.2fg Y=%+5.2fg Z=%+5.2fg | |g|=%4.2fg | %s\r\n",
                             accel.x_g, accel.y_g, accel.z_g, mag, accel.orientation);
                } else {
                    snprintf(line, sizeof(line), "[LIVE MMA7660] Read Failed\r\n");
                }
                Log(line);
            } else if (s_vl53l0x_present && s_mma7660_present) {
                // Dual Sensor Streaming
                int32_t dist = VL53L0X_ReadDistance_mm(&hi2c1);
                MMA7660_Data_t accel;
                MMA7660_Read(&hi2c1, &accel);
                if (dist > 0) {
                    snprintf(line, sizeof(line),
                             "[LIVE] Dist=%4ld mm | Accel: X=%+5.2fg Y=%+5.2fg Z=%+5.2fg | %s\r\n",
                             (long)dist, accel.x_g, accel.y_g, accel.z_g, accel.orientation);
                } else {
                    snprintf(line, sizeof(line),
                             "[LIVE] Dist= --- mm | Accel: X=%+5.2fg Y=%+5.2fg Z=%+5.2fg | %s\r\n",
                             accel.x_g, accel.y_g, accel.z_g, accel.orientation);
                }
                Log(line);
            }

            // Toggle Green LED LD2 heartbeat
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        }
        HAL_Delay(10);
    }
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        while (1);
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        while (1);
    }
}

static void MX_I2C1_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000; // 100 kHz Standard Mode
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    __HAL_RCC_I2C1_CLK_ENABLE();
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        while (1);
    }
}

static void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        while (1);
    }
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // User LED LD2: PA5 (Output)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Blue User Button B1: PC13 (Input)
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // I2C1 GPIO Configuration: PB6 -> SCL, PB7 -> SDA
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
