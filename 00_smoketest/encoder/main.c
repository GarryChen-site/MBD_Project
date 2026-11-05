/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : TIM3 Encoder Mode Smoketest for N20 Geared Motor
 ******************************************************************************
 */
/* USER CODE END Header */
#include "main.h"
#include "encoder_hal.h"
#include <stdio.h>
#include <string.h>

TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);

static void Log(const char *s) {
    HAL_UART_Transmit(&huart2, (const uint8_t *)s, strlen(s), 1000);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM3_Init();
    MX_USART2_UART_Init();

    Log("\r\n=======================================================\r\n");
    Log("  N20 DC Geared Motor Encoder Smoketest (TIM3 TI1/TI2) \r\n");
    Log("=======================================================\r\n");
    Log("Spec: N20 Hall Encoder = 7 PPR (28 counts / motor rotor rev)\r\n");
    Log("Goal: 1 turn of output shaft -> count = N_rev +- 2%%\r\n\r\n");
    Log("Procedure:\r\n");
    Log("  1. Continuous telemetry prints TIM3 count every 100 ms.\r\n");
    Log("  2. Place tape mark on output shaft.\r\n");
    Log("  3. Press Blue Button (B1) to ZERO counter.\r\n");
    Log("  4. Turn shaft by hand exactly 1 full turn.\r\n");
    Log("  5. Press Blue Button (B1) to SAVE trial (repeat 3-5 times).\r\n");
    Log("-------------------------------------------------------\r\n\r\n");

    Encoder_State_t enc = {0};
    Encoder_Init(&htim3);
    Encoder_Zero(&enc);

    uint32_t last_print = HAL_GetTick();
    bool last_button_state = GPIO_PIN_SET;
    bool waiting_for_turn = false;

    while (1) {
        Encoder_Update(&enc);

        // Check Blue User Button (B1 on PC13, active LOW)
        bool btn_pressed = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET);
        if (btn_pressed && last_button_state == GPIO_PIN_SET) {
            HAL_Delay(30); // Debounce
            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
                if (!waiting_for_turn) {
                    // Step 3: Zero the counter at tape mark position
                    Encoder_Zero(&enc);
                    waiting_for_turn = true;
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED ON

                    char msg[128];
                    snprintf(msg, sizeof(msg),
                             "\r\n>>> [TRIAL %u START] Counter ZEROED. Turn shaft exactly 1 turn, then press B1 again... <<<\r\n",
                             (unsigned)(enc.trial_count + 1));
                    Log(msg);
                } else {
                    // Step 4 & 5: Record the trial count
                    uint8_t count = Encoder_RecordTrial(&enc);
                    waiting_for_turn = false;
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // LED OFF

                    char msg[192];
                    snprintf(msg, sizeof(msg),
                             "\r\n[TRIAL %u RECORDED] Counts = %+ld (Dir: %s)\r\n",
                             (unsigned)count,
                             (long)enc.total_counts,
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
                            snprintf(line_buf, sizeof(line_buf),
                                     "  Trial %u            : %+ld counts\r\n",
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

                        snprintf(line_buf, sizeof(line_buf),
                                 "  Direction Sign      : %s\r\n",
                                 (avg_n_rev >= 0) ? "CW (+, PASS)" : "CCW (-, PASS)");
                        Log(line_buf);

                        snprintf(line_buf, sizeof(line_buf),
                                 "  VERDICT             : %s\r\n",
                                 pass ? ">>> [PASS] <<<" : ">>> [FAIL] <<<");
                        Log(line_buf);

                        Log("=======================================================\r\n");
                        Log("Press B1 to begin another trial if desired.\r\n\r\n");
                    } else {
                        Log("-> Complete at least 3 trials to compute average. Press B1 to zero for next trial.\r\n\r\n");
                    }
                }
            }
        }
        last_button_state = (btn_pressed ? GPIO_PIN_RESET : GPIO_PIN_SET);

        // Requirement 2: Print counter value every 100 ms to the serial terminal
        if ((HAL_GetTick() - last_print) >= 100) {
            last_print = HAL_GetTick();

            char line[128];
            snprintf(line, sizeof(line),
                     "TIM3_CNT=%5u | Total=%+6ld | Dir=%s\r\n",
                     enc.raw_cnt,
                     (long)enc.total_counts,
                     (enc.direction == ENCODER_DIR_FORWARD) ? "CW (+)" : "CCW (-)");
            Log(line);
        }
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

static void MX_TIM3_Init(void) {
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 65535; // 16-bit auto-reload counter (0xFFFF)
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    // Requirement 1: Configure TIM3 in Encoder mode, TI1 and TI2 (the x4 count)
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 10; // Digital noise filter (filters contact bounce / motor noise)

    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 10;

    if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
        while (1);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
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
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    // User LED LD2: PA5
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

    // TIM3 Encoder Inputs: PA6 (TIM3_CH1) and PA7 (TIM3_CH2)
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
