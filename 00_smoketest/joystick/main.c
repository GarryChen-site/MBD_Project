/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : KY-023 Dual-Axis Joystick Smoketest (STM32F446RE)
 ******************************************************************************
 */
/* USER CODE END Header */
#include "main.h"
#include "joystick_hal.h"
#include <stdio.h>
#include <string.h>

ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);

static void Log(const char *s) {
    HAL_UART_Transmit(&huart2, (const uint8_t *)s, strlen(s), 1000);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_USART2_UART_Init();

    Log("\r\n==========================================\r\n");
    Log("  KY-023 Joystick Smoketest (12-bit ADC)  \r\n");
    Log("  Target: X/Y 0-4095, Centre Noise σ <= 10\r\n");
    Log("==========================================\r\n");

    // Initialize joystick HAL (PA0 = VRx, PA1 = VRy, PA10 = SW)
    Joystick_State_t joy;
    Joystick_Init(&hadc1, GPIOA, GPIO_PIN_10);
    Joystick_ResetTravel(&joy);

    // 1. Centre Noise Benchmark at Rest (100 samples)
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
            Log("[RESULT] Centre Noise: [PASS] (σ <= 10 counts / 8 mV)\r\n\r\n");
        } else {
            Log("[RESULT] Centre Noise: [FAIL] (σ > 10 counts). Check ground / wiring!\r\n\r\n");
        }
    }

    Log("[TEST 2] Live Monitor & Full Travel Test\r\n");
    Log("         Move joystick to all 4 edges to verify range 0 -> 4095.\r\n");
    Log("         Press down on joystick to verify SW button.\r\n\r\n");

    uint32_t last_print = HAL_GetTick();

    while (1) {
        Joystick_Read(&joy);

        // Turn LD2 on when button pressed (active LOW)
        if (joy.btn_sw == 0) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        }

        // Print every 100 ms
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

static void MX_ADC1_Init(void) {
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
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

    // LD2 Green LED: PA5
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Pushbutton SW: PA10 (Input with Internal Pull-Up)
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // ADC Pins: PA0 (A0 / VRx), PA1 (A1 / VRy)
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
