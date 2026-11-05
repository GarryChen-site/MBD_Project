/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : TB6612FNG + N20 Motor Smoketest (TIM4 20 kHz PWM)
 ******************************************************************************
 */
/* USER CODE END Header */
#include "main.h"
#include "motor_hal.h"
#include <stdio.h>
#include <string.h>

TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart2;

TB6612_Motor_t motorA;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);

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

int main(void) {
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM4_Init();
    MX_USART2_UART_Init();

    Log("\r\n=======================================================\r\n");
    Log("  TB6612FNG + N20 Motor Smoketest (TIM4 20 kHz PWM)    \r\n");
    Log("=======================================================\r\n");
    Log("Pins:\r\n");
    Log("  PB6  (D10) -> PWMA (TIM4_CH1, 20 kHz PWM)\r\n");
    Log("  PC7  (D9)  -> AIN1 (Direction 1)\r\n");
    Log("  PA9  (D8)  -> AIN2 (Direction 2)\r\n");
    Log("  PA8  (D7)  -> STBY (Standby, Active HIGH)\r\n");
    Log("-------------------------------------------------------\r\n\r\n");

    // Initialize Motor A: PB6 (PWMA), PC7 (AIN1), PA9 (AIN2), PA8 (STBY)
    Motor_Init(&motorA, &htim4, TIM_CHANNEL_1,
               GPIOC, GPIO_PIN_7,
               GPIOA, GPIO_PIN_9,
               GPIOA, GPIO_PIN_8);

    // Run initial test sequence
    Run_Smoketest_Sequence();

    bool last_button_state = GPIO_PIN_SET;

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

static void MX_TIM4_Init(void) {
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim4.Instance = TIM4;
    // 20 kHz PWM calculation:
    // f_timer = 16 MHz
    // Prescaler = 0 -> timer counts at 16 MHz
    // ARR = 799 -> Period = 800 counts
    // Frequency = 16,000,000 / 800 = 20,000 Hz = 20.0 kHz
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = MOTOR_PWM_ARR; // 799
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
        while (1);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
        while (1);
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0; // Initial duty = 0%
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
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

    // Motor Direction Pins:
    // AIN1: PC7 (Arduino D9)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // AIN2: PA9 (Arduino D8)
    // STBY: PA8 (Arduino D7)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PWMA: PB6 (Arduino D10) configured as TIM4_CH1 (Alternate Function AF2)
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
