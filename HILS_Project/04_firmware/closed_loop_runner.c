/* closed_loop_runner.c - Pass 2 (Stage 5/6 Core) 10 ms closed-loop hardware runner
 * Uses the exact peripheral handles and pins from smoketest/{joystick,encoder,motor}.
 */
#include "closed_loop_runner.h"
#include "app_wrapper.h"
#include "prof.h"
#include "main.h"
#if defined(HAL_CAN_MODULE_ENABLED) || defined(USE_CAN_TELEMETRY)
#include "can_telemetry.h"
#endif
#include <stdio.h>
#include <string.h>

extern TIM_HandleTypeDef  htim4;    /* PWM, PB6 = TIM4_CH1, 20 kHz (smoketest/motor) */
extern TIM_HandleTypeDef  htim3;    /* Encoder mode, PA6/PA7, 16-bit counter (smoketest/encoder) */
extern ADC_HandleTypeDef  hadc1;    /* PA0 = IN0 (X), PA1 = IN1 (Y) (smoketest/joystick) */
extern UART_HandleTypeDef huart2;   /* ST-LINK VCP, 115200 8N1 */

/* TB6612 direction and standby pins matching smoketest/motor */
#define MOT_AIN1_PORT GPIOC
#define MOT_AIN1_PIN  GPIO_PIN_7    /* Arduino D9 */
#define MOT_AIN2_PORT GPIOA
#define MOT_AIN2_PIN  GPIO_PIN_9    /* Arduino D8 */
#define MOT_STBY_PORT GPIOA
#define MOT_STBY_PIN  GPIO_PIN_8    /* Arduino D7 */

static void uart_puts(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 200);
}

static uint16_t adc1_read_channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        return 0xFFFFu;
    }
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
        HAL_ADC_Stop(&hadc1);
        return 0xFFFFu;
    }
    uint16_t v = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return v;
}

static void motor_set_duty_pct(float duty_pct)
{
    if (duty_pct < 0.0f)            duty_pct = 0.0f;
    if (duty_pct > CL_DUTY_MAX_PCT) duty_pct = CL_DUTY_MAX_PCT;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim4);
    uint32_t ccr = (uint32_t)((duty_pct * 0.01f) * (float)(arr + 1u) + 0.5f);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccr);
}

void ClosedLoop_Init(void)
{
    HAL_GPIO_WritePin(MOT_AIN1_PORT, MOT_AIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOT_AIN2_PORT, MOT_AIN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOT_STBY_PORT, MOT_STBY_PIN, GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    motor_set_duty_pct(0.0f);
#ifdef STM32F446xx
    Prof_Init();
#endif
    App_Init();
#if defined(HAL_CAN_MODULE_ENABLED) || defined(USE_CAN_TELEMETRY)
    CanTelemetry_Init();
#endif
}

void ClosedLoop_RunLive(uint32_t seconds, const char *file_name)
{
    char line[128];
    ClosedLoop_Init();

    uart_puts("# === CLOSED-LOOP LIVE CONTROL START (Move Joystick to control speed!) ===\r\n");
    snprintf(line, sizeof(line),
             "#BEGIN %s\r\nt_ms,adcX,adcY,speed_req_rpm,speed_meas_rpm,duty_cmd_pct,fault_bitmap,mode,wcet_us\r\n",
             file_name);
    uart_puts(line);

    uint16_t last_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    uint32_t n_ticks  = (seconds * 1000u) / CL_PERIOD_MS;
    uint32_t t0       = HAL_GetTick();
    const float rpm_scale = 60.0f / (CL_ENCODER_CPR * ((float)CL_PERIOD_MS * 0.001f));

    for (uint32_t k = 0; k < n_ticks; k++) {
        while ((HAL_GetTick() - t0) < (k * CL_PERIOD_MS)) { /* wait for 10 ms tick */ }

        uint16_t adcX = adc1_read_channel(ADC_CHANNEL_0);
        uint16_t adcY = adc1_read_channel(ADC_CHANNEL_1);
        uint16_t cnt  = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
        int16_t  raw_dN = (int16_t)(cnt - last_cnt);
        int16_t  dN   = (raw_dN < 0) ? (int16_t)(-raw_dN) : raw_dN;
        last_cnt = cnt;

        float speed_meas_rpm = (float)dN * rpm_scale;

        Rte_AppIn_t  in  = { (float)adcX, (float)adcY, speed_meas_rpm };
        Rte_AppOut_t out = { 0 };

        uint32_t wcet_us = 0u;
#ifdef STM32F446xx
        uint32_t c0 = Prof_Now();
#endif
        App_Step10ms(&in, &out);
#ifdef STM32F446xx
        wcet_us = Prof_CyclesToUs(Prof_Now() - c0);
#endif
        motor_set_duty_pct(out.duty_cmd_pct);
#if defined(HAL_CAN_MODULE_ENABLED) || defined(USE_CAN_TELEMETRY)
        CanTelemetry_Step10ms(k, &in, &out, 1000u, 0u, wcet_us);
#endif

        snprintf(line, sizeof(line), "%lu,%u,%u,%.2f,%.2f,%.2f,%u,%u,%lu\r\n",
                 (unsigned long)(k * CL_PERIOD_MS),
                 adcX, adcY,
                 out.speed_req_rpm, speed_meas_rpm, out.duty_cmd_pct,
                 out.fault_bitmap, out.mode, (unsigned long)wcet_us);
        uart_puts(line);
    }

    motor_set_duty_pct(0.0f);
    uart_puts("#END\r\n");
    uart_puts("# === CLOSED-LOOP LIVE CONTROL COMPLETE ===\r\n");
}

void ClosedLoop_RunAutoStep(const char *file_name)
{
    char line[128];
    ClosedLoop_Init();

    uart_puts("# === CLOSED-LOOP 6-SECOND AUTO STEP TEST START ===\r\n");
    snprintf(line, sizeof(line),
             "#BEGIN %s\r\nt_ms,adcX,adcY,speed_req_rpm,speed_meas_rpm,duty_cmd_pct,fault_bitmap,mode,wcet_us\r\n",
             file_name);
    uart_puts(line);

    uint16_t last_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    uint32_t n_ticks  = 600u; /* 6.0 s at 10 ms */
    uint32_t t0       = HAL_GetTick();
    const float rpm_scale = 60.0f / (CL_ENCODER_CPR * ((float)CL_PERIOD_MS * 0.001f));

    for (uint32_t k = 0; k < n_ticks; k++) {
        while ((HAL_GetTick() - t0) < (k * CL_PERIOD_MS)) { }

        uint32_t t_ms = k * CL_PERIOD_MS;
        uint16_t adcX = (t_ms >= 500u && t_ms < 3500u) ? 3209u : 2044u;
        uint16_t adcY = (t_ms >= 500u && t_ms < 3500u) ? 3222u : 2071u;

        uint16_t cnt    = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
        int16_t  raw_dN = (int16_t)(cnt - last_cnt);
        int16_t  dN     = (raw_dN < 0) ? (int16_t)(-raw_dN) : raw_dN;
        last_cnt = cnt;

        float speed_meas_rpm = (float)dN * rpm_scale;

        Rte_AppIn_t  in  = { (float)adcX, (float)adcY, speed_meas_rpm };
        Rte_AppOut_t out = { 0 };

        uint32_t wcet_us = 0u;
#ifdef STM32F446xx
        uint32_t c0 = Prof_Now();
#endif
        App_Step10ms(&in, &out);
#ifdef STM32F446xx
        wcet_us = Prof_CyclesToUs(Prof_Now() - c0);
#endif
        motor_set_duty_pct(out.duty_cmd_pct);
#if defined(HAL_CAN_MODULE_ENABLED) || defined(USE_CAN_TELEMETRY)
        CanTelemetry_Step10ms(k, &in, &out, 1000u, 0u, wcet_us);
#endif

        snprintf(line, sizeof(line), "%lu,%u,%u,%.2f,%.2f,%.2f,%u,%u,%lu\r\n",
                 (unsigned long)t_ms,
                 adcX, adcY,
                 out.speed_req_rpm, speed_meas_rpm, out.duty_cmd_pct,
                 out.fault_bitmap, out.mode, (unsigned long)wcet_us);
        uart_puts(line);
    }

    motor_set_duty_pct(0.0f);
    uart_puts("#END\r\n");
    uart_puts("# === CLOSED-LOOP AUTO STEP TEST COMPLETE ===\r\n");
}
