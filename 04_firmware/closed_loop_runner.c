/* closed_loop_runner.c - Pass 2 (Stage 5/6) 10 ms closed-loop hardware runner
 * Uses the exact peripheral handles and pins from
 * 00_smoketest/{joystick,encoder,motor,i2c,servo}.
 */
#include "closed_loop_runner.h"
#include "app_wrapper.h"
#include "main.h"
#include "prof.h"
#if defined(HAL_CAN_MODULE_ENABLED) || defined(USE_CAN_TELEMETRY)
#include "can_telemetry.h"
#endif
#if defined(HAL_I2C_MODULE_ENABLED) || defined(USE_AEB_HARDWARE)
#include "i2c_sensors_hal.h"
#include "servo_hal.h"
#endif
#include <stdio.h>
#include <string.h>

extern TIM_HandleTypeDef
    htim4; /* PWM, PB6 = TIM4_CH1, 20 kHz (smoketest/motor) */
extern TIM_HandleTypeDef
    htim3; /* Encoder mode, PA6/PA7, 16-bit counter (smoketest/encoder) */
extern ADC_HandleTypeDef
    hadc1; /* PA0 = IN0 (X), PA1 = IN1 (Y) (smoketest/joystick) */
extern UART_HandleTypeDef huart2; /* ST-LINK VCP, 115200 8N1 */

#if defined(HAL_I2C_MODULE_ENABLED) || defined(USE_AEB_HARDWARE)
extern I2C_HandleTypeDef
    hi2c1; /* PB8 = I2C1_SCL (D15), PB9 = I2C1_SDA (D14) (Option 3B ToF) */
extern TIM_HandleTypeDef
    htim2; /* PB10 = TIM2_CH3 (D6), 50 Hz Servo PWM (PSC=15, ARR=19999) */
#ifndef SERVO_TIM_HANDLE
#define SERVO_TIM_HANDLE (&htim2)
#endif
#ifndef SERVO_TIM_CHANNEL
#define SERVO_TIM_CHANNEL TIM_CHANNEL_3
#endif
#endif

/* TB6612 direction and standby pins matching smoketest/motor */
#define MOT_AIN1_PORT GPIOC
#define MOT_AIN1_PIN GPIO_PIN_7 /* Arduino D9 */
#define MOT_AIN2_PORT GPIOA
#define MOT_AIN2_PIN GPIO_PIN_9 /* Arduino D8 */
#define MOT_STBY_PORT GPIOA
#define MOT_STBY_PIN GPIO_PIN_8 /* Arduino D7 */

static void uart_puts(const char *s) {
  HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 200);
}

static uint16_t adc1_read_channel(uint32_t channel) {
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

static void motor_set_duty_pct(float duty_pct) {
  if (duty_pct < 0.0f)
    duty_pct = 0.0f;
  if (duty_pct > CL_DUTY_MAX_PCT)
    duty_pct = CL_DUTY_MAX_PCT;
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim4);
  uint32_t ccr = (uint32_t)((duty_pct * 0.01f) * (float)(arr + 1u) + 0.5f);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccr);
}

void ClosedLoop_Init(void) {
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

void ClosedLoop_RunLive(uint32_t seconds, const char *file_name) {
  char line[128];
  ClosedLoop_Init();

  uart_puts("# === CLOSED-LOOP LIVE CONTROL START (Move Joystick to control "
            "speed!) ===\r\n");
  snprintf(line, sizeof(line),
           "#BEGIN "
           "%s\r\nt_ms,adcX,adcY,speed_req_rpm,speed_meas_rpm,duty_cmd_pct,"
           "fault_bitmap,mode,wcet_us\r\n",
           file_name);
  uart_puts(line);

  uint16_t last_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  uint32_t n_ticks = (seconds * 1000u) / CL_PERIOD_MS;
  uint32_t t0 = HAL_GetTick();
  const float rpm_scale =
      60.0f / (CL_ENCODER_CPR * ((float)CL_PERIOD_MS * 0.001f));

  for (uint32_t k = 0; k < n_ticks; k++) {
    while ((HAL_GetTick() - t0) <
           (k * CL_PERIOD_MS)) { /* wait for 10 ms tick */
    }

    uint16_t adcX = adc1_read_channel(ADC_CHANNEL_0);
    uint16_t adcY = adc1_read_channel(ADC_CHANNEL_1);
    uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t raw_dN = (int16_t)(cnt - last_cnt);
    int16_t dN = (raw_dN < 0) ? (int16_t)(-raw_dN) : raw_dN;
    last_cnt = cnt;

    float speed_meas_rpm = (float)dN * rpm_scale;

    Rte_AppIn_t in = {(float)adcX, (float)adcY, speed_meas_rpm};
    Rte_AppOut_t out = {0};

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
             (unsigned long)(k * CL_PERIOD_MS), adcX, adcY, out.speed_req_rpm,
             speed_meas_rpm, out.duty_cmd_pct, out.fault_bitmap, out.mode,
             (unsigned long)wcet_us);
    uart_puts(line);
  }

  motor_set_duty_pct(0.0f);
  uart_puts("#END\r\n");
  uart_puts("# === CLOSED-LOOP LIVE CONTROL COMPLETE ===\r\n");
}

void ClosedLoop_RunAutoStep(const char *file_name) {
  char line[128];
  ClosedLoop_Init();

  uart_puts("# === CLOSED-LOOP 6-SECOND AUTO STEP TEST START ===\r\n");
  snprintf(line, sizeof(line),
           "#BEGIN "
           "%s\r\nt_ms,adcX,adcY,speed_req_rpm,speed_meas_rpm,duty_cmd_pct,"
           "fault_bitmap,mode,wcet_us\r\n",
           file_name);
  uart_puts(line);

  uint16_t last_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  uint32_t n_ticks = 600u; /* 6.0 s at 10 ms */
  uint32_t t0 = HAL_GetTick();
  const float rpm_scale =
      60.0f / (CL_ENCODER_CPR * ((float)CL_PERIOD_MS * 0.001f));

  for (uint32_t k = 0; k < n_ticks; k++) {
    while ((HAL_GetTick() - t0) < (k * CL_PERIOD_MS)) {
    }

    uint32_t t_ms = k * CL_PERIOD_MS;
    uint16_t adcX = (t_ms >= 500u && t_ms < 3500u) ? 3209u : 2044u;
    uint16_t adcY = (t_ms >= 500u && t_ms < 3500u) ? 3222u : 2071u;

    uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t raw_dN = (int16_t)(cnt - last_cnt);
    int16_t dN = (raw_dN < 0) ? (int16_t)(-raw_dN) : raw_dN;
    last_cnt = cnt;

    float speed_meas_rpm = (float)dN * rpm_scale;

    Rte_AppIn_t in = {(float)adcX, (float)adcY, speed_meas_rpm};
    Rte_AppOut_t out = {0};

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
             (unsigned long)t_ms, adcX, adcY, out.speed_req_rpm, speed_meas_rpm,
             out.duty_cmd_pct, out.fault_bitmap, out.mode,
             (unsigned long)wcet_us);
    uart_puts(line);
  }

  motor_set_duty_pct(0.0f);
  uart_puts("#END\r\n");
  uart_puts("# === CLOSED-LOOP AUTO STEP TEST COMPLETE ===\r\n");
}

void ClosedLoop_RunAEB(uint32_t seconds, const char *file_name) {
#if defined(HAL_I2C_MODULE_ENABLED) || defined(USE_AEB_HARDWARE)
  char line[160];
  ClosedLoop_Init();

  SG90_Servo_t servo;
  Servo_Init(&servo, SERVO_TIM_HANDLE, SERVO_TIM_CHANNEL);
  Servo_SetAngle(&servo, AEB_SERVO_RELEASE_DEG);
  Servo_Start(&servo);

  bool tof_ok = (VL53L0X_Init(&hi2c1) == I2C_SENSORS_OK);
  if (tof_ok) {
    uart_puts("# [AEB] VL53L0X ToF (0x29) initialized OK on I2C1 (PB8/PB9); "
              "SG90 Servo ready on TIM2_CH3 (PB10)\r\n");
  } else {
    uart_puts("# [AEB] WARNING: VL53L0X init failed on I2C1! Running with "
              "F_TOF_INVALID protection.\r\n");
  }

  uart_puts("# === CLOSED-LOOP + AEB (VL53L0X + SG90 SERVO) START ===\r\n");
  snprintf(line, sizeof(line),
           "#BEGIN "
           "%s\r\nt_ms,adcX,adcY,speed_req_rpm,speed_meas_rpm,duty_cmd_pct,"
           "fault_bitmap,mode,wcet_us,d_mm,tof_valid,aeb_state,servo_deg\r\n",
           file_name);
  uart_puts(line);

  uint16_t last_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  uint32_t n_ticks = (seconds * 1000u) / CL_PERIOD_MS;
  uint32_t t0 = HAL_GetTick();
  const float rpm_scale =
      60.0f / (CL_ENCODER_CPR * ((float)CL_PERIOD_MS * 0.001f));

  uint16_t last_d_mm = 1000u;
  bool last_valid = false;
  uint8_t aeb_state = (uint8_t)AEB_STATE_NORMAL;

  for (uint32_t k = 0; k < n_ticks; k++) {
    while ((HAL_GetTick() - t0) <
           (k * CL_PERIOD_MS)) { /* wait for 10 ms tick */
    }

    uint16_t adcX = adc1_read_channel(ADC_CHANNEL_0);
    uint16_t adcY = adc1_read_channel(ADC_CHANNEL_1);
    uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t raw_dN = (int16_t)(cnt - last_cnt);
    int16_t dN = (raw_dN < 0) ? (int16_t)(-raw_dN) : raw_dN;
    last_cnt = cnt;

    float speed_meas_rpm = (float)dN * rpm_scale;

    /* Non-blocking 50 ms VL53L0X schedule:
     *   k % 5 == 0 (0, 50, 100 ms...): Arm single-shot measurement (~0.3 ms)
     *   k % 5 == 4 (40, 90, 140 ms...): Read ready distance & step 50 ms AEB
     * FSM (~0.2 ms)
     */
    if ((k % 5u) == 0u) {
      if (tof_ok) {
        (void)VL53L0X_StartSingleShot(&hi2c1);
      }
    } else if ((k % 5u) == 4u) {
      if (tof_ok) {
        int32_t d = VL53L0X_ReadReadyDistance_mm(&hi2c1);
        if (d > 0 && d <= 4000) {
          last_d_mm = (uint16_t)d;
          last_valid = true;
        } else {
          last_valid = false;
        }
      } else {
        last_valid = false;
      }
      aeb_state = App_AebStep50ms(last_d_mm, last_valid, speed_meas_rpm, NULL);
    }

    Rte_AppIn_t in = {(float)adcX, (float)adcY, speed_meas_rpm};
    Rte_AppOut_t out = {0};
    float servo_deg = AEB_SERVO_RELEASE_DEG;

    uint32_t wcet_us = 0u;
#ifdef STM32F446xx
    uint32_t c0 = Prof_Now();
#endif
    App_Step10ms(&in, &out);
    App_ApplyAebOverride(&out, &servo_deg);
#ifdef STM32F446xx
    wcet_us = Prof_CyclesToUs(Prof_Now() - c0);
#endif

    motor_set_duty_pct(out.duty_cmd_pct);
    Servo_SetAngle(&servo, servo_deg);

#if defined(HAL_CAN_MODULE_ENABLED) || defined(USE_CAN_TELEMETRY)
    CanTelemetry_Step10ms(k, &in, &out, last_d_mm, aeb_state, wcet_us);
#endif

    snprintf(line, sizeof(line),
             "%lu,%u,%u,%.2f,%.2f,%.2f,%u,%u,%lu,%u,%u,%u,%.1f\r\n",
             (unsigned long)(k * CL_PERIOD_MS), adcX, adcY, out.speed_req_rpm,
             speed_meas_rpm, out.duty_cmd_pct, out.fault_bitmap, out.mode,
             (unsigned long)wcet_us, last_d_mm, last_valid ? 1u : 0u, aeb_state,
             servo_deg);
    uart_puts(line);
  }

  motor_set_duty_pct(0.0f);
  Servo_SetAngle(&servo, AEB_SERVO_RELEASE_DEG);
  Servo_Release(&servo);
  uart_puts("#END\r\n");
  uart_puts("# === CLOSED-LOOP + AEB COMPLETE ===\r\n");
#else
  (void)seconds;
  (void)file_name;
  uart_puts("# ERROR: ClosedLoop_RunAEB requires I2C1 (hi2c1) and TIM2 (htim2) "
            "enabled in CubeMX!\r\n");
#endif
}