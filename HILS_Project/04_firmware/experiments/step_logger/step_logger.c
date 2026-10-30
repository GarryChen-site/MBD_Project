/* step_logger.c - Stage 2 open-loop motor + encoder calibration logger
 * Pinout aligned 100% with smoketest/motor and smoketest/encoder:
 *   - PWMA : PB6 (TIM4_CH1, 20 kHz, PSC=0, ARR=799) -> extern TIM_HandleTypeDef htim4
 *   - AIN1 : PC7 (Arduino D9)
 *   - AIN2 : PA9 (Arduino D8)
 *   - STBY : PA8 (Arduino D7)
 *   - ENC  : PA6/PA7 (TIM3_CH1/CH2, TI1&TI2 X4)     -> extern TIM_HandleTypeDef htim3
 *   - UART : PA2/PA3 (USART2, 115200 8N1)           -> extern UART_HandleTypeDef huart2
 */
#include "step_logger.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

extern TIM_HandleTypeDef  htim4;    /* PWM, PB6 = TIM4_CH1, 20 kHz (from smoketest/motor) */
extern TIM_HandleTypeDef  htim3;    /* Encoder mode, PA6/PA7, 16-bit counter (from smoketest/encoder) */
extern UART_HandleTypeDef huart2;   /* ST-LINK VCP, 115200 8N1 */

/* TB6612 control pins matching smoketest/motor */
#define MOT_AIN1_PORT GPIOC
#define MOT_AIN1_PIN  GPIO_PIN_7    /* Arduino D9 */
#define MOT_AIN2_PORT GPIOA
#define MOT_AIN2_PIN  GPIO_PIN_9    /* Arduino D8 */
#define MOT_STBY_PORT GPIOA
#define MOT_STBY_PIN  GPIO_PIN_8    /* Arduino D7 */

typedef struct { uint16_t duty_pm; int16_t dN; } Sample_t;
static Sample_t s_buf[STEP_LOGGER_MAX_SAMPLES];

static void uart_puts(const char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 200);
}

static void motor_set_duty_permille(uint16_t pm)
{
  if (pm > STEP_DUTY_MAX_PERMILLE) pm = STEP_DUTY_MAX_PERMILLE;
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim4);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ((uint32_t)pm * (arr + 1u)) / 1000u);
}

void StepLogger_Init(void)
{
  HAL_GPIO_WritePin(MOT_AIN1_PORT, MOT_AIN1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOT_AIN2_PORT, MOT_AIN2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOT_STBY_PORT, MOT_STBY_PIN, GPIO_PIN_SET);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  motor_set_duty_permille(0);
}

void StepLogger_Run(const StepExp_t *cfg, const char *file_name)
{
  uint32_t n = (cfg->t_pre_ms + cfg->t_step_ms + cfg->t_post_ms) / cfg->period_ms;
  if (n == 0u || n > STEP_LOGGER_MAX_SAMPLES || cfg->period_ms == 0u) {
    motor_set_duty_permille(0);
    return;
  }

  uint16_t last = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  uint32_t t0 = HAL_GetTick();
  for (uint32_t k = 0; k < n; k++) {
    uint32_t t_ms = k * cfg->period_ms;
    uint16_t d = (t_ms >= cfg->t_pre_ms && t_ms < cfg->t_pre_ms + cfg->t_step_ms)
                 ? cfg->duty_step_permille : cfg->duty_start_permille;
    motor_set_duty_permille(d);
    while ((HAL_GetTick() - t0) < (k + 1u) * cfg->period_ms) { }
    uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t raw_dN = (int16_t)(cnt - last);
    /* Use magnitude so forward spin is always positive even if A/B phases count down (smoketest/encoder) */
    s_buf[k].duty_pm = d;
    s_buf[k].dN = (raw_dN < 0) ? (int16_t)(-raw_dN) : raw_dN;
    last = cnt;
  }
  motor_set_duty_permille(0);

  char line[64];
  snprintf(line, sizeof line, "#BEGIN %s\r\nt_ms,duty_permille,dN\r\n", file_name);
  uart_puts(line);
  for (uint32_t k = 0; k < n; k++) {
    snprintf(line, sizeof line, "%lu,%u,%d\r\n",
             (unsigned long)(k * cfg->period_ms), s_buf[k].duty_pm, s_buf[k].dN);
    uart_puts(line);
  }
  uart_puts("#END\r\n");
}

void StepLogger_RunAll(void)
{
  char name[32];
  char msg[120];
  StepExp_t e = { .duty_start_permille = 0, .period_ms = 10 };

  /* 0. 500 ms Motor + Encoder Pre-Flight Spin Check */
  uart_puts("# === MOTOR PRE-FLIGHT CHECK (40% duty for 0.5 s) ===\r\n");
  uint16_t c0 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  motor_set_duty_permille(400);
  HAL_Delay(500);
  uint16_t c1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  motor_set_duty_permille(0);
  int16_t pre_dN = (int16_t)(c1 - c0);
  if (pre_dN < 0) pre_dN = -pre_dN;
  snprintf(msg, sizeof msg, "# [PRE-CHECK] Encoder delta in 0.5 s @ 40%% duty = %d counts\r\n", (int)pre_dN);
  uart_puts(msg);
  if (pre_dN == 0) {
    uart_puts("# [WARNING] Motor did NOT spin or Encoder read 0! Check 6V Battery ON/OFF switch, VM, VCC, and PB6/PC7/PA9/PA8!\r\n");
  }
  HAL_Delay(1500);

  uart_puts("# === STAGE 2 MOTOR CALIBRATION START (19 runs, ~90 s total) ===\r\n");

  /* 1. Static map D05..D50 (10 runs, 2.5 s hold, 2.0 s rest between runs) */
  e.t_pre_ms = 200; e.t_step_ms = 2500; e.t_post_ms = 0;
  for (uint16_t d = 50; d <= 500; d += 50) {
    e.duty_step_permille = d;
    snprintf(name, sizeof name, "static_D%02u.csv", d / 10u);
    snprintf(msg, sizeof msg, "# [Static %u/10] Running %s (Duty = %u%%)...\r\n",
             (unsigned)(d / 50u), name, (unsigned)(d / 10u));
    uart_puts(msg);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    StepLogger_Run(&e, name);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_Delay(2000); /* 2 s coast to full stop */
  }

  /* 2. Dynamic step response: 3 amplitudes (15%, 25%, 35%) x 3 repeats (9 runs) */
  e.t_pre_ms = 500; e.t_step_ms = 2000; e.t_post_ms = 1000;
  const uint16_t amp[3] = {150, 250, 350};
  int run_idx = 0;
  for (int a = 0; a < 3; a++) {
    for (int r = 1; r <= 3; r++) {
      run_idx++;
      e.duty_step_permille = amp[a];
      snprintf(name, sizeof name, "step_%02u_r%d.csv", amp[a] / 10u, r);
      snprintf(msg, sizeof msg, "# [Step %d/9] Running %s (Step 0 -> %u%%)...\r\n",
               run_idx, name, (unsigned)(amp[a] / 10u));
      uart_puts(msg);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
      StepLogger_Run(&e, name);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
      HAL_Delay(2000);
    }
  }
  uart_puts("# === STAGE 2 MOTOR CALIBRATION COMPLETE ===\r\n");
}
