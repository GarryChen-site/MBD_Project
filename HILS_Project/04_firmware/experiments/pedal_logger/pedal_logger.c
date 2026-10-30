/* pedal_logger.c - Stage 2 KY-023 dual-axis joystick logger
 * Pinout aligned 100% with smoketest/joystick:
 *   - VRx : PA0 (ADC1_IN0) on hadc1 (84 cycles sampling time)
 *   - VRy : PA1 (ADC1_IN1) on hadc1 (84 cycles sampling time)
 *   - SW  : PA10 (GPIO_Input, Pull-up)
 *   - LD2 : PA5  (Green LED countdown indicator)
 */
#include "pedal_logger.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

extern ADC_HandleTypeDef  hadc1;    /* PA0 = IN0 (X), PA1 = IN1 (Y) — matches smoketest/joystick */
extern UART_HandleTypeDef huart2;

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

void Mcal_Adc_ReadPedals(uint16_t *x_counts, uint16_t *y_counts)
{
  *x_counts = adc1_read_channel(ADC_CHANNEL_0); /* PA0 = VRx */
  *y_counts = adc1_read_channel(ADC_CHANNEL_1); /* PA1 = VRy */
}

static void uart_puts(const char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 200);
}

void PedalLogger_Run(const char *file_name, uint32_t seconds)
{
  char line[64];
  snprintf(line, sizeof line, "#BEGIN %s\r\nt_ms,adcX,adcY\r\n", file_name);
  uart_puts(line);
  uint32_t t0 = HAL_GetTick(), n = seconds * 100u;
  for (uint32_t k = 0; k < n; k++) {
    while ((HAL_GetTick() - t0) < k * 10u) { }
    uint16_t x, y;
    Mcal_Adc_ReadPedals(&x, &y);
    snprintf(line, sizeof line, "%lu,%u,%u\r\n", (unsigned long)(k * 10u), x, y);
    uart_puts(line);
  }
  uart_puts("#END\r\n");
}

static void countdown(const char *msg)
{
  char buf[96];
  uart_puts("\r\n");
  uart_puts(msg);
  uart_puts("\r\n");
  for (int s = 5; s > 0; s--) {
    snprintf(buf, sizeof buf, "#   Starting in %d s...\r\n", s);
    uart_puts(buf);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_Delay(1000);
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

void PedalLogger_RunSessions(void)
{
  uart_puts("# === STAGE 2 JOYSTICK CALIBRATION START (Hold module with 5 pins at BOTTOM) ===\r\n");
  countdown("# [Session 1/4] joy_rest.csv (10 s): HANDS OFF (leave stick at CENTRE)!");
  PedalLogger_Run("joy_rest.csv", 10);

  countdown("# [Session 2/4] joy_diag_full.csv (25 s): Push UP-RIGHT (45 deg) all the way to the corner ~10 times!");
  PedalLogger_Run("joy_diag_full.csv", 25);

  countdown("# [Session 3/4] joy_diag_valid.csv (45 s): Push UP-RIGHT (45 deg) ~20 times (mix half-way & full-way)!");
  PedalLogger_Run("joy_diag_valid.csv", 45);

  countdown("# [Session 4/4] joy_xonly.csv (25 s): Push straight RIGHT only (~10 times, do NOT push UP/DOWN)!");
  PedalLogger_Run("joy_xonly.csv", 25);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  uart_puts("# === STAGE 2 JOYSTICK CALIBRATION COMPLETE ===\r\n");
}
