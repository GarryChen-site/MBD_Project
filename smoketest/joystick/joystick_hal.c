#include "joystick_hal.h"
#include <math.h>
#include <string.h>

static ADC_HandleTypeDef *p_hadc = NULL;
static GPIO_TypeDef *p_sw_port = NULL;
static uint16_t p_sw_pin = 0;

static uint16_t Read_ADC_Channel(uint32_t channel) {
    if (!p_hadc) return 0;

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES; // Longer sampling time reduces noise
    HAL_ADC_ConfigChannel(p_hadc, &sConfig);

    HAL_ADC_Start(p_hadc);
    if (HAL_ADC_PollForConversion(p_hadc, 10) == HAL_OK) {
        uint16_t val = (uint16_t)HAL_ADC_GetValue(p_hadc);
        HAL_ADC_Stop(p_hadc);
        return val;
    }
    HAL_ADC_Stop(p_hadc);
    return 0;
}

int Joystick_Init(ADC_HandleTypeDef *hadc, GPIO_TypeDef *sw_port, uint16_t sw_pin) {
    if (!hadc) return JOYSTICK_ERROR;
    p_hadc = hadc;
    p_sw_port = sw_port;
    p_sw_pin = sw_pin;
    return JOYSTICK_OK;
}

void Joystick_ResetTravel(Joystick_State_t *state) {
    if (!state) return;
    state->min_x = 4095;
    state->max_x = 0;
    state->min_y = 4095;
    state->max_y = 0;
}

int Joystick_Read(Joystick_State_t *state) {
    if (!p_hadc || !state) return JOYSTICK_ERROR;

    // Read VRx (PA0 / ADC_CHANNEL_0) and VRy (PA1 / ADC_CHANNEL_1)
    state->raw_x = Read_ADC_Channel(ADC_CHANNEL_0);
    state->raw_y = Read_ADC_Channel(ADC_CHANNEL_1);

    // Read digital pushbutton (SW) with pull-up (0 = Pressed, 1 = Released)
    if (p_sw_port) {
        state->btn_sw = (HAL_GPIO_ReadPin(p_sw_port, p_sw_pin) == GPIO_PIN_SET) ? 1 : 0;
    } else {
        state->btn_sw = 1;
    }

    // Update travel bounds
    if (state->raw_x < state->min_x) state->min_x = state->raw_x;
    if (state->raw_x > state->max_x) state->max_x = state->raw_x;
    if (state->raw_y < state->min_y) state->min_y = state->raw_y;
    if (state->raw_y > state->max_y) state->max_y = state->raw_y;

    return JOYSTICK_OK;
}

int Joystick_ComputeNoise(Joystick_State_t *state, uint32_t samples) {
    if (!p_hadc || !state || samples < 2) return JOYSTICK_ERROR;

    double sum_x = 0.0, sum_y = 0.0;
    double sum_sq_x = 0.0, sum_sq_y = 0.0;

    for (uint32_t i = 0; i < samples; i++) {
        uint16_t x = Read_ADC_Channel(ADC_CHANNEL_0);
        uint16_t y = Read_ADC_Channel(ADC_CHANNEL_1);

        sum_x += x;
        sum_y += y;
        sum_sq_x += ((double)x * (double)x);
        sum_sq_y += ((double)y * (double)y);

        HAL_Delay(2); // 2 ms delay between samples
    }

    double n = (double)samples;
    double mean_x = sum_x / n;
    double mean_y = sum_y / n;

    double var_x = (sum_sq_x / n) - (mean_x * mean_x);
    double var_y = (sum_sq_y / n) - (mean_y * mean_y);

    state->mean_x = (float)mean_x;
    state->mean_y = (float)mean_y;
    state->std_x = (var_x > 0.0) ? (float)sqrt(var_x) : 0.0f;
    state->std_y = (var_y > 0.0) ? (float)sqrt(var_y) : 0.0f;

    return JOYSTICK_OK;
}
