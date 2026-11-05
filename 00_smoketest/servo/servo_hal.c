/**
 ******************************************************************************
 * @file    servo_hal.c
 * @brief   SG90 180-Degree Micro Servo Driver Implementation
 *          Target: STM32F446RE (Nucleo-64)
 ******************************************************************************
 */

#include "servo_hal.h"

void Servo_Init(SG90_Servo_t *servo, TIM_HandleTypeDef *htim, uint32_t channel) {
    if (!servo || !htim) return;

    servo->htim = htim;
    servo->channel = channel;
    servo->current_angle_deg = 90.0f;
    servo->current_pulse_us = SERVO_MID_PULSE_US;
    servo->is_active = false;
}

void Servo_Start(SG90_Servo_t *servo) {
    if (!servo || !servo->htim) return;

    // Apply neutral position before enabling output
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, servo->current_pulse_us);
    HAL_TIM_PWM_Start(servo->htim, servo->channel);
    servo->is_active = true;
}

void Servo_Stop(SG90_Servo_t *servo) {
    if (!servo || !servo->htim) return;

    HAL_TIM_PWM_Stop(servo->htim, servo->channel);
    servo->is_active = false;
}

void Servo_SetPulseWidth_us(SG90_Servo_t *servo, uint16_t pulse_us) {
    if (!servo || !servo->htim) return;

    // Clamp strictly within the 500 us .. 2500 us mechanical safety limits
    if (pulse_us < SERVO_MIN_PULSE_US) {
        pulse_us = SERVO_MIN_PULSE_US;
    } else if (pulse_us > SERVO_MAX_PULSE_US) {
        pulse_us = SERVO_MAX_PULSE_US;
    }

    servo->current_pulse_us = pulse_us;
    // Map pulse back to angle
    servo->current_angle_deg = ((float)(pulse_us - SERVO_MIN_PULSE_US) / 2000.0f) * 180.0f;

    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse_us);
}

void Servo_SetAngle(SG90_Servo_t *servo, float angle_deg) {
    if (!servo || !servo->htim) return;

    // Clamp angle to valid [0.0 .. 180.0] range
    if (angle_deg < SERVO_MIN_ANGLE_DEG) {
        angle_deg = SERVO_MIN_ANGLE_DEG;
    } else if (angle_deg > SERVO_MAX_ANGLE_DEG) {
        angle_deg = SERVO_MAX_ANGLE_DEG;
    }

    // Convert angle to microseconds:
    // 0 deg   -> 500 us
    // 90 deg  -> 1500 us
    // 180 deg -> 2500 us
    uint16_t pulse = (uint16_t)(SERVO_MIN_PULSE_US + (angle_deg / 180.0f) * 2000.0f + 0.5f);
    Servo_SetPulseWidth_us(servo, pulse);
}

void Servo_SweepSmooth(SG90_Servo_t *servo, float start_deg, float end_deg, uint32_t step_delay_ms) {
    if (!servo || !servo->htim) return;

    float step = (start_deg < end_deg) ? 1.0f : -1.0f;
    float current = start_deg;

    while (true) {
        Servo_SetAngle(servo, current);
        HAL_Delay(step_delay_ms);

        if ((step > 0.0f && current >= end_deg) || (step < 0.0f && current <= end_deg)) {
            break;
        }
        current += step;
    }
    Servo_SetAngle(servo, end_deg);
}

void Servo_Release(SG90_Servo_t *servo) {
    if (!servo || !servo->htim) return;

    // Setting compare to 0 de-energizes the internal H-bridge,
    // stopping holding torque and completely silencing gear buzzing/humming.
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, 0);
}
