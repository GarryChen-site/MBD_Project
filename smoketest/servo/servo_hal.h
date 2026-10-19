/**
 ******************************************************************************
 * @file    servo_hal.h
 * @brief   SG90 180-Degree Micro Servo Driver (50 Hz PWM on TIM3)
 *          Target: STM32F446RE (Nucleo-64)
 ******************************************************************************
 */

#ifndef SERVO_HAL_H
#define SERVO_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* =========================================================================
 * SG90 PWM Timing Specifications (50 Hz / 20 ms Period)
 * Timer Counter Clock: 1 MHz (1 tick = 1.0 us)
 * ARR: 19999 (20,000 us = 20.0 ms period -> 50.0 Hz)
 * ========================================================================= */
#define SERVO_PWM_PERIOD_US         20000U  /* 20 ms period (50 Hz) */
#define SERVO_MIN_PULSE_US          500U    /* 0 degrees (0.5 ms) */
#define SERVO_MID_PULSE_US          1500U   /* 90 degrees neutral (1.5 ms) */
#define SERVO_MAX_PULSE_US          2500U   /* 180 degrees (2.5 ms) */
#define SERVO_DEADBAND_US           8U      /* Dead band width (from spec) */

#define SERVO_MIN_ANGLE_DEG         0.0f
#define SERVO_MAX_ANGLE_DEG         180.0f

/**
 * @brief SG90 Servo Handle Structure
 */
typedef struct {
    TIM_HandleTypeDef *htim;     /* Timer handle (e.g. &htim3) */
    uint32_t channel;            /* Timer channel (e.g. TIM_CHANNEL_1) */
    float current_angle_deg;     /* Current commanded angle [0..180] */
    uint16_t current_pulse_us;   /* Current active pulse width [500..2500] */
    bool is_active;              /* PWM generation active flag */
} SG90_Servo_t;

/* Public API */
void Servo_Init(SG90_Servo_t *servo, TIM_HandleTypeDef *htim, uint32_t channel);
void Servo_Start(SG90_Servo_t *servo);
void Servo_Stop(SG90_Servo_t *servo);
void Servo_SetPulseWidth_us(SG90_Servo_t *servo, uint16_t pulse_us);
void Servo_SetAngle(SG90_Servo_t *servo, float angle_deg);
void Servo_SweepSmooth(SG90_Servo_t *servo, float start_deg, float end_deg, uint32_t step_delay_ms);
void Servo_Release(SG90_Servo_t *servo);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_HAL_H */
