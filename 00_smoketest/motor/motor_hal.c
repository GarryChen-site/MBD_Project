#include "motor_hal.h"
#include <math.h>

int Motor_Init(TB6612_Motor_t *motor,
               TIM_HandleTypeDef *htim, uint32_t pwm_channel,
               GPIO_TypeDef *ain1_port, uint16_t ain1_pin,
               GPIO_TypeDef *ain2_port, uint16_t ain2_pin,
               GPIO_TypeDef *stby_port, uint16_t stby_pin) {
    if (!motor || !htim || !ain1_port || !ain2_port || !stby_port) {
        return MOTOR_ERROR;
    }

    motor->htim = htim;
    motor->pwm_channel = pwm_channel;
    motor->ain1_port = ain1_port;
    motor->ain1_pin = ain1_pin;
    motor->ain2_port = ain2_port;
    motor->ain2_pin = ain2_pin;
    motor->stby_port = stby_port;
    motor->stby_pin = stby_pin;

    motor->current_duty = 0.0f;
    motor->current_dir = MOTOR_DIR_COAST;
    motor->is_enabled = false;

    // Start with driver disabled in Standby mode (safe state)
    HAL_GPIO_WritePin(motor->stby_port, motor->stby_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->ain1_port, motor->ain1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->ain2_port, motor->ain2_pin, GPIO_PIN_RESET);

    // Set PWM pulse width to 0
    __HAL_TIM_SET_COMPARE(motor->htim, motor->pwm_channel, 0);

    // Start hardware PWM channel
    if (HAL_TIM_PWM_Start(motor->htim, motor->pwm_channel) != HAL_OK) {
        return MOTOR_ERROR;
    }

    return MOTOR_OK;
}

int Motor_SetDuty(TB6612_Motor_t *motor, float duty_pct) {
    if (!motor || !motor->htim) return MOTOR_ERROR;

    if (duty_pct < 0.0f) duty_pct = 0.0f;
    if (duty_pct > 100.0f) duty_pct = 100.0f;

    motor->current_duty = duty_pct;

    // Calculate CCR register value: 0% -> 0, 100% -> (ARR + 1)
    uint32_t pulse = (uint32_t)lroundf((duty_pct / 100.0f) * (float)(MOTOR_PWM_ARR + 1));
    if (pulse > (MOTOR_PWM_ARR + 1)) pulse = MOTOR_PWM_ARR + 1;

    __HAL_TIM_SET_COMPARE(motor->htim, motor->pwm_channel, pulse);
    return MOTOR_OK;
}

int Motor_SetDirection(TB6612_Motor_t *motor, Motor_Dir_t dir) {
    if (!motor) return MOTOR_ERROR;

    motor->current_dir = dir;

    switch (dir) {
        case MOTOR_DIR_FORWARD:
            // AIN1 = HIGH, AIN2 = LOW (Clockwise)
            HAL_GPIO_WritePin(motor->ain1_port, motor->ain1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor->ain2_port, motor->ain2_pin, GPIO_PIN_RESET);
            break;

        case MOTOR_DIR_REVERSE:
            // AIN1 = LOW, AIN2 = HIGH (Counter-Clockwise)
            HAL_GPIO_WritePin(motor->ain1_port, motor->ain1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor->ain2_port, motor->ain2_pin, GPIO_PIN_SET);
            break;

        case MOTOR_DIR_BRAKE:
            // AIN1 = HIGH, AIN2 = HIGH (Short brake)
            HAL_GPIO_WritePin(motor->ain1_port, motor->ain1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor->ain2_port, motor->ain2_pin, GPIO_PIN_SET);
            break;

        case MOTOR_DIR_COAST:
        default:
            // AIN1 = LOW, AIN2 = LOW (Coast / Stop)
            HAL_GPIO_WritePin(motor->ain1_port, motor->ain1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor->ain2_port, motor->ain2_pin, GPIO_PIN_RESET);
            break;
    }

    return MOTOR_OK;
}

void Motor_SetStandby(TB6612_Motor_t *motor, bool enable) {
    if (!motor) return;

    motor->is_enabled = enable;
    HAL_GPIO_WritePin(motor->stby_port, motor->stby_pin,
                      enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

int Motor_SetSpeed(TB6612_Motor_t *motor, float speed_pct) {
    if (!motor) return MOTOR_ERROR;

    if (fabsf(speed_pct) < 0.1f) {
        Motor_SetDuty(motor, 0.0f);
        Motor_SetDirection(motor, MOTOR_DIR_COAST);
    } else if (speed_pct > 0.0f) {
        Motor_SetDirection(motor, MOTOR_DIR_FORWARD);
        Motor_SetDuty(motor, speed_pct);
    } else {
        Motor_SetDirection(motor, MOTOR_DIR_REVERSE);
        Motor_SetDuty(motor, -speed_pct);
    }

    return MOTOR_OK;
}

void Motor_Stop(TB6612_Motor_t *motor) {
    if (!motor) return;
    Motor_SetDuty(motor, 0.0f);
    Motor_SetDirection(motor, MOTOR_DIR_COAST);
}

void Motor_Brake(TB6612_Motor_t *motor) {
    if (!motor) return;
    Motor_SetDuty(motor, 100.0f);
    Motor_SetDirection(motor, MOTOR_DIR_BRAKE);
}
