#ifndef MOTOR_HAL_H
#define MOTOR_HAL_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define MOTOR_OK       0
#define MOTOR_ERROR  (-1)

// 20 kHz PWM Configuration (Ultrasonic / Silent PWM)
// With TIM4 on APB1 (16 MHz) and Prescaler = 0:
// ARR = (16,000,000 / 20,000) - 1 = 799
#define MOTOR_PWM_ARR           799
#define MOTOR_PWM_FREQ_HZ       20000

typedef enum {
    MOTOR_DIR_FORWARD = 0, // Clockwise (CW): AIN1 = HIGH, AIN2 = LOW
    MOTOR_DIR_REVERSE = 1, // Counter-Clockwise (CCW): AIN1 = LOW, AIN2 = HIGH
    MOTOR_DIR_BRAKE   = 2, // Active brake: AIN1 = HIGH, AIN2 = HIGH
    MOTOR_DIR_COAST   = 3  // Passive stop: AIN1 = LOW, AIN2 = LOW
} Motor_Dir_t;

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           pwm_channel; // e.g. TIM_CHANNEL_1
    
    GPIO_TypeDef      *ain1_port;
    uint16_t           ain1_pin;
    
    GPIO_TypeDef      *ain2_port;
    uint16_t           ain2_pin;
    
    GPIO_TypeDef      *stby_port;
    uint16_t           stby_pin;
    
    float              current_duty; // 0.0% to 100.0%
    Motor_Dir_t        current_dir;
    bool               is_enabled;   // STBY pin state
} TB6612_Motor_t;

/**
 * @brief  Initialize TB6612 motor driver instance.
 * @param  motor        Pointer to TB6612_Motor_t structure
 * @param  htim         Pointer to initialized TIM_HandleTypeDef (TIM4)
 * @param  pwm_channel  Timer PWM channel (e.g., TIM_CHANNEL_1)
 * @param  ain1_port    GPIO Port for AIN1
 * @param  ain1_pin     GPIO Pin for AIN1
 * @param  ain2_port    GPIO Port for AIN2
 * @param  ain2_pin     GPIO Pin for AIN2
 * @param  stby_port    GPIO Port for STBY
 * @param  stby_pin     GPIO Pin for STBY
 */
int Motor_Init(TB6612_Motor_t *motor,
               TIM_HandleTypeDef *htim, uint32_t pwm_channel,
               GPIO_TypeDef *ain1_port, uint16_t ain1_pin,
               GPIO_TypeDef *ain2_port, uint16_t ain2_pin,
               GPIO_TypeDef *stby_port, uint16_t stby_pin);

/**
 * @brief  Set motor speed and direction (-100.0% to +100.0%).
 *         Positive = Forward (CW), Negative = Reverse (CCW), 0.0 = Stop.
 */
int Motor_SetSpeed(TB6612_Motor_t *motor, float speed_pct);

/**
 * @brief  Set PWM duty cycle (0.0% to 100.0%).
 */
int Motor_SetDuty(TB6612_Motor_t *motor, float duty_pct);

/**
 * @brief  Set motor direction (FORWARD, REVERSE, BRAKE, COAST).
 */
int Motor_SetDirection(TB6612_Motor_t *motor, Motor_Dir_t dir);

/**
 * @brief  Set Standby (STBY) pin.
 *         true  = Driver Enabled (STBY = HIGH)
 *         false = Driver Disabled / Standby (STBY = LOW -> motor immediately stops)
 */
void Motor_SetStandby(TB6612_Motor_t *motor, bool enable);

/**
 * @brief  Stop motor (coasts to a halt, duty = 0).
 */
void Motor_Stop(TB6612_Motor_t *motor);

/**
 * @brief  Active brake motor (shorts motor terminals, fast stop).
 */
void Motor_Brake(TB6612_Motor_t *motor);

#endif /* MOTOR_HAL_H */
