#ifndef JOYSTICK_HAL_H
#define JOYSTICK_HAL_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define JOYSTICK_OK       0
#define JOYSTICK_ERROR  (-1)

#define NOISE_SAMPLE_COUNT 100 // Samples to calculate standard deviation

typedef struct {
    uint16_t raw_x;     // Current 12-bit ADC value X (0 - 4095)
    uint16_t raw_y;     // Current 12-bit ADC value Y (0 - 4095)
    uint8_t  btn_sw;    // Switch state (0 = Pressed, 1 = Released)
    
    // Travel tracking (min/max over session)
    uint16_t min_x;
    uint16_t max_x;
    uint16_t min_y;
    uint16_t max_y;
    
    // Noise metrics at rest (calculated over N samples)
    float    mean_x;
    float    std_x;     // Noise sigma X (counts)
    float    mean_y;
    float    std_y;     // Noise sigma Y (counts)
} Joystick_State_t;

/**
 * @brief  Initialize the joystick module (ADC handle and SW GPIO pin).
 * @param  hadc      Pointer to initialized ADC_HandleTypeDef (12-bit)
 * @param  sw_port   GPIO port for pushbutton SW (e.g., GPIOA)
 * @param  sw_pin    GPIO pin for pushbutton SW (e.g., GPIO_PIN_10)
 */
int Joystick_Init(ADC_HandleTypeDef *hadc, GPIO_TypeDef *sw_port, uint16_t sw_pin);

/**
 * @brief  Read one instantaneous pair of (X, Y) and button state.
 * @param  state  Pointer to Joystick_State_t to update
 */
int Joystick_Read(Joystick_State_t *state);

/**
 * @brief  Collect N samples at rest and compute center mean and noise sigma.
 * @param  state  Pointer to Joystick_State_t to update with std_x and std_y
 * @param  samples Number of samples to collect (e.g., 100)
 */
int Joystick_ComputeNoise(Joystick_State_t *state, uint32_t samples);

/**
 * @brief  Reset travel extremes (min_x, max_x, min_y, max_y).
 */
void Joystick_ResetTravel(Joystick_State_t *state);

#endif /* JOYSTICK_HAL_H */
