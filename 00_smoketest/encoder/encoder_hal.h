#ifndef ENCODER_HAL_H
#define ENCODER_HAL_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define ENCODER_OK       0
#define ENCODER_ERROR  (-1)

// N20 Geared Motor Magnetic Hall Encoder Spec:
// Base PPR = 7 ppr (pulses per motor rotor revolution).
// In TIM3 4x mode (TI1 and TI2), counts per motor rotor rev = 7 * 4 = 28 counts.
// After gearbox with ratio G, output shaft counts per rev: N_rev = 28 * G.
#define ENCODER_BASE_PPR            7
#define ENCODER_MOTOR_COUNTS_PER_REV (ENCODER_BASE_PPR * 4) // 28 counts/motor rev

#define MAX_CALIBRATION_TRIALS      5
#define ENCODER_TOLERANCE_PCT       2.0f // Acceptance milestone: +-2%

typedef enum {
    ENCODER_DIR_FORWARD = 1,  // Clockwise (CW)
    ENCODER_DIR_BACKWARD = -1 // Counter-Clockwise (CCW)
} Encoder_Direction_t;

typedef struct {
    uint16_t raw_cnt;         // Raw 16-bit timer register (TIM3->CNT)
    uint16_t last_raw_cnt;    // Previous 16-bit reading
    int32_t  total_counts;    // Continuous accumulated signed 32-bit position
    int8_t   direction;       // +1 = CW, -1 = CCW
    bool     is_moving;       // True if delta detected in last update
    
    // Calibrated output shaft parameters (from 3-5 tape trials)
    int32_t  calibrated_n_rev;// Measured average N_rev per 1 shaft turn
    float    estimated_ratio; // Estimated gear ratio G = N_rev / 28
    
    // Multi-trial tape calibration storage
    int32_t  trial_counts[MAX_CALIBRATION_TRIALS];
    uint8_t  trial_count;
} Encoder_State_t;

/**
 * @brief  Initialize encoder module bound to TIM_HandleTypeDef (TIM3).
 * @param  htim  Pointer to TIM_HandleTypeDef configured in Encoder Mode (TI1 and TI2)
 */
int Encoder_Init(TIM_HandleTypeDef *htim);

/**
 * @brief  Update encoder state, handling 16-bit modular wrap-around.
 *         Called periodically (every 1 to 20 ms).
 * @param  state Pointer to Encoder_State_t
 */
int Encoder_Update(Encoder_State_t *state);

/**
 * @brief  Zero the counter and reset total accumulated counts to 0.
 * @param  state Pointer to Encoder_State_t
 */
void Encoder_Zero(Encoder_State_t *state);

/**
 * @brief  Record current total counts as one completed 1-turn trial (for 3-5 trial averaging).
 * @param  state Pointer to Encoder_State_t
 * @return Number of trials recorded so far (1 to 5)
 */
uint8_t Encoder_RecordTrial(Encoder_State_t *state);

/**
 * @brief  Compute the average N_rev and check if all trials fall within +-2% tolerance.
 * @param  state           Pointer to Encoder_State_t
 * @param  avg_n_rev_out   Output pointer to average N_rev
 * @param  max_error_pct   Output pointer to maximum percentage deviation across trials
 * @return true if all trials are within +-2% and direction is consistent, false otherwise
 */
bool Encoder_ComputeAverage(Encoder_State_t *state, float *avg_n_rev_out, float *max_error_pct);

#endif /* ENCODER_HAL_H */
