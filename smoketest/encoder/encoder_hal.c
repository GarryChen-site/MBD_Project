#include "encoder_hal.h"
#include <math.h>
#include <stdlib.h>

static TIM_HandleTypeDef *p_htim = NULL;

int Encoder_Init(TIM_HandleTypeDef *htim) {
    if (!htim) return ENCODER_ERROR;
    p_htim = htim;

    // Start TIM3 in Encoder Interface Mode for both channels (TI1 and TI2)
    if (HAL_TIM_Encoder_Start(p_htim, TIM_CHANNEL_ALL) != HAL_OK) {
        return ENCODER_ERROR;
    }
    return ENCODER_OK;
}

void Encoder_Zero(Encoder_State_t *state) {
    if (!state || !p_htim) return;

    __HAL_TIM_SET_COUNTER(p_htim, 0);
    state->raw_cnt = 0;
    state->last_raw_cnt = 0;
    state->total_counts = 0;
    state->direction = 1;
    state->is_moving = false;
}

int Encoder_Update(Encoder_State_t *state) {
    if (!p_htim || !state) return ENCODER_ERROR;

    uint16_t current = (uint16_t)__HAL_TIM_GET_COUNTER(p_htim);

    // Compute signed 16-bit delta with automatic modular wrap-around handling
    // (e.g. 65535 -> 0 wraps cleanly to +1; 0 -> 65535 wraps cleanly to -1)
    int16_t delta = (int16_t)(current - state->last_raw_cnt);

    state->raw_cnt = current;
    state->last_raw_cnt = current;
    state->total_counts += delta;
    state->is_moving = (delta != 0);

    // Read hardware direction bit from TIM3 CR1 register
    // 0 = Counter counts UP (CW), 1 = Counter counts DOWN (CCW)
    state->direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(p_htim) ? ENCODER_DIR_BACKWARD : ENCODER_DIR_FORWARD;

    return ENCODER_OK;
}

uint8_t Encoder_RecordTrial(Encoder_State_t *state) {
    if (!state) return 0;
    if (state->trial_count < MAX_CALIBRATION_TRIALS) {
        state->trial_counts[state->trial_count] = state->total_counts;
        state->trial_count++;
    }
    return state->trial_count;
}

bool Encoder_ComputeAverage(Encoder_State_t *state, float *avg_n_rev_out, float *max_error_pct) {
    if (!state || state->trial_count < 3) return false;

    float sum = 0.0f;
    for (uint8_t i = 0; i < state->trial_count; i++) {
        sum += (float)state->trial_counts[i];
    }
    float avg = sum / (float)state->trial_count;

    if (avg_n_rev_out) {
        *avg_n_rev_out = avg;
    }

    state->calibrated_n_rev = (int32_t)lroundf(fabsf(avg));
    state->estimated_ratio = (float)state->calibrated_n_rev / (float)ENCODER_MOTOR_COUNTS_PER_REV;

    float max_err = 0.0f;
    for (uint8_t i = 0; i < state->trial_count; i++) {
        float err = fabsf((float)state->trial_counts[i] - avg) / fabsf(avg) * 100.0f;
        if (err > max_err) {
            max_err = err;
        }
    }

    if (max_error_pct) {
        *max_error_pct = max_err;
    }

    return (max_err <= ENCODER_TOLERANCE_PCT);
}
