/* step_logger.h - Stage 2 experiment logger (open-loop duty steps, 10 ms, encoder counts)
 *
 * Output format (matches MATLAB stage2_lib('load_step_log')):
 *     #BEGIN step_25_r1.csv
 *     t_ms,duty_permille,dN
 *     0,0,0
 *     ...
 *     #END
 * Timing definition (ISS-2-06): the duty of period k is applied at the START of period k,
 * dN[k] = encoder counts accumulated DURING period k.
 */
#ifndef STEP_LOGGER_H
#define STEP_LOGGER_H

#include <stdint.h>

#define STEP_LOGGER_MAX_SAMPLES   600u    /* 6 s at 10 ms */
#define STEP_DUTY_MAX_PERMILLE    500u    /* hard cap = Dmax = Vrated/Vbat (P-005), recompute if Vbat changes */

typedef struct {
  uint16_t duty_start_permille;
  uint16_t duty_step_permille;
  uint32_t t_pre_ms, t_step_ms, t_post_ms;
  uint32_t period_ms;           /* 10 */
} StepExp_t;

/* Enable the TB6612 (STBY=1, IN1=1, IN2=0 = forward) and start PWM + encoder timers. Call once. */
void StepLogger_Init(void);

/* Blocking: run one experiment, then print it over UART2 under the given file name.
 * The motor is forced to 0 at the end and on any argument error. */
void StepLogger_Run(const StepExp_t *cfg, const char *file_name);

/* Convenience: static map D05..D50 (hold 2.5 s) and 3 amplitudes x 3 repeats. */
void StepLogger_RunAll(void);

#endif
