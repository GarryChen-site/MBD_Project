/* closed_loop_runner.h - Pass 2 (Stage 5/6 Core) 10 ms closed-loop hardware runner
 *
 * Pinout aligned 100% with smoketest/joystick, smoketest/encoder, and smoketest/motor:
 *   - Joystick : PA0 (ADC1_IN0 = VRx), PA1 (ADC1_IN1 = VRy) on hadc1 (84 cycles)
 *   - Encoder  : PA6/PA7 (TIM3_CH1/CH2, TI1&TI2 X4) on htim3 (N_rev = 1426 counts/rev)
 *   - Motor PWM: PB6 (TIM4_CH1, 20 kHz, PSC=0, ARR=799) on htim4
 *   - TB6612   : AIN1 = PC7 (D9), AIN2 = PA9 (D8), STBY = PA8 (D7)
 *   - UART     : PA2/PA3 (USART2, 115200 8N1) on huart2
 *
 * Output format (compatible with 04_firmware/experiments/capture_log.py):
 *     #BEGIN hil_closed_loop.csv
 *     t_ms,adcX,adcY,speed_req_rpm,speed_meas_rpm,duty_cmd_pct,fault_bitmap,mode,wcet_us
 *     0,2051,2039,0.00,0.00,0.00,0,0,4
 *     ...
 *     #END
 */
#ifndef CLOSED_LOOP_RUNNER_H
#define CLOSED_LOOP_RUNNER_H

#include <stdint.h>

#define CL_PERIOD_MS          10u
#define CL_ENCODER_CPR        1426.0f   /* Measured in smoketest/encoder/encoder_test_log.txt (50:1 N20) */
#define CL_DUTY_MAX_PCT       50.0f     /* P-005 hard safety cap (Vrated / Vbat = 3V / 6V) */

/* Initialize TB6612 pins, TIM4 PWM, TIM3 encoder, DWT cycle counter, and App_Init(). */
void ClosedLoop_Init(void);

/* Run live closed-loop control from physical joystick (ADC1 PA0/PA1) for `seconds`
 * and stream #BEGIN <file_name> ... #END over UART2. */
void ClosedLoop_RunLive(uint32_t seconds, const char *file_name);

/* Run a deterministic 6-second virtual-pedal step test on the real STM32 + real motor
 * (no hand movement required) and stream #BEGIN <file_name> ... #END over UART2. */
void ClosedLoop_RunAutoStep(const char *file_name);

#endif /* CLOSED_LOOP_RUNNER_H */
