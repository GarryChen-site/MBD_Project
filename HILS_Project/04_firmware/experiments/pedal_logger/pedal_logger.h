/* pedal_logger.h - Stage 2 joystick logger: X on ADC1 (PA0, IN0), Y on ADC2 (PA1, IN1), 100 Hz.
 *
 * Output (matches the MATLAB joystick files):
 *     #BEGIN joy_rest.csv
 *     t_ms,adcX,adcY
 *     ...
 *     #END
 * Streams each line immediately (no RAM buffer): 2 kB/s is far below the UART limit.
 */
#ifndef PEDAL_LOGGER_H
#define PEDAL_LOGGER_H
#include <stdint.h>

/* Mcal-style reader used later by the application too (signature fixed by PRODUCT_SPEC section 5). */
void Mcal_Adc_ReadPedals(uint16_t *x_counts, uint16_t *y_counts);

/* Log for `seconds` at 100 Hz under the given file name. */
void PedalLogger_Run(const char *file_name, uint32_t seconds);

/* The four sessions Stage 2 needs, with UART prompts and a 5 s countdown before each.
 *   joy_rest.csv        10 s  hands off
 *   joy_diag_full.csv   40 s  10 smooth FULL diagonal pushes (corner of the gate)
 *   joy_diag_valid.csv 180 s  50 diagonal pushes, mixed peak (40-100 %) and speed (slow/normal/fast)
 *   joy_xonly.csv       40 s  10 pushes along X only to ~60-70 %, Y stays at rest
 */
void PedalLogger_RunSessions(void);
#endif
