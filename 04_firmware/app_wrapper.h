/* app_wrapper.h - Stage 4 clean boundary between BSW/RTE and Embedded Coder ctrl_codegen */
#ifndef APP_WRAPPER_H
#define APP_WRAPPER_H
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float pedal1_adc;
    float pedal2_adc;
    float speed_meas_rpm;
} Rte_AppIn_t;

typedef struct {
    float    duty_cmd_pct;
    float    speed_req_rpm;
    uint16_t fault_bitmap;
    uint8_t  mode;
    float    s1_pct;
    float    p2_pct;
    float    e_pct;
    float    p_cmd_pct;
} Rte_AppOut_t;

void App_Init(void);
void App_Step10ms(const Rte_AppIn_t *in, Rte_AppOut_t *out);
#endif /* APP_WRAPPER_H */
