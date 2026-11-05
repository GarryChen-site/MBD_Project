/* app_wrapper.c - Stage 4 wrapper calling generated ctrl_codegen_step() */
#include <string.h>
#include "app_wrapper.h"
#include "app_generated/ctrl_codegen.h"

void App_Init(void) {
    memset(&ctrl_codegen_DW, 0, sizeof(ctrl_codegen_DW));
    memset(&ctrl_codegen_U,  0, sizeof(ctrl_codegen_U));
    memset(&ctrl_codegen_Y,  0, sizeof(ctrl_codegen_Y));
    ctrl_codegen_initialize();
}

void App_Step10ms(const Rte_AppIn_t *in, Rte_AppOut_t *out) {
    ctrl_codegen_U.pedal1_adc     = in->pedal1_adc;
    ctrl_codegen_U.pedal2_adc     = in->pedal2_adc;
    ctrl_codegen_U.speed_meas_rpm = in->speed_meas_rpm;
    ctrl_codegen_step();
    out->duty_cmd_pct  = ctrl_codegen_Y.duty_cmd_pct;
    out->speed_req_rpm = ctrl_codegen_Y.speed_req_rpm;
    out->fault_bitmap  = ctrl_codegen_Y.fault_bitmap;
    out->mode          = ctrl_codegen_Y.mode;
    out->s1_pct        = ctrl_codegen_Y.s1_pct;
    out->p2_pct        = ctrl_codegen_Y.p2_pct;
    out->e_pct         = ctrl_codegen_Y.e_pct;
    out->p_cmd_pct     = ctrl_codegen_Y.p_cmd_pct;
}
