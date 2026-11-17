/* app_wrapper.c - Stage 4/6 wrapper calling generated ctrl_codegen_step()
 * plus AEB 50 ms state machine (matching 03_simulink/stage3/aeb_ref.m).
 */
#include "app_wrapper.h"
#include "app_generated/ctrl_codegen.h"
#include <string.h>

static App_AebCtx_t g_aeb;

void App_AebInit(void) {
  memset(&g_aeb, 0, sizeof(g_aeb));
  g_aeb.state = (uint8_t)AEB_STATE_NORMAL;
}

void App_Init(void) {
  memset(&ctrl_codegen_DW, 0, sizeof(ctrl_codegen_DW));
  memset(&ctrl_codegen_U, 0, sizeof(ctrl_codegen_U));
  memset(&ctrl_codegen_Y, 0, sizeof(ctrl_codegen_Y));
  ctrl_codegen_initialize();
  App_AebInit();
}

void App_Step10ms(const Rte_AppIn_t *in, Rte_AppOut_t *out) {
  ctrl_codegen_U.pedal1_adc = in->pedal1_adc;
  ctrl_codegen_U.pedal2_adc = in->pedal2_adc;
  ctrl_codegen_U.speed_meas_rpm = in->speed_meas_rpm;
  ctrl_codegen_step();
  out->duty_cmd_pct = ctrl_codegen_Y.duty_cmd_pct;
  out->speed_req_rpm = ctrl_codegen_Y.speed_req_rpm;
  out->fault_bitmap = ctrl_codegen_Y.fault_bitmap;
  out->mode = ctrl_codegen_Y.mode;
  out->s1_pct = ctrl_codegen_Y.s1_pct;
  out->p2_pct = ctrl_codegen_Y.p2_pct;
  out->e_pct = ctrl_codegen_Y.e_pct;
  out->p_cmd_pct = ctrl_codegen_Y.p_cmd_pct;
}

/* Exact C translation of 03_simulink/stage3/aeb_ref.m */
uint8_t App_AebStep50ms(uint16_t d_mm, bool valid, float speed_meas_rpm,
                        bool *tof_invalid_out) {
  if (valid) {
    g_aeb.cInv = 0u;
  } else {
    if (g_aeb.cInv < 0xFFFFu) {
      g_aeb.cInv++;
    }
  }
  g_aeb.tof_invalid = (g_aeb.cInv >= AEB_INV_TICKS);

  if (valid) {
    g_aeb.cW = (d_mm < AEB_WARN_DIST_MM) ? (uint16_t)(g_aeb.cW + 1u) : 0u;
    g_aeb.cB = (d_mm < AEB_BRAKE_DIST_MM) ? (uint16_t)(g_aeb.cB + 1u) : 0u;
    g_aeb.cR = (d_mm > (uint16_t)(AEB_WARN_DIST_MM + AEB_HYST_DIST_MM))
                   ? (uint16_t)(g_aeb.cR + 1u)
                   : 0u;

    switch (g_aeb.state) {
    case AEB_STATE_NORMAL:
      if (g_aeb.cW >= AEB_DEB_TICKS) {
        g_aeb.state = (uint8_t)AEB_STATE_WARNING;
      }
      break;

    case AEB_STATE_WARNING:
      if (g_aeb.cB >= AEB_DEB_TICKS) {
        g_aeb.state = (uint8_t)AEB_STATE_BRAKING;
      } else if (g_aeb.cR >= AEB_REL_TICKS) {
        g_aeb.state = (uint8_t)AEB_STATE_NORMAL;
      }
      break;

    case AEB_STATE_BRAKING:
      if ((g_aeb.cR >= AEB_REL_TICKS) && (speed_meas_rpm < AEB_WSTOP_RPM)) {
        g_aeb.state = (uint8_t)AEB_STATE_NORMAL;
      }
      break;

    default:
      g_aeb.state = (uint8_t)AEB_STATE_NORMAL;
      break;
    }
  }

  if (tof_invalid_out != NULL) {
    *tof_invalid_out = g_aeb.tof_invalid;
  }
  return g_aeb.state;
}

void App_ApplyAebOverride(Rte_AppOut_t *out, float *servo_angle_deg) {
  if (!out)
    return;

  if (g_aeb.tof_invalid) {
    out->fault_bitmap |= (uint16_t)FAULT_BIT_TOF_INVALID;
  }

  float angle = AEB_SERVO_RELEASE_DEG;
  if (g_aeb.state == (uint8_t)AEB_STATE_WARNING) {
    angle = AEB_SERVO_WARN_DEG;
  } else if (g_aeb.state == (uint8_t)AEB_STATE_BRAKING) {
    /* Emergency Brake Override: cut motor PWM to 0%, set SAFE_STOP (2), actuate
     * SG90 to 90 deg */
    out->duty_cmd_pct = 0.0f;
    out->speed_req_rpm = 0.0f;
    out->mode = 2u; /* SAFE_STOP */
    angle = AEB_SERVO_BRAKE_DEG;
  }

  if (servo_angle_deg != NULL) {
    *servo_angle_deg = angle;
  }
}

const App_AebCtx_t *App_GetAebContext(void) { return &g_aeb; }