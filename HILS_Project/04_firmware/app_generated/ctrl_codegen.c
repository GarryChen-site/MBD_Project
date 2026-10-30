/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ctrl_codegen.c
 *
 * Code generated for Simulink model 'ctrl_codegen'.
 *
 * Model version                  : 1.1
 * Simulink Coder version         : 26.1 (R2026a) 20-Nov-2025
 * C/C++ source code generated on : Sat Oct 24 17:02:36 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "ctrl_codegen.h"
#include <math.h>
#include "rtwtypes.h"

/* Block states (default storage) */
DW_ctrl_codegen_T ctrl_codegen_DW;

/* External inputs (root inport signals with default storage) */
ExtU_ctrl_codegen_T ctrl_codegen_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_ctrl_codegen_T ctrl_codegen_Y;

/* Real-time model */
static RT_MODEL_ctrl_codegen_T ctrl_codegen_M_;
RT_MODEL_ctrl_codegen_T *const ctrl_codegen_M = &ctrl_codegen_M_;

/* Model step function */
void ctrl_codegen_step(void)
{
  real32_T rtb_SpeedErr;
  real32_T rtb_U_raw;
  real32_T tmp;

  /* MATLAB Function: '<Root>/PedalCond' incorporates:
   *  Constant: '<Root>/cal'
   *  Inport: '<Root>/pedal1_adc'
   *  Inport: '<Root>/pedal2_adc'
   */
  if (ctrl_codegen_U.pedal1_adc - 2044.0F <= 16.0F) {
    tmp = 0.0F;
  } else {
    tmp = fminf(fmaxf(((ctrl_codegen_U.pedal1_adc - 2044.0F) - 16.0F) / 1915.0F,
                      0.0F), 1.0F);
  }

  ctrl_codegen_Y.s1_pct = 100.0F * tmp;
  if (ctrl_codegen_U.pedal2_adc - 2070.0F <= 22.0F) {
    tmp = 0.0F;
  } else {
    tmp = fminf(fmaxf(((ctrl_codegen_U.pedal2_adc - 2070.0F) - 22.0F) / 1883.0F,
                      0.0F), 1.0F);
  }

  ctrl_codegen_Y.p2_pct = 100.0F - (1.0F - tmp) * 100.0F;
  ctrl_codegen_Y.e_pct = fabsf(ctrl_codegen_Y.s1_pct - ctrl_codegen_Y.p2_pct);

  /* MATLAB Function: '<Root>/FaultMode' incorporates:
   *  Constant: '<Root>/cal'
   *  Constant: '<Root>/fp'
   *  Inport: '<Root>/pedal1_adc'
   *  Inport: '<Root>/pedal2_adc'
   *  MATLAB Function: '<Root>/PedalCond'
   */
  if (ctrl_codegen_Y.e_pct > 15.7F) {
    ctrl_codegen_DW.cPl = fminf(ctrl_codegen_DW.cPl + 1.0F, 5.0F);
  } else {
    ctrl_codegen_DW.cPl = fmaxf(ctrl_codegen_DW.cPl - 1.0F, 0.0F);
  }

  ctrl_codegen_DW.aPl = ((ctrl_codegen_DW.cPl >= 5.0F) || ctrl_codegen_DW.aPl);
  ctrl_codegen_DW.aPl = ((ctrl_codegen_DW.cPl > 0.0F) && ctrl_codegen_DW.aPl);
  if ((ctrl_codegen_U.pedal1_adc < 100.0F) || ((ctrl_codegen_U.pedal1_adc >
        3995.0F) || ((ctrl_codegen_U.pedal2_adc < 100.0F) ||
                     (ctrl_codegen_U.pedal2_adc > 3995.0F)))) {
    ctrl_codegen_DW.cRg = fminf(ctrl_codegen_DW.cRg + 1.0F, 5.0F);
  } else {
    ctrl_codegen_DW.cRg = fmaxf(ctrl_codegen_DW.cRg - 1.0F, 0.0F);
  }

  ctrl_codegen_DW.aRg = ((ctrl_codegen_DW.cRg >= 5.0F) || ctrl_codegen_DW.aRg);
  ctrl_codegen_DW.aRg = ((ctrl_codegen_DW.cRg > 0.0F) && ctrl_codegen_DW.aRg);
  if (ctrl_codegen_DW.aPl || ctrl_codegen_DW.aRg) {
    ctrl_codegen_DW.cClr = 0.0F;
    ctrl_codegen_DW.modeS = 1U;
  } else if (ctrl_codegen_DW.modeS == 1) {
    ctrl_codegen_DW.cClr++;
    if (ctrl_codegen_DW.cClr >= 100.0F) {
      ctrl_codegen_DW.modeS = 0U;
    }
  }

  ctrl_codegen_Y.fault_bitmap = 0U;
  if (ctrl_codegen_DW.aPl) {
    ctrl_codegen_Y.fault_bitmap = 1U;
  }

  if (ctrl_codegen_DW.aRg) {
    ctrl_codegen_Y.fault_bitmap = (uint16_T)(ctrl_codegen_Y.fault_bitmap | 2);
  }

  /* Outport: '<Root>/mode' incorporates:
   *  MATLAB Function: '<Root>/FaultMode'
   */
  ctrl_codegen_Y.mode = ctrl_codegen_DW.modeS;

  /* MinMax: '<Root>/PCmd' incorporates:
   *  MATLAB Function: '<Root>/PedalCond'
   */
  ctrl_codegen_Y.p_cmd_pct = fminf(ctrl_codegen_Y.s1_pct, ctrl_codegen_Y.p2_pct);

  /* DiscreteFilter: '<Root>/PedalLPF' */
  ctrl_codegen_DW.PedalLPF_states = ctrl_codegen_Y.p_cmd_pct - -0.8187308F *
    ctrl_codegen_DW.PedalLPF_states;
  rtb_U_raw = 0.18126924F * ctrl_codegen_DW.PedalLPF_states;

  /* RateLimiter: '<Root>/PedalRateLim' incorporates:
   *  DiscreteFilter: '<Root>/PedalLPF'
   */
  rtb_SpeedErr = rtb_U_raw - ctrl_codegen_DW.PrevY;
  if (rtb_SpeedErr > 4.0F) {
    rtb_U_raw = ctrl_codegen_DW.PrevY + 4.0F;
  } else if (rtb_SpeedErr < -6.0F) {
    rtb_U_raw = ctrl_codegen_DW.PrevY - 6.0F;
  }

  ctrl_codegen_DW.PrevY = rtb_U_raw;

  /* End of RateLimiter: '<Root>/PedalRateLim' */

  /* MATLAB Function: '<Root>/FaultMode' incorporates:
   *  Constant: '<Root>/fp'
   */
  if (ctrl_codegen_DW.modeS == 0) {
    tmp = 147.75397F;
  } else {
    tmp = 30.0F;
  }

  /* MinMax: '<Root>/ReqLimit' incorporates:
   *  Gain: '<Root>/ReqScale'
   *  MATLAB Function: '<Root>/FaultMode'
   */
  ctrl_codegen_Y.speed_req_rpm = fminf(1.4775397F * rtb_U_raw, tmp);

  /* Sum: '<Root>/SpeedErr' incorporates:
   *  Inport: '<Root>/speed_meas_rpm'
   */
  rtb_SpeedErr = ctrl_codegen_Y.speed_req_rpm - ctrl_codegen_U.speed_meas_rpm;

  /* Switch: '<Root>/FF_switch' incorporates:
   *  Constant: '<Root>/FF_dz'
   *  Constant: '<Root>/FF_zero'
   *  Sum: '<Root>/FF_sum'
   */
  if (ctrl_codegen_Y.speed_req_rpm > 0.0F) {
    tmp = 1.9340934F;
  } else {
    tmp = 0.0F;
  }

  /* Sum: '<Root>/U_raw' incorporates:
   *  Gain: '<Root>/KiTs_gain'
   *  Gain: '<Root>/Kp_gain'
   *  Switch: '<Root>/FF_switch'
   *  UnitDelay: '<Root>/I_prev'
   */
  rtb_U_raw = ((0.14782012F * rtb_SpeedErr + tmp) +
               ctrl_codegen_DW.I_prev_DSTATE) + 0.054218404F * rtb_SpeedErr;

  /* Saturate: '<Root>/Sat' */
  if (rtb_U_raw > 50.0F) {
    /* Saturate: '<Root>/Sat' */
    ctrl_codegen_Y.duty_cmd_pct = 50.0F;
  } else if (rtb_U_raw < 0.0F) {
    /* Saturate: '<Root>/Sat' */
    ctrl_codegen_Y.duty_cmd_pct = 0.0F;
  } else {
    /* Saturate: '<Root>/Sat' */
    ctrl_codegen_Y.duty_cmd_pct = rtb_U_raw;
  }

  /* End of Saturate: '<Root>/Sat' */

  /* Sum: '<Root>/I_acc' incorporates:
   *  Gain: '<Root>/Kb_gain'
   *  Gain: '<Root>/Ki_gain'
   *  Gain: '<Root>/Ts_gain'
   *  Sum: '<Root>/I_rate'
   *  Sum: '<Root>/SatDiff'
   *  UnitDelay: '<Root>/I_prev'
   */
  ctrl_codegen_DW.I_prev_DSTATE += ((ctrl_codegen_Y.duty_cmd_pct - rtb_U_raw) *
    36.67864F + 5.4218407F * rtb_SpeedErr) * 0.01F;
}

/* Model initialize function */
void ctrl_codegen_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void ctrl_codegen_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
