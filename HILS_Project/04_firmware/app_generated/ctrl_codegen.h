/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ctrl_codegen.h
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

#ifndef ctrl_codegen_h_
#define ctrl_codegen_h_
#ifndef ctrl_codegen_COMMON_INCLUDES_
#define ctrl_codegen_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* ctrl_codegen_COMMON_INCLUDES_ */

#include "ctrl_codegen_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T PedalLPF_states;            /* '<Root>/PedalLPF' */
  real32_T I_prev_DSTATE;              /* '<Root>/I_prev' */
  real32_T PrevY;                      /* '<Root>/PedalRateLim' */
  real32_T cPl;                        /* '<Root>/FaultMode' */
  real32_T cRg;                        /* '<Root>/FaultMode' */
  real32_T cClr;                       /* '<Root>/FaultMode' */
  uint8_T modeS;                       /* '<Root>/FaultMode' */
  boolean_T aPl;                       /* '<Root>/FaultMode' */
  boolean_T aRg;                       /* '<Root>/FaultMode' */
} DW_ctrl_codegen_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T pedal1_adc;                 /* '<Root>/pedal1_adc' */
  real32_T pedal2_adc;                 /* '<Root>/pedal2_adc' */
  real32_T speed_meas_rpm;             /* '<Root>/speed_meas_rpm' */
} ExtU_ctrl_codegen_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T duty_cmd_pct;               /* '<Root>/duty_cmd_pct' */
  real32_T speed_req_rpm;              /* '<Root>/speed_req_rpm' */
  uint16_T fault_bitmap;               /* '<Root>/fault_bitmap' */
  uint8_T mode;                        /* '<Root>/mode' */
  real32_T s1_pct;                     /* '<Root>/s1_pct' */
  real32_T p2_pct;                     /* '<Root>/p2_pct' */
  real32_T e_pct;                      /* '<Root>/e_pct' */
  real32_T p_cmd_pct;                  /* '<Root>/p_cmd_pct' */
} ExtY_ctrl_codegen_T;

/* Real-time Model Data Structure */
struct tag_RTM_ctrl_codegen_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_ctrl_codegen_T ctrl_codegen_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_ctrl_codegen_T ctrl_codegen_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_ctrl_codegen_T ctrl_codegen_Y;

/* Model entry point functions */
extern void ctrl_codegen_initialize(void);
extern void ctrl_codegen_step(void);
extern void ctrl_codegen_terminate(void);

/* Real-time Model object */
extern RT_MODEL_ctrl_codegen_T *const ctrl_codegen_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/W_single' : Eliminate redundant data type conversion
 * Block '<Root>/X_single' : Eliminate redundant data type conversion
 * Block '<Root>/Y_single' : Eliminate redundant data type conversion
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'ctrl_codegen'
 * '<S1>'   : 'ctrl_codegen/FaultMode'
 * '<S2>'   : 'ctrl_codegen/PedalCond'
 */
#endif                                 /* ctrl_codegen_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
