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

/* AEB states matching 03_simulink/stage3/aeb_ref.m & 05_can/vehicle.dbc */
typedef enum {
    AEB_STATE_NORMAL  = 0u,
    AEB_STATE_WARNING = 1u,
    AEB_STATE_BRAKING = 2u
} App_AebState_t;

/* Fault bit for VL53L0X ToF invalid/out-of-range timeout (Tinv >= 4 ticks = 200 ms) */
#define FAULT_BIT_TOF_INVALID  (1u << 3)  /* 0x0008 */

/* Default Stage 3 AEB parameters (matching run_stage3_tests.m line 205) */
#define AEB_WARN_DIST_MM       300u       /* P-016: warning distance threshold [mm] */
#define AEB_BRAKE_DIST_MM      150u       /* P-017: emergency brake threshold [mm] */
#define AEB_HYST_DIST_MM        50u       /* release hysteresis above dw (300+50 = 350 mm) */
#define AEB_DEB_TICKS            3u       /* consecutive 50 ms ticks to enter WARNING/BRAKING */
#define AEB_REL_TICKS            5u       /* consecutive 50 ms ticks > 350 mm to release */
#define AEB_INV_TICKS            4u       /* consecutive invalid 50 ms ticks to set F_TOF_INVALID */
#define AEB_WSTOP_RPM          5.0f       /* max motor speed [rpm] allowed to release BRAKING */

/* SG90 Servo Brake Angles [deg] */
#define AEB_SERVO_RELEASE_DEG   0.0f      /* 500 us pulse  - brake released */
#define AEB_SERVO_WARN_DEG     30.0f      /* 833 us pulse  - pre-fill warning */
#define AEB_SERVO_BRAKE_DEG    90.0f      /* 1500 us pulse - full mechanical brake */

typedef struct {
    uint8_t  state;          /* App_AebState_t: 0=NORMAL, 1=WARNING, 2=BRAKING */
    bool     tof_invalid;    /* true when consecutive invalid samples >= AEB_INV_TICKS */
    uint16_t cW;             /* consecutive valid samples with d < dw */
    uint16_t cB;             /* consecutive valid samples with d < db */
    uint16_t cR;             /* consecutive valid samples with d > dw + dh */
    uint16_t cInv;           /* consecutive invalid samples */
} App_AebCtx_t;

void App_Init(void);
void App_Step10ms(const Rte_AppIn_t *in, Rte_AppOut_t *out);

void App_AebInit(void);
uint8_t App_AebStep50ms(uint16_t d_mm, bool valid, float speed_meas_rpm, bool *tof_invalid_out);
void App_ApplyAebOverride(Rte_AppOut_t *out, float *servo_angle_deg);
const App_AebCtx_t *App_GetAebContext(void);

#endif /* APP_WRAPPER_H */
