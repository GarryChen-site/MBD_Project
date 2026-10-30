/**
 * @file can_telemetry.h
 * @brief Stage 5 BSW — CAN Communication, E2E Supervision, Dem-Lite & UDS 0x22 Handler
 *
 * Bridges the generated Simulink application (`Rte_AppIn_t` / `Rte_AppOut_t`)
 * with the auto-generated DBC pack/unpack layer (`com_can_signals.h`) and STM32 `bxCAN1`.
 *
 * Works seamlessly in two modes:
 *   1. Physical STM32 `bxCAN1` (when `HAL_CAN_MODULE_ENABLED` is defined in STM32CubeIDE)
 *   2. Software loopback / shadow mode (when `CAN1` is not yet enabled in `.ioc`)
 */
#ifndef CAN_TELEMETRY_H
#define CAN_TELEMETRY_H

#include <stdint.h>
#include "app_wrapper.h"
#include "com_can_signals.h"

#ifdef __cplusplus
extern "C" {
#endif

/* UDS-Lite Diagnostic Identifiers (Service 0x22 ReadDataByIdentifier) */
#define UDS_CAN_ID_REQ          (0x7E0U)
#define UDS_CAN_ID_RESP         (0x7E8U)
#define UDS_DID_ACTIVE_FAULTS   (0xF100U)
#define UDS_DID_STORED_FAULTS   (0xF101U)
#define UDS_DID_WCET_US         (0xF102U)
#define UDS_DID_SW_VERSION      (0xF103U)

typedef struct {
    /* TX Alive Counters (4-bit 0..15) */
    uint8_t tx_cnt_drive;
    uint8_t tx_cnt_diag;
    uint8_t tx_cnt_aeb;
    uint32_t hb_counter;

    /* RX E2E State & Timeout Supervision for 0x100 Pedal_Cmd */
    E2E_State_t rx_e2e_pedal;
    Com_Pedal_Cmd_t last_pedal_cmd;
    uint16_t can_age_ms;

    /* Dem-Lite Fault Memory */
    uint16_t active_fault_bitmap;
    uint16_t stored_fault_bitmap;
    uint16_t max_wcet_us;

    /* Frame Counters */
    uint32_t tx_frame_count;
    uint32_t rx_frame_count;

    /* Last Packed Payloads (for inspection / unit testing) */
    uint8_t last_drive_buf[CAN_DLC_DRIVE_STATUS];
    uint8_t last_diag_buf[CAN_DLC_DIAG_STATUS];
    uint8_t last_aeb_buf[CAN_DLC_AEB_STATUS];
} CanTelemetry_State_t;

/**
 * @brief Initialize BSW CAN telemetry state and configure STM32 CAN1 filter if enabled.
 */
void CanTelemetry_Init(void);

/**
 * @brief Execute 10 ms cyclic CAN TX schedule (`0x200` @ 20 ms, `0x210` @ 50 ms,
 *        `0x201` @ 100 ms, `0x123` @ 500 ms) and update RX timeout age (`can_age_ms`).
 *
 * @param tick_10ms  Zero-based 10 ms step counter (0, 1, 2, ...)
 * @param in         Current application inputs (`pedal1_adc`, `pedal2_adc`, `speed_meas_rpm`)
 * @param out        Current application outputs (`duty_cmd_pct`, `speed_req_rpm`, `fault_bitmap`, `mode`)
 * @param d_mm       Target distance in mm (for `0x210 AEB_Status`)
 * @param aeb_state  Current AEB state (`0=NORMAL, 1=WARNING, 2=BRAKING`)
 * @param wcet_us    Measured execution time of `App_Step10ms` (microseconds)
 */
void CanTelemetry_Step10ms(uint32_t tick_10ms,
                           const Rte_AppIn_t *in,
                           const Rte_AppOut_t *out,
                           uint16_t d_mm,
                           uint8_t aeb_state,
                           uint32_t wcet_us);

/**
 * @brief Process an incoming CAN frame (`0x100 Pedal_Cmd` or `0x7E0 Diag_Req`).
 */
void CanTelemetry_OnRxFrame(uint32_t can_id, const uint8_t *data, uint8_t dlc);

/**
 * @brief Read the current BSW telemetry state (for diagnostics or unit tests).
 */
const CanTelemetry_State_t *CanTelemetry_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_TELEMETRY_H */
