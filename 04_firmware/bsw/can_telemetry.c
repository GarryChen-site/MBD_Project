/**
 * @file can_telemetry.c
 * @brief Stage 5 BSW — CAN Communication, E2E Supervision, Dem-Lite & UDS 0x22 Handler
 */
#include "can_telemetry.h"
#include <string.h>

#if defined(USE_HAL_DRIVER)
#include "main.h"
#ifdef HAL_CAN_MODULE_ENABLED
extern CAN_HandleTypeDef hcan1;
#endif
#endif

static CanTelemetry_State_t g_can_state;

static void bsw_can_transmit(uint32_t std_id, const uint8_t *data, uint8_t dlc)
{
    g_can_state.tx_frame_count++;
#if defined(USE_HAL_DRIVER) && defined(HAL_CAN_MODULE_ENABLED)
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U) {
        CAN_TxHeaderTypeDef hdr;
        uint32_t mailbox = 0U;
        hdr.StdId = std_id;
        hdr.ExtId = 0U;
        hdr.IDE   = CAN_ID_STD;
        hdr.RTR   = CAN_RTR_DATA;
        hdr.DLC   = dlc;
        hdr.TransmitGlobalTime = DISABLE;
        (void)HAL_CAN_AddTxMessage(&hcan1, &hdr, (uint8_t *)data, &mailbox);
    }
#else
    (void)std_id;
    (void)data;
    (void)dlc;
#endif
}

static void bsw_can_poll_rx_fifo(void)
{
#if defined(USE_HAL_DRIVER) && defined(HAL_CAN_MODULE_ENABLED)
    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U) {
        CAN_RxHeaderTypeDef rx_hdr;
        uint8_t rx_data[8];
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_hdr, rx_data) == HAL_OK) {
            CanTelemetry_OnRxFrame(rx_hdr.StdId, rx_data, (uint8_t)rx_hdr.DLC);
        } else {
            break;
        }
    }
#endif
}

void CanTelemetry_Init(void)
{
    memset(&g_can_state, 0, sizeof(g_can_state));

#if defined(USE_HAL_DRIVER) && defined(HAL_CAN_MODULE_ENABLED)
    CAN_FilterTypeDef filter;
    filter.FilterBank           = 0U;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000U;
    filter.FilterIdLow          = 0x0000U;
    filter.FilterMaskIdHigh     = 0x0000U;
    filter.FilterMaskIdLow      = 0x0000U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14U;
    (void)HAL_CAN_ConfigFilter(&hcan1, &filter);
    (void)HAL_CAN_Start(&hcan1);
#endif
}

void CanTelemetry_OnRxFrame(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    g_can_state.rx_frame_count++;

    if ((can_id == CAN_ID_PEDAL_CMD) && (dlc >= CAN_DLC_PEDAL_CMD)) {
        Com_Pedal_Cmd_t decoded;
        E2E_Status_t st = Com_Unpack_Pedal_Cmd(data, &decoded, &g_can_state.rx_e2e_pedal);
        if (st == E2E_OK) {
            g_can_state.last_pedal_cmd = decoded;
            g_can_state.can_age_ms = 0U;
        }
        return;
    }

    /* Minimal UDS Service 0x22 ReadDataByIdentifier on 0x7E0 -> 0x7E8 */
    if ((can_id == UDS_CAN_ID_REQ) && (dlc >= 4U)) {
        uint8_t pci = data[0];
        uint8_t sid = data[1];
        uint16_t did = (uint16_t)(((uint16_t)data[2] << 8U) | data[3]);
        uint8_t resp[8] = {0};

        if ((pci == 0x03U) && (sid == 0x22U)) {
            uint16_t val = 0U;
            uint8_t ok = 1U;
            if (did == UDS_DID_ACTIVE_FAULTS) {
                val = g_can_state.active_fault_bitmap;
            } else if (did == UDS_DID_STORED_FAULTS) {
                val = g_can_state.stored_fault_bitmap;
            } else if (did == UDS_DID_WCET_US) {
                val = g_can_state.max_wcet_us;
            } else if (did == UDS_DID_SW_VERSION) {
                val = 0x0100U; /* v1.0 */
            } else {
                ok = 0U;
            }

            if (ok != 0U) {
                resp[0] = 0x05U;
                resp[1] = 0x62U; /* Positive response SID (0x22 + 0x40) */
                resp[2] = data[2];
                resp[3] = data[3];
                resp[4] = (uint8_t)((val >> 8U) & 0xFFU);
                resp[5] = (uint8_t)(val & 0xFFU);
                bsw_can_transmit(UDS_CAN_ID_RESP, resp, 8U);
            } else {
                resp[0] = 0x03U;
                resp[1] = 0x7FU; /* Negative response */
                resp[2] = 0x22U;
                resp[3] = 0x31U; /* NRC 0x31: RequestOutOfRange */
                bsw_can_transmit(UDS_CAN_ID_RESP, resp, 8U);
            }
        }
    }
}

void CanTelemetry_Step10ms(uint32_t tick_10ms,
                           const Rte_AppIn_t *in,
                           const Rte_AppOut_t *out,
                           uint16_t d_mm,
                           uint8_t aeb_state,
                           uint32_t wcet_us)
{
    /* 1. Drain any received CAN frames */
    bsw_can_poll_rx_fifo();

    /* 2. Update RX timeout age (saturate at 60000 ms) */
    if (g_can_state.can_age_ms <= 59990U) {
        g_can_state.can_age_ms = (uint16_t)(g_can_state.can_age_ms + 10U);
    }

    /* 3. Update Dem-Lite fault memory and max WCET */
    g_can_state.active_fault_bitmap = out->fault_bitmap;
    g_can_state.stored_fault_bitmap |= out->fault_bitmap;
    if (wcet_us > (uint32_t)g_can_state.max_wcet_us) {
        g_can_state.max_wcet_us = (wcet_us > 0xFFFFUL) ? 0xFFFFU : (uint16_t)wcet_us;
    }

    /* 4. 20 ms Cyclic TX: 0x200 Drive_Status (every 2 ticks) */
    if ((tick_10ms % 2U) == 0U) {
        Com_Drive_Status_t drive_msg;
        drive_msg.speed_rpm     = in->speed_meas_rpm;
        drive_msg.duty_pct      = out->duty_cmd_pct;
        drive_msg.mode          = out->mode;
        drive_msg.speed_req_rpm = out->speed_req_rpm;
        Com_Pack_Drive_Status(&drive_msg, g_can_state.last_drive_buf, &g_can_state.tx_cnt_drive);
        bsw_can_transmit(CAN_ID_DRIVE_STATUS, g_can_state.last_drive_buf, CAN_DLC_DRIVE_STATUS);
    }

    /* 5. 50 ms Cyclic TX: 0x210 AEB_Status (every 5 ticks, offset 1 tick) */
    if ((tick_10ms % 5U) == 1U) {
        Com_AEB_Status_t aeb_msg;
        aeb_msg.distance_mm = (d_mm > 2000U) ? 2000U : d_mm;
        aeb_msg.aeb_state   = aeb_state;
        aeb_msg.tof_valid   = 1U;
        Com_Pack_AEB_Status(&aeb_msg, g_can_state.last_aeb_buf, &g_can_state.tx_cnt_aeb);
        bsw_can_transmit(CAN_ID_AEB_STATUS, g_can_state.last_aeb_buf, CAN_DLC_AEB_STATUS);
    }

    /* 6. 100 ms Cyclic TX: 0x201 Diag_Status (every 10 ticks, offset 3 ticks) */
    if ((tick_10ms % 10U) == 3U) {
        Com_Diag_Status_t diag_msg;
        diag_msg.fault_bitmap = g_can_state.active_fault_bitmap;
        diag_msg.rx_err_count = (g_can_state.rx_e2e_pedal.err_total > 255U)
                                    ? 255U
                                    : (uint8_t)g_can_state.rx_e2e_pedal.err_total;
        Com_Pack_Diag_Status(&diag_msg, g_can_state.last_diag_buf, &g_can_state.tx_cnt_diag);
        bsw_can_transmit(CAN_ID_DIAG_STATUS, g_can_state.last_diag_buf, CAN_DLC_DIAG_STATUS);
    }

    /* 7. 500 ms Cyclic TX: 0x123 ECU_Heartbeat (every 50 ticks, offset 7 ticks) */
    if ((tick_10ms % 50U) == 7U) {
        Com_ECU_Heartbeat_t hb_msg;
        uint8_t hb_buf[CAN_DLC_ECU_HEARTBEAT];
        hb_msg.hb_counter    = g_can_state.hb_counter++;
        hb_msg.loopback_flag = 0U;
        Com_Pack_ECU_Heartbeat(&hb_msg, hb_buf);
        bsw_can_transmit(CAN_ID_ECU_HEARTBEAT, hb_buf, CAN_DLC_ECU_HEARTBEAT);
    }
}

const CanTelemetry_State_t *CanTelemetry_GetState(void)
{
    return &g_can_state;
}
