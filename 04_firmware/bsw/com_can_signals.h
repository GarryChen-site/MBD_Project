/**
 * @file com_can_signals.h
 * @brief Auto-generated CAN signal pack/unpack & E2E protection from `05_can/vehicle.dbc`.
 * @note  DO NOT EDIT MANUALLY — regenerate via `python3 05_can/scripts/dbc_codegen.py`.
 */
#ifndef COM_CAN_SIGNALS_H
#define COM_CAN_SIGNALS_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/* 1. CAN Message IDs & Metadata                                             */
/* ========================================================================= */
#define CAN_ID_PEDAL_CMD          (0x100U) /* 256 */
#define CAN_DLC_PEDAL_CMD         (4U)
#define CAN_CYCLE_PEDAL_CMD       (20U)
#define CAN_ID_ECU_HEARTBEAT      (0x123U) /* 291 */
#define CAN_DLC_ECU_HEARTBEAT     (5U)
#define CAN_CYCLE_ECU_HEARTBEAT   (500U)
#define CAN_ID_DRIVE_STATUS       (0x200U) /* 512 */
#define CAN_DLC_DRIVE_STATUS      (7U)
#define CAN_CYCLE_DRIVE_STATUS    (20U)
#define CAN_ID_DIAG_STATUS        (0x201U) /* 513 */
#define CAN_DLC_DIAG_STATUS       (5U)
#define CAN_CYCLE_DIAG_STATUS     (100U)
#define CAN_ID_AEB_STATUS         (0x210U) /* 528 */
#define CAN_DLC_AEB_STATUS        (5U)
#define CAN_CYCLE_AEB_STATUS      (50U)

typedef enum {
    COM_MSG_PEDAL_CMD,
    COM_MSG_ECU_HEARTBEAT,
    COM_MSG_DRIVE_STATUS,
    COM_MSG_DIAG_STATUS,
    COM_MSG_AEB_STATUS,
    COM_MSG_COUNT
} Com_MsgId_t;

typedef struct {
    uint16_t can_id;
    uint8_t  dlc;
    uint16_t cycle_ms;
    uint8_t  is_tx;
    uint8_t  e2e_data_id;
    int8_t   crc_byte;
    int8_t   cnt_start_bit;
} Com_MsgCfg_t;

extern const Com_MsgCfg_t Com_MsgCfg[COM_MSG_COUNT];

/* ========================================================================= */
/* 2. E2E Protection (CRC-8 SAE J1850 + 4-bit Alive Counter)                 */
/* ========================================================================= */
typedef enum {
    E2E_OK = 0,
    E2E_ERR_CRC,
    E2E_ERR_REPEAT,
    E2E_ERR_SEQUENCE,
    E2E_ERR_NO_NEW
} E2E_Status_t;

typedef struct {
    uint8_t  last_cnt;
    uint8_t  initialised;
    uint16_t lost;
    uint16_t err_total;
} E2E_State_t;

uint8_t      E2E_Crc8_Raw(const uint8_t *data, uint8_t len);
uint8_t      E2E_Crc8(const uint8_t *data, uint8_t len, uint8_t data_id);
void         E2E_Protect(uint8_t *msg, uint8_t dlc, uint8_t crc_byte_idx,
                         uint8_t cnt_start_bit, uint8_t data_id, uint8_t *counter);
E2E_Status_t E2E_Check(const uint8_t *msg, uint8_t dlc, uint8_t crc_byte_idx,
                       uint8_t cnt_start_bit, uint8_t data_id, E2E_State_t *st);

/* ========================================================================= */
/* 3. Low-Level Intel (@1+) Bit Field Pack / Unpack                          */
/* ========================================================================= */
void     Com_PackBits(uint8_t *buf, uint8_t start_bit, uint8_t len, uint32_t raw);
uint32_t Com_UnpackBits(const uint8_t *buf, uint8_t start_bit, uint8_t len);
int32_t  Com_SignExtend(uint32_t raw, uint8_t len);

/* ========================================================================= */
/* 4. Typed Message Payloads & Pack/Unpack API                               */
/* ========================================================================= */
typedef struct {
    float      pedal_pct; /* [0.0..100.0] % */
} Com_Pedal_Cmd_t;

void         Com_Pack_Pedal_Cmd(const Com_Pedal_Cmd_t *in, uint8_t *out_buf, uint8_t *alive_cnt);
E2E_Status_t Com_Unpack_Pedal_Cmd(const uint8_t *in_buf, Com_Pedal_Cmd_t *out, E2E_State_t *e2e_state);

typedef struct {
    uint32_t   hb_counter; /* [0.0..4294967295.0]  */
    uint8_t    loopback_flag; /* [0.0..255.0]  */
} Com_ECU_Heartbeat_t;

void         Com_Pack_ECU_Heartbeat(const Com_ECU_Heartbeat_t *in, uint8_t *out_buf);
void         Com_Unpack_ECU_Heartbeat(const uint8_t *in_buf, Com_ECU_Heartbeat_t *out);

typedef struct {
    float      speed_rpm; /* [0.0..300.0] rpm */
    float      duty_pct; /* [0.0..100.0] % */
    uint8_t    mode; /* [0.0..3.0]  */
    float      speed_req_rpm; /* [0.0..300.0] rpm */
} Com_Drive_Status_t;

void         Com_Pack_Drive_Status(const Com_Drive_Status_t *in, uint8_t *out_buf, uint8_t *alive_cnt);
E2E_Status_t Com_Unpack_Drive_Status(const uint8_t *in_buf, Com_Drive_Status_t *out, E2E_State_t *e2e_state);

typedef struct {
    uint16_t   fault_bitmap; /* [0.0..65535.0]  */
    uint8_t    rx_err_count; /* [0.0..255.0]  */
} Com_Diag_Status_t;

void         Com_Pack_Diag_Status(const Com_Diag_Status_t *in, uint8_t *out_buf, uint8_t *alive_cnt);
E2E_Status_t Com_Unpack_Diag_Status(const uint8_t *in_buf, Com_Diag_Status_t *out, E2E_State_t *e2e_state);

typedef struct {
    uint16_t   distance_mm; /* [0.0..2000.0] mm */
    uint8_t    aeb_state; /* [0.0..2.0]  */
    uint8_t    tof_valid; /* [0.0..1.0]  */
} Com_AEB_Status_t;

void         Com_Pack_AEB_Status(const Com_AEB_Status_t *in, uint8_t *out_buf, uint8_t *alive_cnt);
E2E_Status_t Com_Unpack_AEB_Status(const uint8_t *in_buf, Com_AEB_Status_t *out, E2E_State_t *e2e_state);

#ifdef __cplusplus
}
#endif

#endif /* COM_CAN_SIGNALS_H */
