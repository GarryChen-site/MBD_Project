/**
 * @file com_can_signals.c
 * @brief Auto-generated CAN signal pack/unpack & E2E protection from `05_can/vehicle.dbc`.
 */
#include "com_can_signals.h"
#include <string.h>

const Com_MsgCfg_t Com_MsgCfg[COM_MSG_COUNT] = {
    { 0x100U, 4U, 20U, 0U, 0x00U, 3, 16 }, /* Pedal_Cmd */
    { 0x123U, 5U, 500U, 1U, 0x23U, -1, -1 }, /* ECU_Heartbeat */
    { 0x200U, 7U, 20U, 1U, 0x00U, 6, 28 }, /* Drive_Status */
    { 0x201U, 5U, 100U, 1U, 0x01U, 4, 24 }, /* Diag_Status */
    { 0x210U, 5U, 50U, 1U, 0x10U, 4, 24 }, /* AEB_Status */
};

static uint32_t clamp_and_quantize_u32(float phys, float factor, float offset, float min_v, float max_v, uint8_t len)
{
    float clamped = phys;
    if (clamped < min_v) { clamped = min_v; }
    if (clamped > max_v) { clamped = max_v; }
    float scaled = (clamped - offset) / factor;
    uint32_t raw = (scaled >= 0.0F) ? (uint32_t)(scaled + 0.5F) : 0U;
    uint32_t max_raw = (len >= 32U) ? 0xFFFFFFFFUL : ((1UL << len) - 1UL);
    if (raw > max_raw) { raw = max_raw; }
    return raw;
}

void Com_PackBits(uint8_t *buf, uint8_t start_bit, uint8_t len, uint32_t raw)
{
    for (uint8_t i = 0U; i < len; i++) {
        uint8_t bit_pos = (uint8_t)(start_bit + i);
        uint8_t byte_idx = (uint8_t)(bit_pos >> 3U);
        uint8_t bit_idx  = (uint8_t)(bit_pos & 0x07U);
        if (((raw >> i) & 1UL) != 0UL) {
            buf[byte_idx] |= (uint8_t)(1U << bit_idx);
        } else {
            buf[byte_idx] &= (uint8_t)~(1U << bit_idx);
        }
    }
}

uint32_t Com_UnpackBits(const uint8_t *buf, uint8_t start_bit, uint8_t len)
{
    uint32_t raw = 0UL;
    for (uint8_t i = 0U; i < len; i++) {
        uint8_t bit_pos = (uint8_t)(start_bit + i);
        uint8_t byte_idx = (uint8_t)(bit_pos >> 3U);
        uint8_t bit_idx  = (uint8_t)(bit_pos & 0x07U);
        if (((buf[byte_idx] >> bit_idx) & 1U) != 0U) {
            raw |= (1UL << i);
        }
    }
    return raw;
}

int32_t Com_SignExtend(uint32_t raw, uint8_t len)
{
    if ((len == 0U) || (len >= 32U)) {
        return (int32_t)raw;
    }
    uint32_t sign_bit = 1UL << (len - 1U);
    if ((raw & sign_bit) != 0UL) {
        uint32_t mask = ~((1UL << len) - 1UL);
        raw |= mask;
    }
    return (int32_t)raw;
}

uint8_t E2E_Crc8_Raw(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFFU;
    for (uint8_t i = 0U; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0U; b < 8U; b++) {
            crc = ((crc & 0x80U) != 0U) ? (uint8_t)((crc << 1U) ^ 0x1DU) : (uint8_t)(crc << 1U);
        }
    }
    return (uint8_t)(crc ^ 0xFFU);
}

uint8_t E2E_Crc8(const uint8_t *data, uint8_t len, uint8_t data_id)
{
    uint8_t crc = 0xFFU;
    for (uint8_t i = 0U; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0U; b < 8U; b++) {
            crc = ((crc & 0x80U) != 0U) ? (uint8_t)((crc << 1U) ^ 0x1DU) : (uint8_t)(crc << 1U);
        }
    }
    crc ^= data_id;
    for (uint8_t b = 0U; b < 8U; b++) {
        crc = ((crc & 0x80U) != 0U) ? (uint8_t)((crc << 1U) ^ 0x1DU) : (uint8_t)(crc << 1U);
    }
    return (uint8_t)(crc ^ 0xFFU);
}

void E2E_Protect(uint8_t *msg, uint8_t dlc, uint8_t crc_byte_idx,
                 uint8_t cnt_start_bit, uint8_t data_id, uint8_t *counter)
{
    (void)dlc;
    uint8_t cnt = (uint8_t)((*counter) & 0x0FU);
    Com_PackBits(msg, cnt_start_bit, 4U, (uint32_t)cnt);
    msg[crc_byte_idx] = E2E_Crc8(msg, crc_byte_idx, data_id);
    *counter = (uint8_t)((cnt + 1U) & 0x0FU);
}

E2E_Status_t E2E_Check(const uint8_t *msg, uint8_t dlc, uint8_t crc_byte_idx,
                       uint8_t cnt_start_bit, uint8_t data_id, E2E_State_t *st)
{
    (void)dlc;
    uint8_t expected_crc = E2E_Crc8(msg, crc_byte_idx, data_id);
    if (msg[crc_byte_idx] != expected_crc) {
        st->err_total++;
        return E2E_ERR_CRC;
    }
    uint8_t cnt = (uint8_t)(Com_UnpackBits(msg, cnt_start_bit, 4U) & 0x0FU);
    if (st->initialised == 0U) {
        st->initialised = 1U;
        st->last_cnt = cnt;
        return E2E_OK;
    }
    uint8_t delta = (uint8_t)((cnt - st->last_cnt) & 0x0FU);
    if (delta == 0U) {
        st->err_total++;
        return E2E_ERR_REPEAT;
    }
    st->last_cnt = cnt;
    if (delta == 1U) {
        return E2E_OK;
    }
    if (delta == 2U) {
        st->lost++;
        return E2E_OK;
    }
    st->err_total++;
    return E2E_ERR_SEQUENCE;
}

void Com_Pack_Pedal_Cmd(const Com_Pedal_Cmd_t *in, uint8_t *out_buf, uint8_t *alive_cnt)
{
    memset(out_buf, 0, 4U);
    Com_PackBits(out_buf, 0U, 16U, clamp_and_quantize_u32(in->pedal_pct, 0.01F, 0.0F, 0.0F, 100.0F, 16U));
    E2E_Protect(out_buf, 4U, 3U, 16U, 0x00U, alive_cnt);
}

E2E_Status_t Com_Unpack_Pedal_Cmd(const uint8_t *in_buf, Com_Pedal_Cmd_t *out, E2E_State_t *e2e_state)
{
    E2E_Status_t st = E2E_Check(in_buf, 4U, 3U, 16U, 0x00U, e2e_state);
    if (st != E2E_OK) {
        return st;
    }
    out->pedal_pct = (float)Com_UnpackBits(in_buf, 0U, 16U) * 0.01F + 0.0F;
    return E2E_OK;
}

void Com_Pack_ECU_Heartbeat(const Com_ECU_Heartbeat_t *in, uint8_t *out_buf)
{
    memset(out_buf, 0, 5U);
    Com_PackBits(out_buf, 0U, 32U, (uint32_t)in->hb_counter);
    Com_PackBits(out_buf, 32U, 8U, (uint32_t)in->loopback_flag);
}

void Com_Unpack_ECU_Heartbeat(const uint8_t *in_buf, Com_ECU_Heartbeat_t *out)
{
    out->hb_counter = (uint32_t)Com_UnpackBits(in_buf, 0U, 32U);
    out->loopback_flag = (uint8_t)Com_UnpackBits(in_buf, 32U, 8U);
}

void Com_Pack_Drive_Status(const Com_Drive_Status_t *in, uint8_t *out_buf, uint8_t *alive_cnt)
{
    memset(out_buf, 0, 7U);
    Com_PackBits(out_buf, 0U, 16U, clamp_and_quantize_u32(in->speed_rpm, 0.1F, 0.0F, 0.0F, 300.0F, 16U));
    Com_PackBits(out_buf, 16U, 8U, clamp_and_quantize_u32(in->duty_pct, 0.5F, 0.0F, 0.0F, 100.0F, 8U));
    Com_PackBits(out_buf, 24U, 4U, (uint32_t)in->mode);
    Com_PackBits(out_buf, 32U, 16U, clamp_and_quantize_u32(in->speed_req_rpm, 0.1F, 0.0F, 0.0F, 300.0F, 16U));
    E2E_Protect(out_buf, 7U, 6U, 28U, 0x00U, alive_cnt);
}

E2E_Status_t Com_Unpack_Drive_Status(const uint8_t *in_buf, Com_Drive_Status_t *out, E2E_State_t *e2e_state)
{
    E2E_Status_t st = E2E_Check(in_buf, 7U, 6U, 28U, 0x00U, e2e_state);
    if (st != E2E_OK) {
        return st;
    }
    out->speed_rpm = (float)Com_UnpackBits(in_buf, 0U, 16U) * 0.1F + 0.0F;
    out->duty_pct = (float)Com_UnpackBits(in_buf, 16U, 8U) * 0.5F + 0.0F;
    out->mode = (uint8_t)Com_UnpackBits(in_buf, 24U, 4U);
    out->speed_req_rpm = (float)Com_UnpackBits(in_buf, 32U, 16U) * 0.1F + 0.0F;
    return E2E_OK;
}

void Com_Pack_Diag_Status(const Com_Diag_Status_t *in, uint8_t *out_buf, uint8_t *alive_cnt)
{
    memset(out_buf, 0, 5U);
    Com_PackBits(out_buf, 0U, 16U, (uint32_t)in->fault_bitmap);
    Com_PackBits(out_buf, 16U, 8U, (uint32_t)in->rx_err_count);
    E2E_Protect(out_buf, 5U, 4U, 24U, 0x01U, alive_cnt);
}

E2E_Status_t Com_Unpack_Diag_Status(const uint8_t *in_buf, Com_Diag_Status_t *out, E2E_State_t *e2e_state)
{
    E2E_Status_t st = E2E_Check(in_buf, 5U, 4U, 24U, 0x01U, e2e_state);
    if (st != E2E_OK) {
        return st;
    }
    out->fault_bitmap = (uint16_t)Com_UnpackBits(in_buf, 0U, 16U);
    out->rx_err_count = (uint8_t)Com_UnpackBits(in_buf, 16U, 8U);
    return E2E_OK;
}

void Com_Pack_AEB_Status(const Com_AEB_Status_t *in, uint8_t *out_buf, uint8_t *alive_cnt)
{
    memset(out_buf, 0, 5U);
    Com_PackBits(out_buf, 0U, 16U, (uint32_t)in->distance_mm);
    Com_PackBits(out_buf, 16U, 4U, (uint32_t)in->aeb_state);
    Com_PackBits(out_buf, 20U, 1U, (uint32_t)in->tof_valid);
    E2E_Protect(out_buf, 5U, 4U, 24U, 0x10U, alive_cnt);
}

E2E_Status_t Com_Unpack_AEB_Status(const uint8_t *in_buf, Com_AEB_Status_t *out, E2E_State_t *e2e_state)
{
    E2E_Status_t st = E2E_Check(in_buf, 5U, 4U, 24U, 0x10U, e2e_state);
    if (st != E2E_OK) {
        return st;
    }
    out->distance_mm = (uint16_t)Com_UnpackBits(in_buf, 0U, 16U);
    out->aeb_state = (uint8_t)Com_UnpackBits(in_buf, 16U, 4U);
    out->tof_valid = (uint8_t)Com_UnpackBits(in_buf, 20U, 1U);
    return E2E_OK;
}
