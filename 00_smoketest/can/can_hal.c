#include "can_hal.h"
#include <string.h>

static CAN_HandleTypeDef *p_hcan = NULL;

int CAN_HAL_Init(CAN_HandleTypeDef *hcan) {
  if (!hcan) {
    return CAN_HAL_ERROR;
  }
  p_hcan = hcan;

  // Accept-all filter: 32-bit mask mode, ID = 0 / Mask = 0 -> every frame
  // passes
  CAN_FilterTypeDef filter;
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(p_hcan, &filter) != HAL_OK) {
    return CAN_HAL_ERROR;
  }
  if (HAL_CAN_Start(p_hcan) != HAL_OK) {
    return CAN_HAL_ERROR;
  }
  return CAN_HAL_OK;
}

int CAN_HAL_Send(uint32_t id, const uint8_t *data, uint8_t dlc) {
  if (!p_hcan || dlc > 8 || (dlc > 0 && !data)) {
    return CAN_HAL_ERROR;
  }

  CAN_TxHeaderTypeDef hdr;
  uint32_t mailbox;
  uint8_t buf[8] = {0};

  hdr.StdId = id & 0x7FF;
  hdr.ExtId = 0;
  hdr.IDE = CAN_ID_STD;
  hdr.RTR = CAN_RTR_DATA;
  hdr.DLC = dlc;
  hdr.TransmitGlobalTime = DISABLE;

  if (dlc > 0) {
    memcpy(buf, data, dlc);
  }

  // Wait briefly for a free mailbox (3 mailboxes available)
  uint32_t start = HAL_GetTick();
  while (HAL_CAN_GetTxMailboxesFreeLevel(p_hcan) == 0) {
    if ((HAL_GetTick() - start) > 10) {
      return CAN_HAL_TIMEOUT;
    }
  }

  if (HAL_CAN_AddTxMessage(p_hcan, &hdr, buf, &mailbox) != HAL_OK) {
    return CAN_HAL_ERROR;
  }
  return CAN_HAL_OK;
}

int CAN_HAL_Receive(CAN_Frame_t *frame) {
  if (!p_hcan || !frame) {
    return CAN_HAL_ERROR;
  }
  if (HAL_CAN_GetRxFifoFillLevel(p_hcan, CAN_RX_FIFO0) == 0) {
    return 0;
  }

  CAN_RxHeaderTypeDef hdr;
  if (HAL_CAN_GetRxMessage(p_hcan, CAN_RX_FIFO0, &hdr, frame->data) != HAL_OK) {
    return CAN_HAL_ERROR;
  }
  frame->id = (hdr.IDE == CAN_ID_STD) ? hdr.StdId : hdr.ExtId;
  frame->dlc = (uint8_t)hdr.DLC;
  return 1;
}

int CAN_HAL_WaitFor(uint32_t id, CAN_Frame_t *frame, uint32_t timeoutMs) {
  uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < timeoutMs) {
    CAN_Frame_t f;
    int r = CAN_HAL_Receive(&f);
    if (r < 0) {
      return CAN_HAL_ERROR;
    }
    if (r == 1 && f.id == id) {
      if (frame) {
        *frame = f;
      }
      return CAN_HAL_OK;
    }
  }
  return CAN_HAL_TIMEOUT;
}

uint32_t CAN_HAL_GetError(void) {
  return p_hcan ? HAL_CAN_GetError(p_hcan) : 0xFFFFFFFFu;
}
