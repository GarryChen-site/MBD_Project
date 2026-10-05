#ifndef CAN_HAL_H
#define CAN_HAL_H

#include "main.h"
#include <stdint.h>

#define CAN_HAL_OK 0
#define CAN_HAL_ERROR (-1)
#define CAN_HAL_TIMEOUT (-2)

/** Classic CAN 2.0A (11-bit ID) frame */
typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
} CAN_Frame_t;

/**
 * @brief  Configure an accept-all filter (FIFO0) and start the CAN peripheral.
 * @param  hcan  Handle already initialised by MX_CAN1_Init().
 */
int CAN_HAL_Init(CAN_HandleTypeDef *hcan);

/**
 * @brief  Queue one standard-ID data frame for transmission.
 */
int CAN_HAL_Send(uint32_t id, const uint8_t *data, uint8_t dlc);

/**
 * @brief  Non-blocking receive from FIFO0.
 * @retval 1 if a frame was read, 0 if FIFO empty, <0 on error.
 */
int CAN_HAL_Receive(CAN_Frame_t *frame);

/**
 * @brief  Poll FIFO0 until a frame with the given ID arrives or timeout.
 */
int CAN_HAL_WaitFor(uint32_t id, CAN_Frame_t *frame, uint32_t timeoutMs);

/** @brief Return HAL_CAN_GetError() bitmask of the bound handle. */
uint32_t CAN_HAL_GetError(void);

#endif /* CAN_HAL_H */
