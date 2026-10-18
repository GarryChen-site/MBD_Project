#include "i2c_sensors_hal.h"
#include <stdio.h>
#include <string.h>

static uint8_t vl53l0x_stop_variable = 0;

void I2C_RecoverBus(I2C_HandleTypeDef *hi2c) {
    if (!hi2c || !hi2c->Instance) return;

    // Reset peripheral via software reset bit
    hi2c->Instance->CR1 |= I2C_CR1_SWRST;
    HAL_Delay(2);
    hi2c->Instance->CR1 &= ~I2C_CR1_SWRST;

    // Re-initialize I2C
    HAL_I2C_Init(hi2c);
}

int I2C_ScanBus(I2C_HandleTypeDef *hi2c, uint8_t *found_addresses, uint8_t max_addresses) {
    if (!hi2c || !found_addresses || max_addresses == 0) return 0;

    // If hardware is stuck in BUSY, perform hardware recovery
    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY)) {
        I2C_RecoverBus(hi2c);
    }

    int count = 0;
    for (uint8_t addr = 1; addr < 128; addr++) {
        if (hi2c->State != HAL_I2C_STATE_READY) {
            hi2c->State = HAL_I2C_STATE_READY;
        }

        HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(hi2c, (addr << 1), 2, 10);
        if (status == HAL_OK) {
            if (count < max_addresses) {
                found_addresses[count++] = addr;
            }
        } else if (status == HAL_BUSY) {
            I2C_RecoverBus(hi2c);
        }
    }
    return count;
}

// ---------------------------------------------------------------------------
// MMA7660 3-Axis Accelerometer Implementation
// ---------------------------------------------------------------------------

static int8_t MMA7660_Convert6BitSigned(uint8_t raw) {
    // Bits 5:0 are 6-bit two's complement value
    if (raw & 0x20) {
        return (int8_t)(raw | 0xC0); // Sign-extend to 8-bit
    } else {
        return (int8_t)(raw & 0x1F);
    }
}

int MMA7660_Init(I2C_HandleTypeDef *hi2c) {
    if (!hi2c) return I2C_SENSORS_ERROR;

    // Check device readiness
    if (HAL_I2C_IsDeviceReady(hi2c, MMA7660_ADDR_8BIT, 3, 20) != HAL_OK) {
        return I2C_SENSORS_ERROR;
    }

    uint8_t data;

    // 1. Put MMA7660 in Standby mode before modifying configuration
    data = 0x00;
    if (HAL_I2C_Mem_Write(hi2c, MMA7660_ADDR_8BIT, MMA7660_REG_MODE, 1, &data, 1, 100) != HAL_OK) {
        return I2C_SENSORS_ERROR;
    }

    // 2. Set sample rate: 64 samples/second (SRST = 0x00)
    data = 0x00;
    if (HAL_I2C_Mem_Write(hi2c, MMA7660_ADDR_8BIT, MMA7660_REG_SRST, 1, &data, 1, 100) != HAL_OK) {
        return I2C_SENSORS_ERROR;
    }

    // 3. Put MMA7660 in Active Mode (bit 0 = 1)
    data = 0x01;
    if (HAL_I2C_Mem_Write(hi2c, MMA7660_ADDR_8BIT, MMA7660_REG_MODE, 1, &data, 1, 100) != HAL_OK) {
        return I2C_SENSORS_ERROR;
    }

    return I2C_SENSORS_OK;
}

int MMA7660_Read(I2C_HandleTypeDef *hi2c, MMA7660_Data_t *data) {
    if (!hi2c || !data) return I2C_SENSORS_ERROR;

    uint8_t raw_buf[4] = {0}; // X, Y, Z, TILT

    // Bit 6 is the Alert bit. If Alert bit is 1, a register refresh collided with read.
    // Retry up to 5 times to ensure clean reading.
    int retries = 5;
    bool success = false;

    while (retries-- > 0) {
        if (HAL_I2C_Mem_Read(hi2c, MMA7660_ADDR_8BIT, MMA7660_REG_XOUT, 1, raw_buf, 4, 50) == HAL_OK) {
            // Check if alert bit (bit 6) is cleared on X, Y, Z
            if ((raw_buf[0] & 0x40) == 0 &&
                (raw_buf[1] & 0x40) == 0 &&
                (raw_buf[2] & 0x40) == 0) {
                success = true;
                break;
            }
        }
        HAL_Delay(2);
    }

    if (!success) return I2C_SENSORS_ERROR;

    data->raw_x = MMA7660_Convert6BitSigned(raw_buf[0] & 0x3F);
    data->raw_y = MMA7660_Convert6BitSigned(raw_buf[1] & 0x3F);
    data->raw_z = MMA7660_Convert6BitSigned(raw_buf[2] & 0x3F);

    // MMA7660 sensitivity: 21.33 counts / g
    data->x_g = (float)data->raw_x / 21.33f;
    data->y_g = (float)data->raw_y / 21.33f;
    data->z_g = (float)data->raw_z / 21.33f;

    data->tilt_status = raw_buf[3];

    // Decode orientation from TILT register (bits 4:2 = BaFro, bits 1:0 = PoLa)
    uint8_t p_l = data->tilt_status & 0x03;
    uint8_t b_f = (data->tilt_status >> 2) & 0x03;

    if (b_f == 0x01) {
        data->orientation = "Front";
    } else if (b_f == 0x02) {
        data->orientation = "Back";
    } else if (p_l == 0x01) {
        data->orientation = "Left";
    } else if (p_l == 0x02) {
        data->orientation = "Right";
    } else if (data->z_g > 0.6f) {
        data->orientation = "Flat (Level)";
    } else if (data->z_g < -0.6f) {
        data->orientation = "Inverted";
    } else {
        data->orientation = "Tilted";
    }

    return I2C_SENSORS_OK;
}

// ---------------------------------------------------------------------------
// VL53L0X Time-of-Flight Ranging Sensor Implementation
// ---------------------------------------------------------------------------

static HAL_StatusTypeDef VL53L0X_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(hi2c, VL53L0X_ADDR_8BIT, reg, 1, &val, 1, 100);
}

static uint8_t VL53L0X_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg) {
    uint8_t val = 0;
    HAL_I2C_Mem_Read(hi2c, VL53L0X_ADDR_8BIT, reg, 1, &val, 1, 100);
    return val;
}

int VL53L0X_Init(I2C_HandleTypeDef *hi2c) {
    if (!hi2c) return I2C_SENSORS_ERROR;

    // Check device readiness
    if (HAL_I2C_IsDeviceReady(hi2c, VL53L0X_ADDR_8BIT, 3, 20) != HAL_OK) {
        return I2C_SENSORS_ERROR;
    }

    // Verify Model ID: register 0xC0 should return 0xEE
    uint8_t model_id = VL53L0X_ReadReg(hi2c, VL53L0X_REG_IDENTIFICATION_MODEL_ID);
    if (model_id != 0xEE) {
        return I2C_SENSORS_ERROR; // Sensor not found or incorrect chip ID
    }

    // Configure 2V8 I/O Mode (bit 0 of register 0x89)
    uint8_t vhv = VL53L0X_ReadReg(hi2c, 0x89);
    VL53L0X_WriteReg(hi2c, 0x89, vhv | 0x01);

    // Standard I2C Mode Tuning Sequence
    VL53L0X_WriteReg(hi2c, 0x88, 0x00);
    VL53L0X_WriteReg(hi2c, 0x80, 0x01);
    VL53L0X_WriteReg(hi2c, 0xFF, 0x01);
    VL53L0X_WriteReg(hi2c, 0x00, 0x00);

    // Read and save factory stop_variable from register 0x91
    vl53l0x_stop_variable = VL53L0X_ReadReg(hi2c, 0x91);

    VL53L0X_WriteReg(hi2c, 0x00, 0x01);
    VL53L0X_WriteReg(hi2c, 0xFF, 0x00);
    VL53L0X_WriteReg(hi2c, 0x80, 0x00);

    // Disable MSRC and PRE_RANGE limit checks
    uint8_t msrc = VL53L0X_ReadReg(hi2c, 0x60);
    VL53L0X_WriteReg(hi2c, 0x60, msrc | 0x12);

    // Enable full internal measurement sequence
    VL53L0X_WriteReg(hi2c, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xFF);

    return I2C_SENSORS_OK;
}

int32_t VL53L0X_ReadDistance_mm(I2C_HandleTypeDef *hi2c) {
    if (!hi2c) return I2C_SENSORS_ERROR;

    // Load static tuning with saved stop_variable
    VL53L0X_WriteReg(hi2c, 0x80, 0x01);
    VL53L0X_WriteReg(hi2c, 0xFF, 0x01);
    VL53L0X_WriteReg(hi2c, 0x00, 0x00);
    VL53L0X_WriteReg(hi2c, 0x91, vl53l0x_stop_variable);
    VL53L0X_WriteReg(hi2c, 0x00, 0x01);
    VL53L0X_WriteReg(hi2c, 0xFF, 0x00);
    VL53L0X_WriteReg(hi2c, 0x80, 0x00);

    // Arm single-shot measurement: SYSRANGE_START (0x00) = 0x01
    VL53L0X_WriteReg(hi2c, VL53L0X_REG_SYSRANGE_START, 0x01);

    // Poll until start bit is cleared
    uint32_t start_tick = HAL_GetTick();
    while (VL53L0X_ReadReg(hi2c, VL53L0X_REG_SYSRANGE_START) & 0x01) {
        if (HAL_GetTick() - start_tick > 200) {
            return I2C_SENSORS_TIMEOUT;
        }
    }

    // Poll until measurement data is ready (RESULT_INTERRUPT_STATUS & 0x07 != 0)
    start_tick = HAL_GetTick();
    while ((VL53L0X_ReadReg(hi2c, VL53L0X_REG_RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (HAL_GetTick() - start_tick > 500) {
            return I2C_SENSORS_TIMEOUT;
        }
    }

    // Read 2-byte distance from 0x1E (RESULT_RANGE_STATUS + 10)
    uint8_t dist_raw[2] = {0};
    if (HAL_I2C_Mem_Read(hi2c, VL53L0X_ADDR_8BIT, 0x1E, 1, dist_raw, 2, 100) != HAL_OK) {
        return I2C_SENSORS_ERROR;
    }

    // Clear interrupt to allow next measurement
    VL53L0X_WriteReg(hi2c, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    uint16_t dist_mm = ((uint16_t)dist_raw[0] << 8) | dist_raw[1];

    // Filter out sensor out-of-range flag (8190 or 8191 mm)
    if (dist_mm > 4000) {
        return I2C_SENSORS_ERROR; // Out of range or no target reflection
    }

    return (int32_t)dist_mm;
}
