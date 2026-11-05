#ifndef I2C_SENSORS_HAL_H
#define I2C_SENSORS_HAL_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define I2C_SENSORS_OK       0
#define I2C_SENSORS_ERROR  (-1)
#define I2C_SENSORS_TIMEOUT (-2)

// 7-bit and 8-bit I2C Addresses
#define VL53L0X_ADDR_7BIT       0x29
#define VL53L0X_ADDR_8BIT       (VL53L0X_ADDR_7BIT << 1) // 0x52

#define MMA7660_ADDR_7BIT       0x4C
#define MMA7660_ADDR_8BIT       (MMA7660_ADDR_7BIT << 1) // 0x98

// MMA7660 Registers
#define MMA7660_REG_XOUT        0x00
#define MMA7660_REG_YOUT        0x01
#define MMA7660_REG_ZOUT        0x02
#define MMA7660_REG_TILT        0x03
#define MMA7660_REG_SRST        0x04
#define MMA7660_REG_SPCNT       0x05
#define MMA7660_REG_INTSU       0x06
#define MMA7660_REG_MODE        0x07

// VL53L0X Key Registers
#define VL53L0X_REG_SYSRANGE_START          0x00
#define VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG  0x01
#define VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR  0x0B
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS 0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS     0x14
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID 0xC0

typedef struct {
    float x_g;            // Acceleration in g (-1.5g to +1.5g)
    float y_g;
    float z_g;
    int8_t raw_x;         // Raw 6-bit signed count (-32 to +31)
    int8_t raw_y;
    int8_t raw_z;
    uint8_t tilt_status;  // Raw tilt register
    const char *orientation; // Human-readable (e.g., "Flat", "Left", "Right", etc.)
} MMA7660_Data_t;

typedef struct {
    uint16_t distance_mm; // Measured distance in millimeters
    bool is_valid;        // Range valid / within operating envelope
    uint8_t range_status; // Hardware range status
} VL53L0X_Data_t;

/**
 * @brief Reset and recover I2C bus from hardware BUSY lockup.
 */
void I2C_RecoverBus(I2C_HandleTypeDef *hi2c);

/**
 * @brief Scan the I2C bus across all 7-bit addresses (0x01 to 0x7F).
 * @param hi2c Pointer to I2C_HandleTypeDef
 * @param found_addresses Array to store detected 7-bit addresses
 * @param max_addresses Maximum capacity of found_addresses array
 * @return Number of responding ACK devices found
 */
int I2C_ScanBus(I2C_HandleTypeDef *hi2c, uint8_t *found_addresses, uint8_t max_addresses);

/**
 * @brief Initialize MMA7660 3-Axis Accelerometer.
 *        Configures device into Active Mode at 64 samples/sec.
 */
int MMA7660_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Read acceleration from MMA7660.
 */
int MMA7660_Read(I2C_HandleTypeDef *hi2c, MMA7660_Data_t *data);

/**
 * @brief Initialize VL53L0X Time-of-Flight Sensor.
 *        Verifies Model ID (0xEE) and configures standard 2V8 single-shot mode.
 */
int VL53L0X_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Trigger and read a single-shot distance measurement.
 * @return Measured distance in millimeters, or negative error code
 */
int32_t VL53L0X_ReadDistance_mm(I2C_HandleTypeDef *hi2c);

#endif /* I2C_SENSORS_HAL_H */
