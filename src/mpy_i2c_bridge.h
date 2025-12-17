/*
 * Copyright 2025 NXP + Community
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef MPY_I2C_BRIDGE_H
#define MPY_I2C_BRIDGE_H

#include "py/runtime.h"
#include "fsl_common.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the MicroPython I2C bridge with a Python I2C object
 *
 * Stores the Python I2C object for later use in C callbacks.
 * The object is registered as a GC root to prevent collection.
 *
 * @param i2c_obj MicroPython I2C object (machine.I2C instance)
 */
void mpy_i2c_bridge_init(mp_obj_t i2c_obj);

/**
 * @brief Deinitialize the I2C bridge
 *
 * Clears the stored I2C object reference.
 */
void mpy_i2c_bridge_deinit(void);

/**
 * @brief Check if I2C bridge is initialized
 *
 * @return true if bridge has a valid I2C object
 */
bool mpy_i2c_bridge_is_initialized(void);

/**
 * @brief Send data to I2C device using MicroPython I2C object
 *
 * Calls i2c.writeto(addr, data) on the stored MicroPython I2C object.
 *
 * @param device_address 7-bit I2C device address
 * @param data Pointer to data buffer to send
 * @param data_size Number of bytes to send
 * @return kStatus_Success on success, kStatus_Fail on error
 */
status_t mpy_i2c_bridge_send(uint8_t device_address, const uint8_t *data, size_t data_size);

/**
 * @brief Send data with register address to I2C device
 *
 * Calls i2c.writeto_mem(addr, reg, data) on the stored MicroPython I2C object.
 *
 * @param device_address 7-bit I2C device address
 * @param reg_address Register address within device
 * @param reg_addr_size Size of register address (1 or 2 bytes)
 * @param data Pointer to data buffer to send
 * @param data_size Number of bytes to send
 * @return kStatus_Success on success, kStatus_Fail on error
 */
status_t mpy_i2c_bridge_send_reg(uint8_t device_address, uint32_t reg_address,
                                  size_t reg_addr_size, const uint8_t *data, size_t data_size);

/**
 * @brief Receive data from I2C device using MicroPython I2C object
 *
 * Calls i2c.readfrom(addr, len) on the stored MicroPython I2C object.
 *
 * @param device_address 7-bit I2C device address
 * @param data Pointer to buffer to receive data
 * @param data_size Number of bytes to receive
 * @return kStatus_Success on success, kStatus_Fail on error
 */
status_t mpy_i2c_bridge_receive(uint8_t device_address, uint8_t *data, size_t data_size);

/**
 * @brief Receive data from I2C device register
 *
 * Calls i2c.readfrom_mem(addr, reg, len) on the stored MicroPython I2C object.
 *
 * @param device_address 7-bit I2C device address
 * @param reg_address Register address within device
 * @param reg_addr_size Size of register address (1 or 2 bytes)
 * @param data Pointer to buffer to receive data
 * @param data_size Number of bytes to receive
 * @return kStatus_Success on success, kStatus_Fail on error
 */
status_t mpy_i2c_bridge_receive_reg(uint8_t device_address, uint32_t reg_address,
                                     size_t reg_addr_size, uint8_t *data, size_t data_size);

#ifdef __cplusplus
}
#endif

#endif /* MPY_I2C_BRIDGE_H */
