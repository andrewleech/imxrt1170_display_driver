/*
 * Copyright 2025 NXP + Community
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mpy_i2c_bridge.h"
#include "py/runtime.h"
#include "py/obj.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef struct {
    mp_obj_t i2c_obj;  // MicroPython I2C object
    bool initialized;
} mpy_i2c_bridge_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

// Global bridge state - registered as GC root to prevent collection
MP_REGISTER_ROOT_POINTER(mp_obj_t mpy_i2c_bridge_obj);
static mpy_i2c_bridge_t g_i2c_bridge = {
    .i2c_obj = MP_OBJ_NULL,
    .initialized = false
};

/*******************************************************************************
 * Code
 ******************************************************************************/

void mpy_i2c_bridge_init(mp_obj_t i2c_obj)
{
    if (i2c_obj == mp_const_none || i2c_obj == MP_OBJ_NULL) {
        g_i2c_bridge.i2c_obj = MP_OBJ_NULL;
        g_i2c_bridge.initialized = false;
        MP_STATE_VM(mpy_i2c_bridge_obj) = MP_OBJ_NULL;
        return;
    }

    // Store I2C object
    g_i2c_bridge.i2c_obj = i2c_obj;
    g_i2c_bridge.initialized = true;

    // Register as GC root to prevent collection
    MP_STATE_VM(mpy_i2c_bridge_obj) = i2c_obj;
}

void mpy_i2c_bridge_deinit(void)
{
    g_i2c_bridge.i2c_obj = MP_OBJ_NULL;
    g_i2c_bridge.initialized = false;
    MP_STATE_VM(mpy_i2c_bridge_obj) = MP_OBJ_NULL;
}

bool mpy_i2c_bridge_is_initialized(void)
{
    return g_i2c_bridge.initialized && g_i2c_bridge.i2c_obj != MP_OBJ_NULL;
}

status_t mpy_i2c_bridge_send(uint8_t device_address, const uint8_t *data, size_t data_size)
{
    if (!mpy_i2c_bridge_is_initialized()) {
        return kStatus_Fail;
    }

    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        // Call i2c.writeto(addr, data)
        mp_obj_t args[2] = {
            MP_OBJ_NEW_SMALL_INT(device_address),
            mp_obj_new_bytes(data, data_size)
        };
        mp_call_method_n_kw(2, 0, args);

        nlr_pop();
        return kStatus_Success;
    } else {
        // Exception occurred
        mp_obj_print_exception(&mp_plat_print, MP_OBJ_FROM_PTR(nlr.ret_val));
        return kStatus_Fail;
    }
}

status_t mpy_i2c_bridge_send_reg(uint8_t device_address, uint32_t reg_address,
                                  size_t reg_addr_size, const uint8_t *data, size_t data_size)
{
    if (!mpy_i2c_bridge_is_initialized()) {
        return kStatus_Fail;
    }

    if (reg_addr_size > 2) {
        return kStatus_InvalidArgument;
    }

    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        // Call i2c.writeto_mem(addr, memaddr, data, addrsize=reg_addr_size)
        mp_obj_t args[4] = {
            MP_OBJ_NEW_SMALL_INT(device_address),
            MP_OBJ_NEW_SMALL_INT(reg_address),
            mp_obj_new_bytes(data, data_size),
            MP_OBJ_NEW_SMALL_INT(reg_addr_size * 8)  // Convert bytes to bits
        };

        // Load writeto_mem method
        mp_obj_t writeto_mem_method = mp_load_attr(g_i2c_bridge.i2c_obj, MP_QSTR_writeto_mem);

        // Call method with 4 arguments (self is implicit)
        mp_call_function_n_kw(writeto_mem_method, 3, 1, args);

        nlr_pop();
        return kStatus_Success;
    } else {
        // Exception occurred
        mp_obj_print_exception(&mp_plat_print, MP_OBJ_FROM_PTR(nlr.ret_val));
        return kStatus_Fail;
    }
}

status_t mpy_i2c_bridge_receive(uint8_t device_address, uint8_t *data, size_t data_size)
{
    if (!mpy_i2c_bridge_is_initialized()) {
        return kStatus_Fail;
    }

    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        // Call i2c.readfrom(addr, len)
        mp_obj_t args[2] = {
            MP_OBJ_NEW_SMALL_INT(device_address),
            MP_OBJ_NEW_SMALL_INT(data_size)
        };

        // Load readfrom method
        mp_obj_t readfrom_method = mp_load_attr(g_i2c_bridge.i2c_obj, MP_QSTR_readfrom);

        // Call method
        mp_obj_t result = mp_call_function_n_kw(readfrom_method, 2, 0, args);

        // Extract bytes from result
        mp_buffer_info_t bufinfo;
        mp_get_buffer_raise(result, &bufinfo, MP_BUFFER_READ);

        if (bufinfo.len != data_size) {
            nlr_pop();
            return kStatus_Fail;
        }

        memcpy(data, bufinfo.buf, data_size);

        nlr_pop();
        return kStatus_Success;
    } else {
        // Exception occurred
        mp_obj_print_exception(&mp_plat_print, MP_OBJ_FROM_PTR(nlr.ret_val));
        return kStatus_Fail;
    }
}

status_t mpy_i2c_bridge_receive_reg(uint8_t device_address, uint32_t reg_address,
                                     size_t reg_addr_size, uint8_t *data, size_t data_size)
{
    if (!mpy_i2c_bridge_is_initialized()) {
        return kStatus_Fail;
    }

    if (reg_addr_size > 2) {
        return kStatus_InvalidArgument;
    }

    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        // Call i2c.readfrom_mem(addr, memaddr, len, addrsize=reg_addr_size)
        mp_obj_t args[4] = {
            MP_OBJ_NEW_SMALL_INT(device_address),
            MP_OBJ_NEW_SMALL_INT(reg_address),
            MP_OBJ_NEW_SMALL_INT(data_size),
            MP_OBJ_NEW_SMALL_INT(reg_addr_size * 8)  // Convert bytes to bits
        };

        // Load readfrom_mem method
        mp_obj_t readfrom_mem_method = mp_load_attr(g_i2c_bridge.i2c_obj, MP_QSTR_readfrom_mem);

        // Call method with 4 arguments
        mp_obj_t result = mp_call_function_n_kw(readfrom_mem_method, 3, 1, args);

        // Extract bytes from result
        mp_buffer_info_t bufinfo;
        mp_get_buffer_raise(result, &bufinfo, MP_BUFFER_READ);

        if (bufinfo.len != data_size) {
            nlr_pop();
            return kStatus_Fail;
        }

        memcpy(data, bufinfo.buf, data_size);

        nlr_pop();
        return kStatus_Success;
    } else {
        // Exception occurred
        mp_obj_print_exception(&mp_plat_print, MP_OBJ_FROM_PTR(nlr.ret_val));
        return kStatus_Fail;
    }
}
