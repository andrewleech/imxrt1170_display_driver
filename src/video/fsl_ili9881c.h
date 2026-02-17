/*
 * Copyright 2026, Planet Innovation
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _FSL_ILI9881C_H_
#define _FSL_ILI9881C_H_

#include "fsl_display.h"
#include "fsl_mipi_dsi_cmd.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief ILI9881C resource. */
typedef struct _ili9881c_resource
{
    mipi_dsi_device_t *dsiDevice; /*!< MIPI DSI device. */
} ili9881c_resource_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initialize the ILI9881C display (WF50DTYA3MNG10000 module).
 *
 * This function initializes the ILI9881C LCD controller. The ILI9881C has
 * its initialization sequence pre-programmed in non-volatile memory (NVM),
 * so only standard MIPI DSI commands are needed (Sleep Out, Display On).
 *
 * Display specifications:
 * - Resolution: 720x1200 pixels
 * - Interface: 2-lane MIPI DSI
 * - Backlight: PWM control (handled in Python layer)
 * - Touch: GT928 controller @ I2C address 0x5D
 *
 * @param handle Pointer to the display handle.
 * @param config Pointer to the display configuration.
 * @return Returns @ref kStatus_Success if success, otherwise returns error code.
 */
status_t ILI9881C_Init(display_handle_t *handle, const display_config_t *config);

/*!
 * @brief Deinitialize the ILI9881C display.
 *
 * @param handle Pointer to the display handle.
 * @return Returns @ref kStatus_Success if success, otherwise returns error code.
 */
status_t ILI9881C_Deinit(display_handle_t *handle);

/*!
 * @brief Start the ILI9881C display.
 *
 * This function sends the Display On command to the ILI9881C.
 *
 * @param handle Pointer to the display handle.
 * @return Returns @ref kStatus_Success if success, otherwise returns error code.
 */
status_t ILI9881C_Start(display_handle_t *handle);

/*!
 * @brief Stop the ILI9881C display.
 *
 * This function sends the Display Off command to the ILI9881C.
 *
 * @param handle Pointer to the display handle.
 * @return Returns @ref kStatus_Success if success, otherwise returns error code.
 */
status_t ILI9881C_Stop(display_handle_t *handle);

extern const display_operations_t ili9881c_ops;

#if defined(__cplusplus)
}
#endif

#endif /* _FSL_ILI9881C_H_ */
