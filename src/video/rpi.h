/*
 * Copyright  2019 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _RPI_H_
#define _RPI_H_

#include "fsl_display.h"
#include "fsl_mipi_dsi_cmd.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief RaspberryPi 7" Display resource.
 */
typedef struct _rpi_resource
{
    mipi_dsi_device_t *dsiDevice;      /*!< MIPI DSI device. */
    void (*pullResetPin)(bool pullUp); /*!< Function to pull reset pin high or low. */
    void (*pullPowerPin)(bool pullUp); /*!< Function to pull power pin high or low. */
} rpi_resource_t;

extern const display_operations_t rpi_ops;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

status_t RPI_Init(display_handle_t *handle, const display_config_t *config);

status_t RPI_Deinit(display_handle_t *handle);

status_t RPI_Start(display_handle_t *handle);

status_t RPI_Stop(display_handle_t *handle);

#if defined(__cplusplus)
}
#endif

#endif /* _RPI_H_ */
