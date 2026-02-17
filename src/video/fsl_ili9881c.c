/*
 * Copyright 2026, Planet Innovation
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_ili9881c.h"
#include "fsl_mipi_dsi_cmd.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ILI9881C_DelayMs VIDEO_DelayMs

/* MIPI DCS commands */
#define MIPI_DCS_EXIT_SLEEP_MODE  0x11
#define MIPI_DCS_SET_DISPLAY_ON   0x29
#define MIPI_DCS_SET_DISPLAY_OFF  0x28
#define MIPI_DCS_ENTER_SLEEP_MODE 0x10

/*******************************************************************************
 * Variables
 ******************************************************************************/

const display_operations_t ili9881c_ops = {
    .init   = ILI9881C_Init,
    .deinit = ILI9881C_Deinit,
    .start  = ILI9881C_Start,
    .stop   = ILI9881C_Stop,
};

/*******************************************************************************
 * Code
 ******************************************************************************/

status_t ILI9881C_Init(display_handle_t *handle, const display_config_t *config)
{
    const ili9881c_resource_t *resource = (const ili9881c_resource_t *)(handle->resource);
    mipi_dsi_device_t *dsiDevice = resource->dsiDevice;

    /* Verify resolution */
    if (config->resolution != FSL_VIDEO_RESOLUTION(720, 1200))
    {
        return kStatus_InvalidArgument;
    }

    /* The ILI9881C on WF50DTYA3MNG10000 module has its initialization sequence
     * pre-programmed in non-volatile memory (NVM). No custom init commands needed.
     * Just power up the panel and send standard MIPI DSI commands. */

    /* Wait for display to stabilize after power-on */
    ILI9881C_DelayMs(120);

    /* Exit sleep mode */
    uint8_t sleepOutCmd = MIPI_DCS_EXIT_SLEEP_MODE;
    MIPI_DSI_GenericWrite(dsiDevice, &sleepOutCmd, 1);

    /* Wait for display to wake up (spec requires 120ms after sleep out) */
    ILI9881C_DelayMs(120);

    /* Display On command is sent in ILI9881C_Start() */

    return kStatus_Success;
}

status_t ILI9881C_Deinit(display_handle_t *handle)
{
    const ili9881c_resource_t *resource = (const ili9881c_resource_t *)(handle->resource);
    mipi_dsi_device_t *dsiDevice = resource->dsiDevice;

    /* Turn off display */
    uint8_t displayOffCmd = MIPI_DCS_SET_DISPLAY_OFF;
    MIPI_DSI_GenericWrite(dsiDevice, &displayOffCmd, 1);
    ILI9881C_DelayMs(20);

    /* Enter sleep mode */
    uint8_t sleepInCmd = MIPI_DCS_ENTER_SLEEP_MODE;
    MIPI_DSI_GenericWrite(dsiDevice, &sleepInCmd, 1);
    ILI9881C_DelayMs(120);

    return kStatus_Success;
}

status_t ILI9881C_Start(display_handle_t *handle)
{
    const ili9881c_resource_t *resource = (const ili9881c_resource_t *)(handle->resource);
    mipi_dsi_device_t *dsiDevice = resource->dsiDevice;

    /* Turn on display */
    uint8_t displayOnCmd = MIPI_DCS_SET_DISPLAY_ON;
    MIPI_DSI_GenericWrite(dsiDevice, &displayOnCmd, 1);
    ILI9881C_DelayMs(20);

    return kStatus_Success;
}

status_t ILI9881C_Stop(display_handle_t *handle)
{
    const ili9881c_resource_t *resource = (const ili9881c_resource_t *)(handle->resource);
    mipi_dsi_device_t *dsiDevice = resource->dsiDevice;

    /* Turn off display */
    uint8_t displayOffCmd = MIPI_DCS_SET_DISPLAY_OFF;
    MIPI_DSI_GenericWrite(dsiDevice, &displayOffCmd, 1);
    ILI9881C_DelayMs(20);

    return kStatus_Success;
}
