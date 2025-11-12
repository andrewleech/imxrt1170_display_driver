/*
 * Copyright  2019 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_display.h"
#include "rpi.h"
#include "rpi_support.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define RPI_DelayMs VIDEO_DelayMs

/*******************************************************************************
 * Variables
 ******************************************************************************/

const display_operations_t rpi_ops = {
    .init   = RPI_Init,
    .deinit = RPI_Deinit,
    .start  = RPI_Start,
    .stop   = RPI_Stop,
};

/*******************************************************************************
 * Code
 ******************************************************************************/

status_t RPI_Init(display_handle_t *handle, const display_config_t *config)
{
    uint8_t param[6];
//    status_t status                    = kStatus_Success;
    const rpi_resource_t *resource = (const rpi_resource_t *)(handle->resource);
    mipi_dsi_device_t *dsiDevice       = resource->dsiDevice;
    if (config->resolution != FSL_VIDEO_RESOLUTION(800, 480))
    {
        return kStatus_InvalidArgument;
    }
    talk_to_raspi();
    // from tc358762_dsi_init

    //	rpi_touchscreen_write(dsi, DSI_LANEENABLE,
    //			      DSI_LANEENABLE_CLOCK |
    //			      DSI_LANEENABLE_D0);
    param[0] = 0x10;
    param[1] = 0x02;
    param[2] = 0x03;
    param[3] = 0x00;
    param[4] = 0x00;
    param[5] = 0x00;
    MIPI_DSI_GenericWrite(dsiDevice, param, 6);

    //rpi_touchscreen_write(dsi, PPI_D0S_CLRSIPOCOUNT, 0x05);
    param[0] = 0x64;
    param[1] = 0x01;
    param[2] = 0x05;
    param[3] = 0x00;
    param[4] = 0x00;
    param[5] = 0x00;
    MIPI_DSI_GenericWrite(dsiDevice, param, 6);

    //rpi_touchscreen_write(dsi, PPI_D1S_CLRSIPOCOUNT, 0x05);
    param[0] = 0x68;
    param[1] = 0x01;
    param[2] = 0x05;
    param[3] = 0x00;
    param[4] = 0x00;
    param[5] = 0x00;
    MIPI_DSI_GenericWrite(dsiDevice, param, 6);

    //rpi_touchscreen_write(dsi, PPI_D0S_ATMR, 0x00);
    param[0] = 0x44;
    param[1] = 0x01;
    param[2] = 0x00;
    param[3] = 0x00;
    param[4] = 0x00;
    param[5] = 0x00;
    MIPI_DSI_GenericWrite(dsiDevice, param, 6);

    //rpi_touchscreen_write(dsi, PPI_D1S_ATMR, 0x00);
    param[0] = 0x48;
    param[1] = 0x01;
    param[2] = 0x00;
    param[3] = 0x00;
    param[4] = 0x00;
    param[5] = 0x00;
    MIPI_DSI_GenericWrite(dsiDevice, param, 6);

    //rpi_touchscreen_write(dsi, PPI_LPTXTIMECNT, 0x03);
    param[0] = 0x14;
    param[1] = 0x01;
    param[2] = 0x03;
    param[3] = 0x00;
    param[4] = 0x00;
    param[5] = 0x00;
    MIPI_DSI_GenericWrite(dsiDevice, param, 6);

    //rpi_touchscreen_write(dsi, SPICMR, 0x00);
    param[0] = 0x50;
    param[1] = 0x04;
    param[2] = 0x00;
    param[3] = 0x00;
    param[4] = 0x00;
    param[5] = 0x00;
    MIPI_DSI_GenericWrite(dsiDevice, param, 6);

    //rpi_touchscreen_write(dsi, LCDCTRL, 0x00100150);
    param[0] = 0x20;
    param[1] = 0x04;
    param[2] = 0x50;
    param[3] = 0x01;
    param[4] = 0x10;
    param[5] = 0x00;
    MIPI_DSI_GenericWrite(dsiDevice, param, 6);

    //rpi_touchscreen_write(dsi, SYSCTRL, 0x040f);
    param[0] = 0x64;
    param[1] = 0x04;
    param[2] = 0x0f;
    param[3] = 0x04;
    param[4] = 0x00;
    param[5] = 0x00;
    MIPI_DSI_GenericWrite(dsiDevice, param, 6);

    RPI_DelayMs(100);

    //rpi_touchscreen_write(dsi, PPI_STARTPPI, 0x01);
    param[0] = 0x04;
    param[1] = 0x01;
    param[2] = 0x01;
    param[3] = 0x00;
    param[4] = 0x00;
    param[5] = 0x00;
    MIPI_DSI_GenericWrite(dsiDevice, param, 6);

    //rpi_touchscreen_write(dsi, DSI_STARTDSI, 0x01);
    param[0] = 0x04;
    param[1] = 0x02;
    param[2] = 0x01;
    param[3] = 0x00;
    param[4] = 0x00;
    param[5] = 0x00;
    MIPI_DSI_GenericWrite(dsiDevice, param, 6);

    RPI_DelayMs(100);

    return kStatus_Success;
}

status_t RPI_Deinit(display_handle_t *handle)
{
    const rpi_resource_t *resource = (const rpi_resource_t *)(handle->resource);
    mipi_dsi_device_t *dsiDevice       = resource->dsiDevice;

    (void)MIPI_DSI_DCS_EnterSleepMode(dsiDevice, true);

    //resource->pullResetPin(false);
    //resource->pullPowerPin(false);

    return kStatus_Success;
}

status_t RPI_Start(display_handle_t *handle)
{
    const rpi_resource_t *resource = (const rpi_resource_t *)(handle->resource);
    mipi_dsi_device_t *dsiDevice       = resource->dsiDevice;

    return MIPI_DSI_DCS_SetDisplayOn(dsiDevice, true);
}

status_t RPI_Stop(display_handle_t *handle)
{
    const rpi_resource_t *resource = (const rpi_resource_t *)(handle->resource);
    mipi_dsi_device_t *dsiDevice       = resource->dsiDevice;

    return MIPI_DSI_DCS_SetDisplayOn(dsiDevice, false);
}
