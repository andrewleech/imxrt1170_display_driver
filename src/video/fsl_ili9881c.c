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
#define MIPI_DCS_SET_TEAR_ON      0x35

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static status_t ILI9881C_SwitchPage(mipi_dsi_device_t *dsiDevice, uint8_t page);
static status_t ILI9881C_WriteCmd(mipi_dsi_device_t *dsiDevice, uint8_t cmd, uint8_t data);

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

/**
 * Switch ILI9881C register page.
 *
 * The ILI9881C has 5 register pages (0-4). Page switching is done by writing
 * {0xFF, 0x98, 0x81, page} as a DCS long write.
 */
static status_t ILI9881C_SwitchPage(mipi_dsi_device_t *dsiDevice, uint8_t page)
{
    uint8_t buf[4] = {0xFF, 0x98, 0x81, page};
    return MIPI_DSI_GenericWrite(dsiDevice, buf, 4);
}

/**
 * Write a single register on the current page.
 */
static status_t ILI9881C_WriteCmd(mipi_dsi_device_t *dsiDevice, uint8_t cmd, uint8_t data)
{
    uint8_t buf[2] = {cmd, data};
    return MIPI_DSI_GenericWrite(dsiDevice, buf, 2);
}

/**
 * Send the full panel initialization sequence.
 *
 * Based on the rpi_5inch panel from the mainline Linux panel-ilitek-ili9881c.c
 * driver. This is a 5" 720x1280 2-lane MIPI DSI ILI9881C panel (62x110mm
 * active area) matching the WF50DTYA3MNG10000 module specifications.
 *
 * The sequence configures:
 * - Page 3: GIP (Gate-In-Panel) timing and signal mapping
 * - Page 4: Power control and voltage settings
 * - Page 1: Panel operation, VCOM, VREG, gamma correction
 * - Page 0: Return to default page for normal operation
 */
static status_t ILI9881C_InitSequence(mipi_dsi_device_t *dsiDevice)
{
    status_t status;

    /* Skip Page 3 (GIP) — leave NVM-programmed gate driver mapping intact.
     * The GIP settings are panel-specific and the NVM should have correct values
     * for the WF50DTYA3MNG10000 module. */

    /* ---- Page 4: Power control ---- */
    status = ILI9881C_SwitchPage(dsiDevice, 4);
    if (status != kStatus_Success) return status;

    ILI9881C_WriteCmd(dsiDevice, 0x6C, 0x15);
    ILI9881C_WriteCmd(dsiDevice, 0x6E, 0x2a);
    ILI9881C_WriteCmd(dsiDevice, 0x6F, 0x57);
    ILI9881C_WriteCmd(dsiDevice, 0x3A, 0xa4);
    ILI9881C_WriteCmd(dsiDevice, 0x8D, 0x1a);
    ILI9881C_WriteCmd(dsiDevice, 0x87, 0xba);
    ILI9881C_WriteCmd(dsiDevice, 0x26, 0x76);
    ILI9881C_WriteCmd(dsiDevice, 0xB2, 0xd1);

    /* ---- Page 1: Panel operation, VCOM, VREG, gamma ---- */
    status = ILI9881C_SwitchPage(dsiDevice, 1);
    if (status != kStatus_Success) return status;

    /* Skip reg 0x22 (panel operation) — keep NVM default 0x30 (BGR mode).
     * Color order compensated by LCDIF kLCDIFV2_LineOrderBGR.
     * Skip reg 0x2E (NL) — NVM default 0xC8 (1280 lines). Setting 0xB4 causes dimness. */
    ILI9881C_WriteCmd(dsiDevice, 0x31, 0x00);  /* Display inversion */
    ILI9881C_WriteCmd(dsiDevice, 0x53, 0x35);  /* VCOM1 */
    ILI9881C_WriteCmd(dsiDevice, 0x55, 0x50);  /* VCOM2 */
    ILI9881C_WriteCmd(dsiDevice, 0x50, 0xaf);  /* VREG1 */
    ILI9881C_WriteCmd(dsiDevice, 0x51, 0xaf);  /* VREG2 */
    ILI9881C_WriteCmd(dsiDevice, 0x60, 0x14);  /* Source timing */

    /* Positive gamma correction */
    ILI9881C_WriteCmd(dsiDevice, 0xA0, 0x08);
    ILI9881C_WriteCmd(dsiDevice, 0xA1, 0x1d);
    ILI9881C_WriteCmd(dsiDevice, 0xA2, 0x2c);
    ILI9881C_WriteCmd(dsiDevice, 0xA3, 0x14);
    ILI9881C_WriteCmd(dsiDevice, 0xA4, 0x19);
    ILI9881C_WriteCmd(dsiDevice, 0xA5, 0x2e);
    ILI9881C_WriteCmd(dsiDevice, 0xA6, 0x22);
    ILI9881C_WriteCmd(dsiDevice, 0xA7, 0x23);
    ILI9881C_WriteCmd(dsiDevice, 0xA8, 0x97);
    ILI9881C_WriteCmd(dsiDevice, 0xA9, 0x1e);
    ILI9881C_WriteCmd(dsiDevice, 0xAA, 0x29);
    ILI9881C_WriteCmd(dsiDevice, 0xAB, 0x7b);
    ILI9881C_WriteCmd(dsiDevice, 0xAC, 0x18);
    ILI9881C_WriteCmd(dsiDevice, 0xAD, 0x17);
    ILI9881C_WriteCmd(dsiDevice, 0xAE, 0x4b);
    ILI9881C_WriteCmd(dsiDevice, 0xAF, 0x1f);
    ILI9881C_WriteCmd(dsiDevice, 0xB0, 0x27);
    ILI9881C_WriteCmd(dsiDevice, 0xB1, 0x52);
    ILI9881C_WriteCmd(dsiDevice, 0xB2, 0x63);
    ILI9881C_WriteCmd(dsiDevice, 0xB3, 0x39);

    /* Negative gamma correction */
    ILI9881C_WriteCmd(dsiDevice, 0xC0, 0x08);
    ILI9881C_WriteCmd(dsiDevice, 0xC1, 0x1d);
    ILI9881C_WriteCmd(dsiDevice, 0xC2, 0x2c);
    ILI9881C_WriteCmd(dsiDevice, 0xC3, 0x14);
    ILI9881C_WriteCmd(dsiDevice, 0xC4, 0x19);
    ILI9881C_WriteCmd(dsiDevice, 0xC5, 0x2e);
    ILI9881C_WriteCmd(dsiDevice, 0xC6, 0x22);
    ILI9881C_WriteCmd(dsiDevice, 0xC7, 0x23);
    ILI9881C_WriteCmd(dsiDevice, 0xC8, 0x97);
    ILI9881C_WriteCmd(dsiDevice, 0xC9, 0x1e);
    ILI9881C_WriteCmd(dsiDevice, 0xCA, 0x29);
    ILI9881C_WriteCmd(dsiDevice, 0xCB, 0x7b);
    ILI9881C_WriteCmd(dsiDevice, 0xCC, 0x18);
    ILI9881C_WriteCmd(dsiDevice, 0xCD, 0x17);
    ILI9881C_WriteCmd(dsiDevice, 0xCE, 0x4b);
    ILI9881C_WriteCmd(dsiDevice, 0xCF, 0x1f);
    ILI9881C_WriteCmd(dsiDevice, 0xD0, 0x27);
    ILI9881C_WriteCmd(dsiDevice, 0xD1, 0x52);
    ILI9881C_WriteCmd(dsiDevice, 0xD2, 0x63);
    ILI9881C_WriteCmd(dsiDevice, 0xD3, 0x39);

    /* ---- Switch back to Page 0 for normal operation ---- */
    status = ILI9881C_SwitchPage(dsiDevice, 0);
    if (status != kStatus_Success) return status;

    return kStatus_Success;
}

status_t ILI9881C_Init(display_handle_t *handle, const display_config_t *config)
{
    const ili9881c_resource_t *resource = (const ili9881c_resource_t *)(handle->resource);
    mipi_dsi_device_t *dsiDevice = resource->dsiDevice;

    /* Verify resolution */
    if (config->resolution != FSL_VIDEO_RESOLUTION(720, 1200))
    {
        return kStatus_InvalidArgument;
    }

    /* Hardware reset sequence (if reset/power pins are available) */
    if (resource->pullPowerPin) {
        resource->pullPowerPin(true);
        ILI9881C_DelayMs(5);
    }
    if (resource->pullResetPin) {
        resource->pullResetPin(true);
        ILI9881C_DelayMs(5);
        resource->pullResetPin(false);  /* Assert reset (active low) */
        ILI9881C_DelayMs(1);            /* Hold reset for >10us (datasheet) */
        resource->pullResetPin(true);   /* Release reset */
        ILI9881C_DelayMs(120);          /* Wait for NVM load after HW reset */
    } else {
        /* No HW reset — use SW reset instead */
        ILI9881C_DelayMs(10);
        uint8_t swResetCmd = 0x01;
        MIPI_DSI_GenericWrite(dsiDevice, &swResetCmd, 1);
        ILI9881C_DelayMs(120);
    }

    /* NVM defaults used. VCOM/VREG/gamma from Linux rpi_5inch init causes
     * green tint — values don't match this panel. Color order: LCDIF BGR
     * line order compensates for NVM BGR mode (reg 0x22=0x30). */

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
