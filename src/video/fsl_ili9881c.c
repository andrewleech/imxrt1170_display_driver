/*
 * Copyright 2026, Planet Innovation
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_ili9881c.h"
#include "fsl_mipi_dsi_cmd.h"
#include "py/mphal.h"
#include "py/mpconfig.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ILI9881C_DelayMs VIDEO_DelayMs
#define ILI9881C_PRINTF(...)   mp_printf(&mp_plat_print, __VA_ARGS__)

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
 * Send the panel initialization sequence.
 *
 * Minimal init found via bottom-up search (tests 58-60).
 *
 * NVM (factory-programmed OTP) loads on every power cycle. Our writes
 * override specific values in volatile SRAM. Only override what's needed.
 *
 * Required overrides (everything else from NVM):
 * - Page 3 GIP: NVM GIP broken at 1280 lines (test 58: vertical bars)
 * - P1 0x22: BGR+SS+DSI video mode (NVM=0x30, wrong for our HW)
 * - P1 0x2E: NL=1280 gate lines (NVM=0xB4/1200)
 * - P1 0x50/51: VREG=0xB7 (NVM VREG too low for kd050 GIP, test 59: blank)
 * - P1 0x53/55: VCOM left at NVM factory default (test 60: looks great)
 */
static status_t ILI9881C_InitSequence(mipi_dsi_device_t *dsiDevice)
{
    status_t status;

    /* Page 3: GIP timing from kd050hdfia020 — enabled for test 60.
     * Test 58: NVM GIP broken (vertical bars). Test 59: VREG alone = blank.
     * GIP is required. */
    status = ILI9881C_SwitchPage(dsiDevice, 3);
    if (status != kStatus_Success) return status;

    ILI9881C_WriteCmd(dsiDevice, 0x01, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x02, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x03, 0x72);
    ILI9881C_WriteCmd(dsiDevice, 0x04, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x05, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x06, 0x09);
    ILI9881C_WriteCmd(dsiDevice, 0x07, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x08, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x09, 0x01);
    ILI9881C_WriteCmd(dsiDevice, 0x0A, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x0B, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x0C, 0x01);
    ILI9881C_WriteCmd(dsiDevice, 0x0D, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x0E, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x0F, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x10, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x11, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x12, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x13, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x14, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x15, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x16, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x17, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x18, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x19, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x1A, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x1B, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x1C, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x1D, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x1E, 0x40);
    ILI9881C_WriteCmd(dsiDevice, 0x1F, 0x80);
    ILI9881C_WriteCmd(dsiDevice, 0x20, 0x05);
    ILI9881C_WriteCmd(dsiDevice, 0x21, 0x02);
    ILI9881C_WriteCmd(dsiDevice, 0x22, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x23, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x24, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x25, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x26, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x27, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x28, 0x33);
    ILI9881C_WriteCmd(dsiDevice, 0x29, 0x02);
    ILI9881C_WriteCmd(dsiDevice, 0x2A, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x2B, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x2C, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x2D, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x2E, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x2F, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x30, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x31, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x32, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x33, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x34, 0x04);
    ILI9881C_WriteCmd(dsiDevice, 0x35, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x36, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x37, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x38, 0x3C);
    ILI9881C_WriteCmd(dsiDevice, 0x39, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x3A, 0x40);
    ILI9881C_WriteCmd(dsiDevice, 0x3B, 0x40);
    ILI9881C_WriteCmd(dsiDevice, 0x3C, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x3D, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x3E, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x3F, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x40, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x41, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x42, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x43, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x44, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x50, 0x01);
    ILI9881C_WriteCmd(dsiDevice, 0x51, 0x23);
    ILI9881C_WriteCmd(dsiDevice, 0x52, 0x45);
    ILI9881C_WriteCmd(dsiDevice, 0x53, 0x67);
    ILI9881C_WriteCmd(dsiDevice, 0x54, 0x89);
    ILI9881C_WriteCmd(dsiDevice, 0x55, 0xAB);
    ILI9881C_WriteCmd(dsiDevice, 0x56, 0x01);
    ILI9881C_WriteCmd(dsiDevice, 0x57, 0x23);
    ILI9881C_WriteCmd(dsiDevice, 0x58, 0x45);
    ILI9881C_WriteCmd(dsiDevice, 0x59, 0x67);
    ILI9881C_WriteCmd(dsiDevice, 0x5A, 0x89);
    ILI9881C_WriteCmd(dsiDevice, 0x5B, 0xAB);
    ILI9881C_WriteCmd(dsiDevice, 0x5C, 0xCD);
    ILI9881C_WriteCmd(dsiDevice, 0x5D, 0xEF);
    ILI9881C_WriteCmd(dsiDevice, 0x5E, 0x11);
    ILI9881C_WriteCmd(dsiDevice, 0x5F, 0x01);
    ILI9881C_WriteCmd(dsiDevice, 0x60, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x61, 0x15);
    ILI9881C_WriteCmd(dsiDevice, 0x62, 0x14);
    ILI9881C_WriteCmd(dsiDevice, 0x63, 0x0E);
    ILI9881C_WriteCmd(dsiDevice, 0x64, 0x0F);
    ILI9881C_WriteCmd(dsiDevice, 0x65, 0x0C);
    ILI9881C_WriteCmd(dsiDevice, 0x66, 0x0D);
    ILI9881C_WriteCmd(dsiDevice, 0x67, 0x06);
    ILI9881C_WriteCmd(dsiDevice, 0x68, 0x02);
    ILI9881C_WriteCmd(dsiDevice, 0x69, 0x07);
    ILI9881C_WriteCmd(dsiDevice, 0x6A, 0x02);
    ILI9881C_WriteCmd(dsiDevice, 0x6B, 0x02);
    ILI9881C_WriteCmd(dsiDevice, 0x6C, 0x02);
    ILI9881C_WriteCmd(dsiDevice, 0x6D, 0x02);
    ILI9881C_WriteCmd(dsiDevice, 0x6E, 0x02);
    ILI9881C_WriteCmd(dsiDevice, 0x6F, 0x02);
    ILI9881C_WriteCmd(dsiDevice, 0x70, 0x02);
    ILI9881C_WriteCmd(dsiDevice, 0x71, 0x02);
    ILI9881C_WriteCmd(dsiDevice, 0x72, 0x02);
    ILI9881C_WriteCmd(dsiDevice, 0x73, 0x02);
    ILI9881C_WriteCmd(dsiDevice, 0x74, 0x02);
    ILI9881C_WriteCmd(dsiDevice, 0x75, 0x01);
    ILI9881C_WriteCmd(dsiDevice, 0x76, 0x00);
    ILI9881C_WriteCmd(dsiDevice, 0x77, 0x14);
    ILI9881C_WriteCmd(dsiDevice, 0x78, 0x15);
    ILI9881C_WriteCmd(dsiDevice, 0x79, 0x0E);
    ILI9881C_WriteCmd(dsiDevice, 0x7A, 0x0F);
    ILI9881C_WriteCmd(dsiDevice, 0x7B, 0x0C);
    ILI9881C_WriteCmd(dsiDevice, 0x7C, 0x0D);
    ILI9881C_WriteCmd(dsiDevice, 0x7D, 0x06);
    ILI9881C_WriteCmd(dsiDevice, 0x7E, 0x02);
    ILI9881C_WriteCmd(dsiDevice, 0x7F, 0x07);
    ILI9881C_WriteCmd(dsiDevice, 0x80, 0x02);
    ILI9881C_WriteCmd(dsiDevice, 0x81, 0x02);
    ILI9881C_WriteCmd(dsiDevice, 0x83, 0x02);
    ILI9881C_WriteCmd(dsiDevice, 0x84, 0x02);
    ILI9881C_WriteCmd(dsiDevice, 0x85, 0x02);
    ILI9881C_WriteCmd(dsiDevice, 0x86, 0x02);
    ILI9881C_WriteCmd(dsiDevice, 0x87, 0x02);
    ILI9881C_WriteCmd(dsiDevice, 0x88, 0x02);
    ILI9881C_WriteCmd(dsiDevice, 0x89, 0x02);
    ILI9881C_WriteCmd(dsiDevice, 0x8A, 0x02);

#if 0  /* TEST 59+: Page 4 power — enable if needed */
    /* Page 4: Datasheet defaults (p120) for power control. */
    status = ILI9881C_SwitchPage(dsiDevice, 4);
    if (status != kStatus_Success) return status;

    ILI9881C_WriteCmd(dsiDevice, 0x69, 0xD7);  /* Power Control 1 */
    ILI9881C_WriteCmd(dsiDevice, 0x6C, 0x15);  /* VCORE */
    ILI9881C_WriteCmd(dsiDevice, 0x6E, 0x6A);  /* Power Control 2 / VGH clamp */
    ILI9881C_WriteCmd(dsiDevice, 0x6F, 0x34);  /* Power Control 3 / VGH+VGL step-up */
    ILI9881C_WriteCmd(dsiDevice, 0x8D, 0x14);  /* Power Control 4 / VGL clamp */
#endif /* Page 4 power */

    /* Page 1: NL + panel mode + VREG (test 60) */
    status = ILI9881C_SwitchPage(dsiDevice, 1);
    if (status != kStatus_Success) return status;

    ILI9881C_WriteCmd(dsiDevice, 0x22, 0x0A);  /* BGR, SS, DSI video mode (required — NVM=0x30) */
    ILI9881C_WriteCmd(dsiDevice, 0x2E, 0xC8);  /* NL=1280 gate lines (required — NVM=0xB4/1200) */

#if 0  /* TEST 62: Column inversion — enable only if wrong inv behaviour */
    ILI9881C_WriteCmd(dsiDevice, 0x31, 0x00);  /* Column inversion */
#endif

    /* VREG — added for test 59 (test 58 had vertical bars + whining + shutdown) */
    ILI9881C_WriteCmd(dsiDevice, 0x50, 0xB7);  /* VREG1OUT */
    ILI9881C_WriteCmd(dsiDevice, 0x51, 0xB7);  /* VREG2OUT */

#if 0  /* VCOM — NVM factory-calibrated value used. Only override if needed. */
    ILI9881C_WriteCmd(dsiDevice, 0x53, 0x96);  /* VCOM1 */
    ILI9881C_WriteCmd(dsiDevice, 0x55, 0x96);  /* VCOM2 */
#endif

#if 0  /* Pump clocks — datasheet defaults */
    ILI9881C_WriteCmd(dsiDevice, 0x40, 0x33);  /* Pump clock A */
    ILI9881C_WriteCmd(dsiDevice, 0x41, 0x33);  /* Pump clock B */
    ILI9881C_WriteCmd(dsiDevice, 0x42, 0x44);  /* Pump clock C */
    ILI9881C_WriteCmd(dsiDevice, 0x43, 0x55);  /* Pump clock D */
#endif

#if 0  /* Source timing — datasheet defaults */
    ILI9881C_WriteCmd(dsiDevice, 0x60, 0x14);  /* SDT */
    ILI9881C_WriteCmd(dsiDevice, 0x61, 0x00);  /* CRT */
    ILI9881C_WriteCmd(dsiDevice, 0x62, 0x19);  /* EQT */
    ILI9881C_WriteCmd(dsiDevice, 0x63, 0x10);  /* PCT */
#endif

#if 0  /* Gamma — kd050hdfia020 */
    /* Positive gamma */
    ILI9881C_WriteCmd(dsiDevice, 0xA0, 0x08);
    ILI9881C_WriteCmd(dsiDevice, 0xA1, 0x1A);
    ILI9881C_WriteCmd(dsiDevice, 0xA2, 0x27);
    ILI9881C_WriteCmd(dsiDevice, 0xA3, 0x15);
    ILI9881C_WriteCmd(dsiDevice, 0xA4, 0x17);
    ILI9881C_WriteCmd(dsiDevice, 0xA5, 0x2A);
    ILI9881C_WriteCmd(dsiDevice, 0xA6, 0x1E);
    ILI9881C_WriteCmd(dsiDevice, 0xA7, 0x1F);
    ILI9881C_WriteCmd(dsiDevice, 0xA8, 0x8B);
    ILI9881C_WriteCmd(dsiDevice, 0xA9, 0x1B);
    ILI9881C_WriteCmd(dsiDevice, 0xAA, 0x27);
    ILI9881C_WriteCmd(dsiDevice, 0xAB, 0x78);
    ILI9881C_WriteCmd(dsiDevice, 0xAC, 0x18);
    ILI9881C_WriteCmd(dsiDevice, 0xAD, 0x18);
    ILI9881C_WriteCmd(dsiDevice, 0xAE, 0x4C);
    ILI9881C_WriteCmd(dsiDevice, 0xAF, 0x21);
    ILI9881C_WriteCmd(dsiDevice, 0xB0, 0x27);
    ILI9881C_WriteCmd(dsiDevice, 0xB1, 0x54);
    ILI9881C_WriteCmd(dsiDevice, 0xB2, 0x67);
    ILI9881C_WriteCmd(dsiDevice, 0xB3, 0x39);

    /* Negative gamma */
    ILI9881C_WriteCmd(dsiDevice, 0xC0, 0x08);
    ILI9881C_WriteCmd(dsiDevice, 0xC1, 0x1A);
    ILI9881C_WriteCmd(dsiDevice, 0xC2, 0x27);
    ILI9881C_WriteCmd(dsiDevice, 0xC3, 0x15);
    ILI9881C_WriteCmd(dsiDevice, 0xC4, 0x17);
    ILI9881C_WriteCmd(dsiDevice, 0xC5, 0x2A);
    ILI9881C_WriteCmd(dsiDevice, 0xC6, 0x1E);
    ILI9881C_WriteCmd(dsiDevice, 0xC7, 0x1F);
    ILI9881C_WriteCmd(dsiDevice, 0xC8, 0x8B);
    ILI9881C_WriteCmd(dsiDevice, 0xC9, 0x1B);
    ILI9881C_WriteCmd(dsiDevice, 0xCA, 0x27);
    ILI9881C_WriteCmd(dsiDevice, 0xCB, 0x78);
    ILI9881C_WriteCmd(dsiDevice, 0xCC, 0x18);
    ILI9881C_WriteCmd(dsiDevice, 0xCD, 0x18);
    ILI9881C_WriteCmd(dsiDevice, 0xCE, 0x4C);
    ILI9881C_WriteCmd(dsiDevice, 0xCF, 0x21);
    ILI9881C_WriteCmd(dsiDevice, 0xD0, 0x27);
    ILI9881C_WriteCmd(dsiDevice, 0xD1, 0x54);
    ILI9881C_WriteCmd(dsiDevice, 0xD2, 0x67);
    ILI9881C_WriteCmd(dsiDevice, 0xD3, 0x39);
#endif /* Gamma */

    /* Page 0: return to default page */
    status = ILI9881C_SwitchPage(dsiDevice, 0);
    if (status != kStatus_Success) return status;

    return kStatus_Success;
}

status_t ILI9881C_Init(display_handle_t *handle, const display_config_t *config)
{
    const ili9881c_resource_t *resource = (const ili9881c_resource_t *)(handle->resource);
    mipi_dsi_device_t *dsiDevice = resource->dsiDevice;
    status_t status;
    /* Verify resolution */
    if (config->resolution != FSL_VIDEO_RESOLUTION(720, 1280))
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
        /* Skip SW reset — we write all datasheet defaults explicitly in
         * ILI9881C_InitSequence(), so NVM values get overridden anyway.
         * SW reset via GenericWrite/DCS_Write hangs the DSI link. */
        ILI9881C_DelayMs(10);
    }

    /* Minimal init: GIP + NL + panel mode + VREG + VCOM. No Page4/gamma/pump/SDT. */
    status = ILI9881C_InitSequence(dsiDevice);
    if (status != kStatus_Success) return status;

    /* Exit sleep mode — use DCS write for standard DCS commands */
    uint8_t sleepOutCmd = MIPI_DCS_EXIT_SLEEP_MODE;
    MIPI_DSI_DCS_Write(dsiDevice, &sleepOutCmd, 1);

    /* Wait for display to wake up (spec requires 120ms after sleep out) */
    ILI9881C_DelayMs(120);

    /* Display On command is sent in ILI9881C_Start() */

    return kStatus_Success;
}

status_t ILI9881C_Deinit(display_handle_t *handle)
{
    const ili9881c_resource_t *resource = (const ili9881c_resource_t *)(handle->resource);
    mipi_dsi_device_t *dsiDevice = resource->dsiDevice;

    /* Turn off display — use DCS write for standard DCS commands */
    uint8_t displayOffCmd = MIPI_DCS_SET_DISPLAY_OFF;
    MIPI_DSI_DCS_Write(dsiDevice, &displayOffCmd, 1);
    ILI9881C_DelayMs(20);

    /* Enter sleep mode */
    uint8_t sleepInCmd = MIPI_DCS_ENTER_SLEEP_MODE;
    MIPI_DSI_DCS_Write(dsiDevice, &sleepInCmd, 1);
    ILI9881C_DelayMs(120);

    return kStatus_Success;
}

status_t ILI9881C_Start(display_handle_t *handle)
{
    /* Send Display ON after LCDIF starts HS video (standard NXP pattern).
     * The DSI bridge handles LP commands during HS blanking intervals. */
    const ili9881c_resource_t *resource = (const ili9881c_resource_t *)(handle->resource);
    mipi_dsi_device_t *dsiDevice = resource->dsiDevice;

    uint8_t displayOnCmd = MIPI_DCS_SET_DISPLAY_ON;
    status_t status = MIPI_DSI_DCS_Write(dsiDevice, &displayOnCmd, 1);
    ILI9881C_PRINTF("ILI9881C: Display ON (in Start, after HS) status=%d\r\n", (int)status);

    return status;
}

status_t ILI9881C_Stop(display_handle_t *handle)
{
    const ili9881c_resource_t *resource = (const ili9881c_resource_t *)(handle->resource);
    mipi_dsi_device_t *dsiDevice = resource->dsiDevice;

    /* Turn off display */
    uint8_t displayOffCmd = MIPI_DCS_SET_DISPLAY_OFF;
    MIPI_DSI_DCS_Write(dsiDevice, &displayOffCmd, 1);
    ILI9881C_DelayMs(20);

    return kStatus_Success;
}
