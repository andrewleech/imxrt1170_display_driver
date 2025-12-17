/*
 * Copyright 2019-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _DISPLAY_SUPPORT_H_
#define _DISPLAY_SUPPORT_H_

#include "fsl_dc_fb.h"

#include "py/mphal.h"
#include "py/mpconfig.h"

#include "display_board_config.h"

#define PRINTF(...)   mp_printf(&mp_plat_print, "lvgl: " __VA_ARGS__)

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* @TEST_ANCHOR */

#define DEMO_DISPLAY_CONTROLLER_ELCDIF  0
#define DEMO_DISPLAY_CONTROLLER_LCDIFV2 1

#ifndef DEMO_DISPLAY_CONTROLLER
/* Use LCDIFV2 by default, could use ELCDIF by changing this macro. */
#define DEMO_DISPLAY_CONTROLLER DEMO_DISPLAY_CONTROLLER_LCDIFV2
#endif

#define DEMO_BUFFER_FIXED_ADDRESS 0

#if DEMO_BUFFER_FIXED_ADDRESS
#define DEMO_BUFFER0_ADDR 0x80000000
#define DEMO_BUFFER1_ADDR 0x80200000
#endif

/*
 * Use the MIPI dumb panel
 */

/* Definitions for the frame buffer. */
#define DEMO_BUFFER_COUNT 2 /* 2 is enough for DPI interface display. */

#ifndef DEMO_USE_XRGB8888
#define DEMO_USE_XRGB8888 0
#endif

#if DEMO_USE_XRGB8888
#define DEMO_BUFFER_PIXEL_FORMAT   kVIDEO_PixelFormatXRGB8888
#define DEMO_BUFFER_BYTE_PER_PIXEL 4
#else
#define DEMO_BUFFER_PIXEL_FORMAT   kVIDEO_PixelFormatRGB565
#define DEMO_BUFFER_BYTE_PER_PIXEL 2
#endif

/*
 * Frame buffer size for compile-time allocation.
 * Set to maximum supported panel size to accommodate all panels:
 * - RPI 7": 800x480 (widest)
 * - RK055AHD091/MHD091: 720x1280 (tallest)
 * Actual panel size configured at runtime via panel_config_t.
 */
#define DEMO_BUFFER_WIDTH  (800)
#define DEMO_BUFFER_HEIGHT (1280)

/* Where the frame buffer is shown in the screen. */
#define DEMO_BUFFER_START_X 0U
#define DEMO_BUFFER_START_Y 0U

#define DEMO_BUFFER_STRIDE_BYTE (DEMO_BUFFER_WIDTH * DEMO_BUFFER_BYTE_PER_PIXEL)
/* There is not frame buffer aligned requirement, consider the 64-bit AXI data
 * bus width and 32-byte cache line size, the frame buffer alignment is set to
 * 32 byte.
 */
#define FRAME_BUFFER_ALIGN 32

extern const dc_fb_t g_dc;

/*******************************************************************************
 * Runtime Panel Configuration
 ******************************************************************************/

/**
 * @brief Runtime panel configuration structure
 *
 * Allows panels to be configured at runtime from Python without preprocessor conditionals.
 * Python passes this config to init_with_config() in mpy_api.c.
 */
typedef struct {
    const char *name;      // Panel identifier (e.g. "rpi_7inch", "rk055ahd091")
    uint16_t width;        // Horizontal resolution in pixels
    uint16_t height;       // Vertical resolution in pixels
    uint8_t hsw;           // Horizontal sync width
    uint8_t hfp;           // Horizontal front porch
    uint8_t hbp;           // Horizontal back porch
    uint8_t vsw;           // Vertical sync width
    uint8_t vfp;           // Vertical front porch
    uint8_t vbp;           // Vertical back porch
    uint8_t dsi_lanes;     // Number of DSI lanes (1 or 2)
} panel_config_t;

/**
 * @brief Initialize display with runtime panel configuration
 *
 * Required for display initialization. Panels are configured at runtime
 * from Python with timing and resolution parameters.
 *
 * @param config Panel configuration from Python
 */
void BOARD_InitDisplayWithConfig(const panel_config_t *config);

/**
 * @brief Get current runtime panel configuration
 *
 * @return Pointer to current panel config, or NULL if not initialized
 */
const panel_config_t* BOARD_GetPanelConfig(void);

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

status_t BOARD_PrepareDisplayController(void);
status_t BOARD_DeinitLcdPanel(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _DISPLAY_SUPPORT_H_ */
