/*
 * Copyright 2019-2021, 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "display_support.h"
#include "fsl_gpio.h"
#include "fsl_mipi_dsi.h"
#include <string.h>
// Include all panel drivers for runtime selection
#include "fsl_rm68200.h"
#include "fsl_rm68191.h"
#include "fsl_hx8394.h"
#include "rpi.h"
#include "pca6416.h"
#include "pca9530.h"
#include "pin_mux.h"
#include "board.h"

#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)
#include "fsl_dc_fb_lcdifv2.h"
#else
#include "fsl_dc_fb_elcdif.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)

#define DEMO_LCDIF_POL_FLAGS                                                             \
    (kLCDIFV2_DataEnableActiveHigh | kLCDIFV2_VsyncActiveLow | kLCDIFV2_HsyncActiveLow | \
     kLCDIFV2_DriveDataOnFallingClkEdge)

#define DEMO_LCDIF LCDIFV2

#else

#define DEMO_LCDIF_POL_FLAGS \
    (kELCDIF_DataEnableActiveHigh | kELCDIF_VsyncActiveLow | kELCDIF_HsyncActiveLow | kELCDIF_DriveDataOnFallingClkEdge)

#define DEMO_LCDIF LCDIF

#endif

/* Definitions for MIPI. */
#define DEMO_MIPI_DSI (&g_mipiDsi)

/*
 * The DPHY bit clock must be fast enough to send out the pixels, it should be
 * larger than:
 *
 *         (Pixel clock * bit per output pixel) / number of MIPI data lane
 *
 * Here the desired DPHY bit clock multiplied by ( 9 / 8 = 1.125) to ensure
 * it is fast enough.
 */
#define DEMO_MIPI_DPHY_BIT_CLK_ENLARGE(origin) (((origin) / 8) * 9)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void BOARD_PullPanelResetPin(bool pullUp);
static void BOARD_PullPanelPowerPin(bool pullUp);
static void BOARD_InitLcdifClock(void);
static void BOARD_InitMipiDsiClock(void);
static status_t BOARD_DSI_Transfer(dsi_transfer_t *xfer);

/*******************************************************************************
 * Variables
 ******************************************************************************/

// Runtime panel configuration (required - no backward compatibility)
static const panel_config_t *g_runtime_panel_config = NULL;

// Helper macros to get panel parameters from runtime config
#define PANEL_WIDTH (g_runtime_panel_config->width)
#define PANEL_HEIGHT (g_runtime_panel_config->height)
#define PANEL_HSW (g_runtime_panel_config->hsw)
#define PANEL_HFP (g_runtime_panel_config->hfp)
#define PANEL_HBP (g_runtime_panel_config->hbp)
#define PANEL_VSW (g_runtime_panel_config->vsw)
#define PANEL_VFP (g_runtime_panel_config->vfp)
#define PANEL_VBP (g_runtime_panel_config->vbp)
#define PANEL_DSI_LANES (g_runtime_panel_config->dsi_lanes)

static uint32_t mipiDsiTxEscClkFreq_Hz;
static uint32_t mipiDsiDphyBitClkFreq_Hz;
static uint32_t mipiDsiDphyRefClkFreq_Hz;
static uint32_t mipiDsiDpiClkFreq_Hz;

const MIPI_DSI_Type g_mipiDsi = {
    .host = DSI_HOST,
    .apb  = DSI_HOST_APB_PKT_IF,
    .dpi  = DSI_HOST_DPI_INTFC,
    .dphy = DSI_HOST_DPHY_INTFC,
};

// All panel handles compiled in for runtime selection
static mipi_dsi_device_t dsiDevice = {
    .virtualChannel = 0,
    .xferFunc       = BOARD_DSI_Transfer,
};

// RK055AHD091 (720x1280, RM68200)
static const rm68200_resource_t rm68200Resource = {
    .dsiDevice    = &dsiDevice,
    .pullResetPin = BOARD_PullPanelResetPin,
    .pullPowerPin = BOARD_PullPanelPowerPin,
};

static display_handle_t rm68200Handle = {
    .resource = &rm68200Resource,
    .ops      = &rm68200_ops,
};

// RK055MHD091 (720x1280, HX8394)
static const hx8394_resource_t hx8394Resource = {
    .dsiDevice    = &dsiDevice,
    .pullResetPin = BOARD_PullPanelResetPin,
    .pullPowerPin = BOARD_PullPanelPowerPin,
};

static display_handle_t hx8394Handle = {
    .resource = &hx8394Resource,
    .ops      = &hx8394_ops,
};

// RPI 7" (800x480)
static const rpi_resource_t rpiResource = {
    .dsiDevice    = &dsiDevice,
    .pullResetPin = BOARD_PullPanelResetPin,
    .pullPowerPin = BOARD_PullPanelPowerPin,
};

static display_handle_t rpiHandle = {
    .resource = &rpiResource,
    .ops      = &rpi_ops,
};

// RK055IQH091 (540x960, RM68191)
static const rm68191_resource_t rm68191Resource = {
    .dsiDevice    = &dsiDevice,
    .pullResetPin = BOARD_PullPanelResetPin,
    .pullPowerPin = BOARD_PullPanelPowerPin,
};

static display_handle_t rm68191Handle = {
    .resource = &rm68191Resource,
    .ops      = &rm68191_ops,
};

#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)

static dc_fb_lcdifv2_handle_t s_dcFbLcdifv2Handle = {0};

static dc_fb_lcdifv2_config_t s_dcFbLcdifv2Config = {
    .lcdifv2       = DEMO_LCDIF,
    .width         = DEMO_BUFFER_WIDTH,   // Default, updated at runtime
    .height        = DEMO_BUFFER_HEIGHT,  // Default, updated at runtime
    .hsw           = 6,    // Default timing (RK055MHD091), updated at runtime
    .hfp           = 12,
    .hbp           = 24,
    .vsw           = 2,
    .vfp           = 16,
    .vbp           = 14,
    .polarityFlags = DEMO_LCDIF_POL_FLAGS,
    .lineOrder     = kLCDIFV2_LineOrderRGB,
/* CM4 is domain 1, CM7 is domain 0. */
#if (__CORTEX_M <= 4)
    .domain = 1,
#else
    .domain = 0,
#endif
};

const dc_fb_t g_dc = {
    .ops     = &g_dcFbOpsLcdifv2,
    .prvData = &s_dcFbLcdifv2Handle,
    .config  = &s_dcFbLcdifv2Config,
};

#else

dc_fb_elcdif_handle_t s_dcFbElcdifHandle = {0}; /* The handle must be initialized to 0. */

dc_fb_elcdif_config_t s_dcFbElcdifConfig = {
    .elcdif        = DEMO_LCDIF,
    .width         = DEMO_BUFFER_WIDTH,   // Default, updated at runtime
    .height        = DEMO_BUFFER_HEIGHT,  // Default, updated at runtime
    .hsw           = 6,    // Default timing (RK055MHD091), updated at runtime
    .hfp           = 12,
    .hbp           = 24,
    .vsw           = 2,
    .vfp           = 16,
    .vbp           = 14,
    .polarityFlags = DEMO_LCDIF_POL_FLAGS,
    .dataBus       = kELCDIF_DataBus24Bit,
};

const dc_fb_t g_dc = {
    .ops     = &g_dcFbOpsElcdif,
    .prvData = &s_dcFbElcdifHandle,
    .config  = &s_dcFbElcdifConfig,
};
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

static void BOARD_PullPanelResetPin(bool pullUp)
{
    // Skip for RPI panel - Python handles all I2C (PCA6416)
    if (PANEL_DSI_LANES == 1) {
        return;
    }

    // Non-RPI panels use direct GPIO
    GPIO_PinWrite(BOARD_MIPI_PANEL_RST_GPIO, BOARD_MIPI_PANEL_RST_PIN, pullUp ? 1 : 0);
}

static void BOARD_PullPanelPowerPin(bool pullUp)
{
    // Skip for RPI panel - Python handles all I2C (PCA6416)
    if (PANEL_DSI_LANES == 1) {
        return;
    }

    // Non-RPI panels use direct GPIO
    GPIO_PinWrite(BOARD_MIPI_PANEL_POWER_GPIO, BOARD_MIPI_PANEL_POWER_PIN, pullUp ? 1 : 0);
}

static status_t BOARD_DSI_Transfer(dsi_transfer_t *xfer)
{
    return DSI_TransferBlocking(DEMO_MIPI_DSI, xfer);
}

static void BOARD_InitLcdifClock(void)
{
    /*
     * The pixel clock is (height + VSW + VFP + VBP) * (width + HSW + HFP + HBP) * frame rate.
     *
     * For 60Hz frame rate, the RK055IQH091 pixel clock should be 36MHz.
     * the RK055AHD091 pixel clock should be 62MHz.
     * the RPI 7" pixel clock should be 26MHz.
     *
     * Clock divider calculation (from PLL_528):
     * - 720x1280 panels: div=9  (58.7MHz)
     * - 540x960 panels:  div=15 (35.2MHz)
     * - 800x480 panels:  div=20 (26.4MHz)
     */
    // Calculate divider based on panel resolution
    uint32_t pixel_count = PANEL_WIDTH * PANEL_HEIGHT;
    uint8_t clock_div;

    if (pixel_count >= 700000) {
        clock_div = 9;  // Large panels (720x1280 = 921k pixels)
    } else if (pixel_count >= 400000) {
        clock_div = 15; // Medium panels (540x960 = 518k pixels)
    } else {
        clock_div = 20; // Small panels (800x480 = 384k pixels)
    }

    const clock_root_config_t lcdifClockConfig = {
        .clockOff = false,
        .mux      = 4, /*!< PLL_528. */
        .div      = clock_div,
    };

#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)
    CLOCK_SetRootClock(kCLOCK_Root_Lcdifv2, &lcdifClockConfig);

    mipiDsiDpiClkFreq_Hz = CLOCK_GetRootClockFreq(kCLOCK_Root_Lcdifv2);

#else

    CLOCK_SetRootClock(kCLOCK_Root_Lcdif, &lcdifClockConfig);

    mipiDsiDpiClkFreq_Hz = CLOCK_GetRootClockFreq(kCLOCK_Root_Lcdif);
#endif
}

static void BOARD_InitMipiDsiClock(void)
{
    uint32_t mipiDsiEscClkFreq_Hz;

    /* RxClkEsc max 60MHz, TxClkEsc 12 to 20MHz. */
    /* RxClkEsc = 528MHz / 11 = 48MHz. */
    /* TxClkEsc = 528MHz / 11 / 4 = 16MHz. */
    const clock_root_config_t mipiEscClockConfig = {
        .clockOff = false,
        .mux      = 4, /*!< PLL_528. */
        .div      = 11,
    };

    CLOCK_SetRootClock(kCLOCK_Root_Mipi_Esc, &mipiEscClockConfig);

    mipiDsiEscClkFreq_Hz = CLOCK_GetRootClockFreq(kCLOCK_Root_Mipi_Esc);

    const clock_group_config_t mipiEscClockGroupConfig = {
        .clockOff = false, .resetDiv = 2, .div0 = 2, /* TX esc clock. */
    };

    CLOCK_SetGroupConfig(kCLOCK_Group_MipiDsi, &mipiEscClockGroupConfig);

    mipiDsiTxEscClkFreq_Hz = mipiDsiEscClkFreq_Hz / 3;

    /* DPHY reference clock, use OSC 24MHz clock. */
    const clock_root_config_t mipiDphyRefClockConfig = {
        .clockOff = false,
        .mux      = 1, /*!< OSC_24M. */
        .div      = 1,
    };

    CLOCK_SetRootClock(kCLOCK_Root_Mipi_Ref, &mipiDphyRefClockConfig);

    mipiDsiDphyRefClkFreq_Hz = BOARD_XTAL0_CLK_HZ;
}

static status_t BOARD_InitLcdPanel(void)
{
    status_t status;
    const display_config_t displayConfig = {
        .resolution   = FSL_VIDEO_RESOLUTION(PANEL_WIDTH, PANEL_HEIGHT),
        .hsw          = PANEL_HSW,
        .hfp          = PANEL_HFP,
        .hbp          = PANEL_HBP,
        .vsw          = PANEL_VSW,
        .vfp          = PANEL_VFP,
        .vbp          = PANEL_VBP,
        .controlFlags = 0,
        .dsiLanes     = PANEL_DSI_LANES,
    };

    // Runtime panel selection based on config->name
    if (g_runtime_panel_config == NULL || g_runtime_panel_config->name == NULL) {
        // Runtime configuration is required
        return kStatus_InvalidArgument;
    }

    const char *panel_name = g_runtime_panel_config->name;

    if (strcmp(panel_name, "rpi_7inch") == 0) {
        status = RPI_Init(&rpiHandle, &displayConfig);
    } else if (strcmp(panel_name, "rk055ahd091") == 0) {
        // Init GPIO pins for non-RPI panels
        const gpio_pin_config_t pinConfig = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
        GPIO_PinInit(BOARD_MIPI_PANEL_POWER_GPIO, BOARD_MIPI_PANEL_POWER_PIN, &pinConfig);
        GPIO_PinInit(BOARD_MIPI_PANEL_BL_GPIO, BOARD_MIPI_PANEL_BL_PIN, &pinConfig);
        GPIO_PinInit(BOARD_MIPI_PANEL_RST_GPIO, BOARD_MIPI_PANEL_RST_PIN, &pinConfig);

        status = RM68200_Init(&rm68200Handle, &displayConfig);

        if (status == kStatus_Success) {
            GPIO_PinWrite(BOARD_MIPI_PANEL_BL_GPIO, BOARD_MIPI_PANEL_BL_PIN, 1);
        }
    } else if (strcmp(panel_name, "rk055mhd091") == 0) {
        const gpio_pin_config_t pinConfig = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
        GPIO_PinInit(BOARD_MIPI_PANEL_POWER_GPIO, BOARD_MIPI_PANEL_POWER_PIN, &pinConfig);
        GPIO_PinInit(BOARD_MIPI_PANEL_BL_GPIO, BOARD_MIPI_PANEL_BL_PIN, &pinConfig);
        GPIO_PinInit(BOARD_MIPI_PANEL_RST_GPIO, BOARD_MIPI_PANEL_RST_PIN, &pinConfig);

        status = HX8394_Init(&hx8394Handle, &displayConfig);

        if (status == kStatus_Success) {
            GPIO_PinWrite(BOARD_MIPI_PANEL_BL_GPIO, BOARD_MIPI_PANEL_BL_PIN, 1);
        }
    } else if (strcmp(panel_name, "rk055iqh091") == 0) {
        const gpio_pin_config_t pinConfig = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
        GPIO_PinInit(BOARD_MIPI_PANEL_POWER_GPIO, BOARD_MIPI_PANEL_POWER_PIN, &pinConfig);
        GPIO_PinInit(BOARD_MIPI_PANEL_BL_GPIO, BOARD_MIPI_PANEL_BL_PIN, &pinConfig);
        GPIO_PinInit(BOARD_MIPI_PANEL_RST_GPIO, BOARD_MIPI_PANEL_RST_PIN, &pinConfig);

        status = RM68191_Init(&rm68191Handle, &displayConfig);

        if (status == kStatus_Success) {
            GPIO_PinWrite(BOARD_MIPI_PANEL_BL_GPIO, BOARD_MIPI_PANEL_BL_PIN, 1);
        }
    } else {
        // Unknown panel name
        PRINTF("Error: Unknown panel name '%s'\r\n", panel_name);
        PRINTF("Supported panels: rpi_7inch, rk055ahd091, rk055mhd091, rk055iqh091\r\n");
        return kStatus_InvalidArgument;
    }

    return status;
}

status_t BOARD_DeinitLcdPanel(void) {
    // TODO missing full support to match init. Might also need a reset
    status_t status;

    if (g_runtime_panel_config == NULL || g_runtime_panel_config->name == NULL) {
        return kStatus_InvalidArgument;
    }

    const char *panel_name = g_runtime_panel_config->name;

    if (strcmp(panel_name, "rpi_7inch") == 0) {
        status = RPI_Deinit(&rpiHandle);
    } else if (strcmp(panel_name, "rk055ahd091") == 0) {
        status = RM68200_Deinit(&rm68200Handle);
    } else if (strcmp(panel_name, "rk055mhd091") == 0) {
        status = HX8394_Deinit(&hx8394Handle);
    } else if (strcmp(panel_name, "rk055iqh091") == 0) {
        status = RM68191_Deinit(&rm68191Handle);
    } else {
        PRINTF("Error: Unknown panel name '%s'\r\n", panel_name);
        return kStatus_InvalidArgument;
    }

    return status;
}

static void BOARD_SetMipiDsiConfig(void)
{
    dsi_config_t dsiConfig;
    dsi_dphy_config_t dphyConfig;

    // Determine if this is a 1-lane panel (RPI 7") based on runtime config
    bool is_single_lane = (PANEL_DSI_LANES == 1);

    const dsi_dpi_config_t dpiConfig = {.pixelPayloadSize = PANEL_WIDTH,
                                        .dpiColorCoding   = kDSI_Dpi24Bit,
                                        .pixelPacket      = kDSI_PixelPacket24Bit,
                                        .videoMode        = is_single_lane ? kDSI_DpiNonBurstWithSyncPulse : kDSI_DpiBurst,
                                        .bllpMode         = kDSI_DpiBllpLowPower,
                                        .polarityFlags    = kDSI_DpiVsyncActiveLow | kDSI_DpiHsyncActiveLow,
                                        .hfp              = PANEL_HFP,
                                        .hbp              = PANEL_HBP,
                                        .hsw              = PANEL_HSW,
                                        .vfp              = PANEL_VFP,
                                        .vbp              = PANEL_VBP,
                                        .panelHeight      = PANEL_HEIGHT,
                                        .virtualChannel   = 0};

    /*
     * dsiConfig.numLanes = 4;
     * dsiConfig.enableNonContinuousHsClk = false;
     * dsiConfig.autoInsertEoTp = true;
     * dsiConfig.numExtraEoTp = 0;
     * dsiConfig.htxTo_ByteClk = 0;
     * dsiConfig.lrxHostTo_ByteClk = 0;
     * dsiConfig.btaTo_ByteClk = 0;
     */
    DSI_GetDefaultConfig(&dsiConfig);
    dsiConfig.numLanes = PANEL_DSI_LANES;

    if (is_single_lane) {
        // RPI 7" specific settings
        dsiConfig.autoInsertEoTp = false;
        dsiConfig.enableNonContinuousHsClk = false;
    } else {
        dsiConfig.autoInsertEoTp = true;
    }

    /* Init the DSI module. */
    DSI_Init(DEMO_MIPI_DSI, &dsiConfig);

    /* Init DPHY.
     *
     * The DPHY bit clock must be fast enough to send out the pixels, it should be
     * larger than:
     *
     *         (Pixel clock * bit per output pixel) / number of MIPI data lane
     *
     * Here the desired DPHY bit clock multiplied by ( 9 / 8 = 1.125) to ensure
     * it is fast enough.
     *
     * Note that the DSI output pixel is 24bit per pixel.
     */
    mipiDsiDphyBitClkFreq_Hz = mipiDsiDpiClkFreq_Hz * (24 / PANEL_DSI_LANES);

    // Don't enlarge bit clock for single-lane panels (RPI 7")
    if (!is_single_lane) {
        mipiDsiDphyBitClkFreq_Hz = DEMO_MIPI_DPHY_BIT_CLK_ENLARGE(mipiDsiDphyBitClkFreq_Hz);
    }

    DSI_GetDphyDefaultConfig(&dphyConfig, mipiDsiDphyBitClkFreq_Hz, mipiDsiTxEscClkFreq_Hz);

    mipiDsiDphyBitClkFreq_Hz = DSI_InitDphy(DEMO_MIPI_DSI, &dphyConfig, mipiDsiDphyRefClkFreq_Hz);

    /* Init DPI interface. */
    DSI_SetDpiConfig(DEMO_MIPI_DSI, &dpiConfig, PANEL_DSI_LANES, mipiDsiDpiClkFreq_Hz, mipiDsiDphyBitClkFreq_Hz);
}

status_t BOARD_InitDisplayInterface(void)
{
    CLOCK_EnableClock(kCLOCK_Video_Mux);

#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)
    /* LCDIF v2 output to MIPI DSI. */
    VIDEO_MUX->VID_MUX_CTRL.SET = VIDEO_MUX_VID_MUX_CTRL_MIPI_DSI_SEL_MASK;
#else
    /* ELCDIF output to MIPI DSI. */
    VIDEO_MUX->VID_MUX_CTRL.CLR = VIDEO_MUX_VID_MUX_CTRL_MIPI_DSI_SEL_MASK;
#endif

    /* 1. Power on and isolation off. */
    PGMC_BPC4->BPC_POWER_CTRL |= (PGMC_BPC_BPC_POWER_CTRL_PSW_ON_SOFT_MASK | PGMC_BPC_BPC_POWER_CTRL_ISO_OFF_SOFT_MASK);

    /* 2. Assert MIPI reset. */
    IOMUXC_GPR->GPR62 &=
        ~(IOMUXC_GPR_GPR62_MIPI_DSI_PCLK_SOFT_RESET_N_MASK | IOMUXC_GPR_GPR62_MIPI_DSI_ESC_SOFT_RESET_N_MASK |
          IOMUXC_GPR_GPR62_MIPI_DSI_BYTE_SOFT_RESET_N_MASK | IOMUXC_GPR_GPR62_MIPI_DSI_DPI_SOFT_RESET_N_MASK);

    /* 3. Setup clock. */
    BOARD_InitMipiDsiClock();

    /* 4. Deassert PCLK and ESC reset. */
    IOMUXC_GPR->GPR62 |=
        (IOMUXC_GPR_GPR62_MIPI_DSI_PCLK_SOFT_RESET_N_MASK | IOMUXC_GPR_GPR62_MIPI_DSI_ESC_SOFT_RESET_N_MASK);

    /* 5. Configures peripheral. */
    BOARD_SetMipiDsiConfig();

    /* 6. Deassert BYTE and DBI reset. */
    IOMUXC_GPR->GPR62 |=
        (IOMUXC_GPR_GPR62_MIPI_DSI_BYTE_SOFT_RESET_N_MASK | IOMUXC_GPR_GPR62_MIPI_DSI_DPI_SOFT_RESET_N_MASK);

    /* 7. Configure the panel. */
    return BOARD_InitLcdPanel();
}

#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)
void LCDIFv2_IRQHandler(void)
{
    DC_FB_LCDIFV2_IRQHandler(&g_dc);
}
#else
void eLCDIF_IRQHandler(void)
{
    DC_FB_ELCDIF_IRQHandler(&g_dc);
}
#endif

void BOARD_InitDisplayWithConfig(const panel_config_t *config)
{
    // Store runtime configuration
    g_runtime_panel_config = config;

    // Update dc_fb config structures with runtime values
#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)
    s_dcFbLcdifv2Config.width = config->width;
    s_dcFbLcdifv2Config.height = config->height;
    s_dcFbLcdifv2Config.hsw = config->hsw;
    s_dcFbLcdifv2Config.hfp = config->hfp;
    s_dcFbLcdifv2Config.hbp = config->hbp;
    s_dcFbLcdifv2Config.vsw = config->vsw;
    s_dcFbLcdifv2Config.vfp = config->vfp;
    s_dcFbLcdifv2Config.vbp = config->vbp;
#else
    s_dcFbElcdifConfig.width = config->width;
    s_dcFbElcdifConfig.height = config->height;
    s_dcFbElcdifConfig.hsw = config->hsw;
    s_dcFbElcdifConfig.hfp = config->hfp;
    s_dcFbElcdifConfig.hbp = config->hbp;
    s_dcFbElcdifConfig.vsw = config->vsw;
    s_dcFbElcdifConfig.vfp = config->vfp;
    s_dcFbElcdifConfig.vbp = config->vbp;
#endif
}

status_t BOARD_VerifyDisplayClockSource(void)
{
    status_t status;
    uint32_t srcClkFreq;

    /*
     * In this implementation, the SYSPLL2 (528M) clock is used as the source
     * of LCDIFV2 pixel clock and MIPI DSI ESC clock. The OSC24M clock is used
     * as the MIPI DSI DPHY PLL reference clock. This function checks the clock
     * source are valid. OSC24M is always valid, so only verify the SYSPLL2.
     */
    srcClkFreq = CLOCK_GetPllFreq(kCLOCK_PllSys2);
    if (528 != (srcClkFreq / 1000000))
    {
        status = kStatus_Fail;
    }
    else
    {
        status = kStatus_Success;
    }

    return status;
}

status_t BOARD_PrepareDisplayController(void)
{
    status_t status;

    status = BOARD_VerifyDisplayClockSource();

    if (status != kStatus_Success)
    {
        PRINTF("Error: Invalid display clock source.\r\n");
        return status;
    }

    BOARD_InitLcdifClock();

    status = BOARD_InitDisplayInterface();

    if (kStatus_Success == status)
    {
#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)
        NVIC_ClearPendingIRQ(LCDIFv2_IRQn);
        NVIC_SetPriority(LCDIFv2_IRQn, 3);
        EnableIRQ(LCDIFv2_IRQn);
#else
        NVIC_ClearPendingIRQ(eLCDIF_IRQn);
        NVIC_SetPriority(eLCDIF_IRQn, 3);
        EnableIRQ(eLCDIF_IRQn);
#endif
    }

    return kStatus_Success;
}
