/*
 * Copyright 2019-2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "lvgl_support.h"
#include "lvgl.h"
#if defined(SDK_OS_FREE_RTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#endif
#include "board.h"
#include "pin_mux.h"

#include "fsl_gpio.h"
#include "fsl_cache.h"

#include "fsl_gt911.h"

#if LV_USE_GPU_NXP_VG_LITE
#include "vg_lite.h"
#include "vglite_support.h"
#endif

#if LV_USE_GPU_NXP_PXP
#include "draw/nxp/pxp/lv_draw_pxp.h"
#endif

#include "py/runtime.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Ratate panel or not. */
#ifndef DEMO_USE_ROTATE
#if LV_USE_GPU_NXP_PXP
#define DEMO_USE_ROTATE 1
#else
#define DEMO_USE_ROTATE 0
#endif
#endif

/* Cache line size. */
#ifndef FSL_FEATURE_L2CACHE_LINESIZE_BYTE
#define FSL_FEATURE_L2CACHE_LINESIZE_BYTE 0
#endif
#ifndef FSL_FEATURE_L1DCACHE_LINESIZE_BYTE
#define FSL_FEATURE_L1DCACHE_LINESIZE_BYTE 0
#endif

#if (FSL_FEATURE_L2CACHE_LINESIZE_BYTE > FSL_FEATURE_L1DCACHE_LINESIZE_BYTE)
#define DEMO_CACHE_LINE_SIZE FSL_FEATURE_L2CACHE_LINESIZE_BYTE
#else
#define DEMO_CACHE_LINE_SIZE FSL_FEATURE_L1DCACHE_LINESIZE_BYTE
#endif

#if (DEMO_CACHE_LINE_SIZE > FRAME_BUFFER_ALIGN)
#define DEMO_FB_ALIGN DEMO_CACHE_LINE_SIZE
#else
#define DEMO_FB_ALIGN FRAME_BUFFER_ALIGN
#endif

#if (LV_ATTRIBUTE_MEM_ALIGN_SIZE > DEMO_FB_ALIGN)
#undef DEMO_FB_ALIGN
#define DEMO_FB_ALIGN LV_ATTRIBUTE_MEM_ALIGN_SIZE
#endif

#define DEMO_FB_SIZE \
    (((DEMO_BUFFER_WIDTH * DEMO_BUFFER_HEIGHT * LCD_FB_BYTE_PER_PIXEL) + DEMO_FB_ALIGN - 1) & ~(DEMO_FB_ALIGN - 1))

#if DEMO_USE_ROTATE
#define LVGL_BUFFER_WIDTH  DEMO_BUFFER_HEIGHT
#define LVGL_BUFFER_HEIGHT DEMO_BUFFER_WIDTH
#else
#define LVGL_BUFFER_WIDTH  DEMO_BUFFER_WIDTH
#define LVGL_BUFFER_HEIGHT DEMO_BUFFER_HEIGHT
#endif

#if __CORTEX_M == 4
#define DEMO_FLUSH_DCACHE() L1CACHE_CleanInvalidateSystemCache()
#else
#define DEMO_FLUSH_DCACHE() SCB_CleanInvalidateDCache()
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void DEMO_FlushDisplay(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map);

#if (LV_USE_GPU_NXP_VG_LITE || LV_USE_GPU_NXP_PXP)
static void DEMO_CleanInvalidateCache(lv_display_t * disp);
#endif

static void DEMO_InitTouch(void);

static void DEMO_ReadTouch(lv_indev_t * indev_drv, lv_indev_data_t * data);

static void DEMO_BufferSwitchOffCallback(void *param, void *switchOffBuffer);

static void BOARD_PullMIPIPanelTouchResetPin(bool pullUp);

static void BOARD_ConfigMIPIPanelTouchIntPin(gt911_int_pin_mode_t mode);

static void DEMO_WaitBufferSwitchOff(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

#define DYNAMIC_FB_ALLOC 1

#if DYNAMIC_FB_ALLOC
MP_REGISTER_ROOT_POINTER(uint8_t *s_frameBuffer_alloc);  // malloc output goes here
uint8_t (*s_frameBuffer)[DEMO_FB_SIZE];  // this holds aligned framebuffer pointer for use

#if DEMO_USE_ROTATE
MP_REGISTER_ROOT_POINTER(uint8_t *s_lvglBuffer_alloc);
uint8_t (*s_lvglBuffer)[DEMO_FB_SIZE];
#endif

#else
SDK_ALIGN(static uint8_t __attribute__((section(".sdram"))) s_frameBuffer[2][DEMO_FB_SIZE], DEMO_FB_ALIGN);
#if DEMO_USE_ROTATE
SDK_ALIGN(static uint8_t __attribute__((section(".sdram"))) s_lvglBuffer[1][DEMO_FB_SIZE], DEMO_FB_ALIGN);
#endif
#endif

#if defined(SDK_OS_FREE_RTOS)
static SemaphoreHandle_t s_transferDone;
#else
static volatile bool s_transferDone;
static bool s_touchInitialized = false;
static volatile uint32_t s_vsyncCount = 0;
#endif

#if DEMO_USE_ROTATE
/*
 * When rotate is used, LVGL stack draws in one buffer (s_lvglBuffer), and LCD
 * driver uses two buffers (s_frameBuffer) to remove tearing effect.
 */
static void *volatile s_inactiveFrameBuffer;
#endif

static gt911_handle_t s_touchHandle;
static const gt911_config_t s_touchConfig = {
    .I2C_SendFunc     = BOARD_MIPIPanelTouch_I2C_Send,
    .I2C_ReceiveFunc  = BOARD_MIPIPanelTouch_I2C_Receive,
    .pullResetPinFunc = BOARD_PullMIPIPanelTouchResetPin,
    .intPinFunc       = BOARD_ConfigMIPIPanelTouchIntPin,
    .timeDelayMsFunc  = VIDEO_DelayMs,
    .touchPointNum    = 1,
    .i2cAddrMode      = kGT911_I2cAddrMode0,
    .intTrigMode      = kGT911_IntRisingEdge,
};
static int s_touchResolutionX;
static int s_touchResolutionY;

/*******************************************************************************
 * Code
 ******************************************************************************/

static bool s_lvgl_initialized = false;
static bool s_disp_initialized = false;

void lv_port_pre_init(void) {
}

void lv_port_disp_init(void) {

    /* Guard against multiple initialization */
    if (s_disp_initialized) {
        PRINTF("Display already initialized, skipping\r\n");
        return;
    }

    PRINTF("Display config: %dx%d, %d BPP, FB size: %d bytes\r\n",
           DEMO_BUFFER_WIDTH, DEMO_BUFFER_HEIGHT, DEMO_BUFFER_BYTE_PER_PIXEL, DEMO_FB_SIZE);

    /* Initialize LVGL library */
    if (!s_lvgl_initialized) {
        lv_init();
        s_lvgl_initialized = true;
        PRINTF("LVGL initialized\r\n");
    }

    BOARD_InitMipiPanelPins();

    #if DYNAMIC_FB_ALLOC
    
    #define align_up(num, align) (((num) + ((align) - 1)) & ~((align) - 1))

    MP_STATE_VM(s_frameBuffer_alloc) = m_new0(uint8_t, 2 * DEMO_FB_SIZE + DEMO_FB_ALIGN);
    s_frameBuffer = (uint8_t(*)[DEMO_FB_SIZE]) align_up((uintptr_t)MP_STATE_VM(s_frameBuffer_alloc), DEMO_FB_ALIGN);

    #if DEMO_USE_ROTATE
    MP_STATE_VM(s_lvglBuffer_alloc) = m_new0(uint8_t, DEMO_FB_SIZE + DEMO_FB_ALIGN);
    s_lvglBuffer = (uint8_t(*)[DEMO_FB_SIZE]) align_up((uintptr_t)MP_STATE_VM(s_lvglBuffer_alloc), DEMO_FB_ALIGN);
    #endif
    
	#else // static FB alloc
	
    memset(s_frameBuffer, 0, sizeof(s_frameBuffer));
    #if DEMO_USE_ROTATE
    memset(s_lvglBuffer, 0, sizeof(s_lvglBuffer));
    #endif
    #endif

    status_t status;
    dc_fb_info_t fbInfo;

    #if LV_USE_GPU_NXP_VG_LITE
    /* Initialize GPU. */
    BOARD_PrepareVGLiteController();
    #endif

    /*-------------------------
     * Initialize your display
     * -----------------------*/
    BOARD_PrepareDisplayController();

    status = g_dc.ops->init(&g_dc);
    if (kStatus_Success != status) {
        assert(0);
    }

    g_dc.ops->getLayerDefaultConfig(&g_dc, 0, &fbInfo);
    fbInfo.pixelFormat = DEMO_BUFFER_PIXEL_FORMAT;
    fbInfo.width = DEMO_BUFFER_WIDTH;
    fbInfo.height = DEMO_BUFFER_HEIGHT;
    fbInfo.startX = DEMO_BUFFER_START_X;
    fbInfo.startY = DEMO_BUFFER_START_Y;
    fbInfo.strideBytes = DEMO_BUFFER_STRIDE_BYTE;
    g_dc.ops->setLayerConfig(&g_dc, 0, &fbInfo);

    g_dc.ops->setCallback(&g_dc, 0, DEMO_BufferSwitchOffCallback, NULL);

    #if defined(SDK_OS_FREE_RTOS)
    s_transferDone = xSemaphoreCreateBinary();
    if (NULL == s_transferDone) {
        PRINTF("Frame semaphore create failed\r\n");
        assert(0);
    }
    #else
    s_transferDone = false;
    #endif

    #if DEMO_USE_ROTATE
    /* s_frameBuffer[1] is first shown in the panel, s_frameBuffer[0] is inactive. */
    s_inactiveFrameBuffer = (void *)s_frameBuffer[0];
    #endif

    /* DIAGNOSTIC: Fill both framebuffers with color blocks (RGB565)
     * Top third: RED (0xF800), Middle third: GREEN (0x07E0), Bottom third: BLUE (0x001F) */
    {
        uint16_t *fb0 = (uint16_t *)s_frameBuffer[0];
        uint16_t *fb1 = (uint16_t *)s_frameBuffer[1];
        uint32_t pixels_per_row = DEMO_PANEL_WIDTH;
        uint32_t total_rows = DEMO_PANEL_HEIGHT;
        uint32_t third = total_rows / 3;
        for (uint32_t row = 0; row < total_rows; row++) {
            uint16_t color;
            if (row < third)
                color = 0xF800; /* RED */
            else if (row < 2 * third)
                color = 0x07E0; /* GREEN */
            else
                color = 0x001F; /* BLUE */
            for (uint32_t col = 0; col < pixels_per_row; col++) {
                uint32_t idx = row * pixels_per_row + col;
                fb0[idx] = color;
                fb1[idx] = color;
            }
        }
        PRINTF("DIAG: FB[0]=%p FB[1]=%p size=%u (RGB block fill: R/G/B thirds)\r\n",
               fb0, fb1, DEMO_FB_SIZE);
    }

    /* Flush both frame buffers from DCache to SDRAM */
    SCB_CleanInvalidateDCache_by_Addr(s_frameBuffer[0], DEMO_FB_SIZE);
    SCB_CleanInvalidateDCache_by_Addr(s_frameBuffer[1], DEMO_FB_SIZE);

    /* Readback verification: read from SDRAM (invalidate DCache first) */
    {
        SCB_CleanInvalidateDCache_by_Addr(s_frameBuffer[1], 64);
        volatile uint16_t *fb = (volatile uint16_t *)s_frameBuffer[1];
        PRINTF("DIAG: FB[1] readback: [0]=%04x [1]=%04x [2]=%04x [719]=%04x [720]=%04x\r\n",
               fb[0], fb[1], fb[2], fb[719], fb[720]);
    }

    /* Pre-init LCDIF regs (before setFrameBuffer/enableLayer) */
    {
        LCDIFV2_Type *lcdif = LCDIFV2;
        PRINTF("DIAG-PRE: LCDIF CTRL=0x%08x DISP_SIZE=0x%08x\r\n",
               lcdif->CTRL, lcdif->DISP_SIZE);
        PRINTF("DIAG-PRE: L0 DESC1=0x%08x DESC3=0x%08x DESC4=0x%08x DESC5=0x%08x\r\n",
               lcdif->LAYER[0].CTRLDESCL1, lcdif->LAYER[0].CTRLDESCL3,
               lcdif->LAYER[0].CTRLDESCL4, lcdif->LAYER[0].CTRLDESCL5);
    }

    /* lvgl starts render in frame buffer 0, so show frame buffer 1 first. */
    g_dc.ops->setFrameBuffer(&g_dc, 0, (void *)s_frameBuffer[1]);

    /* Wait for frame buffer sent to display controller video memory. */
    if ((g_dc.ops->getProperty(&g_dc) & kDC_FB_ReserveFrameBuffer) == 0) {
        DEMO_WaitBufferSwitchOff();
    }

    g_dc.ops->enableLayer(&g_dc, 0);

    /* Start the LCD panel (send Display On command for ILI9881C) */
    BOARD_StartLcdPanel();

    /* Post-init LCDIF regs (after setFrameBuffer/enableLayer/startPanel) */
    {
        LCDIFV2_Type *lcdif = LCDIFV2;
        PRINTF("DIAG-POST: L0 DESC1=0x%08x DESC3=0x%08x DESC4=0x%08x DESC5=0x%08x\r\n",
               lcdif->LAYER[0].CTRLDESCL1, lcdif->LAYER[0].CTRLDESCL3,
               lcdif->LAYER[0].CTRLDESCL4, lcdif->LAYER[0].CTRLDESCL5);
        /* DSI DPI interface registers */
        DSI_HOST_DPI_INTFC_Type *dpi = DSI_HOST_DPI_INTFC;
        PRINTF("DIAG-POST: DSI PAYLOAD=%u PIXEL_FMT=%u COLOR=%u VIDEO_MODE=%u\r\n",
               dpi->PIXEL_PAYLOAD_SIZE, dpi->PIXEL_FORMAT,
               dpi->INTERFACE_COLOR_CODING, dpi->VIDEO_MODE);
        PRINTF("DIAG-POST: DSI HFP=%u HBP=%u HSA=%u VBP=%u VFP=%u VACTIVE=%u\r\n",
               dpi->HFP, dpi->HBP, dpi->HSA, dpi->VBP, dpi->VFP, dpi->VACTIVE);
        PRINTF("DIAG-POST: DSI MULT_PKTS=%u FIFO_SEND=%u\r\n",
               dpi->ENABLE_MULT_PKTS, dpi->PIXEL_FIFO_SEND_LEVEL);
        /* DSI error status */
        PRINTF("DIAG-POST: DSI RX_ERROR=0x%08x APB_IRQ=0x%08x APB_IRQ2=0x%08x\r\n",
               DSI_HOST->RX_ERROR_STATUS,
               DSI_HOST_APB_PKT_IF->IRQ_STATUS2,  /* Read STATUS2 first (per docs) */
               DSI_HOST_APB_PKT_IF->IRQ_STATUS);   /* Then STATUS (clears both) */
        /* VIDEO_MUX register — verify LCDIFv2 is routed to DSI */
        PRINTF("DIAG-POST: VIDEO_MUX_CTRL=0x%08x\r\n",
               VIDEO_MUX->VID_MUX_CTRL.RW);
    }

    /* Wait 500ms and check VSYNC frame count */
    {
        uint32_t count1 = s_vsyncCount;
        VIDEO_DelayMs(500);
        uint32_t count2 = s_vsyncCount;
        PRINTF("DIAG: VSYNC count: %u -> %u in 500ms (%u frames, ~%u Hz)\r\n",
               count1, count2, count2 - count1, (count2 - count1) * 2);
    }

    /*-----------------------------------
     * Register the display in LittlevGL
     *----------------------------------*/

    // Changes in master (v9 development) https://github.com/lvgl/lvgl/issues/4011

    lv_display_t * disp = lv_display_create(LCD_WIDTH, LCD_HEIGHT);

    // Set color format to match framebuffer pixel format
    #if DEMO_USE_XRGB8888
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_XRGB8888);
    #else
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
    #endif

    lv_display_set_flush_cb(disp, (void *)DEMO_FlushDisplay);

    #if DEMO_USE_ROTATE
    lv_display_set_buffers(disp, s_lvglBuffer[0], NULL, DEMO_BUFFER_WIDTH*DEMO_BUFFER_HEIGHT*DEMO_BUFFER_BYTE_PER_PIXEL, LCD_RENDER_MODE);
    #else
    lv_display_set_buffers(disp, s_frameBuffer[0], s_frameBuffer[1], DEMO_BUFFER_WIDTH*DEMO_BUFFER_HEIGHT*DEMO_BUFFER_BYTE_PER_PIXEL, LCD_RENDER_MODE);
    #endif

#if LV_USE_GPU_NXP_VG_LITE
    if (vg_lite_init(DEFAULT_VG_LITE_TW_WIDTH, DEFAULT_VG_LITE_TW_HEIGHT) != VG_LITE_SUCCESS)
    {
        PRINTF("VGLite init error. STOP.");
        vg_lite_close();
        while (1)
            ;
    }

    if (vg_lite_set_command_buffer_size(VG_LITE_COMMAND_BUFFER_SIZE) != VG_LITE_SUCCESS)
    {
        PRINTF("VGLite set command buffer. STOP.");
        vg_lite_close();
        while (1)
            ;
    }
#endif

    s_disp_initialized = true;
    PRINTF("Display initialization complete\r\n");
}

void lv_port_disp_deinit(void) {
    BOARD_DeinitLcdPanel();
    s_disp_initialized = false;
}

static void DEMO_BufferSwitchOffCallback(void *param, void *switchOffBuffer) {
    #if defined(SDK_OS_FREE_RTOS)
    BaseType_t taskAwake = pdFALSE;

    xSemaphoreGiveFromISR(s_transferDone, &taskAwake);
    portYIELD_FROM_ISR(taskAwake);
    #else
    s_transferDone = true;
    #endif
    s_vsyncCount++;

    #if DEMO_USE_ROTATE
    s_inactiveFrameBuffer = switchOffBuffer;
    #endif
}

#if (LV_USE_GPU_NXP_VG_LITE || LV_USE_GPU_NXP_PXP)
// static void DEMO_CleanInvalidateCache(lv_disp_drv_t *disp_drv) {
    // DEMO_FLUSH_DCACHE();
// }
#endif

static void DEMO_WaitBufferSwitchOff(void) {
    #if defined(SDK_OS_FREE_RTOS)
    if (xSemaphoreTake(s_transferDone, portMAX_DELAY) != pdTRUE) {
        PRINTF("Display flush failed\r\n");
        assert(0);
    }
    #else
    while (false == s_transferDone) {
    }
    s_transferDone = false;
    #endif
}

void DEMO_FlushDisplay(lv_display_t * disp_drv, const lv_area_t * area, uint8_t * color_p) {

    bool is_last = lv_disp_flush_is_last(disp_drv);

    /* No PRINTF in flush callback — USB CDC output can block and deadlock. */

    if (!is_last) {
        lv_disp_flush_ready(disp_drv);
        return;
    }
    #if DEMO_USE_ROTATE

    /*
     * Work flow:
     *
     * 1. Wait for the available inactive frame buffer to draw.
     * 2. Draw the ratated frame to inactive buffer.
     * 3. Pass inactive to LCD controller to show.
     */

    static bool firstFlush = true;

    /* Only wait for the first time. */
    if (firstFlush) {
        firstFlush = false;
    } else {
        /* Wait frame buffer. */
        DEMO_WaitBufferSwitchOff();
    }

    /* Copy buffer. */
    void *inactiveFrameBuffer = s_inactiveFrameBuffer;

    #if __CORTEX_M == 4
    L1CACHE_CleanInvalidateSystemCacheByRange((uint32_t)s_inactiveFrameBuffer, DEMO_FB_SIZE);
    #else
    SCB_CleanInvalidateDCache_by_Addr(inactiveFrameBuffer, DEMO_FB_SIZE);
    #endif

    lv_color_t * dest_buf = ((lv_color_t *)inactiveFrameBuffer);

    int32_t w = LVGL_BUFFER_WIDTH; //lv_area_get_width(area);
    int32_t h = LVGL_BUFFER_HEIGHT; //lv_area_get_height(area);
    lv_color_format_t cf = lv_display_get_color_format(disp_drv);
    // uint32_t px_size = lv_color_format_get_size(cf);
    uint32_t w_stride = lv_draw_buf_width_to_stride(w, cf);
    uint32_t h_stride = lv_draw_buf_width_to_stride(h, cf);

    lv_display_rotation_t rotation = LV_DISPLAY_ROTATION_270;

    uint32_t dest_stride = (rotation == LV_DISPLAY_ROTATION_270 || rotation == LV_DISPLAY_ROTATION_90) ? h_stride : w_stride;


    #if LV_USE_GPU_NXP_PXP /* Use PXP to rotate the panel. */
    // lv_area_t dest_area = {
    //     .x1 = 0,
    //     .x2 = DEMO_BUFFER_HEIGHT - 1,
    //     .y1 = 0,
    //     .y2 = DEMO_BUFFER_WIDTH - 1,
    // };

    // const lv_color_t * src_buf = color_p;
    // const lv_area_t * dest_area = &dest_area;
    // lv_coord_t dest_stride = DEMO_BUFFER_WIDTH;
    // const lv_area_t * src_area = area;
    // int32_t src_width = 
    // int32_t src_height = lv_area_get_height(area);
    // lv_coord_t src_stride = lv_area_get_width(area);
    // lv_opa_t opa = LV_OPA_COVER;
    // // lv_disp_rot_t angle = LV_DISP_ROT_270;

    lv_draw_pxp_rotate(color_p, dest_buf, w, h, w_stride, dest_stride, rotation, cf);
    // lv_gpu_nxp_pxp_wait();

    #else /* Use CPU to rotate the panel. */
    lv_draw_sw_rotate(color_p, dest_buf, w, h, w_stride, dest_stride, rotation, cf);

    // for (uint32_t y = 0; y < LVGL_BUFFER_HEIGHT; y++)
    // {
    //     for (uint32_t x = 0; x < LVGL_BUFFER_WIDTH; x++)
    //     {
    //         ((uint16_t *)inactiveFrameBuffer)[(DEMO_BUFFER_HEIGHT - x) * DEMO_BUFFER_WIDTH + y] =
    //             ((uint16_t *)color_p)[y * LVGL_BUFFER_WIDTH + x];
    //     }
    // }
    #endif

#if __CORTEX_M == 4
    L1CACHE_CleanInvalidateSystemCacheByRange((uint32_t)s_inactiveFrameBuffer, DEMO_FB_SIZE);
#else
    SCB_CleanInvalidateDCache_by_Addr(inactiveFrameBuffer, DEMO_FB_SIZE);
#endif

    g_dc.ops->setFrameBuffer(&g_dc, 0, inactiveFrameBuffer);

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);

    #else /* DEMO_USE_ROTATE */

    /* Direct mode: color_p points into one of the two framebuffers.
     * Flush DCache so LCDIF DMA reads the rendered pixels from SDRAM. */
    SCB_CleanInvalidateDCache_by_Addr(color_p, DEMO_FB_SIZE);

    g_dc.ops->setFrameBuffer(&g_dc, 0, (void *)color_p);

    /* Wait for the buffer switch (VSYNC) before LVGL writes to the
     * other buffer, so we don't tear. */
    DEMO_WaitBufferSwitchOff();

    lv_disp_flush_ready(disp_drv);
    #endif /* DEMO_USE_ROTATE */
}

void lv_port_indev_init(void) {
	BOARD_MIPIPanelTouch_I2C_Init();
    // static lv_indev_drv_t indev_drv;

    /*------------------
     * Touchpad
     * -----------------*/

    /*Initialize your touchpad */
    DEMO_InitTouch();

    /*Register a touchpad input device*/
    lv_indev_t * indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, DEMO_ReadTouch);
}

static void BOARD_PullMIPIPanelTouchResetPin(bool pullUp)
{
    #ifdef BOARD_MIPI_PANEL_TOUCH_RST_GPIO
    if (pullUp)
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_TOUCH_RST_GPIO, BOARD_MIPI_PANEL_TOUCH_RST_PIN, 1);
    }
    else
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_TOUCH_RST_GPIO, BOARD_MIPI_PANEL_TOUCH_RST_PIN, 0);
    }
    #endif
}

static void BOARD_ConfigMIPIPanelTouchIntPin(gt911_int_pin_mode_t mode)
{
    #ifdef BOARD_MIPI_PANEL_TOUCH_INT_GPIO
    if (mode == kGT911_IntPinInput)
    {
        BOARD_MIPI_PANEL_TOUCH_INT_GPIO->GDIR &= ~(1UL << BOARD_MIPI_PANEL_TOUCH_INT_PIN);
    }
    else
    {
        if (mode == kGT911_IntPinPullDown)
        {
            GPIO_PinWrite(BOARD_MIPI_PANEL_TOUCH_INT_GPIO, BOARD_MIPI_PANEL_TOUCH_INT_PIN, 0);
        }
        else
        {
            GPIO_PinWrite(BOARD_MIPI_PANEL_TOUCH_INT_GPIO, BOARD_MIPI_PANEL_TOUCH_INT_PIN, 1);
        }

        BOARD_MIPI_PANEL_TOUCH_INT_GPIO->GDIR |= (1UL << BOARD_MIPI_PANEL_TOUCH_INT_PIN);
    }
    #endif
}

/*Initialize your touchpad*/
static void DEMO_InitTouch(void)
{
    status_t status;

    const gpio_pin_config_t resetPinConfig = {
        .direction = kGPIO_DigitalOutput, .outputLogic = 0, .interruptMode = kGPIO_NoIntmode
    };
    #ifdef BOARD_MIPI_PANEL_TOUCH_INT_GPIO
    GPIO_PinInit(BOARD_MIPI_PANEL_TOUCH_INT_GPIO, BOARD_MIPI_PANEL_TOUCH_INT_PIN, &resetPinConfig);
    #endif
    #ifdef BOARD_MIPI_PANEL_TOUCH_RST_GPIO
    GPIO_PinInit(BOARD_MIPI_PANEL_TOUCH_RST_GPIO, BOARD_MIPI_PANEL_TOUCH_RST_PIN, &resetPinConfig);
    #endif

    status = GT911_Init(&s_touchHandle, &s_touchConfig);

    if (kStatus_Success != status)
    {
        PRINTF("ERROR: Touch IC initialization failed\r\n");
        return;
    }

    s_touchInitialized = true;
    GT911_GetResolution(&s_touchHandle, &s_touchResolutionX, &s_touchResolutionY);
}

/* Will be called by the library to read the touchpad */
static void DEMO_ReadTouch(lv_indev_t * drv, lv_indev_data_t * data) {
    static int touch_x = 0;
    static int touch_y = 0;

    if (!s_touchInitialized) {
        data->state = LV_INDEV_STATE_REL;
        return;
    }

    if (kStatus_Success == GT911_GetSingleTouch(&s_touchHandle, &touch_x, &touch_y))
    {
        data->state = LV_INDEV_STATE_PR;
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
    }

    /*Set the last pressed coordinates*/
    if (DEMO_PANEL_WIDTH != s_touchResolutionX) {
        touch_x = touch_x * DEMO_PANEL_WIDTH / s_touchResolutionX;
    }
    data->point.x = s_touchResolutionX - 1 - touch_x;
    
    if (DEMO_PANEL_HEIGHT != s_touchResolutionY) {
        touch_y = touch_y * DEMO_PANEL_HEIGHT / s_touchResolutionY;
    }
    data->point.y = s_touchResolutionY - 1 - touch_y;
}
