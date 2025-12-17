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

#if LV_USE_DRAW_VG_LITE
#include "vg_lite.h"
#include "vglite_support.h"
#endif

#if LV_USE_ROTATE_PXP
#include "draw/nxp/pxp/lv_draw_pxp.h"
#include "display/lv_display_private.h"
#endif

#include "py/runtime.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Ratate panel or not. */
#ifndef DEMO_USE_ROTATE
#if LV_USE_ROTATE_PXP
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

#define DEMO_BUFFER_STRIDE_BYTE ((DEMO_BUFFER_WIDTH * LCD_FB_BYTE_PER_PIXEL + LV_DRAW_BUF_ALIGN - 1) & ~(LV_DRAW_BUF_ALIGN - 1))
#define DEMO_FB_SIZE_STATIC (DEMO_BUFFER_STRIDE_BYTE * DEMO_BUFFER_HEIGHT)  // Compile-time size
#define COMPUTE_STRIDE(x) ((x * LCD_FB_BYTE_PER_PIXEL + LV_DRAW_BUF_ALIGN - 1) & ~(LV_DRAW_BUF_ALIGN - 1))

// Helper function to get frame buffer size (runtime or compile-time)
static inline size_t get_fb_size(void) {
    // Note: This function uses runtime config if available, but could be refactored later
    // to properly handle all runtime sizing needs. For now, it returns compile-time size
    // which works because we use DYNAMIC_FB_ALLOC and allocate based on DEMO_PANEL.
    // TODO: In future, calculate stride from runtime panel width/height if available
    return DEMO_FB_SIZE_STATIC;
}
#define DEMO_FB_SIZE get_fb_size()  // Macro wraps function for easy refactoring later

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

#if (LV_USE_DRAW_VGLITE || LV_USE_DRAW_VG_LITE || LV_USE_DRAW_PXP)
void DEMO_CleanInvalidateCache(void);
void DEMO_CleanInvalidateCacheByAddr(void *addr, int32_t dsize);
#endif

static void DEMO_BufferSwitchOffCallback(void *param, void *switchOffBuffer);

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
#endif

#if DEMO_USE_ROTATE
/*
 * When rotate is used, LVGL stack draws in one buffer (s_lvglBuffer), and LCD
 * driver uses two buffers (s_frameBuffer) to remove tearing effect.
 */
static void *volatile s_inactiveFrameBuffer;
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

void lv_port_pre_init(void) {
}

void lv_port_disp_init(void) {

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

    #if LV_USE_DRAW_VG_LITE
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

    /* lvgl starts render in frame buffer 0, so show frame buffer 1 first. */
    g_dc.ops->setFrameBuffer(&g_dc, 0, (void *)s_frameBuffer[1]);

    /* Wait for frame buffer sent to display controller video memory. */
    if ((g_dc.ops->getProperty(&g_dc) & kDC_FB_ReserveFrameBuffer) == 0) {
        DEMO_WaitBufferSwitchOff();
    }

    g_dc.ops->enableLayer(&g_dc, 0);

    /*-----------------------------------
     * Register the display in LittlevGL
     *----------------------------------*/

    // Changes in master (v9 development) https://github.com/lvgl/lvgl/issues/4011

    lv_display_t * disp = lv_display_create(LVGL_BUFFER_WIDTH, LVGL_BUFFER_HEIGHT);
    lv_display_set_flush_cb(disp, DEMO_FlushDisplay);
    lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_270);

    #if DEMO_USE_ROTATE
    lv_display_set_buffers(disp, s_lvglBuffer[0], NULL, DEMO_FB_SIZE, LV_DISPLAY_RENDER_MODE_FULL);
    #else
    lv_display_set_buffers(disp, s_frameBuffer[0], s_frameBuffer[1], DEMO_FB_SIZE, LV_DISPLAY_RENDER_MODE_FULL);
    #endif

#if LV_USE_DRAW_VG_LITE
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
}

void lv_port_disp_init_with_config(const panel_config_t *config) {
    // Store runtime panel configuration
    BOARD_InitDisplayWithConfig(config);

    // Initialize display with runtime config
    // (uses same code path as lv_port_disp_init, but with runtime parameters)
    lv_port_disp_init();
}

void lv_port_disp_deinit(void) {
    BOARD_DeinitLcdPanel();
}

static void DEMO_BufferSwitchOffCallback(void *param, void *switchOffBuffer) {
    #if defined(SDK_OS_FREE_RTOS)
    BaseType_t taskAwake = pdFALSE;

    xSemaphoreGiveFromISR(s_transferDone, &taskAwake);
    portYIELD_FROM_ISR(taskAwake);
    #else
    s_transferDone = true;
    #endif

    #if DEMO_USE_ROTATE
    s_inactiveFrameBuffer = switchOffBuffer;
    #endif
}

#if (LV_USE_DRAW_VGLITE || LV_USE_DRAW_VG_LITE || LV_USE_DRAW_PXP)
void DEMO_CleanInvalidateCache(void)
{
    DEMO_FLUSH_DCACHE();
}

void DEMO_CleanInvalidateCacheByAddr(void *addr, int32_t dsize)
{
    SCB_CleanInvalidateDCache_by_Addr(addr, dsize);
}
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

void DEMO_FlushDisplay(lv_display_t * disp, const lv_area_t * area, uint8_t * color_p) {

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

    DEMO_FLUSH_DCACHE();

    /* Copy buffer. */
    void *inactiveFrameBuffer = s_inactiveFrameBuffer;

    #if LV_USE_ROTATE_PXP /* Use PXP to rotate the panel. */
    lv_draw_pxp_rotate(color_p, inactiveFrameBuffer,
                       LVGL_BUFFER_WIDTH, LVGL_BUFFER_HEIGHT,
                       COMPUTE_STRIDE(LVGL_BUFFER_WIDTH),
                       COMPUTE_STRIDE(DEMO_BUFFER_WIDTH),
                       LV_DISPLAY_ROTATION_270, disp->color_format);

    #else /* Use CPU to rotate the panel. */
    lv_draw_sw_rotate(color_p, inactiveFrameBuffer,
                      LVGL_BUFFER_WIDTH, LVGL_BUFFER_HEIGHT,
                      COMPUTE_STRIDE(LVGL_BUFFER_WIDTH),
                      COMPUTE_STRIDE(DEMO_BUFFER_WIDTH),
                      LV_DISPLAY_ROTATION_270, disp->color_format);
    #endif

    DEMO_FLUSH_DCACHE();

    g_dc.ops->setFrameBuffer(&g_dc, 0, inactiveFrameBuffer);

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing*/
    lv_display_flush_ready(disp);

    #else /* DEMO_USE_ROTATE */

    DEMO_FLUSH_DCACHE();

    g_dc.ops->setFrameBuffer(&g_dc, 0, (void *)color_p);

    DEMO_WaitBufferSwitchOff();

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing*/
    lv_display_flush_ready(disp);
    #endif /* DEMO_USE_ROTATE */
}

