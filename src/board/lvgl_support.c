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

    lv_display_t * disp = lv_display_create(LCD_WIDTH, LCD_HEIGHT);
    lv_display_set_flush_cb(disp, (void *)DEMO_FlushDisplay);
    lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_270);

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

    if (!lv_disp_flush_is_last(disp_drv)) {
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

#if __CORTEX_M == 4
    L1CACHE_CleanInvalidateSystemCacheByRange((uint32_t)color_p, DEMO_FB_SIZE);
#else
    SCB_CleanInvalidateDCache_by_Addr(color_p, DEMO_FB_SIZE);
#endif

    g_dc.ops->setFrameBuffer(&g_dc, 0, (void *)color_p);

    DEMO_WaitBufferSwitchOff();

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);
    #endif /* DEMO_USE_ROTATE */
}
