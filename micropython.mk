IMXRT1170_DISP_DRV_MOD_DIR := $(USERMOD_DIR)

# Location of top-level MicroPython directory
MPY_DIR ?= lib/micropython

SRC_DIR = $(realpath $(IMXRT1170_DISP_DRV_MOD_DIR)/src)


MCU_SERIES ?= MIMXRT1176
MCU_VARIANT ?= MIMXRT1176DVMAA
MCU_CORE ?= _cm7

# Set SDK directory based on MCU_SERIES
MCU_DIR = $(MPY_DIR)/lib/nxp_driver/sdk/devices/$(MCU_SERIES)

# Add all C files to SRC_USERMOD.
SRC_USERMOD += $(SRC_DIR)/mpy_api.c

SRC_USERMOD += $(addprefix $(SRC_DIR)/,\
	board/board.c \
	board/display_support.c \
	board/lvgl_support.c \
	board/pin_mux.c \
	touchpanel/fsl_gt911.c \
	video/fsl_dc_fb_elcdif.c \
	video/fsl_dc_fb_lcdifv2.c \
	video/fsl_hx8394.c \
	video/fsl_mipi_dsi_cmd.c \
	video/fsl_rm68191.c \
	video/fsl_rm68200.c \
	video/fsl_video_common.c \
)

# NXP Drivers
SRC_USERMOD += \
	$(MCU_DIR)/drivers/fsl_elcdif.c \
	$(MCU_DIR)/drivers/fsl_lcdifv2.c \
	$(MCU_DIR)/drivers/fsl_mipi_dsi.c \
    $(MCU_DIR)/drivers/fsl_pxp.c

# We can add our module folder to include paths if needed
# This is not actually needed in this example.

INC += -I$(TOP)/$(MCU_DIR)
INC += -I$(TOP)/$(MCU_DIR)/drivers
INC += -I$(SRC_DIR)/board
INC += -I$(SRC_DIR)/touchpanel
INC += -I$(SRC_DIR)/video

CFLAGS_USERMOD += $(INC) -Wno-error=double-promotion
