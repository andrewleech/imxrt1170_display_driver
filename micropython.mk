IMXRT1170_DISP_DRV_MOD_DIR := $(USERMOD_DIR)

#$(foreach v, $(.VARIABLES), $(info $(v) = $($(v))))

# Location of top-level MicroPython directory
MPY_DIR ?= $(TOP)

#SRC_DIR = $(realpath $(IMXRT1170_DISP_DRV_MOD_DIR)/src)
SRC_DIR = $(IMXRT1170_DISP_DRV_MOD_DIR)/src

MICROPY_PORT = $(notdir $(CURDIR))
ifeq ($(MICROPY_PORT),mimxrt)

MCU_SERIES ?= MIMXRT1176
MCU_VARIANT ?= MIMXRT1176DVMAA
MCU_CORE ?= _cm7

# Set SDK directory based on MCU_SERIES
NXP_SDK = $(abspath $(MPY_DIR)/lib/nxp_driver/sdk)
MCU_DIR = $(NXP_SDK)/devices/$(MCU_SERIES)

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
	$(NXP_SDK)/drivers/elcdif/fsl_elcdif.c \
	$(NXP_SDK)/drivers/lcdifv2/fsl_lcdifv2.c \
	$(NXP_SDK)/drivers/mipi_dsi_split/fsl_mipi_dsi.c \
    $(NXP_SDK)/drivers/pxp/fsl_pxp.c

# We can add our module folder to include paths if needed
# This is not actually needed in this example.

INC += -I$(MCU_DIR)
INC += -I$(MCU_DIR)/drivers
#INC += -I$(NXP_SDK)/drivers/common
INC += -I$(SRC_DIR)/board
INC += -I$(SRC_DIR)/touchpanel
INC += -I$(SRC_DIR)/video
INC += -I$(NXP_SDK)/drivers/elcdif
INC += -I$(NXP_SDK)/drivers/lcdifv2
INC += -I$(NXP_SDK)/drivers/mipi_dsi_split
INC += -I$(NXP_SDK)/drivers/pxp

CFLAGS_USERMOD += $(INC) -D_FSL_COMMON_ARM_H_=FSL_COMMON_ARM_H_ -D_FSL_COMMON_H_=FSL_COMMON_H_ 

#$(BUILD)/$(MOD_DIRNAME)/lvgl/src/draw/nxp/pxp/lv_draw_pxp_img.o: CFLAGS_USERMOD += -D_FSL_CLOCK_H_=FSL_CLOCK_H_

#$(BUILD)/machine_bitstream.o: CFLAGS_USERMOD += -D_FSL_CLOCK_H_=FSL_CLOCK_H_
#$(BUILD)/eth.o: CFLAGS_USERMOD += -D_FSL_CLOCK_H_=FSL_CLOCK_H_
#$(BUILD)/extmod/machine_pwm.o: CFLAGS_USERMOD += -D_FSL_CLOCK_H_=FSL_CLOCK_H_

MOD_DIRNAME := $(notdir $(abspath $(USERMOD_DIR)))

$(info PATH:$(BUILD)/$(NXP_SDK)/drivers/mipi_dsi_split/fsl_mipi_dsi.o)
$(BUILD)/$(NXP_SDK)/drivers/mipi_dsi_split/fsl_mipi_dsi.o: CFLAGS_USERMOD += -Wno-error=float-conversion -Wno-error=double-promotion



endif
