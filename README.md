Micropython LVGL Display Driver for the imx-rt1170

This repo provides LVGL drivers for the MIPI display driver in the NXP i.MX RT1170.
It can be built for micropython as either a native module or user c module.

## Supported Display Panels

This driver supports multiple display panels:

1. **RK055AHD091** (720x1280) - RK055AHD091-CTG(RK055HDMIPI4M)
2. **RK055IQH091** (540x960) - RK055IQH091-CTG
3. **RK055MHD091** (720x1280) - RK055MHD091A0-CTG(RK055HDMIPI4MA0) - **Default**
4. **DEMO_PANEL_RASPI_7INCH** (800x480) - Raspberry Pi 7" Display

To change the display panel, modify the `DEMO_PANEL` definition in `display_support.h`.

## Raspberry Pi 7" Display Support

The driver includes support for the Raspberry Pi 7" display, including:

- **RPI Display Driver** (`rpi.c`, `rpi.h`) - Main display initialization and control
- **PCA6416 GPIO Expander** (`pca6416.c`, `pca6416.h`) - 16-bit I2C GPIO expander for display control
- **PCA9530 LED Controller** (`pca9530.c`, `pca9530.h`) - 2-bit LED controller
- **RPI Support Functions** (`rpi_support.c`, `rpi_support.h`) - Display communication and initialization

## Board-Specific Pin Configuration

The display configuration / driver pin definitions in `display_support.h` and `board.h` can be overridden for custom hardware by creating a `display_board_config.h` file in your board directory that defines the pins before including the display driver headers. See `board.h` for the list of overrideable definitions.

The display drivers themselves are based directly on the source provide by NXP in:
https://github.com/nxp-appcodehub/dm-rt1170evkb-full-appliance/tree/60dc3d517f8a67034b8569fca6dc6c5094e0841a

RPI 7" display support derived from:
https://github.com/nxp-appcodehub/dm-lvgl_cluster_rt1170_evkb_rpi

