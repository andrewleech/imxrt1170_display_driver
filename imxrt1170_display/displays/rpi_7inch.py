# -*- coding: utf-8 -*-
#
# Copyright (c) 2025, Planet Innovation
# 436 Elgar Road, Box Hill, 3128, VIC, Australia
# Phone: +61 3 9945 7510
#
# The copyright to the computer program(s) herein is the property of
# Planet Innovation, Australia.
# The program(s) may be used and/or copied only with the written permission
# of Planet Innovation or in accordance with the terms and conditions
# stipulated in the agreement/contract under which the program(s) have been
# supplied.

"""
Raspberry Pi 7" Touchscreen Display Driver.

This module provides a Python driver for the official Raspberry Pi 7" touchscreen
display (800x480 resolution) connected via MIPI DSI interface.

The display uses:
- MIPI DSI interface for video
- I2C for backlight and touch controller communication
- Resolution: 800x480 pixels
- Color depth: 24-bit RGB

Hardware specifications:
- Part Number: Raspberry Pi Touch Display
- Panel: 7" IPS LCD
- Touch Controller: FT5406 (I2C address 0x45)
- Backlight Control: I2C via PCA6416 GPIO expander
- Interface: 1-lane MIPI DSI
"""

from ..display import Display


class RPI7InchDisplay(Display):
    """Driver for Raspberry Pi 7" touchscreen display.

    This class provides initialization and control for the official
    Raspberry Pi 7" touchscreen display connected via MIPI DSI.

    The actual low-level initialization is handled by the C driver
    (RPI_Init, RPI_Start, RPI_Stop, RPI_Deinit), which is automatically
    called during the LVGL initialization process.

    Backlight control is handled via I2C through the PCA6416 GPIO expander.

    Example usage:
        from imxrt1170_display.displays import RPI7InchDisplay

        display = RPI7InchDisplay()
        display.init()          # Initialize display hardware
        display.lvgl_init()     # Initialize LVGL graphics

        # Use LVGL to draw graphics here

        display.lvgl_deinit()   # Clean up LVGL
        display.deinit()        # Power down display

    Attributes:
        I2C_ADDR (int): I2C address of the display controller (0x45)
    """

    # I2C address for the Raspberry Pi display controller
    I2C_ADDR = 0x45

    def __init__(self):
        """Initialize RPI 7" display configuration.

        Sets up the display with standard specifications:
        - Resolution: 800x480 pixels
        - Color depth: 24-bit RGB (supports 16-bit mode via LVGL)
        """
        super().__init__(width=800, height=480, color_depth=24)

    def init(self):
        """Initialize the Raspberry Pi 7" display hardware.

        This method performs Python-level initialization. The actual
        hardware initialization (MIPI DSI setup, display controller init,
        power sequencing) is handled by the C driver during lvgl_init().

        For the RPI display, most initialization is handled in C for
        optimal performance and timing-critical operations.

        The initialization sequence in C includes:
        1. Configure MIPI DSI interface (1-lane mode)
        2. Initialize display controller via I2C
        3. Power on the display panel
        4. Configure display timings (800x480, 60Hz)
        5. Start MIPI DSI video streaming

        Returns:
            None

        Raises:
            OSError: If display initialization fails (during lvgl_init())
        """
        # The C driver handles all initialization during lv_port_disp_init()
        # This method is provided for consistency with the Display interface
        # and to allow subclasses to add custom initialization if needed.
        pass

    def deinit(self):
        """Deinitialize the Raspberry Pi 7" display hardware.

        This method performs Python-level cleanup. The actual hardware
        deinitialization (MIPI DSI shutdown, power down) is handled by
        the C driver during lvgl_deinit().

        Returns:
            None
        """
        # The C driver handles all deinitialization during lv_port_disp_deinit()
        pass

    def get_brightness(self):
        """Get current display brightness.

        Note: The RPI display backlight is controlled via I2C through
        the PCA6416 GPIO expander. This implementation does not currently
        support reading the brightness level.

        Returns:
            int: Current brightness level (0-100)

        Raises:
            NotImplementedError: Brightness reading not supported
        """
        raise NotImplementedError("RPI display does not support brightness reading")

    def set_brightness(self, level):
        """Set display brightness.

        Note: The RPI display backlight is controlled via I2C through
        the PCA6416 GPIO expander. This implementation provides basic
        on/off control through the C driver.

        Args:
            level (int): Brightness level (0-100)
                - 0: Backlight off
                - >0: Backlight on (full brightness)

        Raises:
            NotImplementedError: Fine-grained brightness control not supported
        """
        raise NotImplementedError("RPI display only supports on/off backlight control via C driver")
