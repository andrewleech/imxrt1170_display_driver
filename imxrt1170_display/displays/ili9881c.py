# -*- coding: utf-8 -*-
#
# Copyright (c) 2026, Planet Innovation
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
ILI9881C LCD Display Driver (WF50DTYA3MNG10000 module).

This module provides a Python driver for the ILI9881C LCD display
(720x1200 resolution) connected via MIPI DSI interface.

The display uses:
- MIPI DSI interface for video (2-lane)
- PWM for backlight control (GPIO_AD_04 @ 1kHz)
- I2C for touch controller (GT928 @ 0x5D)
- Resolution: 720x1200 pixels
- Color depth: 24-bit RGB

Hardware specifications:
- Part Number: WF50DTYA3MNG10000
- Panel: 5" IPS LCD
- Display Controller: ILI9881C (pre-programmed NVM init)
- Touch Controller: GT928 (I2C address 0x5D)
- Interface: 2-lane MIPI DSI
- Backlight: PWM @ 1kHz (not I2C like RPI display)

Key differences from RPI 7" display:
- Resolution: 720x1200 (portrait) vs 800x480 (landscape)
- Backlight: Direct PWM control vs I2C control
- Touch IC: GT928 vs FT5406
- MIPI lanes: 2 vs 1
"""

from .rpi_7inch import RPI7InchDisplay
from machine import Pin, PWM


class ILI9881CDisplay(RPI7InchDisplay):
    """Driver for ILI9881C LCD display (WF50DTYA3MNG10000 module).

    This class provides initialization and control for the ILI9881C
    LCD display connected via MIPI DSI. It inherits from RPI7InchDisplay
    because both use similar MIPI DSI initialization via the rpi.c driver,
    but overrides backlight control to use PWM instead of I2C.

    The ILI9881C controller has its initialization sequence pre-programmed
    in non-volatile memory (NVM), so only standard MIPI DSI commands are
    needed (Sleep Out, Display On).

    Example usage:
        from imxrt1170_display.displays import ILI9881CDisplay

        display = ILI9881CDisplay()
        display.init()          # Initialize display hardware and backlight
        display.lvgl_init()     # Initialize LVGL graphics

        # Adjust brightness
        display.set_brightness(50)  # 50% brightness

        # Use LVGL to draw graphics here

        display.lvgl_deinit()   # Clean up LVGL
        display.deinit()        # Power down display

    Attributes:
        BACKLIGHT_FREQ (int): PWM frequency for backlight (1000 Hz)
        TOUCH_I2C_ADDR (int): I2C address of GT928 touch controller (0x5D)
    """

    # PWM frequency for backlight control (1kHz)
    BACKLIGHT_FREQ = 1000

    # I2C address for GT928 touch controller
    TOUCH_I2C_ADDR = 0x5D

    def __init__(self):
        """Initialize ILI9881C display configuration.

        Sets up the display with ILI9881C specifications:
        - Resolution: 720x1280 pixels
        - Color depth: 24-bit RGB (supports 16-bit mode via LVGL)
        - Backlight: PWM control
        """
        # Call Display.__init__ directly, not RPI7InchDisplay.__init__
        # to set correct resolution
        super(RPI7InchDisplay, self).__init__(width=720, height=1200, color_depth=24)

        # Backlight PWM object (initialized in init())
        self._backlight_pwm = None
        self._current_brightness = 100

    def lvgl_init(self):
        """Initialize LVGL without event_loop.

        The ILI9881C flush callback blocks waiting for the LCDIFv2
        frame-complete interrupt. Running lv.timer_handler() from a timer
        interrupt (as lv_utils.event_loop does) causes a deadlock because
        the LCDIFv2 IRQ cannot fire while already in an interrupt context.

        Instead, the caller must drive lv.timer_handler() and lv.tick_inc()
        from the main loop.
        """
        import imxrt1170_disp  # noqa: F401 - import triggers C-level init

    def init(self):
        """Initialize the ILI9881C display hardware.

        This method performs display-specific initialization:
        1. Initialize PWM for backlight control (GPIO_AD_04 @ 1kHz)
        2. Set initial brightness to 100%

        The actual hardware initialization (MIPI DSI setup, display controller
        init, power sequencing) is handled by the C driver (rpi.c) during
        lvgl_init(), since the ILI9881C uses the same MIPI DSI init sequence
        as the RPI display (pre-programmed NVM).

        Returns:
            None

        Raises:
            OSError: If PWM initialization fails
        """
        # Initialize backlight PWM on GPIO_AD_04 (mapped as PWM1_2A in board pins)
        # Frequency: 1kHz (1000 Hz)
        # Duty cycle: 0-65535 (16-bit resolution)
        try:
            self._backlight_pwm = PWM(Pin.board.PWM1_2A, freq=self.BACKLIGHT_FREQ)
            self.set_brightness(self._current_brightness)
        except Exception as e:
            raise OSError(f"Failed to initialize ILI9881C backlight PWM: {e}")

    def deinit(self):
        """Deinitialize the ILI9881C display hardware.

        This method performs display-specific cleanup:
        1. Turn off backlight (set to 0%)
        2. Deinitialize PWM

        The actual hardware deinitialization (MIPI DSI shutdown, power down)
        is handled by the C driver during lvgl_deinit().

        Returns:
            None
        """
        if self._backlight_pwm:
            # Turn off backlight before deinit
            self.set_brightness(0)
            self._backlight_pwm.deinit()
            self._backlight_pwm = None

    def get_brightness(self):
        """Get current display brightness.

        Returns:
            int: Current brightness level (0-100)
        """
        return self._current_brightness

    def set_brightness(self, level):
        """Set display brightness.

        Args:
            level (int): Brightness level (0-100)
                - 0: Backlight off
                - 100: Maximum brightness

        Raises:
            ValueError: If level is outside 0-100 range
            RuntimeError: If PWM not initialized
        """
        if not 0 <= level <= 100:
            raise ValueError(f"Brightness level must be 0-100, got {level}")

        if not self._backlight_pwm:
            raise RuntimeError("Backlight PWM not initialized. Call init() first.")

        # Convert percentage to 16-bit duty cycle value
        # 0% = 0, 100% = 65535
        duty = int((level / 100) * 65535)
        self._backlight_pwm.duty_u16(duty)
        self._current_brightness = level
