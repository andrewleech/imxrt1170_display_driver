# -*- coding: utf-8 -*-
#
# PI Background IP
# Copyright (c) 2025, Planet Innovation Pty Ltd
# 436 Elgar Rd, Box Hill, 3128, VIC, Australia
# Phone: +61 3 9945 7510
#
# The copyright to the computer program(s) herein is the property of
# Planet Innovation, Australia.
# The program(s) may be used and/or copied only with the written permission
# of Planet Innovation or in accordance with the terms and conditions
# stipulated in the agreement/contract under which the program(s) have been
# supplied.
#

"""
i.MX RT1170 Display Driver for MicroPython.

This module provides Python and C interfaces for display control on the i.MX RT1170,
supporting various display panels including the Raspberry Pi 7" touchscreen.

The module provides a Pythonic interface over the low-level C driver (imxrt1170_disp),
allowing flexible display configuration and easy integration with LVGL.

Example usage:

    Basic usage (initialize default display):
        import imxrt1170_disp

        imxrt1170_disp.__init__()  # Initialize display with default config
        # Now you can use LVGL to draw on the display
        imxrt1170_disp.deinit()    # Clean up

    Advanced usage with Python display configuration:
        from imxrt1170_display.displays import RPI7InchDisplay
        import imxrt1170_disp

        display = RPI7InchDisplay()
        display.init()  # Custom initialization for this display

        imxrt1170_disp.__init__()  # Initialize LVGL with the display
        # Use LVGL for graphics
        imxrt1170_disp.deinit()
        display.deinit()

    Custom display configuration:
        from imxrt1170_display import Display

        class CustomDisplay(Display):
            def __init__(self):
                super().__init__()
                self.width = 800
                self.height = 480

            def init(self):
                # Custom display initialization
                pass

        display = CustomDisplay()
        display.init()
"""

# Import display base classes and specific drivers
from .display import Display

# Import specific display drivers
try:
    from .displays import RPI7InchDisplay
    __all__ = ["Display", "RPI7InchDisplay"]
except ImportError:
    # If specific displays aren't available, just export the base class
    __all__ = ["Display"]
