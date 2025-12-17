"""
IMXRT1170 Display Driver - Python Package

Provides Python-based configuration for MIPI DSI display panels on NXP i.MX RT1170,
wrapping the C-based LVGL display driver with runtime panel configuration.

Usage:
    from imxrt1170_display.panels import RPI7InchDisplay

    display = RPI7InchDisplay(i2c=machine.I2C(1))
    display.init()
    # Display is now ready for LVGL

Architecture:
    - Python layer: Panel configuration, I2C setup, GPIO control
    - C layer: MIPI DSI, LCDIF, LVGL integration, clocks, DMA
"""

# Import the C module
import imxrt1170_disp

# Export panel classes
from .panels.rpi_7inch import RPI7InchDisplay
from .panels.rk055ahd091 import RK055AHD091Display
from .panels.rk055iqh091 import RK055IQH091Display
from .panels.rk055mhd091 import RK055MHD091Display

__version__ = "1.0.0"
__all__ = [
    "RPI7InchDisplay",
    "RK055AHD091Display",
    "RK055IQH091Display",
    "RK055MHD091Display",
]
