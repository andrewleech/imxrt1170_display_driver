"""
Display Panel Implementations

Each panel class provides:
- Panel timing configuration (width, height, sync pulses, porches)
- I2C initialization for display controllers (if needed)
- Hardware-specific initialization sequences

Panels can be subclassed to override configuration for custom boards.
"""

from .base import BasePanel
from .rpi_7inch import RPI7InchDisplay
from .rk055ahd091 import RK055AHD091Display
from .rk055iqh091 import RK055IQH091Display
from .rk055mhd091 import RK055MHD091Display

__all__ = [
    "BasePanel",
    "RPI7InchDisplay",
    "RK055AHD091Display",
    "RK055IQH091Display",
    "RK055MHD091Display",
]
