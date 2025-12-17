"""
RK055AHD091 - 720x1280 MIPI DSI Display Panel

NXP evaluation display panel with:
- Resolution: 720x1280 (portrait)
- Interface: MIPI DSI (2 lanes)
- Driver IC: HX8394
"""

from .base import BasePanel


class RK055AHD091Display(BasePanel):
    """RK055AHD091 720x1280 MIPI DSI Display"""

    def get_panel_config(self):
        """Return RK055AHD091 panel timing configuration"""
        return {
            'name': 'rk055ahd091',
            'width': 720,
            'height': 1280,
            'hsw': 8,    # Horizontal sync width
            'hfp': 32,   # Horizontal front porch
            'hbp': 32,   # Horizontal back porch
            'vsw': 2,    # Vertical sync width
            'vfp': 16,   # Vertical front porch
            'vbp': 14,   # Vertical back porch
            'dsi_lanes': 2
        }

    def init_hardware(self):
        """
        Initialize RK055AHD091 hardware.

        This panel typically requires no I2C initialization.
        Driver IC (HX8394) is configured via MIPI DSI commands (handled in C layer).
        """
        pass
