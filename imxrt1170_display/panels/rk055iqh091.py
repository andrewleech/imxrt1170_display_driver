"""
RK055IQH091 - 540x960 MIPI DSI Display Panel

NXP evaluation display panel with:
- Resolution: 540x960 (portrait)
- Interface: MIPI DSI (2 lanes)
- Driver IC: RM68191
"""

from .base import BasePanel


class RK055IQH091Display(BasePanel):
    """RK055IQH091 540x960 MIPI DSI Display"""

    def get_panel_config(self):
        """Return RK055IQH091 panel timing configuration"""
        return {
            'name': 'rk055iqh091',
            'width': 540,
            'height': 960,
            'hsw': 2,    # Horizontal sync width
            'hfp': 32,   # Horizontal front porch
            'hbp': 30,   # Horizontal back porch
            'vsw': 2,    # Vertical sync width
            'vfp': 16,   # Vertical front porch
            'vbp': 14,   # Vertical back porch
            'dsi_lanes': 2
        }

    def init_hardware(self):
        """
        Initialize RK055IQH091 hardware.

        This panel typically requires no I2C initialization.
        Driver IC (RM68191) is configured via MIPI DSI commands (handled in C layer).
        """
        pass
