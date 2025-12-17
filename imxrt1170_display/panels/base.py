"""
Base Panel Class

Provides the common interface for all display panels.
"""

import imxrt1170_disp


class BasePanel:
    """
    Base class for MIPI DSI display panels.

    Subclasses must implement:
        - get_panel_config(): Return panel timing configuration dict
        - init_hardware(): Initialize panel-specific hardware (optional)
    """

    def get_panel_config(self):
        """
        Return panel configuration dictionary.

        Returns:
            dict: Panel configuration with keys:
                - name (str): Panel identifier
                - width (int): Horizontal resolution
                - height (int): Vertical resolution
                - hsw (int): Horizontal sync width
                - hfp (int): Horizontal front porch
                - hbp (int): Horizontal back porch
                - vsw (int): Vertical sync width
                - vfp (int): Vertical front porch
                - vbp (int): Vertical back porch
                - dsi_lanes (int): Number of DSI lanes (1 or 2)
        """
        raise NotImplementedError("Subclass must implement get_panel_config()")

    def init_hardware(self):
        """
        Initialize panel-specific hardware.

        Called before display controller initialization.
        Override to configure I2C peripherals, GPIO expanders, etc.
        """
        pass

    def init(self):
        """
        Initialize the display with this panel configuration.

        1. Calls init_hardware() for panel-specific setup (Python does ALL I2C)
        2. Gets panel configuration
        3. Calls C layer to initialize display controller (MIPI DSI, LCDIF, LVGL)
        """
        # Panel-specific hardware initialization (ALL I2C done in Python)
        self.init_hardware()

        # Get panel configuration
        config = self.get_panel_config()

        # Initialize display controller via C module
        # C only handles MIPI DSI, LCDIF, and LVGL setup
        # Python has already done ALL I2C initialization
        imxrt1170_disp.init_with_config(config)

        return config
