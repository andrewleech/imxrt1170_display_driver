"""
Raspberry Pi 7" Touch Display (800x480)

Official Raspberry Pi 7-inch touchscreen display with:
- Resolution: 800x480 @ 60Hz
- Interface: MIPI DSI (1 lane)
- Backlight Controller: Attiny88 (I2C address 0x45)
- GPIO Expander: PCA6416 (I2C address 0x20)
- Touch Controller: FT5406 (separate driver, not handled here)
"""

from .base import BasePanel
import machine
import time


class RPI7InchDisplay(BasePanel):
    """Raspberry Pi 7" MIPI DSI Display"""

    # I2C addresses
    ATTINY88_ADDR = 0x45
    PCA6416_ADDR = 0x20

    # Attiny88 registers
    REG_ID = 0x80
    REG_POWERON = 0x85
    REG_PWM = 0x86
    REG_PORTA = 0x81

    # PCA6416 registers
    PCA_CONFIG = 0x06
    PCA_OUTPUT = 0x02

    def __init__(self, display_i2c=None, gpio_i2c=None):
        """
        Initialize RPI 7" display panel.

        Args:
            display_i2c: I2C bus for Attiny88 display controller (default: I2C(1))
            gpio_i2c: I2C bus for PCA6416 GPIO expander (default: I2C(6))
        """
        self.display_i2c = display_i2c
        self.gpio_i2c = gpio_i2c

    def get_panel_config(self):
        """Return RPI 7" panel timing configuration"""
        return {
            'name': 'rpi_7inch',
            'width': 800,
            'height': 480,
            'hsw': 20,   # Horizontal sync width
            'hfp': 70,   # Horizontal front porch
            'hbp': 23,   # Horizontal back porch
            'vsw': 2,    # Vertical sync width
            'vfp': 7,    # Vertical front porch
            'vbp': 21,   # Vertical back porch
            'dsi_lanes': 1
        }

    def init_hardware(self):
        """
        Initialize RPI 7" display hardware.

        - Configures I2C pins (via C helper)
        - Initializes PCA6416 GPIO expander
        - Powers on display via Attiny88
        - Sets backlight brightness
        """
        # Initialize I2C buses if not provided
        if self.display_i2c is None:
            # TODO: Call configure_i2c_pins() once implemented in Phase 2
            self.display_i2c = machine.I2C(1, freq=100000)

        if self.gpio_i2c is None:
            # TODO: Call configure_i2c_pins() once implemented in Phase 2
            self.gpio_i2c = machine.I2C(6, freq=100000)

        # Initialize PCA6416 GPIO expander
        self._init_pca6416()

        # Power on display via Attiny88
        self._init_attiny()

    def _init_pca6416(self):
        """Initialize PCA6416 GPIO expander"""
        try:
            # Configure all pins as outputs
            self.gpio_i2c.writeto_mem(self.PCA6416_ADDR, self.PCA_CONFIG, bytes([0x00]))

            # Set initial output state (all low)
            self.gpio_i2c.writeto_mem(self.PCA6416_ADDR, self.PCA_OUTPUT, bytes([0x00]))
        except OSError as e:
            print(f"Warning: PCA6416 initialization failed: {e}")

    def _init_attiny(self):
        """
        Initialize Attiny88 display controller.

        Power-on sequence for RPI 7" display.
        """
        try:
            # Wait for Attiny to be ready
            time.sleep_ms(2000)

            # Check ID register
            reg_id = self.display_i2c.readfrom_mem(self.ATTINY88_ADDR, self.REG_ID, 1)
            print(f"Attiny88 ID: 0x{reg_id[0]:02x}")

            # Power on sequence
            # Step 1: Power off
            self.display_i2c.writeto_mem(self.ATTINY88_ADDR, self.REG_POWERON, bytes([0x00]))
            time.sleep_ms(800)

            # Step 2: Power on
            self.display_i2c.writeto_mem(self.ATTINY88_ADDR, self.REG_POWERON, bytes([0x01]))
            time.sleep_ms(800)

            # Step 3: Enable display output
            self.display_i2c.writeto_mem(self.ATTINY88_ADDR, self.REG_PORTA, bytes([0x04]))

            # Set backlight brightness (0x00-0xFF, 0x80 = 50%)
            brightness = 0x80
            self.display_i2c.writeto_mem(self.ATTINY88_ADDR, self.REG_PWM, bytes([brightness]))

        except OSError as e:
            print(f"Warning: Attiny88 initialization failed: {e}")

    def set_brightness(self, level):
        """
        Set backlight brightness.

        Args:
            level (int): Brightness level 0-255 (0=off, 255=max)
        """
        if not 0 <= level <= 255:
            raise ValueError("Brightness must be 0-255")

        try:
            self.display_i2c.writeto_mem(self.ATTINY88_ADDR, self.REG_PWM, bytes([level]))
        except OSError as e:
            print(f"Warning: Failed to set brightness: {e}")
