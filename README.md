# MicroPython LVGL Display Driver for i.MX RT1170

MicroPython display driver for NXP i.MX RT1170 with LVGL 9.x support. Supports multiple MIPI DSI panels with runtime configuration via Python.

## Architecture

**Python Layer:**
- All I2C initialization (display controllers, GPIO expanders)
- Panel-specific configuration and timing
- Runtime panel selection

**C Layer:**
- MIPI DSI interface
- LCDIF/LCDIFv2 display controller
- LVGL integration
- Frame buffer management
- Hardware acceleration (PXP)

## Supported Display Panels

| Panel | Resolution | DSI Lanes | Controller | Notes |
|-------|-----------|-----------|------------|-------|
| **RPI 7"** | 800x480 | 1 | Attiny88 | Requires PCA6416 GPIO expander |
| **RK055AHD091** | 720x1280 | 2 | RM68200 | RK055HDMIPI4M |
| **RK055MHD091** | 720x1280 | 2 | HX8394 | RK055HDMIPI4MA0 (Default) |
| **RK055IQH091** | 540x960 | 2 | RM68191 | - |

## Quick Start

### Using RPI 7" Display

```python
from imxrt1170_display.panels import RPI7InchDisplay

# Create display instance (uses default I2C buses)
display = RPI7InchDisplay()

# Initialize display
display.init()

# LVGL is now ready to use
import lvgl as lv
scr = lv.screen_active()
label = lv.label(scr)
label.set_text("Hello World")
```

### Using RK055MHD091 (Default Panel)

```python
from imxrt1170_display.panels import RK055MHD091Display

display = RK055MHD091Display()
display.init()
```

### Using RK055AHD091

```python
from imxrt1170_display.panels import RK055AHD091Display

display = RK055AHD091Display()
display.init()
```

### Using RK055IQH091

```python
from imxrt1170_display.panels import RK055IQH091Display

display = RK055IQH091Display()
display.init()
```

## Board Customization

### Custom I2C Bus

```python
from imxrt1170_display.panels import RPI7InchDisplay
import machine

# Override I2C buses for custom board
display = RPI7InchDisplay(
    display_i2c=machine.I2C(2, freq=100000),  # Attiny88 on I2C2
    gpio_i2c=machine.I2C(3, freq=100000)       # PCA6416 on I2C3
)
display.init()
```

### Custom PCA6416 Address

```python
# Default PCA6416 address is 0x21
display = RPI7InchDisplay(pca6416_addr=0x20)  # Override to 0x20
display.init()
```

### Custom Panel Timing

```python
from imxrt1170_display.panels import RK055MHD091Display

class CustomPanelTiming(RK055MHD091Display):
    def get_panel_config(self):
        config = super().get_panel_config()
        # Adjust horizontal front porch
        config['hfp'] = 20
        config['hbp'] = 30
        return config

display = CustomPanelTiming()
display.init()
```

### Custom Panel (New Driver IC)

```python
from imxrt1170_display.panels.base import BasePanel
import machine

class CustomDisplay(BasePanel):
    def __init__(self, i2c=None):
        self.i2c = i2c or machine.I2C(1, freq=100000)

    def get_panel_config(self):
        return {
            'name': 'custom_panel',
            'width': 1024,
            'height': 600,
            'hsw': 10,   # Horizontal sync width
            'hfp': 40,   # Horizontal front porch
            'hbp': 40,   # Horizontal back porch
            'vsw': 3,    # Vertical sync width
            'vfp': 13,   # Vertical front porch
            'vbp': 29,   # Vertical back porch
            'dsi_lanes': 2
        }

    def init_hardware(self):
        # Custom I2C initialization for panel controller
        self.i2c.writeto_mem(0x45, 0x10, bytes([0x01]))
        # ... panel-specific setup

display = CustomDisplay()
display.init()
```

## Panel Timing Reference

### RPI 7" Display (800x480)
```python
{
    'width': 800, 'height': 480,
    'hsw': 20, 'hfp': 70, 'hbp': 23,
    'vsw': 2, 'vfp': 7, 'vbp': 21,
    'dsi_lanes': 1
}
```

### RK055AHD091 (720x1280)
```python
{
    'width': 720, 'height': 1280,
    'hsw': 8, 'hfp': 32, 'hbp': 32,
    'vsw': 2, 'vfp': 16, 'vbp': 14,
    'dsi_lanes': 2
}
```

### RK055MHD091 (720x1280)
```python
{
    'width': 720, 'height': 1280,
    'hsw': 6, 'hfp': 12, 'hbp': 24,
    'vsw': 2, 'vfp': 16, 'vbp': 14,
    'dsi_lanes': 2
}
```

### RK055IQH091 (540x960)
```python
{
    'width': 540, 'height': 960,
    'hsw': 2, 'hfp': 32, 'hbp': 30,
    'vsw': 2, 'vfp': 16, 'vbp': 14,
    'dsi_lanes': 2
}
```

## API Reference

### BasePanel Class

```python
class BasePanel:
    def get_panel_config(self):
        """Return panel configuration dictionary.

        Returns:
            dict: Panel configuration with keys:
                - name: str - Panel identifier
                - width: int - Panel width in pixels
                - height: int - Panel height in pixels
                - hsw: int - Horizontal sync width
                - hfp: int - Horizontal front porch
                - hbp: int - Horizontal back porch
                - vsw: int - Vertical sync width
                - vfp: int - Vertical front porch
                - vbp: int - Vertical back porch
                - dsi_lanes: int - Number of MIPI DSI lanes (1 or 2)
        """
        pass

    def init_hardware(self):
        """Initialize panel-specific hardware (I2C, GPIO, etc.)"""
        pass

    def init(self):
        """Initialize display (calls init_hardware() then C layer)"""
        pass
```

### Panel Classes

All panel classes inherit from `BasePanel`:

- `RPI7InchDisplay(display_i2c=None, gpio_i2c=None, pca6416_addr=None)`
- `RK055AHD091Display()`
- `RK055MHD091Display()`
- `RK055IQH091Display()`

## Board-Specific Pin Configuration

Pin definitions in `src/board/board.h` can be overridden by creating a `display_board_config.h` file in your board directory. Include this file before the display driver headers.

Example `display_board_config.h`:
```c
#ifndef DISPLAY_BOARD_CONFIG_H
#define DISPLAY_BOARD_CONFIG_H

// Override panel reset pin
#define BOARD_MIPI_PANEL_RST_GPIO GPIO9
#define BOARD_MIPI_PANEL_RST_PIN  2

// Override backlight pin
#define BOARD_MIPI_PANEL_BL_GPIO GPIO9
#define BOARD_MIPI_PANEL_BL_PIN  30

#endif
```

## Touch Input

Touch input is NOT included in this driver. Use a separate MicroPython touch controller module:

- GT911 (for RK055 panels): Separate `gt911` module
- FT5406 (for RPI 7" display): Separate `ft5406` module

Example:
```python
from imxrt1170_display.panels import RPI7InchDisplay
from ft5406 import FT5406  # Separate touch module

# Initialize display
display = RPI7InchDisplay()
display.init()

# Initialize touch separately
touch = FT5406(i2c=machine.I2C(5))
```

## Troubleshooting

### Display shows garbage/static

**Cause**: Timing mismatch or incorrect DSI lane count

**Fix**: Verify panel timing parameters match your physical panel. Check datasheet for correct HSW/HFP/HBP/VSW/VFP/VBP values.

### Black screen (no output)

**Cause**: MIPI DSI not initialized or panel reset/power not asserted

**Fix**:
- Check that `init_hardware()` is called
- For RPI panel, verify I2C communication with Attiny88
- Check that panel reset/power GPIOs are correct for your board

### RPI 7" Display not working

**Cause**: PCA6416 address mismatch or I2C bus issue

**Fix**:
```python
# Try alternate PCA6416 address
display = RPI7InchDisplay(pca6416_addr=0x20)

# Or specify I2C buses explicitly
display = RPI7InchDisplay(
    display_i2c=machine.I2C(1),
    gpio_i2c=machine.I2C(6)
)
```

### I2C errors on init

**Cause**: Wrong I2C bus or device not connected

**Fix**:
- Verify physical I2C connections
- Check I2C bus number (1-based in MicroPython)
- Scan I2C bus: `machine.I2C(1).scan()` to verify device address

## Project Structure

```
imxrt1170_display_driver/
├── src/
│   ├── board/
│   │   ├── board.c              # Board-level initialization
│   │   ├── display_support.c    # MIPI DSI + LCDIF configuration
│   │   ├── lvgl_support.c       # LVGL integration
│   │   └── pin_mux.c            # Panel GPIO pin muxing
│   ├── video/
│   │   ├── fsl_hx8394.c         # HX8394 driver (RK055MHD091)
│   │   ├── fsl_rm68200.c        # RM68200 driver (RK055AHD091)
│   │   ├── fsl_rm68191.c        # RM68191 driver (RK055IQH091)
│   │   ├── rpi.c                # RPI 7" driver
│   │   ├── rpi_support.c        # Attiny88 communication
│   │   ├── pca6416.c            # PCA6416 GPIO expander
│   │   └── fsl_dc_fb_lcdifv2.c  # Display controller abstraction
│   └── mpy_api.c                # Python<->C binding
├── imxrt1170_display/
│   ├── __init__.py              # Python API entry point
│   └── panels/
│       ├── base.py              # BasePanel class
│       ├── rpi_7inch.py         # RPI 7" implementation
│       ├── rk055ahd091.py       # RK055AHD091 panel
│       ├── rk055mhd091.py       # RK055MHD091 panel
│       └── rk055iqh091.py       # RK055IQH091 panel
├── micropython.mk               # MicroPython build integration
└── manifest.py                  # Package metadata
```

## Building

### As User C Module

Add to your MicroPython build:
```bash
make USER_C_MODULES=/path/to/imxrt1170_display_driver CFLAGS_EXTRA=-DMODULE_IMXRT1170_DISP_ENABLED=1
```

### Manifest Installation

Add to your `manifest.py`:
```python
require("imxrt1170_display_driver")
```

## Backward Compatibility

The old C-only API (`imxrt1170_disp.init()`) still works for backward compatibility when `DEMO_PANEL` is defined at compile time. However, the Python API is recommended for new projects.

Old style (deprecated):
```python
import imxrt1170_disp
imxrt1170_disp.init()  # Uses compile-time DEMO_PANEL
```

New style (recommended):
```python
from imxrt1170_display.panels import RK055MHD091Display
display = RK055MHD091Display()
display.init()
```

## Attribution

Display drivers based on NXP source code:
- https://github.com/nxp-appcodehub/dm-rt1170evkb-full-appliance/tree/60dc3d517f8a67034b8569fca6dc6c5094e0841a

RPI 7" display support derived from:
- https://github.com/nxp-appcodehub/dm-lvgl_cluster_rt1170_evkb_rpi
