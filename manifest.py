"""
MicroPython Package Manifest for imxrt1170_display

This manifest allows the package to be installed via:
    mpremote mip install github:user/repo

Or included in a MicroPython build via:
    include("path/to/manifest.py")
"""

# Package metadata
metadata(
    version="1.0.0",
    description="MIPI DSI display driver for NXP i.MX RT1170 with LVGL support",
    author="NXP + Community",
    license="BSD-3-Clause",
)

# Include all Python files from the package
package(
    "imxrt1170_display",
    base_path=".",
)

# Note: The C module (imxrt1170_disp) is built separately via micropython.mk
# and must be included in the MicroPython firmware build.
