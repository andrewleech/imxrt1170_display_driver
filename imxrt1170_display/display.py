# -*- coding: utf-8 -*-
#
# Copyright (c) 2025, Planet Innovation
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
Base class for i.MX RT1170 display panels.

This module provides an abstract base class for display drivers,
allowing display-specific initialization and configuration to be implemented
in Python rather than C. The low-level display and LVGL initialization is
handled by the C module for performance.
"""

import imxrt1170_disp


class Display:
    """Abstract base class for i.MX RT1170 display panels.

    Subclasses should implement:
    - init(): Initialize display hardware (power-up, configure panel)
    - deinit(): Deinitialize display hardware (power-down, cleanup)

    The base class provides integration with the low-level C driver:
    - lvgl_init(): Initialize LVGL graphics library
    - lvgl_deinit(): Deinitialize LVGL graphics library

    Attributes:
        width (int): Display width in pixels
        height (int): Display height in pixels
        color_depth (int): Color depth in bits (16, 24, 32)
    """

    def __init__(self, width=800, height=480, color_depth=16):
        """Initialize display base class.

        Args:
            width: Display width in pixels (default: 800)
            height: Display height in pixels (default: 480)
            color_depth: Color depth in bits (default: 16)
        """
        self.width = width
        self.height = height
        self.color_depth = color_depth

    def init(self):
        """Initialize the display hardware.

        This method should be implemented by subclasses to:
        - Configure GPIO pins for display control
        - Initialize display controller
        - Configure display timings
        - Power on the display panel
        - Perform any panel-specific initialization

        After this method completes, the display should be ready for
        LVGL initialization via lvgl_init().

        Raises:
            NotImplementedError: Must be implemented by subclass
        """
        raise NotImplementedError("Subclass must implement init()")

    def deinit(self):
        """Deinitialize the display hardware.

        This method should be implemented by subclasses to:
        - Power down the display panel
        - Release GPIO pins
        - Clean up any allocated resources

        This should be called after lvgl_deinit() to ensure proper cleanup.

        Raises:
            NotImplementedError: Must be implemented by subclass
        """
        raise NotImplementedError("Subclass must implement deinit()")

    def lvgl_init(self):
        """Initialize LVGL graphics library.

        This calls the low-level C function to initialize LVGL with the
        display configuration. Call this after init() to start using LVGL.

        Raises:
            OSError: If LVGL initialization fails
        """
        imxrt1170_disp.__init__()

    def lvgl_deinit(self):
        """Deinitialize LVGL graphics library.

        This calls the low-level C function to clean up LVGL resources.
        Call this before deinit() to ensure proper cleanup order.

        Raises:
            OSError: If LVGL deinitialization fails
        """
        imxrt1170_disp.deinit()
