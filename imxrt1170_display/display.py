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
Base Display Driver Class.

This module provides the base Display class that all specific display
drivers inherit from.
"""


class Display:
    """Base class for display drivers.

    This class provides the common interface for all display drivers.
    Specific display implementations should inherit from this class
    and implement display-specific functionality.

    Attributes:
        width (int): Display width in pixels
        height (int): Display height in pixels
        color_depth (int): Color depth in bits per pixel
    """

    def __init__(self, width, height, color_depth=24):
        """Initialize display configuration.

        Args:
            width (int): Display width in pixels
            height (int): Display height in pixels
            color_depth (int): Color depth in bits per pixel (default: 24)
        """
        self.width = width
        self.height = height
        self.color_depth = color_depth

    def init(self):
        """Initialize display hardware.

        This method should be overridden by subclasses to perform
        display-specific hardware initialization.

        Raises:
            NotImplementedError: If not implemented by subclass
        """
        raise NotImplementedError("Subclass must implement init()")

    def deinit(self):
        """Deinitialize display hardware.

        This method should be overridden by subclasses to perform
        display-specific hardware cleanup.

        Raises:
            NotImplementedError: If not implemented by subclass
        """
        raise NotImplementedError("Subclass must implement deinit()")

    def get_brightness(self):
        """Get current display brightness.

        Returns:
            int: Current brightness level (0-100)

        Raises:
            NotImplementedError: If not implemented by subclass
        """
        raise NotImplementedError("Subclass must implement get_brightness()")

    def set_brightness(self, level):
        """Set display brightness.

        Args:
            level (int): Brightness level (0-100)

        Raises:
            NotImplementedError: If not implemented by subclass
        """
        raise NotImplementedError("Subclass must implement set_brightness()")

    def lvgl_init(self):
        """Initialize LVGL graphics library.

        This calls the low-level C function to initialize LVGL with the
        display configuration, then starts the lv_utils event loop to
        drive lv_task_handler() and lv_tick_inc() via a hardware timer.
        Call this after init() to start using LVGL.

        Raises:
            OSError: If LVGL initialization fails
        """
        import imxrt1170_disp
        imxrt1170_disp.__init__()

        # Start LVGL event loop: drives lv.task_handler() + lv.tick_inc()
        # via a periodic machine.Timer interrupt so LVGL actually renders.
        from lv_utils import event_loop
        if not event_loop.is_running():
            self._event_loop = event_loop(timer_id=-1)  # MIMXRT only supports soft timer (-1)
        else:
            self._event_loop = None  # Already running, don't own it

    def lvgl_deinit(self):
        """Deinitialize LVGL graphics library.

        Stops the lv_utils event loop (if started by this instance) and
        calls the low-level C function to clean up LVGL resources.
        Call this before deinit() to ensure proper cleanup order.

        Raises:
            OSError: If LVGL deinitialization fails
        """
        event_loop = getattr(self, '_event_loop', None)
        if event_loop is not None:
            event_loop.deinit()
            self._event_loop = None

        import imxrt1170_disp
        imxrt1170_disp.deinit()
