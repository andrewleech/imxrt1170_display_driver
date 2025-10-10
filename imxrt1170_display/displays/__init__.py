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
Specific display panel drivers for i.MX RT1170.

This subpackage contains implementations for various display panels,
each providing panel-specific initialization and configuration.
"""

from .rpi_7inch import RPI7InchDisplay

__all__ = ["RPI7InchDisplay"]
