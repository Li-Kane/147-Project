#!/usr/bin/env python

# Copyright (c) 2019-2022, NVIDIA CORPORATION. All rights reserved.
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

import Jetson.GPIO as GPIO
import time

# Pin Definitions
output_pin = 6  # BCM pin 18, BOARD pin 12

def main():
    # Pin Setup:
    GPIO.setmode(GPIO.BCM)  # BCM pin-numbering scheme from Raspberry Pi
    # set pin as an output pin with optional initial state of HIGH
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)
    GPIO.output(output_pin, GPIO.LOW)

    try:
        while True:
            continue
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()


#(106, 'PQ.06', "tegra234-gpio", 31, 6, 'GPIO11', 'GP66', None, None),
# PADCTL_G3_SOC_GPIO33_0
# Offset: 0x70
# Read/Write: R/W
# Parity Protection: Y
# Shadow: N
# SCR Protection: PADCTL_G3_SCR_SCR_SOC_GPIO33_0
# Reset: 0x00000474 (0bxxxx,xxxx,xxxx,xxxx,xxx0,x1x0,0111,0100)

# how I set BCM and stuff
# https://docs.nvidia.com/jetson/archives/r36.3/DeveloperGuide/HR/JetsonModuleAdaptationAndBringUp/JetsonAgxOrinSeries.html?highlight=pin%20direction#changing-the-pinmux