# -*- coding: utf-8 -*-
# Based on https://github.com/JanBednarik/micropython-ws2812

# Driver for the WS2812 RGB LEDs. May be used for controlling
# a single LED or chain of LEDs.
#
# Example of use:
#
# from ws2812 import WS2812
#
# chain = WS2812(nleds=4)
# data = [
#     (255, 0, 0),    # red
#     (0, 255, 0),    # green
#     (0, 0, 255),    # blue
#     (85, 85, 85),   # white
# ]
# chain.show(data)
#
# Version: 1.0

from machine import SPI
from machine import Pin
from machine import disable_irq
from machine import enable_irq

class WS2812:
    # values to put inside the SPI register for each bit of color
    bits = (0xE0, 0xFC)

    def __init__(self, nleds=1):
        # params: * nleds = number of LEDs

        self.buf = bytearray(nleds * 3 * 8)  # 1 byte per bit
        # spi init
        # bus 0, 8MHz => 125 ns by bit, each transfer is 10 cycles long due to the
        # CC3200 spi dead cycles, therefore => 125*10=1.25 us as required by the
        # WS2812. Don't do the automatic pin assigment since we only need MOSI
        self.spi = SPI(0, SPI.MASTER, baudrate=8000000, pins=None)
        Pin('GP16', mode=Pin.ALT, pull=Pin.PULL_DOWN, alt=7)
        # turn all the LEDs off
        self.show([])

    def _send(self):
        # send the buffer over SPI.

        disable_irq()
        self.spi.write(self.buf)
        enable_irq()

    def show(self, data):
        # show RGB data on the LEDs. Expected data = [(R, G, B), ...] where
        # R, G and B are the intensities of the colors in the range 0 to 255.
        # the number of tuples may be less than the number of connected LEDs.

        self.fill(data)
        self._send()

    def update(self, data, start=0):
        # fill a part of the buffer with RGB data.
        # Returns the index of the first unfilled LED.

        buf = self.buf
        bits = self.bits
        idx = start * 24
        for colors in data:
            for c in (1, 0, 2): # the WS2812 requires GRB
                color = colors[c]
                for bit in range (0, 8):
                    buf[idx + bit] = bits[color >> (7 - bit) & 0x01]
                idx += 8
        return idx // 24

    def fill(self, data):
        # fill the buffer with RGB data.
        # all the LEDs after the data are turned off.

        end = self.update(data)
        buf = self.buf
        off = self.bits[0]
        for idx in range(end * 24, len(self.buf)):
            buf[idx] = off
