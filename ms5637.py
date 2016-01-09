#!/usr/bin/env python3

# Simple driver for the MS5637 digital barometric pressure sensor

import time

class MS5637:
    def __init__(self, i2c, addr=118):
        self.i2c = i2c
        self.addr = addr
        self.state = 'CAL'
        self.C1 = 0
        self.C2 = 0
        self.C3 = 0
        self.C4 = 0
        self.C5 = 0
        self.C6 = 0
        self.D1 = 0
        self.D2 = 0
        self._bar = 0
        # send a reset command
        self.i2c.writeto(self.addr, 0x1E)

    def _read_mem(self, mem_addr):
        cal = self.i2c.readfrom_mem(self.addr, mem_addr, 2)
        return (cal[0] << 8) + cal[1]

    def _convert_pressure(self):
        # start the convertion
        self.i2c.writeto(self.addr, 0x42)  # 512 OSR

    def _convert_temp(self):
        # start the convertion
        self.i2c.writeto(self.addr, 0x52)  # 512 OSR

    def _read_adc(self):
        time.sleep_ms(2)
        adc = self.i2c.readfrom_mem(self.addr, 0x00, 3)
        return (adc[0] << 16) + (adc[1] << 8) + adc[2]

    def _read_cal(self):
        crc = self._read_mem(0xA0)
        self.C1 = self._read_mem(0xA2)
        self.C2 = self._read_mem(0xA4)
        self.C3 = self._read_mem(0xA6)
        self.C4 = self._read_mem(0xA8)
        self.C5 = self._read_mem(0xAA)
        self.C6 = self._read_mem(0xAC)

    def run(self):
        if self.state == 'CAL':
            self._read_cal()
            self.state = 'PRS_C'
        elif self.state == 'PRS_C':
            self._convert_pressure()
            self.state = 'PRS_R'
        elif self.state == 'PRS_R':
            self.D1 = self._read_adc()
            self.state = 'TMP_C'
        elif self.state == 'TMP_C':
            self._convert_temp()
            self.state = 'TMP_R'
        elif self.state == 'TMP_R':
            self.D2 = self._read_adc()
            self.state = 'BAR'
        else:
            dT = self.D2 - (self.C5 * 256)
            TEMP = 2000 + ((dT * self.C6) // 8388608)
            OFF = (self.C2 * 131072) + ((self.C4 * dT) // 64)
            SENS = (self.C1 * 65536) + ((self.C3 * dT) // 128)
            self._bar = (((self.D1 * SENS) // 2097152) - OFF) // 32768
            self.state = 'PRS_C'

    def bar(self):
        return self._bar
