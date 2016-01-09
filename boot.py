from machine import UART
from machine import Pin
import machine
import os
import time
uart = UART(0, baudrate=57600)
os.dupterm(uart)

bpin = Pin('GP14', mode=Pin.IN, pull=Pin.PULL_UP)
for i in range(10):
    time.sleep_ms(50)
    if bpin():
        break

if bpin():
    # run the demo if the second pushbutton is not pressed
    machine.main('iotdemo.py')
