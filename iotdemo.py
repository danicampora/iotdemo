#!/usr/bin/env python3

# IoT Sensors demo. A config file with the WLAN details
# and the Blynk token must be included. Requires the 
# expansion board, a DS18B20 sensors, a MS5637 barometic
# pressure sensor, and a solid state relay which controlls 
# a light bulb and a BH1750FVI digital light sensor.

import config
import onewire
import BlynkLib
import machine
from machine import Pin
from machine import RTC
from machine import I2C
from machine import WDT
from machine import ADC
from network import WLAN
from bh1750fvi import BH1750FVI
from ms5637 import MS5637
from ws2812 import WS2812

WIFI_SSID  = config.ssid
WIFI_AUTH  = config.auth
BLYNK_AUTH = config.token

MAIN_TASK_PERIOD = const(50)
WDT_TIMEOUT = const(15000)
LEDS_MAX_DELAY = const(10)

def connect_to_wlan(wlan):
    # try connecting to wifi until succeeding
    while True:
        try:
            wlan.connect(WIFI_SSID, auth=WIFI_AUTH, timeout=7500)
            while not wlan.isconnected():
                machine.idle()
            return
        except OSError:
            pass

class BatteryMonitor:
    levels = ((4115, 100), (4050, 95), (3990, 90), (3935, 85),
              (3875, 75), (3825, 70), (3785, 60), (3750, 50),
              (3700, 40), (3675, 30), (3650, 25), (3600, 20),
              (3500, 10), (3490, 5), (3400, 3), (3300, 1), (0, 0))

    def __init__(self, pin):
        adc = ADC(bits=12)
        self.apin = adc.channel(pin=pin)
        self.values = [0 for i in range(5)]
        self.volt = 4200
        self.chrg = 100
        self.idx = 0

    def _charge(self):
        for i in range(len(self.levels)):
            if self.volt >= self.levels[i][0]:
                chrg = self.levels[i][1]
                if chrg < self.chrg:
                    self.chrg = chrg
                break
        return self.chrg

    def read(self):
        # compensate for the resistors value and tolerance
        self.values[self.idx] = self.apin() * 117 // 100
        self.idx = (self.idx + 1) % len(self.values)
        if self.idx == 0: # the list is full
            self.volt = sum(self.values) // len(self.values)
            return self._charge()
        else:
            return None

class VirtualSw:
    def __init__(self, pin):
        self.pin = Pin(pin, Pin.OUT, pull=Pin.PULL_DOWN, value=0)
        self.vsw_value = 0
        self.pin_value = 0

    def _shift(self, value):
        self.pin(value)
        self.pin_value = value

    def __call__(self, value=None):
        if value is None:
            return (self.vsw_value, self.pin_value)
        else:
            self._shift(value) 

    def handler(self, value):
        self.vsw_value = int(value)
        self._shift(self.vsw_value)

class HwSw:
    SW_MAX_UPDATE_TIME = const(1000) # at least one update every second 

    def __init__(self, blynk, hw_pin, v_pin, period):
        self.blynk = blynk
        self.sw = Pin(hw_pin, Pin.IN, pull=Pin.PULL_UP)
        self.v_pin = v_pin
        self.value = self.sw()
        self.period = period
        self.time = 0

    def update(self):
        self.time = 0
        self.blynk.virtual_write(self.v_pin, not self.value)

    def check(self):
        value = self.sw()
        if value != self.value:
            self.value = value
            self.update()
            return not value
        else:
            self.time += self.period
            if self.time > SW_MAX_UPDATE_TIME:
                self.update()
        return False

class LedShow:
    #########    BLUE        GREEN        RED        WHITE         OFF
    colors = ((0, 0, 48), (0, 48, 0), (48, 0, 0), (16, 16, 16), (0, 0, 0))

    def __init__(self, period, nleds=12):
        self.nleds = nleds
        self.period = period
        self.delay = (LEDS_MAX_DELAY + 1) * period
        self.time = 0
        self.idx = 0
        self.chain = WS2812(nleds)
        self._shift(self.idx)

    def _shift(self, idx):
        self.data = [self.colors[idx] if n < (self.nleds - self.nleds // 3) else (0, 0 ,0) for n in range(self.nleds)]
        self.chain.show(self.data)

    def sweep(self):
        self.time += self.period
        if self.time >= self.delay:
            self.time = 0
            self.data = self.data[1:] + self.data[0:1]
            self.chain.show(self.data)

    def shift_handler(self, value):
        if int(value):
            self.idx = (self.idx + 1) % len(self.colors)
            self._shift(self.idx)

    def delay_handler(self, value):
        self.delay = ((LEDS_MAX_DELAY + 1) - int(value)) * self.period

class MainTask:
    BH1750FVI_ADDR = const(35)
    MS5637_ADDR = const(118)
    MAX_LUX_UPDATE_TIME = const(1000)
    LIGHT_DEBOUNCE_TIME = const(2500)
    LIGHT_HOLD_TIME = const(250)

    def __init__(self, blynk, ow_pin, sw1_pin, sw2_pin, relay, email, notify, battery, wdt, nleds, period):
        self.blynk = blynk
        self.ds18b20 = onewire.DS18X20(onewire.OneWire(Pin(ow_pin)))
        i2c = I2C(baudrate=100000, pins=('GP13', 'GP12'))
        self.bh1750fvi = BH1750FVI(i2c, BH1750FVI_ADDR, period)
        self.ms5637 = MS5637(i2c, MS5637_ADDR)
        self.sw1 = HwSw(blynk, sw1_pin, 1, period) # registered on vpin 1
        self.sw2 = HwSw(blynk, sw2_pin, 10, period) # registered on vpin 10

        self.ledshow = LedShow(period, nleds)
        # register the ledshow color handler on V2
        blynk.add_virtual_pin(2, write=self.ledshow.shift_handler)
        # register the ledshow delay handler on V9
        blynk.add_virtual_pin(9, write=self.ledshow.delay_handler)

        self.relay = relay
        self.email = email
        self.notify = notify
        self.battery = battery
        self.wdt = wdt
        self.period = period
        self.ds_state = 'CONV'
        self.lux = 0
        self.lux_time = 0
        self.ld_time = 0
        self.lh_time = 0
        self.lux_turn = False
        self.day_night = ''
        self.lsw = 0
        self.leds_period = period
        self.bar = 0

    def run(self):
        # feed the watchdog
        self.wdt.feed()

        if self.ds_state == 'CONV':
            self.ds18b20.start_convertion(self.ds18b20.roms[0])
            self.ds_state = 'READ'
        elif self.ds_state == 'READ':
            tmp = self.ds18b20.read_temp_async(self.ds18b20.roms[0])
            if tmp != None:
                self.blynk.virtual_write(3, '{:02d}.{:02d}'.format(tmp // 100, tmp % 100))
                self.ds_state = 'CONV'

        # read the debounced push button 1 state
        if self.sw1.check():
            self.notify.send('You pressed the red button and I know it ;)')

        # check also push button 2
        if self.sw2.check():
            self.email.send('[WiPy] IoT Demo', 'You pressed the green button and I know it ;)')

        # only check the Lux value on every other cycle
        if self.lux_turn:
            self.lux_turn = False
            lux = self.bh1750fvi.read()
            if self.lux != lux:
                self.lux = lux
                self.blynk.virtual_write(8, self.lux)
                if self.lux > 100:
                    day_night = 'Day'
                else:
                    day_night = 'Night'
                if self.day_night != day_night:
                    self.day_night = day_night
                    self.blynk.virtual_write(12, self.day_night)
            else:
                self.lux_time += self.period
                if self.lux_time >= MAX_LUX_UPDATE_TIME:
                    self.lux_time = 0
                    self.blynk.virtual_write(8, self.lux)
                    self.blynk.virtual_write(12, self.day_night)
        else:
            self.lux_turn = True

        # keep the led show running
        self.ledshow.sweep()

        # show the battery charge
        charge = self.battery.read()
        if charge != None:
            self.blynk.virtual_write(11, '{} %'.format(charge)) # vpin 11

        # run the barometric sensor task
        self.ms5637.run()
        bar = self.ms5637.bar()
        if self.bar != bar:
            self.bar = bar
            self.blynk.virtual_write(4, '{}'.format(bar // 100)) # vpin 4

        # now some logic to control the light automatically
        lsw, rly = self.relay()
        if self.lsw != lsw: # reset the debounce time
            self.lsw = lsw
            self.ld_time = 0
        if lsw: # is the light switch on?
            if self.ld_time < LIGHT_DEBOUNCE_TIME:
                self.ld_time += self.period
            else:
                if rly: # if the relay is on
                    if self.lux > 250:
                        self.lh_time += self.period
                        if self.lh_time > LIGHT_HOLD_TIME:
                            self.relay(0) # turn the light off
                            self.ld_time = 0
                            self.lh_time = 0
                    else:
                        self.lh_time = 0
                else:
                    if self.lux < 220:
                        self.lh_time += self.period
                        if self.lh_time > LIGHT_HOLD_TIME:
                            self.relay(1) # turn the light on
                            self.ld_time = 0
                            self.lh_time = 0
                    else:
                        self.lh_time = 0

class Email:
    def __init__(self, blynk):
        self.blynk = blynk
        self.enabled = False
        self.to = 'info@wipy.io'

    def handler(self, value):
        self.enabled = int(value)

    def send(self, subject, body, to=None):
        if self.enabled:
            if to == None:
                to = self.to
            self.blynk.email(to, subject, body)

class Notify:
    def __init__(self, blynk):
        self.blynk = blynk
        self.enabled = False

    def handler(self, value):
        self.enabled = int(value)

    def send(self, msg):
        if self.enabled:
            self.blynk.notify(msg)

wdt = WDT(timeout=WDT_TIMEOUT)

wlan = WLAN(mode=WLAN.STA)
connect_to_wlan(wlan) # the WDT will reset if this takes more than WDT_TIMEOUT seconds

wdt.feed()

# set the current time (mandatory to validate certificates)
RTC(datetime=(2016, 1, 1, 0, 0, 0, 0, None))

# initialize Blynk with SSL enabled
blynk = BlynkLib.Blynk(BLYNK_AUTH, wdt=False, ssl=True)

# register the email handler on V5
email = Email(blynk)
blynk.add_virtual_pin(5, write=email.handler)

# register the tweet handler on V6
notify = Notify(blynk)
blynk.add_virtual_pin(6, write=notify.handler)

# register the light switch relay write handler on V7
relay = VirtualSw('GP23')
blynk.add_virtual_pin(7, write=relay.handler)

# instantiate the battery monitor
battery = BatteryMonitor('GP3')

# register the sensors task as the user task (uses V3 and V4)
s_task = MainTask(blynk, 'GP30', 'GP17', 'GP14', relay, email, notify, battery, wdt, 36, MAIN_TASK_PERIOD)
blynk.set_user_task(s_task.run, MAIN_TASK_PERIOD)

# register the terminal REPL on v0
term = blynk.repl(0)
os.dupterm(term)

while True:
    wdt.feed()
    try:
        blynk.run() # run Blynk
    except MemoryError:
        machine.reset()
    except Exception as e:
        print(repr(e))
        if not wlan.isconnected():
            connect_to_wlan(wlan)
