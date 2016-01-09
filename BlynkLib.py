import socket
import struct
import time
import os
import machine
import wipy

HDR_LEN = const(5)
HDR_FMT = "!BHH"

MAX_MSG_PER_SEC = const(20)

MSG_RSP = const(0)
MSG_LOGIN = const(2)
MSG_PING  = const(6)
MSG_TWEET = const(12)
MSG_EMAIL = const(13)
MSG_NOTIFY = const(14)
MSG_BRIDGE = const(15)
MSG_HW = const(20)

STA_SUCCESS = const(200)

HB_PERIOD = const(10)
NON_BLK_SOCK = const(0)
MIN_SOCK_TO = const(1) # 1 second
MAX_SOCK_TO = const(5) # 5 seconds, must be < HB_PERIOD
WDT_TO = const(10000) # 10 seconds
RECONNECT_DELAY = const(1) # 1 second
TASK_PERIOD_RES = const(50) # 50 ms
IDLE_TIME_MS = const(5) # 5 ms

RE_TX_DELAY = const(2)
MAX_TX_RETRIES = const(3)

MAX_VIRTUAL_PINS = const(32)

DISCONNECTED = 0
CONNECTING = 1
AUTHENTICATING = 2
AUTHENTICATED = 3

EAGAIN = const(11)

def sleep_from_until (start, delay):
    while time.ticks_diff(start, time.ticks_ms()) < delay:
        machine.idle()
    return start + delay

class HwPin:
    _PWMMap = {'GP9': 3, 'GP10': 3, 'GP11': 3, 'GP24': 5, 'GP25': 9}
    _TimerMap = { 'GP9': (3, machine.Timer.B),
                 'GP10': (4, machine.Timer.A),
                 'GP11': (4, machine.Timer.B),
                 'GP24': (1, machine.Timer.A),
                 'GP25': (2, machine.Timer.A)}

    _HBPin = 25 if 'WiPy' in os.uname().machine else 9

    def __init__(self, pin_num, mode, pull):
        self._mode = mode
        self._pull = pull
        self._function = ''
        self._pin = None
        self._apin = None
        self._pwm = None
        pin_num = int(pin_num)
        self._name = 'GP' + str(pin_num)
        if pin_num == HwPin._HBPin:
            wipy.heartbeat(False)

    def _config(self, duty_cycle=0):
        if self._function == 'dig':
            _mode = machine.Pin.OUT if self._mode == 'out' else machine.Pin.IN
            if self._pull == 'pu':
                _pull = machine.Pin.PULL_UP
            elif self._pull == 'pd':
                _pull = machine.Pin.PULL_DOWN
            else:
                _pull = None
            self._pin = machine.Pin(self._name, mode=_mode, pull=_pull, drive=machine.Pin.MED_POWER)
        elif self._function == 'ana':
            adc = machine.ADC(bits=12)
            self._apin = adc.channel(pin=self._name)
        else:
            machine.Pin(self._name, mode=machine.Pin.ALT, pull=None, drive=machine.Pin.MED_POWER, alt=HwPin._PWMMap[self._name])
            timer = machine.Timer(HwPin._TimerMap[self._name][0], mode=machine.Timer.PWM)
            self._pwm = timer.channel(HwPin._TimerMap[self._name][1], freq=20000, duty_cycle=duty_cycle)

    def digital_read(self):
        if self._function != 'dig':
            self._function = 'dig'
            self._config()
        return self._pin()

    def digital_write(self, value):
        if self._function != 'dig':
            self._function = 'dig'
            self._config()
        self._pin(value)

    def analog_read(self):
        if self._function != 'ana':
            self._function = 'ana'
            self._config()
        return self._apin()

    def analog_write(self, value):
        if self._function != 'pwm':
            self._function = 'pwm'
            self._config(value)
        else:
            self._pwm.duty_cycle(value)

class VrPin:
    def __init__(self, read=None, write=None):
        self.read = read
        self.write = write

class Terminal:
    def __init__(self, blynk, pin):
        self._blynk = blynk
        self._pin = pin

    def write(self, data):
        self._blynk.virtual_write(self._pin, data)

    def read(self, size):
        return ''

    def virtual_read(self):
        pass

    def virtual_write(self, value):
        try:
            out = eval(value)
            if out != None:
                print(repr(out))
        except:
            try:
                exec(value)
            except Exception as e:
                print('Exception:\n  ' + repr(e))

class Blynk:
    def __init__(self, token, server='cloud.blynk.cc', port=None, connect=True, wdt=True, ssl=False):
        self._wdt = None
        self._vr_pins = {}
        self._do_connect = False
        self._task = None
        self._task_period = 0
        self._token = token
        if isinstance (self._token, str):
            self._token = bytes(token, 'ascii')
        self._server = server
        if port is None:
            if ssl:
                port = 8441
            else:
                port = 8442
        self._port = port
        self._do_connect = connect
        self._wdt = wdt
        self._ssl = ssl
        self.state = DISCONNECTED

    def _format_msg(self, msg_type, *args):
        data = bytes('\0'.join(map(str, args)), 'ascii')
        return struct.pack(HDR_FMT, msg_type, self._new_msg_id(), len(data)) + data

    def _handle_hw(self, data):
        params = list(map(lambda x: x.decode('ascii'), data.split(b'\0')))
        cmd = params.pop(0)
        if cmd == 'info':
            pass
        elif cmd == 'pm':
            pairs = zip(params[0::2], params[1::2])
            for (pin, mode) in pairs:
                pin = int(pin)
                if mode != 'in' and mode != 'out' and mode != 'pu' and mode != 'pd':
                    raise ValueError('')
                self._hw_pins[pin] = HwPin(pin, mode, mode)
            self._pins_configured = True
        elif cmd == 'vw':
            pin = int(params.pop(0))
            if pin in self._vr_pins and self._vr_pins[pin].write:
                for param in params:
                    self._vr_pins[pin].write(param)
        elif cmd == 'vr':
            pin = int(params.pop(0))
            if pin in self._vr_pins and self._vr_pins[pin].read:
                self._vr_pins[pin].read()
        elif self._pins_configured:
            if cmd == 'dw':
                pin = int(params.pop(0))
                val = int(params.pop(0))
                self._hw_pins[pin].digital_write(val)
            elif cmd == 'aw':
                pin = int(params.pop(0))
                val = int(params.pop(0))
                self._hw_pins[pin].analog_write(val)
            elif cmd == 'dr':
                pin = int(params.pop(0))
                val = self._hw_pins[pin].digital_read()
                self._send(self._format_msg(MSG_HW, 'dw', pin, val))
            elif cmd == 'ar':
                pin = int(params.pop(0))
                val = self._hw_pins[pin].analog_read()
                self._send(self._format_msg(MSG_HW, 'aw', pin, val))

    def _new_msg_id(self):
        self._msg_id += 1
        if (self._msg_id > 0xFFFF):
            self._msg_id = 1
        return self._msg_id

    def _settimeout(self, timeout):
        if timeout != self._timeout:
            self._timeout = timeout
            self.conn.settimeout(timeout)

    def _recv(self, length, timeout=0):
        self._settimeout (timeout)
        try:
            self._rx_data += self.conn.recv(length)
        except socket.timeout:
            return b''
        except socket.error as e:
            if e.args[0] ==  EAGAIN:
                return b''
            else:
                raise
        if len(self._rx_data) >= length:
            data = self._rx_data[:length]
            self._rx_data = self._rx_data[length:]
            return data
        else:
            return b''

    def _send(self, data, send_anyway=False):
        if self._tx_count < MAX_MSG_PER_SEC or send_anyway:
            retries = 0
            while retries <= MAX_TX_RETRIES:
                try:
                    self.conn.send(data)
                    self._tx_count += 1
                    break
                except socket.error as er:
                    if er.args[0] != EAGAIN:
                        raise
                    else:
                        time.sleep_ms(RE_TX_DELAY)
                        retries += 1

    def _close(self):
        self.conn.close()
        self.state = DISCONNECTED
        time.sleep(RECONNECT_DELAY)

    def _server_alive(self):
        c_time = int(time.time())
        if self._m_time != c_time:
            self._m_time = c_time
            self._tx_count = 0
            if self._wdt:
                self._wdt.feed()
            if self._last_hb_id != 0 and c_time - self._hb_time >= MAX_SOCK_TO:
                return False
            if c_time - self._hb_time >= HB_PERIOD and self.state == AUTHENTICATED:
                self._hb_time = c_time
                self._last_hb_id = self._new_msg_id()
                self._send(struct.pack(HDR_FMT, MSG_PING, self._last_hb_id, 0), True)
        return True

    def _run_task(self):
        if self._task:
            c_millis = time.ticks_ms()
            if c_millis - self._task_millis >= self._task_period:
                self._task_millis += self._task_period
                self._task()

    def repl(self, pin):
        repl = Terminal(self, pin)
        self.add_virtual_pin(pin, repl.virtual_read, repl.virtual_write)
        return repl

    def notify(self, msg):
        if self.state == AUTHENTICATED:
            self._send(self._format_msg(MSG_NOTIFY, msg))

    def email(self, to, subject, body):
        if self.state == AUTHENTICATED:
            self._send(self._format_msg(MSG_EMAIL, to, subject, body))

    def virtual_write(self, pin, val):
        if self.state == AUTHENTICATED:
            self._send(self._format_msg(MSG_HW, 'vw', pin, val))

    def add_virtual_pin(self, pin, read=None, write=None):
        if isinstance(pin, int) and pin in range(0, MAX_VIRTUAL_PINS):
            self._vr_pins[pin] = VrPin(read, write)
        else:
            raise ValueError('')

    def set_user_task(self, task, ms_period):
        if ms_period % TASK_PERIOD_RES != 0:
            raise ValueError('')
        self._task = task
        self._task_period = ms_period

    def connect(self):
        self._do_connect = True

    def disconnect(self):
        self._do_connect = False

    def run(self):
        self._start_time = time.ticks_ms()
        self._task_millis = self._start_time
        self._hw_pins = {}
        self._rx_data = b''
        self._msg_id = 1
        self._pins_configured = False
        self._timeout = None
        self._tx_count = 0
        self._m_time = 0
        self.state = DISCONNECTED

        if self._wdt:
            self._wdt = machine.WDT(timeout=WDT_TO)

        while True:
            while self.state != AUTHENTICATED:
                self._run_task()
                if self._wdt:
                    self._wdt.feed()
                if self._do_connect:
                    try:
                        self.state = CONNECTING
                        if self._ssl:
                            import ssl
                            ss = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_SEC)
                            self.conn = ssl.wrap_socket(ss, cert_reqs=ssl.CERT_REQUIRED, ca_certs='/flash/cert/ca.pem')
                        else:
                            self.conn = socket.socket()
                        self.conn.connect(socket.getaddrinfo(self._server, self._port)[0][4])
                    except:
                        self._close()
                        continue

                    self.state = AUTHENTICATING
                    hdr = struct.pack(HDR_FMT, MSG_LOGIN, self._new_msg_id(), len(self._token))
                    self._send(hdr + self._token, True)
                    data = self._recv(HDR_LEN, timeout=MAX_SOCK_TO)
                    if not data:
                        self._close()
                        continue

                    msg_type, msg_id, status = struct.unpack(HDR_FMT, data)
                    if status != STA_SUCCESS or msg_id == 0:
                        self._close()
                        continue

                    self.state = AUTHENTICATED
                else:
                    self._start_time = sleep_from_until(self._start_time, TASK_PERIOD_RES)

            self._hb_time = 0
            self._last_hb_id = 0
            self._tx_count = 0
            while self._do_connect:
                data = self._recv(HDR_LEN, NON_BLK_SOCK)
                if data:
                    msg_type, msg_id, msg_len = struct.unpack(HDR_FMT, data)
                    if msg_id == 0:
                        self._close()
                        break
                    if msg_type == MSG_RSP:
                        if msg_id == self._last_hb_id:
                            self._last_hb_id = 0
                    elif msg_type == MSG_PING:
                        self._send(struct.pack(HDR_FMT, MSG_RSP, msg_id, STA_SUCCESS), True)
                    elif msg_type == MSG_HW or msg_type == MSG_BRIDGE:
                        data = self._recv(msg_len, MIN_SOCK_TO)
                        if data:
                            self._handle_hw(data)
                    else:
                        self._close()
                        break
                else:
                    self._start_time = sleep_from_until(self._start_time, IDLE_TIME_MS)
                if not self._server_alive():
                    self._close()
                    break
                self._run_task()

            if not self._do_connect:
                self._close()
