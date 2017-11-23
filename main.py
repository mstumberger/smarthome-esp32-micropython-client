from machine import Pin, ADC, unique_id, I2C
from onewire import OneWire
from dht import DHT11
import ubinascii
from micropython import const
import usocket as socket
import ustruct as struct
from ubinascii import hexlify
import network
import ujson
import time
import framebuf
import config
from time import sleep
from ds18x20 import DS18X20

# register definitions
SET_CONTRAST        = const(0x81)
SET_ENTIRE_ON       = const(0xa4)
SET_NORM_INV        = const(0xa6)
SET_DISP            = const(0xae)
SET_MEM_ADDR        = const(0x20)
SET_COL_ADDR        = const(0x21)
SET_PAGE_ADDR       = const(0x22)
SET_DISP_START_LINE = const(0x40)
SET_SEG_REMAP       = const(0xa0)
SET_MUX_RATIO       = const(0xa8)
SET_COM_OUT_DIR     = const(0xc0)
SET_DISP_OFFSET     = const(0xd3)
SET_COM_PIN_CFG     = const(0xda)
SET_DISP_CLK_DIV    = const(0xd5)
SET_PRECHARGE       = const(0xd9)
SET_VCOM_DESEL      = const(0xdb)
SET_CHARGE_PUMP     = const(0x8d)


class SSD1306:
    def __init__(self, width, height, external_vcc):
        self.width = width
        self.height = height
        self.external_vcc = external_vcc
        self.pages = self.height // 8
        # Note the subclass must initialize self.framebuf to a framebuffer.
        # This is necessary because the underlying data buffer is different
        # between I2C and SPI implementations (I2C needs an extra byte).
        self.poweron()
        self.init_display()

    def init_display(self):
        for cmd in (
            SET_DISP | 0x00, # off
            # address setting
            SET_MEM_ADDR, 0x00, # horizontal
            # resolution and layout
            SET_DISP_START_LINE | 0x00,
            SET_SEG_REMAP | 0x01, # column addr 127 mapped to SEG0
            SET_MUX_RATIO, self.height - 1,
            SET_COM_OUT_DIR | 0x08, # scan from COM[N] to COM0
            SET_DISP_OFFSET, 0x00,
            SET_COM_PIN_CFG, 0x02 if self.height == 32 else 0x12,
            # timing and driving scheme
            SET_DISP_CLK_DIV, 0x80,
            SET_PRECHARGE, 0x22 if self.external_vcc else 0xf1,
            SET_VCOM_DESEL, 0x30, # 0.83*Vcc
            # display
            SET_CONTRAST, 0xff, # maximum
            SET_ENTIRE_ON, # output follows RAM contents
            SET_NORM_INV, # not inverted
            # charge pump
            SET_CHARGE_PUMP, 0x10 if self.external_vcc else 0x14,
            SET_DISP | 0x01): # on
            self.write_cmd(cmd)
        self.fill(0)
        self.show()

    def poweroff(self):
        self.write_cmd(SET_DISP | 0x00)

    def contrast(self, contrast):
        self.write_cmd(SET_CONTRAST)
        self.write_cmd(contrast)

    def invert(self, invert):
        self.write_cmd(SET_NORM_INV | (invert & 1))

    def show(self):
        x0 = 0
        x1 = self.width - 1
        if self.width == 64:
            # displays with width of 64 pixels are shifted by 32
            x0 += 32
            x1 += 32
        self.write_cmd(SET_COL_ADDR)
        self.write_cmd(x0)
        self.write_cmd(x1)
        self.write_cmd(SET_PAGE_ADDR)
        self.write_cmd(0)
        self.write_cmd(self.pages - 1)
        self.write_framebuf()

    def fill(self, col):
        self.framebuf.fill(col)

    def pixel(self, x, y, col):
        self.framebuf.pixel(x, y, col)

    def scroll(self, dx, dy):
        self.framebuf.scroll(dx, dy)

    def text(self, string, x, y, col=1):
        self.framebuf.text(string, x, y, col)


class SSD1306_I2C(SSD1306):
    def __init__(self, width, height, i2c, addr=0x3c, external_vcc=False):
        self.i2c = i2c
        self.addr = addr
        self.temp = bytearray(2)
        # Add an extra byte to the data buffer to hold an I2C data/command byte
        # to use hardware-compatible I2C transactions.  A memoryview of the
        # buffer is used to mask this byte from the framebuffer operations
        # (without a major memory hit as memoryview doesn't copy to a separate
        # buffer).
        self.buffer = bytearray(((height // 8) * width) + 1)
        self.buffer[0] = 0x40  # Set first byte of data buffer to Co=0, D/C=1
        self.framebuf = framebuf.FrameBuffer1(memoryview(self.buffer)[1:], width, height)
        super().__init__(width, height, external_vcc)

    def write_cmd(self, cmd):
        self.temp[0] = 0x80 # Co=1, D/C#=0
        self.temp[1] = cmd
        self.i2c.writeto(self.addr, self.temp)

    def write_framebuf(self):
        # Blast out the frame buffer using a single I2C transaction to support
        # hardware I2C interfaces.
        self.i2c.writeto(self.addr, self.buffer)

    def poweron(self):
        pass


class MQTTException(Exception):
    pass


class MQTTClient:

    def __init__(self, client_id, server, port=0, user=None, password=None, keepalive=0,
                 ssl=False, ssl_params={}):
        if port == 0:
            port = 8883 if ssl else 1883
        self.client_id = client_id
        self.sock = None
        self.server = server
        self.port = port
        self.ssl = ssl
        self.ssl_params = ssl_params
        self.pid = 0
        self.cb = None
        self.user = user
        self.pswd = password
        self.keepalive = keepalive
        self.lw_topic = None
        self.lw_msg = None
        self.lw_qos = 0
        self.lw_retain = False

    def _send_str(self, s):
        self.sock.write(struct.pack("!H", len(s)))
        self.sock.write(s)

    def _recv_len(self):
        n = 0
        sh = 0
        while 1:
            b = self.sock.read(1)[0]
            n |= (b & 0x7f) << sh
            if not b & 0x80:
                return n
            sh += 7

    def set_callback(self, f):
        self.cb = f

    def set_last_will(self, topic, msg, retain=False, qos=0):
        assert 0 <= qos <= 2
        assert topic
        self.lw_topic = topic
        self.lw_msg = msg
        self.lw_qos = qos
        self.lw_retain = retain

    def connect(self, clean_session=True):
        self.sock = socket.socket()
        addr = socket.getaddrinfo(self.server, self.port)[0][-1]
        self.sock.connect(addr)
        if self.ssl:
            import ussl
            self.sock = ussl.wrap_socket(self.sock, **self.ssl_params)
        premsg = bytearray(b"\x10\0\0\0\0\0")
        msg = bytearray(b"\x04MQTT\x04\x02\0\0")

        sz = 10 + 2 + len(self.client_id)
        msg[6] = clean_session << 1
        if self.user is not None:
            sz += 2 + len(self.user) + 2 + len(self.pswd)
            msg[6] |= 0xC0
        if self.keepalive:
            assert self.keepalive < 65536
            msg[7] |= self.keepalive >> 8
            msg[8] |= self.keepalive & 0x00FF
        if self.lw_topic:
            sz += 2 + len(self.lw_topic) + 2 + len(self.lw_msg)
            msg[6] |= 0x4 | (self.lw_qos & 0x1) << 3 | (self.lw_qos & 0x2) << 3
            msg[6] |= self.lw_retain << 5

        i = 1
        while sz > 0x7f:
            premsg[i] = (sz & 0x7f) | 0x80
            sz >>= 7
            i += 1
        premsg[i] = sz

        self.sock.write(premsg, i + 2)
        self.sock.write(msg)
        #print(hex(len(msg)), hexlify(msg, ":"))
        self._send_str(self.client_id)
        if self.lw_topic:
            self._send_str(self.lw_topic)
            self._send_str(self.lw_msg)
        if self.user is not None:
            self._send_str(self.user)
            self._send_str(self.pswd)
        resp = self.sock.read(4)
        assert resp[0] == 0x20 and resp[1] == 0x02
        if resp[3] != 0:
            raise MQTTException(resp[3])
        return resp[2] & 1

    def disconnect(self):
        self.sock.write(b"\xe0\0")
        self.sock.close()

    def ping(self):
        self.sock.write(b"\xc0\0")

    def publish(self, topic, msg, retain=False, qos=0):
        pkt = bytearray(b"\x30\0\0\0")
        pkt[0] |= qos << 1 | retain
        sz = 2 + len(topic) + len(msg)
        if qos > 0:
            sz += 2
        assert sz < 2097152
        i = 1
        while sz > 0x7f:
            pkt[i] = (sz & 0x7f) | 0x80
            sz >>= 7
            i += 1
        pkt[i] = sz
        #print(hex(len(pkt)), hexlify(pkt, ":"))
        self.sock.write(pkt, i + 1)
        self._send_str(topic)
        if qos > 0:
            self.pid += 1
            pid = self.pid
            struct.pack_into("!H", pkt, 0, pid)
            self.sock.write(pkt, 2)
        self.sock.write(msg)
        if qos == 1:
            while 1:
                op = self.wait_msg()
                if op == 0x40:
                    sz = self.sock.read(1)
                    assert sz == b"\x02"
                    rcv_pid = self.sock.read(2)
                    rcv_pid = rcv_pid[0] << 8 | rcv_pid[1]
                    if pid == rcv_pid:
                        return
        elif qos == 2:
            assert 0

    def subscribe(self, topic, qos=0):
        assert self.cb is not None, "Subscribe callback is not set"
        pkt = bytearray(b"\x82\0\0\0")
        self.pid += 1
        struct.pack_into("!BH", pkt, 1, 2 + 2 + len(topic) + 1, self.pid)
        #print(hex(len(pkt)), hexlify(pkt, ":"))
        self.sock.write(pkt)
        self._send_str(topic)
        self.sock.write(qos.to_bytes(1, "little"))
        while 1:
            op = self.wait_msg()
            if op == 0x90:
                resp = self.sock.read(4)
                #print(resp)
                assert resp[1] == pkt[2] and resp[2] == pkt[3]
                if resp[3] == 0x80:
                    raise MQTTException(resp[3])
                return

    # Wait for a single incoming MQTT message and process it.
    # Subscribed messages are delivered to a callback previously
    # set by .set_callback() method. Other (internal) MQTT
    # messages processed internally.
    def wait_msg(self):
        res = self.sock.read(1)
        self.sock.setblocking(True)
        if res is None:
            return None
        if res == b"":
            raise OSError(-1)
        if res == b"\xd0":  # PINGRESP
            sz = self.sock.read(1)[0]
            assert sz == 0
            return None
        op = res[0]
        if op & 0xf0 != 0x30:
            return op
        sz = self._recv_len()
        topic_len = self.sock.read(2)
        topic_len = (topic_len[0] << 8) | topic_len[1]
        topic = self.sock.read(topic_len)
        sz -= topic_len + 2
        if op & 6:
            pid = self.sock.read(2)
            pid = pid[0] << 8 | pid[1]
            sz -= 2
        msg = self.sock.read(sz)
        self.cb(topic, msg)
        if op & 6 == 2:
            pkt = bytearray(b"\x40\x02\0\0")
            struct.pack_into("!H", pkt, 2, pid)
            self.sock.write(pkt)
        elif op & 6 == 4:
            assert 0

    # Checks whether a pending message from server is available.
    # If not, returns immediately with None. Otherwise, does
    # the same processing as wait_msg.
    def check_msg(self):
        self.sock.setblocking(False)
        return self.wait_msg()



# AVAILABLE SENSORS

RELAY = [False, None]
TEMP_DH11 = [False, None]
TEMP_DALLAS = [False, None]
CO_SENSOR = [False, None]
IR_SENSOR = [False, None]
HUMIDITY_SENSOR = [False, None]
I2C_DISPLAY = [False, None]
LED = [False, None]
TOPIC = None
oled = None

# ENABLE WIFI
station = network.WLAN(network.STA_IF)
station.active(True)
# CONNECT
station.connect(config.wifi_name, config.wifi_pass)

# WAIT UNTIL CONNECTED
while not station.isconnected():
    print("sleep")
    time.sleep(1)

# GET MAC ADDRESS
MAC = ubinascii.hexlify(network.WLAN().config('mac'), ':').decode()
CLIENT_ID = ubinascii.hexlify(unique_id())
IP = station.ifconfig()[0]


# CONNECT TO MQTT SERVER
SERVER = config.mqtt_broker

print(CLIENT_ID)


def sub_cb(topic, msg):
    global pins
    global c
    global oled
    global sensor
    global ds
    global led
    print((topic, msg))
    msg = ujson.loads(msg)
    if b'initialize' in topic:
        # RELAY
        if msg['sensor'] == 'RELAY' and not RELAY[0]:
            RELAY[0] = True
            print("enabled", RELAY[0])
            TOPIC = b"esp32/relay/" + bytes(CLIENT_ID)
            c.subscribe(TOPIC)
            print("Subscribed to %s topic" % TOPIC)
            # initialize pins
            if RELAY[0]:
                pins = []
                for pin in sorted(msg['pins']):
                    pins.append([Pin(msg['pins'][pin]["GPIO"], Pin.OUT, Pin.PULL_DOWN, value=1), False])
                RELAY[1] = pins
                print("Initialized relay pins.")
            if msg['new']:
                print("Register RELAY")
                register_relay(msg['pins'])
            else:
                print("Relay is registered.")
            print("RELAY is initialized")

        elif msg['sensor'] == 'RELAY' and RELAY[0]:
            RELAY[0] = False
            print("Disabled RELAY, Unsubscribe not implemented to mqtt lib")

        # I2C DISPLAY
        elif msg['sensor'] == 'I2C_DISPLAY' and not I2C_DISPLAY[0]:
            print(msg['pins']["GPIO"])
            I2C_DISPLAY[0] = True
            print("enabled", I2C_DISPLAY[0])
            I2C_DISPLAY[1] = msg['pins']
            if msg['new']:
                register_sensor("I2C_DISPLAY", I2C_DISPLAY[1])
            oled = SSD1306_I2C(128, 64, I2C(scl=Pin(I2C_DISPLAY[1]["GPIO"][0]), sda=Pin(I2C_DISPLAY[1]["GPIO"][1])))
            oled.fill(0)
            oled.text('Initialized', 0, 0)
            oled.show()

        elif msg['sensor'] == 'I2C_DISPLAY' and I2C_DISPLAY[0]:
            I2C_DISPLAY[0] = False
            print("Disabled I2C_DISPLAY, Unsubscribe not implemented to mqtt lib")
            oled.fill(0)
            oled.show()

        # Dallas sensor
        elif msg['sensor'] == 'TEMP_DALLAS' and not TEMP_DALLAS[0]:
            print(msg['pins'])
            TEMP_DALLAS[0] = True
            print("enabled", TEMP_DALLAS[0])
            TEMP_DALLAS[1] = msg['pins']
            if msg['new']:
                register_sensor("TEMP_DALLAS", TEMP_DALLAS[1])
            if TEMP_DALLAS[0]:
                # Dallas sensors
                dat = Pin(TEMP_DALLAS[1]["GPIO"], Pin.IN, Pin.PULL_DOWN)
                # create the onewire object
                ds = DS18X20(OneWire(dat))
                ds.convert_temp()

        elif msg['sensor'] == 'TEMP_DALLAS' and TEMP_DALLAS[0]:
            print(msg['pins'])
            TEMP_DALLAS[0] = False
            print("Disabled TEMP_DALLAS, Unsubscribe not implemented to mqtt lib")

        # DHT11 TEMP_DH11
        elif msg['sensor'] == 'TEMP_DH11' and not TEMP_DH11[0]:
            print(msg['pins']["GPIO"])
            TEMP_DH11[0] = True
            print("enabled", TEMP_DH11[0])
            TEMP_DH11[1] = msg['pins']
            if msg['new']:
                register_sensor("TEMP_DH11", TEMP_DH11[1])
            if TEMP_DH11[0]:
                sensor = DHT11(Pin(TEMP_DH11[1]["GPIO"], Pin.IN, Pin.PULL_UP))
                # DHT-22 on GPIO 15 (input with internal pull-up resistor)

        elif msg['sensor'] == 'TEMP_DH11' and TEMP_DH11[0]:
            TEMP_DH11[0] = False
            print("Disabled TEMP_DH11, Unsubscribe not implemented to mqtt lib")

        # IR
        elif msg['sensor'] == 'IR_SENSOR' and not IR_SENSOR[0]:
            print(msg['pins'])
            IR_SENSOR[0] = True
            print("enabled", IR_SENSOR[0])
            IR_SENSOR[1] = msg['pins']
            if msg['new']:
                register_sensor("IR_SENSOR", IR_SENSOR[1])

        elif msg['sensor'] == 'IR_SENSOR' and IR_SENSOR[0]:
            IR_SENSOR[0] = False
            print("Disabled IR_SENSOR, Unsubscribe not implemented to mqtt lib")

        # CO
        elif msg['sensor'] == 'CO_SENSOR' and not CO_SENSOR[0]:
            print(msg['pins']["GPIO"])
            CO_SENSOR[0] = True
            print("enabled", CO_SENSOR[0])
            CO_SENSOR[1] = msg['pins']
            print(CO_SENSOR[1])
            if msg['new']:
                register_sensor("CO_SENSOR", CO_SENSOR[1])
            print("CO_SENSOR initialized")

        elif msg['sensor'] == 'CO_SENSOR' and CO_SENSOR[0]:
            CO_SENSOR[0] = False
            print("Disabled CO_SENSOR, Unsubscribe not implemented to mqtt lib")

        # HUMIDITY
        elif msg['sensor'] == 'HUMIDITY_SENSOR' and not HUMIDITY_SENSOR[0]:
            print(msg['pins']["GPIO"])
            HUMIDITY_SENSOR[0] = True
            print("enabled", HUMIDITY_SENSOR[0])
            HUMIDITY_SENSOR[1] = msg['pins']
            if msg['new']:
                register_sensor("HUMIDITY_SENSOR", HUMIDITY_SENSOR[1])

        elif msg['sensor'] == 'HUMIDITY_SENSOR' and HUMIDITY_SENSOR[0]:
            HUMIDITY_SENSOR[0] = False
            print("Disabled CO_SENSOR, Unsubscribe not implemented to mqtt lib")

        # LED
        elif msg['sensor'] == 'LED' and not LED[0]:
            print(msg['pins'])
            LED[0] = True
            print("enabled", LED[0])
            LED[1] = msg['pins']
            if msg['new']:
                register_sensor("LED", LED[1])
            led = Pin(LED[1]["GPIO"], Pin.OUT)

        elif msg['sensor'] == 'LED' and LED[0]:
            LED[0] = False
            print("Disabled LED, Unsubscribe not implemented to mqtt lib")

    else:
        try:

            value = str(msg['value'])
            pin = int(msg['channel'])
            if msg['sensor'] == 'RELAY':
                if "on" in value:
                    pins[pin-1][0].value(0)
                    pins[pin-1][1] = 1
                elif "off" in value:
                    pins[pin-1][0].value(1)
                    pins[pin-1][1] = 0
                elif "toggle" in value:
                    # LED is inversed, so setting it to current state
                    # value will make it toggle
                    pins[pin-1][0].value(pins[pin-1][1])
                    pins[pin-1][1] = not pins[pin-1][1]
        except Exception as e:
            print(e)
            pass


def register_device():
    c.publish(b"register/device", CLIENT_ID)


def initalize_settings():
    global c
    init = "initialize/{}".format(CLIENT_ID)
    print(type(init))
    init = b"initialize/"+CLIENT_ID

    print("Subscribed to {} topic".format(b"initialize/"+CLIENT_ID))
    c.subscribe(init)


def register_relay(pins):
    global c
    msg = {"mac": MAC, "sensor_id": "RELAY", "ip": IP, "client_id": CLIENT_ID, "pins": {}}
    for pin in pins:
        print(pin)
        msg["pins"][pin] = pins[pin]
    print("Publish to register relay")
    c.publish(b"register/relay", ujson.dumps(msg))


def register_sensor(device, pin):
    global c
    msg = {"mac": MAC, "sensor_id": device, "ip": IP, "client_id": CLIENT_ID,
           "pins": pin}
    c.publish(b"register/sensor", ujson.dumps(msg))

print("MAIN")
print("Connecting to MQTT broker")
c = MQTTClient(CLIENT_ID, SERVER)
c.connect()

# Subscribed messages will be delivered to this callback
c.set_callback(sub_cb)

register_device()
initalize_settings()

while 1:
    #micropython.mem_info()
    try:
        c.check_msg()

        if RELAY[0]:
            pass

        if TEMP_DH11[0]:
            sensor.measure()  # Poll sensor
            t = sensor.temperature()
            h = sensor.humidity()
            print(t)
            print(h)
            # msg = {"mac": "hbdsao", "temperature": 22, "humidity": 22}
            msg = {"mac": MAC, "ip": IP}
            msg["humidity"] = h
            msg["temperature"] = t
            msg["sensor_id"] = "DHT11-{}".format(str(ubinascii.hexlify(unique_id())))
            print(msg)
            c.publish(b"measurements", ujson.dumps(msg))  # Publish sensor data to MQTT topic

        if TEMP_DALLAS[0]:
            print(ds.scan())
            msg = {"mac": MAC, "ip": IP}
            for rom in ds.scan():
                t = ds.read_temp(rom)
                print("{}, {}".format(t, ubinascii.hexlify(rom, ':').decode()))

                msg["temperature"] = t
                msg["sensor_id"] = ubinascii.hexlify(rom, ':').decode()
                print(msg)
                c.publish(b"measurements", ujson.dumps(msg))

        if CO_SENSOR[0]:
            adc = ADC(Pin(CO_SENSOR[1]["GPIO"]))
            reading = adc.read()
            print(reading)
            msg = {"mac": MAC, "ip": IP}
            msg["carbon_dioxid"] = reading
            msg["sensor_id"] = "CO_SENSOR"
            c.publish(b"measurements", ujson.dumps(msg))
            if "threshold" in CO_SENSOR[1]:
                threshold = CO_SENSOR[1]["threshold"]
            else:
                threshold = 400
            if reading < threshold:
                if LED[0]:
                    led.value(False)
            else:
                c.publish(b"alerts", bytes("ALERT !!! threshold {} was exceeded, measurement is: {} ".format(threshold, reading)))
                if LED[0]:
                    led.value(True)

        if IR_SENSOR[0]:
            adc = ADC(Pin(IR_SENSOR[1]["GPIO"]))
            reading = adc.read()
            print(reading)
            msg = {"mac": MAC, "ip": IP}
            msg["humidity"] = reading
            msg["sensor_id"] = "IR_SENSOR"
            c.publish(b"measurements", ujson.dumps(msg))
            if "threshold" in IR_SENSOR[1]:
                threshold = IR_SENSOR[1]["threshold"]
            else:
                threshold = 400
            if reading < threshold:
                if LED[0]:
                    led.value(False)

        if HUMIDITY_SENSOR[0]:
            adc = ADC(Pin(HUMIDITY_SENSOR[1]["GPIO"]))
            reading = adc.read()
            print(reading)
            msg = {"mac": MAC, "ip": IP}
            msg["humidity"] = reading
            msg["sensor_id"] = "HUMIDITY_SENSOR"
            c.publish(b"measurements", ujson.dumps(msg))
            if "threshold" in HUMIDITY_SENSOR[1]:
                threshold = HUMIDITY_SENSOR[1]["threshold"]
            else:
                threshold = 400
            if reading < threshold:
                if LED[0]:
                    led.value(False)

        if I2C_DISPLAY[0]:
            oled.fill(0)
            oled.text('MicroPython on', 0, 0)
            oled.text('an ESP32 ', 0, 10)
            oled.text('IP address:', 0, 20)
            oled.text(IP, 0, 30)
            try:
                oled.text('Temperature: {}'.format(t), 0, 40)
                oled.text('Humidity: {}'.format(h), 0, 50)
            except:
                oled.text('Temperature: {}'.format("ERR"), 0, 40)
                oled.text('Humidity: {}'.format("ERR"), 0, 50)
                pass
            oled.show()

        if 0:
            if LED[0]:
                led.value(True)
                sleep(1)
                led.value(False)

    except Exception:
        pass
    time.sleep(1)
