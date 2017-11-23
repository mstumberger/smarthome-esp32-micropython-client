micropython

SHELL:

https://github.com/dhylands/rshell

rshell --buffer-size=30 -p /dev/ttyUSB0 

mkdir /pyboard/umqtt

cp simple.py /pyboard/umqtt/

cp dht_publish.py /pyboard 

repl

import main

Wlan:

http://docs.micropython.org/en/latest/esp8266/esp8266/quickref.html

mqtt:

https://raw.githubusercontent.com/micropython/micropython-lib/master/umqtt.simple/umqtt/simple.py

Temp, humidity:

https://www.youtube.com/watch?v=_vcQTyLU1WY

https://www.rototron.info/raspberry-pi-esp32-micropython-mqtt-dht22-tutorial/


Display:

SSD1306 I2C display. pin 5 is SDA, pin 4 is SCL. 

####display lib

https://github.com/adafruit/micropython-adafruit-ssd1306

####usage

http://docs.micropython.org/en/latest/esp8266/library/machine.I2C.html#machine-i2c

####howto
http://www.instructables.com/id/MicroPython-on-an-ESP32-Board-With-Integrated-SSD1/



#Dependencies:

echo "deb http://apt.postgresql.org/pub/repos/apt/ wheezy-pgdg main" > sudo /etc/apt/sources.list.d/pgdg.list 

wget --quiet -O - http://apt.postgresql.org/pub/repos/apt/ACCC4CF8.asc | sudo apt-key add -

sudo apt-get update

sudo apt-get install -y python-pip libffi-dev libssl-dev postgresql libpq-dev mc && sudo -H pip install crossbar psycopg2

add service:

cd  /etc/systemd/system/

which crossbar

mcedit cbs.service

mcedit forwarder.service

```
[Unit]
Description=Crossbar.io
After=network.target

[Service]
Type=simple
User=pi
Group=pi
StandardInput=null
StandardOutput=journal
StandardError=journal
Environment="MYVAR1=foobar"
ExecStart=/opt/crossbar/bin/crossbar start --cbdir=/home/ubuntu/mynode1/.crossbar
Restart=on-abort

[Install]
WantedBy=multi-user.target
```

```
[Unit]
Description=MQTT forward to Crossbar
After=network.target

[Service]
Type=simple
User=pi
Group=pi
StandardInput=null
StandardOutput=journal
StandardError=journal
ExecStart=/usr/bin/python /home/pi/MQTT_forwarder/main.py
Restart=on-abort

[Install]
WantedBy=multi-user.target
```

sudo systemctl daemon-reload

sudo systemctl enable cbs.service

sudo systemctl start cbs

sudo systemctl enable forwarder.service

sudo systemctl start forwarder

sudo journalctl -f -u cbs

sudo journalctl -f -u forwarder



##DATABASE:

sudo passwd postgres

sudo -i -u postgres

psql

ALTER USER postgres WITH PASSWORD 'supertemp';

createdb -O postgres CBS_DB

psql "CBS_DB" < database.backup

sudo mcedit /etc/postgresql/9.6/main/postgresql.conf

( change to : listen_addresses = '*')

sudo mcedit /etc/postgresql/9.6/main/pg_hba.conf

( In this file you have set, from which computers you can connect to this server and what method of authentication you can use. Usually you will need similar line)

"host    all         all         192.168.1.0/24        md5"

sudo service postgresql restart