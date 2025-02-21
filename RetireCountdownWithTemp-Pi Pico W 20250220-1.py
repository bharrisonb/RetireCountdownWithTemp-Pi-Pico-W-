# MIT License
#
# Copyright (c) 2025 Bob Harrison
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# RPH 20250220

# thanks to --> https://peppe8o.com/using-i2c-lcd-display-with-raspberry-pi-pico-and-micropython/
# for ntp set time see --> https://gist.github.com/aallan/581ecf4dc92cd53e3a415b7c33a1147c
# see Breadboard layout photo at ---> https://photos.app.goo.gl/CSPPqWjCjzYWCRrRA

# requires and/or references these packages:
#  micropython_cpython_micropython
#  micropython_cpython_uzlib
#  micropython_gzip
#  micropython_zlib
#  picozero


import machine
import utime
import struct
from picozero import pico_temp_sensor
from machine import I2C
from machine import Pin, PWM
from lcd_api import (
    LcdApi,
)  # load https://peppe8o.com/download/micropython/LCD/lcd_api.py on device
from i2c_lcd import (
    I2cLcd,
)  # load https://peppe8o.com/download/micropython/LCD/i2c_lcd.py on device

import network
import socket
import urequests, json
import struct

import time

from machine import Pin, I2C

from picozero import LED

NTP_DELTA = (
    2208988800 + 14400
)  # delta (offset) between NTP epoch and Pi Pico epic for US Eastern Time (UTC - 4)
host = "pool.ntp.org"  # ntp server


def set_time():
    NTP_QUERY = bytearray(48)
    NTP_QUERY[0] = 0x1B
    addr = socket.getaddrinfo(host, 123)[0][-1]
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.settimeout(1)
        res = s.sendto(NTP_QUERY, addr)
        msg = s.recv(48)
    finally:
        s.close()
    val = struct.unpack("!I", msg[40:44])[0]
    t = val - NTP_DELTA
    tm = time.gmtime(t)
    machine.RTC().datetime(
        (tm[0], tm[1], tm[2], tm[6] + 1, tm[3], tm[4], tm[5], 0)
    )  # set the RTC based on NTP server


rtc = machine.RTC()
wlan = network.WLAN(network.STA_IF)  # connect to local Wi-Fi router
wlan.active(True)
wlan.connect(
    "SSID", "password"
)  # SSID and password of local Wi-Fi router
time.sleep(5)
print("Connected to internet status:")
print(wlan.isconnected())  # Verify connected to Wi-Fi router

print(wlan.ifconfig())  # print config
print(wlan.config("mac"))  # print MAC address
print(wlan.config("ssid"))  # print MAC address
print(wlan.config("txpower"))  # print ytansmit power


###

set_time()
print(time.localtime())

###


I2C_ADDR = 0x27
I2C_NUM_ROWS = 2
I2C_NUM_COLS = 16

i2c = I2C(0, sda=machine.Pin(0), scl=machine.Pin(1), freq=400000)
lcd = I2cLcd(i2c, I2C_ADDR, I2C_NUM_ROWS, I2C_NUM_COLS)
lcd.clear()

# using : https://www.epochconverter.com/

retire = 1744804800 - 14400  # Wednesday, April 16, 2025 8:00:20 AM GMT-04:00 DST
#retire = 0   # test to go through timeout logic

now = rtc.datetime()  # current Epoch time UTC for Thonny 3.3.3 and earlier

rtc = machine.RTC()
print(rtc.datetime())  # Print the current time for reference
print(now)  # Print the current Epoch time

t = rtc.datetime()
t_string = "{:02d}:{:02d}:{:02d}:{:02d}:{:02d}".format(t[0], t[1], t[2], t[4], t[5])
print(t_string)

lcd.move_to(0, 1)  # to show current Epoch time
lcd.putstr(str(now))
utime.sleep(10)
lcd.clear()
led28 = Pin(28, Pin.OUT)  # external led (red)
led2 = Pin(2, Pin.OUT)  # external led (yellow)

tempC = 0.00
tempF = 0.00

led2.low()
led28.high()
toggle_minutes = 0
minutes = 0

sensor_temp = machine.ADC(4)
conversion_factor = 3.3 / (65535)

while True:

    if toggle_minutes != minutes:  # perform once a minute
        led2.high()

        led28.low()
        toggle_minutes = minutes

        reading = sensor_temp.read_u16() * conversion_factor
        # tempC = round(27 - (reading - 0.706) / 0.001721, 2)

        tempC = (
            pico_temp_sensor.temp - 1
        )  # or, try this library, calibrate by 1 degC for error

        tempF = round(((tempC * 9 / 5) + 32), 2)
        print("Temp Deg C: ", tempC, " Temp Deg F: ", tempF)

        lcd.clear()
        t = rtc.datetime()

        t_string = "{:02d}:{:02d}:{:02d}:{:02d}:{:02d}".format(
            t[0], t[1], t[2], t[4], t[5]
        )
        lcd.putstr(t_string)

        lcd.move_to(0, 1)
        lcd.putstr("DegC: ")
        lcd.putstr(str(tempC))
        utime.sleep_ms(
            13000
        )  # for 13 seconds, display current temp, then continue with countdown

        led2.low()
        led28.high()

        lcd.clear()
        lcd.putstr("Y:D:H:M:S to go")

    lcd.move_to(0, 1)

    now = utime.time()  # current Epoch time

    countdown = retire - now  # Epoch seconds until retirement

    ## check to see if it's time to Retire

    if countdown <= 0:
        lcd.clear()

        lcd.putstr(" Time to Retire")
        lcd.move_to(0, 1)
        lcd.putstr("    Enjoy!")

        led2.high()

        led28.high()

        for i in range(864000000):
            # Notify the great event!
            lcd.backlight_on()
            led2.toggle()
            led28.toggle()
            utime.sleep_ms(500)
            lcd.backlight_off()
            led2.toggle()
            led28.toggle()
            utime.sleep_ms(500)

        break  # all done!

    days = countdown / (24 * 3600)  # full number of days to go

    years = int(days / 365)  # full number of years to go

    lcd.putstr(str(years))
    lcd.putstr(":")

    days1 = int(days - (365 * years))  # remaining whole days

    lcd.putstr(str(days1))
    lcd.putstr(":")

    countdown1 = countdown % (24 * 3600)

    hours = int(countdown1 / 3600)  # remaining whole hours

    if hours < 10:
        lcd.putstr("0")

    lcd.putstr(str(hours))
    lcd.putstr(":")

    countdown1 %= 3600

    minutes = int(countdown1 / 60)  # remaing whole minutes

    if minutes < 10:
        lcd.putstr("0")

    lcd.putstr(str(minutes))
    lcd.putstr(":")

    countdown1 %= 60  # remaining whole seonds

    seconds = countdown1

    if seconds < 10:
        lcd.putstr("0")

    lcd.putstr(str(seconds))
    lcd.putstr("  ")
