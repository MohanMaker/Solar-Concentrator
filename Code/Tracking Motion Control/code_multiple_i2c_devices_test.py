# Code Demonstrating the Use of an Stepper Motor and RTC

# import libraries (general)
import time
import board

# import libraries for stepper motor
from adafruit_motorkit import MotorKit
from adafruit_motor import stepper

# import libraries for RTC
import busio as io  # For hardware I2C (M0 boards)
import adafruit_ds3231

# Setup use of multiple I2C devices
i2c = board.I2C()
kit = MotorKit(i2c=i2c)
rtc = adafruit_ds3231.DS3231(i2c)


# --Simple test for using adafruit_motorkit with a stepper motor-- #
# https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing/circuitpython

for i in range(2):
    kit.stepper2.onestep()
    kit.stepper1.onestep()
    time.sleep(0.5)


# --Simple demo of reading and writing the time for the DS3231 real-time clock-- #
# https://learn.adafruit.com/adafruit-ds3231-precision-rtc-breakout/circuitpython

# Lookup table for names of days (nicer printing).
#days = ("Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday")

# pylint: disable-msg=using-constant-test
#if False:  # change to True if you want to set the time!
    #                     year, mon, date, hour, min, sec, wday, yday, isdst
#    t = time.struct_time((2020, 8, 30, 15, 18, 0, 6, 243, 1))
    # you must set year, mon, date, hour, min, sec and weekday
    # yearday is not supported, isdst can be set but we don't do anything with it at this time
#    print("Setting time to:", t)  # uncomment for debugging
#    rtc.datetime = t
#    print()
# pylint: enable-msg=using-constant-test

# Main loop:
#while True:
#    t = rtc.datetime
    # print(t)     # uncomment for debugging
#    print(
#        "The date is {} {}/{}/{}".format(
#            days[int(t.tm_wday)], t.tm_mday, t.tm_mon, t.tm_year
#        )
#    )
#    print("The time is {}:{:02}:{:02}".format(t.tm_hour, t.tm_min, t.tm_sec))
#    time.sleep(1)  # wait a second