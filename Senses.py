import adafruit_vl6180x
import adafruit_bno055
import time
from digitalio import DigitalInOut, Direction

class Senses:
    def __init__(self, tofPinArray, i2c):
        self.i2c = i2c
        self.initTofs(tofPinArray)
        self.dof = adafruit_bno055.BNO055_I2C(self.i2c)

    def init_switch(self, pin, power_on):
        switch = DigitalInOut(pin)
        switch.direction = Direction.OUTPUT
        switch.value = power_on
        return switch

    def init_TOF_sensor(self, switch, address):
        switch.value = True
        time.sleep(0.05)
        sensor = adafruit_vl6180x.VL6180X(self.i2c)
        sensor.set_address(self.i2c, address)
        return sensor

    def initTofs(self, pinArray):
        s1switch = self.init_switch(pinArray[0], False)
        s2switch = self.init_switch(pinArray[1], False)
        s3switch = self.init_switch(pinArray[2], False)

        s1 = self.init_TOF_sensor(s1switch, 0x30)
        s2 = self.init_TOF_sensor(s2switch, 0x31)
        s3 = self.init_TOF_sensor(s3switch, 0x32)
        self.tofArray = {
            "left": s1,
            "center": s2,
            "right": s3
        }

    @property
    def tofs(self):
        return {
            "left": self.tofArray["left"].range,
            "center": self.tofArray["center"].range,
            "right": self.tofArray["right"].range,
        }

    @property
    def dofAngleX(self):
        return self.dof.euler[0]
