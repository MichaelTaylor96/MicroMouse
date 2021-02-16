from Senses import Senses
import busio
import board
import time
from adafruit_motorkit import MotorKit

# MAX WALL DIST: 124
# MIN WALL DIST: 30

class Mouse:
    def __init__(self, tofPinArray):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.senses = Senses(tofPinArray, i2c)
        motors = MotorKit(i2c=i2c)
        self.motorL = motors.motor4
        self.motorR = motors.motor1
        self.moveState = "stationary"
        self.initDof()
        self.heading = self.angle
        self.trim = {"direction": "", "size": 0}

    @property
    def angle(self):
        return self.senses.dofAngleX

    @property
    def leftDist(self):
        return self.senses.tofs["left"]

    @property
    def frontDist(self):
        return self.senses.tofs["center"]

    @property
    def rightDist(self):
        return self.senses.tofs["right"]

    @property
    def dofCalibration(self):
        return self.senses.dof.calibration_status

    def initDof(self):
        while self.dofCalibration[1] == 0:
            time.sleep(1)
            continue

    def moveForward(self, throttle):
        self.moveState = "forward"
        self.heading = self.angle
        print(f"Heading to maintain: {self.heading}")
        done = False
        while not done:
            error = (self.angle - self.heading) % 360
            if error > 180:
                error = -1*(360-error)
            errorMod = min(2.49*(error/180), 1)
            correction = abs((1-throttle)*errorMod)
            self.motorL.throttle = throttle
            self.motorR.throttle = throttle
            if (error < 0):
                self.motorL.throttle += correction
            else:
                self.motorR.throttle += correction
            done = self.frontDist < 70
        self.stop()

    def moveBackward(self, throttle):
        self.moveState = "backward"
        self.motorL.throttle = -1*throttle
        self.motorR.throttle = -1*throttle

    def stop(self):
        self.moveState = "stationary"
        self.motorL.throttle = 0
        self.motorR.throttle = 0

    def turnAngle(self, degrees, speed):
        self.stop()
        targetAngle = (self.angle + degrees) % 360
        self.moveState = "turning"
        self.motorL.throttle = speed if degrees > 0 else -1*speed
        self.motorR.throttle = -1*speed if degrees > 0 else speed
        while abs(targetAngle - self.angle) > 0.75:
            continue
        self.stop()

    def turn(self, direction):
        if direction == "right":
            angle = 90
        elif direction == "left":
            angle = -90
        else:
            print("Invalid Direction")
            return
        targetAngle = (self.angle + angle) % 360
        while abs(angle) > 0.25:
            self.turnAngle(angle, 0.15)
            angle = targetAngle - self.angle
