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
        self.targetHeading = self.angle
        # Trim use to have direction like {direction: "right", size: 0}
        # However, for now, I know that the right motor is more powerful than the left,
        # and will just always apply the trim as a negative on the right throttle. It
        # makes it easier when the trim has overcorrect to only worry about one.
        # Open to other ideas though (looking at you Greg)
        self.trim = 0

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
        self.targetHeading = self.angle
        previousAngle = self.angle
        done = False
        correctionBase = min(self.trim+throttle, 1-(self.trim+throttle))

        while not done:
            # Check if close to walls
            # if self.leftDist < 50 or 120 > self.rightDist > 70:
            #     self.targetHeading = (self.targetHeading + 5) % 360
            # elif self.rightDist < 50 or 120 > self.leftDist > 70:
            #     self.targetHeading = (self.targetHeading - 5) % 360

            # Evaluate errors
            error = (self.angle - self.targetHeading) % 360
            if error > 180:
                error = -1*(360-error)
            angleDrift = (self.angle - previousAngle) % 360
            if (angleDrift > 180):
                angleDrift = -1*(360-angleDrift)
            previousAngle = self.angle

            correction = correctionBase*(error/180)
            
            # Back off the trim correction if angle is approaching heading
            if error < 0 <= angleDrift or angleDrift <= 0 < error:
                correction = correctionBase*((angleDrift)/180)

            self.trim += correction
            self.trim = max(min(self.trim, 1-throttle), -throttle)
            lTrim = abs(self.trim) if self.trim < 0 else 0
            rTrim = self.trim if self.trim > 0 else 0
            self.motorL.throttle = throttle + lTrim
            self.motorR.throttle = throttle + rTrim
            done = self.frontDist < 70
        self.stop()

    def moveForwardEventuallyStraight(self, throttle):
        self.moveState = "forward"
        previousAngle = self.angle
        done = False
        self.trim = 0
        trimCoefficient = 20
        
        while not done:
            angleDrift = (self.angle - previousAngle) % 360
            if (angleDrift > 180):
                angleDrift = -1*(360-angleDrift)

            self.trim += (angleDrift / 180) * trimCoefficient

            self.motorL.throttle = throttle
            self.motorR.throttle = min(max(throttle + self.trim, 0), 1)
            done = self.frontDist < 70

            previousAngle = self.angle

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
        rSpeed = speed + self.trim if self.trim > 0 else speed
        lSpeed = abs(self.trim) + speed if self.trim < 0 else speed
        self.motorL.throttle = lSpeed if degrees > 0 else -1*lSpeed
        self.motorR.throttle = -1*rSpeed if degrees > 0 else rSpeed
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
        targetAngle = (self.targetHeading + angle) % 360
        while abs(angle) > 0.25:
            self.turnAngle(angle, 0.15)
            angle = targetAngle - self.angle
