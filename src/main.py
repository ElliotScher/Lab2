# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Elliot Scher                                                 #
# 	Created:      1/24/2024, 8:08:30 AM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default


# class StateMachine:
#     def __init__(self) -> None:
#         self.state = 0
    
#     def changeState(self, state):
#         self.state = state

#     def getState(self):
#         return self.state



brain = Brain()

leftReflectance = Line(brain.three_wire_port.d)
rightReflectance = Line(brain.three_wire_port.c)

button = Bumper(brain.three_wire_port.e)

imu = Inertial(Ports.PORT3)

ultrasonic = Sonar(brain.three_wire_port.g)

leftDrive = Motor(Ports.PORT10, True)
rightDrive = Motor(Ports.PORT1, False)

WHEEL_DIAMETER_IN = 4
WHEEL_BASE_IN = 11.625
GEAR_RATIO = 5

FORWARD_SPEED_M_PER_S = 0.1
LINE_KP = 0.15
LINE_KD = 0.15

ROTATION_SPEED_RAD_PER_SEC = math.pi
TARGET_HEADING = 180
TURN_KP = 4
TURN_KD = 2.9375


prevError = 0

state = 0


imu.set_turn_type(TurnType.LEFT)
imu.calibrate()
imu.reset_heading()

while imu.is_calibrating():
    pass

print("IMU CALIBRATED")

def setState(new):
    global state
    state = new
    wait(500)

def idleState():
    global state
    if button.pressing():
        setState(1)

def followState():
    global state
    global prevError
    # dimensional analysis to turn m/s into rpm
    rpm = (FORWARD_SPEED_M_PER_S * 39.3701 * 60) / (WHEEL_DIAMETER_IN * math.pi)

    # calculated such that error to the right is positive
    error = leftReflectance.reflectivity() - rightReflectance.reflectivity()

    # change in error per second
    rate = (error - prevError) / 0.2

    effort = (error * LINE_KP) + (rate * LINE_KD)

    leftDrive.spin(FORWARD, (rpm - effort) * GEAR_RATIO)
    rightDrive.spin(FORWARD, (rpm + effort) * GEAR_RATIO)

    prevError = error

    if ultrasonic.distance(INCHES) <= 6:
        prevError = 0
        leftDrive.stop()
        rightDrive.stop()
        setState(2)

    if button.pressing():
        prevError = 0
        setState(2)

def rotateState():
    global state
    global prevError
    # dimensional analysis to turn robot rad/sec into wheel rpm
    rpm = (ROTATION_SPEED_RAD_PER_SEC * WHEEL_BASE_IN * 60) / (2 * math.pi)

    error = TARGET_HEADING - imu.heading()

    if error > 180:
        error -= 360
    elif error < -180:
        error += 360

    print(imu.heading())

    # change in error per second
    rate = (error - prevError) / 0.2

    effort = (error * TURN_KP) + (rate * TURN_KD)

    if abs(effort) > rpm:
        effort = math.copysign(rpm, effort)

    leftDrive.spin(REVERSE, effort * GEAR_RATIO)
    rightDrive.spin(FORWARD, effort * GEAR_RATIO)

    prevError = error

    if button.pressing():
        leftDrive.stop()
        rightDrive.stop()
        setState(3)

while True:
    print(state)
    if state == 0:
        idleState()
    if state == 1:
        followState()
    if state == 2:
        rotateState()
    if state == 3:
        followState()