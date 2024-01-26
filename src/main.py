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
# import matplotlib.pyplot

# Brain should be defined by default
brain = Brain()

leftReflectance = Line(brain.three_wire_port.d)
rightReflectance = Line(brain.three_wire_port.c)

button = Bumper(brain.three_wire_port.g)

leftDrive = Motor(Ports.PORT10, True)
rightDrive = Motor(Ports.PORT1, False)

WHEEL_DIAMETER_IN = 4
GEAR_RATIO = 5

FORWARD_SPEED_M_PER_S = 0.1
LINE_KP = 0.15
LINE_KD = 0.15

while True:
    while not button.pressing():
        pass

    while button.pressing():
        pass

    prevError = 0
    while True:
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

        if button.pressing():
            break

        wait(20)

    leftDrive.stop()
    rightDrive.stop()

    while button.pressing():
        pass