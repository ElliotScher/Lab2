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
brain = Brain()

frontSonar = Sonar(brain.three_wire_port.g)

button = Bumper(brain.three_wire_port.e)

leftDrive = Motor(Ports.PORT10, True)
rightDrive = Motor(Ports.PORT1, False)

WHEEL_DIAMETER_IN = 4
GEAR_RATIO = 5

FORWARD_SPEED_M_PER_S = 0.05
TARGET_DISTANCE_FROM_WALL_IN = 5
WALL_KP = 15
WALL_KD = 5

while True:
    while not button.pressing():
        pass

    while button.pressing():
        pass

    prevError = 0
    while True:
        # dimensional analysis to turn m/s into rpm
        rpm = (FORWARD_SPEED_M_PER_S * 39.3701 * 60) / (WHEEL_DIAMETER_IN * math.pi)

        error = frontSonar.distance(DistanceUnits.IN) - TARGET_DISTANCE_FROM_WALL_IN

        # change in error per second
        rate = (error - prevError) / 0.2

        effort = (error * WALL_KP) + (rate * WALL_KD)

        if abs(effort) > rpm:
            effort = math.copysign(rpm, effort)

        leftDrive.spin(FORWARD, effort * GEAR_RATIO)
        rightDrive.spin(FORWARD, effort * GEAR_RATIO)

        prevError = error

        print(frontSonar.distance(DistanceUnits.IN))

        if button.pressing():
            break

        wait(20)

    leftDrive.stop()
    rightDrive.stop()

    while button.pressing():
        pass