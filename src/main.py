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

imu = Inertial(Ports.PORT3)

button = Bumper(brain.three_wire_port.e)

leftDrive = Motor(Ports.PORT10, True)
rightDrive = Motor(Ports.PORT1, False)

WHEEL_DIAMETER_IN = 4
WHEEL_BASE_IN = 11.625
GEAR_RATIO = 5

ROTATION_SPEED_RAD_PER_SEC = math.pi
TARGET_HEADING = 180
TURN_KP = 4
TURN_KD = 2.9375

imu.set_turn_type(TurnType.LEFT)
imu.calibrate()
imu.reset_heading()

while imu.is_calibrating():
    pass

print("IMU CALIBRATED")

while True:
    while not button.pressing():
        pass

    while button.pressing():
        pass

    prevError = 0
    while True:
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
            break

        wait(20)

    leftDrive.stop()
    rightDrive.stop()

    while button.pressing():
        pass