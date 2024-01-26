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
import matplotlib.pyplot

# Brain should be defined by default
brain=Brain()

brain.screen.print("Hello V5")

reflectance = Line(brain.three_wire_port.c)
button = Bumper(brain.three_wire_port.g)

reflectances = []
times = []

while True:
    reflectances.append(reflectance.reflectivity())
    times.append(time.time)
    if (button.pressing()):
        break

matplotlib.pyplot.plot(reflectances, times)
matplotlib.pyplot.xlabel("Time")
matplotlib.pyplot.ylabel("Reflectance")
matplotlib.pyplot.title("Reflectance VS Time")
matplotlib.pyplot.show()