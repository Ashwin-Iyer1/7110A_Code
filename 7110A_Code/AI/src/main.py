# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       krish                                                        #
# 	Created:      8/15/2023, 10:56:43 AM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *
# import tensorflow as tf
# Brain should be defined by default
brain=Brain()

brain.screen.print("Hello V5")
frontLeft = Motor(18, 6_1, False)
frontRight = Motor(17, 6_1, True)
backLeft = Motor(20, 6_1, False)
backRight = Motor(19, 6_1, True)

frontLeft.set_velocity(300, RPM)
frontLeft.spin(FORWARD)

