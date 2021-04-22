#!/usr/bin/env python

"""demo_display_iiwa

Basic loading and visualization for the iiwa robot using gepetto viewer.

license: BSD 3-Clause License
copyrights: Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""


from robot_properties_kuka.config import IiwaConfig
import numpy as np 
import time 

# Load the robot urdf.
robot = IiwaConfig.buildRobotWrapper()

# Setup the display (connection to gepetto viewer) and load the robot model.
robot.initDisplay(loadModel=True)

# Create a first initial position for the robot. Both legs are bent inwards.
q = np.matrix(IiwaConfig.initial_configuration).T

# Display the configuration in the viewer.
robot.display(q)

# Example of moving the robot forward and updating the display every time.
for i in range(100):
    print(i)
    q[0] += 0.05
    q[1] -= 0.01
    q[2] += 0.05
    robot.display(q)
    time.sleep(0.1)