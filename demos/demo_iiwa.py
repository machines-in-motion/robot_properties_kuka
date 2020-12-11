import numpy as np 
from robot_properties_kuka.config import IiwaConfig
from robot_properties_kuka.IiwaWrapper import IiwaRobot

import pinocchio as pin
import pybullet as p
import time

# Create a robot instance. This initializes the simulator as well.
robot = IiwaRobot()
tau = np.zeros(robot.pin_robot.model.nq)

# Reset the robot to some initial state.
q0 = np.matrix(IiwaConfig.q0)
v0 = np.matrix(IiwaConfig.v0)
robot.reset_state(q0, v0)

# Run the simulator for 100 steps
for i in range(100):
    # TODO: Implement a controller here.
    robot.send_joint_command(tau)
    
    # Step the simulator.
    p.stepSimulation()
    time.sleep(0.1)
