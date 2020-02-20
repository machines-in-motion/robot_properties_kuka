import numpy as np 
from py_robot_properties_iiwa.config import IiwaConfig
from py_robot_properties_iiwa.robot import IiwaRobot

import pinocchio as pin
import pybullet as p
import time

# Create a robot instance. This initializes the simulator as well.
robot = IiwaRobot()
tau = np.zeros(robot.pin_robot.model.nq)

# Reset the robot to some initial state.
q0 = np.matrix(IiwaConfig.initial_configuration)
dq0 = np.matrix(IiwaConfig.initial_velocity)
print(q0)
robot.reset_state(q0, dq0)

# Run the simulator for 100 steps
for i in range(100):
    # TODO: Implement a controller here.
    robot.send_joint_command(tau)
    
    # Step the simulator.
    p.stepSimulation()
    time.sleep(0.1)
