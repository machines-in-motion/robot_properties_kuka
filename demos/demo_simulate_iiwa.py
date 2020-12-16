#!/usr/bin/env python

"""demo_simulate_iiwa

Simple demo showing how the simulation setup works.

License: BSD 3-Clause License
Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

import time
import numpy as np
from bullet_utils.env import BulletEnvWithGround
from robot_properties_kuka.iiwaWrapper import IiwaRobot, IiwaConfig

if __name__ == "__main__":

    # Create a Pybullet simulation environment
    env = BulletEnvWithGround()

    # Create a robot instance. This initializes the simulator as well.
    robot = env.add_robot(IiwaRobot)
    tau = np.zeros(robot.nb_dof)

    # Reset the robot to some initial state.
    q0 = np.matrix(IiwaConfig.initial_configuration).T
    dq0 = np.matrix(IiwaConfig.initial_velocity).T
    robot.reset_state(q0, dq0)

    # Run the simulator for 100 steps
    for i in range(500):
        # TODO: Implement a controller here.
        robot.send_joint_command(tau)

        # Step the simulator.
        env.step(sleep=True) # You can sleep here if you want to slow down the replay
        time.sleep(.5)
    # Read the final state and forces after the stepping.
    q, dq = robot.get_state()
    active_eff, forces = robot.get_force()
    print("q", q)
    print("dq", dq)
    print("active eff", active_eff)
    print("forces", forces)
