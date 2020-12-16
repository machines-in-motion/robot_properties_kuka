#!/usr/bin/env python

"""demo_display_iiwa

Basic loading and visualization for the iiwa robot using gepetto viewer.

license: BSD 3-Clause License
copyrights: Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""


def with_gepetto_gui_helper():
    import time
    from robot_properties_kuka.config import IiwaConfig
    from gepetto_gui_helper.gepetto_gui_scene import GepettoGuiScene
    from gepetto_gui_helper.robot_visual import RobotVisual
    from gepetto_gui_helper.frame import Frame

    gepetto_gui_scene = GepettoGuiScene("kuka_scene", "kuka_window")
    config = IiwaConfig()
    iiwa_visual = RobotVisual(
        gepetto_gui_scene, "iiwa", config.urdf_path, config.meshes_path
    )
    iiwa_visual.display(config.q0)
    # place the world frame
    world_frame = Frame(gepetto_gui_scene)

    # # Example of moving the robot forward and updating the display every time.
    # q = config.q0.copy()
    # for i in range(10):
    #     q[0] += 0.05
    #     iiwa_visual.display(q)
    #     time.sleep(0.2)


def with_pinocchio_wrapper():
    import numpy as np
    import pinocchio as se3
    import time
    import os

    from robot_properties_kuka.config import IiwaConfig

    # Load the robot urdf.
    robot = IiwaConfig.buildRobotWrapper()

    # Setup the display (connection to gepetto viewer) and load the robot model.
    robot.initDisplay(loadModel=True)

    # Create a first initial position for the robot. Both legs are bent inwards.
    q = np.matrix(IiwaConfig.initial_configuration).T

    # Display the configuration in the viewer.
    robot.display(q)

    # # Example of moving the robot forward and updating the display every time.
    # for i in range(10):
    #     q[0] += 0.05
    #     robot.display(q)
    #     time.sleep(0.2)


if __name__ == "__main__":

    # with_pinocchio_wrapper()
    with_gepetto_gui_helper()