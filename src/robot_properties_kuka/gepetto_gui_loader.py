"""gepetto_gui_loader

Load the robot in the gepetto-gui.

License: BSD 3-Clause License
Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

import eigenpy
import time
from config import IiwaConfig
from py_gepetto_gui_helper.gepetto_gui_scene import GepettoGuiScene
from py_gepetto_gui_helper.robot_visual import RobotVisual
from py_gepetto_gui_helper.frame import Frame

def create_scene():
    """
    Just create a scene for the kuka to be in
    """
    return GepettoGuiScene("kuka_scene", "kuka_window")


def load_kuka_in_gepetto_gui(gepetto_scene, robot_name):
    """
    Load the kuka meshes in the scene
    """
    config = IiwaConfig()
    return RobotVisual(gepetto_scene, robot_name, config.urdf_path,
                       config.meshes_path)

def display_kuka_in_gepetto_gui(launch_gepetto_gui_exec=False):
    """
    Uses the function above to load the urdf model of kuka in gepetto gui
    and load it in the initial configuration
    """

    if launch_gepetto_gui_exec:
        # create a new window
        gepetto_gui_process = GepettoGuiScene.open_gepetto_gui()

    # create a scene in it
    gui_scene = create_scene()
    # load the robot
    kuka_visual = load_kuka_in_gepetto_gui(gui_scene, "kuka")
    # place the robot in initial configuration
    config = IiwaConfig()
    kuka_visual.display(config.q0)
    # place the world frame
    world_frame = Frame(gui_scene)

    if launch_gepetto_gui_exec:
        # close the window after little while
        time.sleep(5)
        GepettoGuiScene.close_gepetto_gui(gepetto_gui_process)

    return gui_scene, kuka_visual, world_frame

if __name__ == "__main__":
    display_kuka_in_gepetto_gui()
    

