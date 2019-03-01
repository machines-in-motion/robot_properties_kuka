import numpy as np
from math import pi
import rospkg
from os.path import join
import pinocchio as se3


class IiwaConfig:
    robot_name = "iiwa"

    urdf_path = (
        join(rospkg.RosPack().get_path("robot_properties_stuggihop"),
             "urdf",
             "iiwa.urdf")
    )

    yaml_path = (
        join(rospkg.RosPack().get_path("robot_properties_stuggihop"),
             "config",
             "iiwa.yaml")
    )
