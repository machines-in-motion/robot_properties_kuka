import numpy as np
from math import pi
import rospkg
from os.path import join, dirname
from os import environ
import pinocchio as se3
from pinocchio.utils import zero
from pinocchio.robot_wrapper import RobotWrapper

class IiwaConfig(object):
    robot_name = "iiwa"

    @classmethod
    def buildRobotWrapper(cls):
        # Rebuild the robot wrapper instead of using the existing model to
        # also load the visuals.
        robot_model, collision_model, visual_model = se3.buildModelsFromUrdf(cls.urdf_path, cls.meshes_path)
        return RobotWrapper(robot_model), collision_model, visual_model
        
    # Here we use the same urdf as for the quadruped but without the freeflyer.
    urdf_path = (
        join(rospkg.RosPack().get_path("robot_properties_" + robot_name),
             "urdf",
             robot_name + ".urdf")
    )

    meshes_path = [
      dirname(rospkg.RosPack().get_path("robot_properties_" + robot_name))
    ]

    yaml_path = (
        join(rospkg.RosPack().get_path("robot_properties_" + robot_name),
             "config",
             "iiwa.yaml")
    )

    # pinocchio model.
    robot_model = se3.buildModelFromUrdf(urdf_path,
                                         se3.JointModelFreeFlyer())

    base_name = robot_model.frames[2].name

    # Define the initial configuration (pos and vel?)
    initial_configuration = se3.utils.zero(robot_model.nq) 
    initial_velocity = se3.utils.zero(robot_model.nv) 