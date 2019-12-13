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
        robot = RobotWrapper.BuildFromURDF(cls.urdf_path, cls.meshes_path)
        robot.model.rotorInertia[6:] = cls.motor_inertia
        robot.model.rotorGearRatio[6:] = cls.motor_gear_ration
        return robot
        
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
    
    # The inertia of a single blmc_motor.
    motor_inertia = 0.0000045

    # The motor gear ratio.
    motor_gear_ration = 9.

    robot_model.rotorInertia[6:] = motor_inertia
    robot_model.rotorGearRatio[6:] = motor_gear_ration

    mass = np.sum([i.mass for i in robot_model.inertias])

    base_name = robot_model.frames[2].name

    # Define the initial configuration (pos and vel?)
    initial_configuration = se3.utils.zero(robot_model.nq) #[0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.] #0.4, 0., 0., 0., 1.] + 4*[0., 0.8, -1.6]
    initial_velocity = se3.utils.zero(robot_model.nv) # [0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.] #(8 + 4 + 6)*[0,]