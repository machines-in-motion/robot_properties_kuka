import numpy as np
from math import pi
import rospkg
from os.path import join, dirname
from os import environ
import pinocchio as se3
from pinocchio.utils import zero
from pinocchio.robot_wrapper import RobotWrapper
from robot_properties_kuka.utils import find_paths

class KukaAbstract(object):
    """ Abstract class for KUKA robots """

    @classmethod
    def buildRobotWrapper(cls):
        # Rebuild the robot wrapper instead of using the existing model to
        # also load the visuals.
        robot = RobotWrapper.BuildFromURDF(
            cls.urdf_path, cls.meshes_path, se3.JointModelFreeFlyer()
        )
        robot.model.rotorInertia[6:] = cls.motor_inertia
        robot.model.rotorGearRatio[6:] = cls.motor_gear_ration
        return robot

    def joint_name_in_single_string(self):
        joint_names = ""
        for name in self.robot_model.names[2:]:
            joint_names += name + " "
        return joint_names


class IiwaConfig(KukaAbstract):
    robot_family = "kuka"   
    robot_name = "iiwa"

    paths = find_paths(robot_name)
    meshes_path = paths["resources"]
    yaml_path = paths["dgm_yaml"]
    urdf_path = paths["urdf"]

    # The inertia of a single blmc_motor.
    motor_inertia = 0.0000045
    # The motor gear ratio.
    motor_gear_ration = 9.0
    
    # Pinocchio model.
    robot_model = se3.buildModelFromUrdf(urdf_path, se3.JointModelFreeFlyer())
    robot_model.rotorInertia[6:] = motor_inertia
    robot_model.rotorGearRatio[6:] = motor_gear_ration

    mass = np.sum([i.mass for i in robot_model.inertias])

    base_name = robot_model.frames[2].name

    # The number of motors, here they are the same as there are only revolute
    # joints.
    nb_joints = robot_model.nq

    joint_names = [
        "A1",
        "A2",
        "A3",
        "A4",
        "A5",
        "A6",
        "A7",
    ]

    # Mapping between the ctrl vector in the device and the urdf indexes.
    urdf_to_dgm = tuple(range(7))

    map_joint_name_to_id = {}
    map_joint_limits = {}
    for i, (name, lb, ub) in enumerate(
        zip(
            robot_model.names[1:],
            robot_model.lowerPositionLimit,
            robot_model.upperPositionLimit,
        )
    ):
        map_joint_name_to_id[name] = i
        map_joint_limits[i] = [float(lb), float(ub)]

    # Define the initial state.
    initial_configuration = [0.]*7
    initial_velocity = [0.]*7

    q0 = zero(robot_model.nq)
    # q0[:] = initial_configuration
    v0 = zero(robot_model.nv)
    a0 = zero(robot_model.nv)


# class Lwr4Config(KukaAbstract):
#     robot_name = "lwr4"

#     @classmethod
#     def buildRobotWrapper(cls):
#         # Rebuild the robot wrapper instead of using the existing model to
#         # also load the visuals.
#         robot_model, collision_model, visual_model = se3.buildModelsFromUrdf(cls.urdf_path, cls.meshes_path)
#         return RobotWrapper(robot_model), collision_model, visual_model
        
#     # Here we use the same urdf as for the quadruped but without the freeflyer.
#     urdf_path = (
#         join(rospkg.RosPack().get_path("robot_properties_" + robot_name),
#              "urdf",
#              robot_name + ".urdf")
#     )

#     meshes_path = [
#       dirname(rospkg.RosPack().get_path("robot_properties_" + robot_name))
#     ]

#     yaml_path = (
#         join(rospkg.RosPack().get_path("robot_properties_" + robot_name),
#              "config",
#              "iiwa.yaml")
#     )

#     # pinocchio model.
#     robot_model = se3.buildModelFromUrdf(urdf_path,
#                                          se3.JointModelFreeFlyer())

#     base_name = robot_model.frames[2].name

#     # Define the initial configuration (pos and vel?)
#     initial_configuration = se3.utils.zero(robot_model.nq) 
#     initial_velocity = se3.utils.zero(robot_model.nv) 
