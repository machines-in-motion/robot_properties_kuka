"""config

Store the configuration of the Kuka family robots.

License: BSD 3-Clause License
Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

import numpy as np
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
            cls.urdf_path, cls.meshes_path)
        robot.model.rotorInertia[:] = cls.motor_inertia
        robot.model.rotorGearRatio[:] = cls.motor_gear_ration
        return robot

    def joint_name_in_single_string(self):
        joint_names = ""
        for name in self.robot_model.names[2:]:
            joint_names += name + " "
        return joint_names


class IiwaConfig(KukaAbstract):
    '''
    Config class for the KUKA LWR iiwa
    '''
    robot_family = "kuka"   
    robot_name = "iiwa"

    paths = find_paths(robot_name)
    meshes_path = paths["package"]
    yaml_path = paths["dgm_yaml"]
    urdf_path = paths["urdf"]

    # The inertia of a single blmc_motor.
    motor_inertia = 0.0000045
    # The motor gear ratio.
    motor_gear_ration = 9.0
    
    # Pinocchio model.
    robot_model = se3.buildModelFromUrdf(urdf_path)
    robot_model.rotorInertia[:] = motor_inertia
    robot_model.rotorGearRatio[:] = motor_gear_ration

    mass = np.sum([i.mass for i in robot_model.inertias])

    base_name = robot_model.frames[2].name

    # The number of motors, here they are the same as there are only revolute
    # joints.
    nb_joints = robot_model.nv

    joint_names = [ "A1",
                    "A2",
                    "A3",
                    "A4",
                    "A5",
                    "A6",
                    "A7" ]

    end_effector_names = ["contact"]

    # Mapping between the ctrl vector in the device and the urdf indexes.
    urdf_to_dgm = tuple(range(robot_model.nv))

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
    initial_configuration = [0.]*robot_model.nq
    initial_velocity = [0.]*robot_model.nv

    q0 = zero(robot_model.nq)
    q0[:] = initial_configuration
    v0 = zero(robot_model.nv)
    a0 = zero(robot_model.nv)


class IiwaReducedConfig(IiwaConfig):
    '''
    Config class for the iiwa reduced model
    '''
    # Override build_robot_wrapper to generate reduced model
    @classmethod
    def buildRobotWrapper(cls, controlled_joints):
        # Rebuild the robot wrapper instead of using the existing model to
        # also load the visuals.
        robot_full = RobotWrapper.BuildFromURDF(
            cls.urdf_path, cls.meshes_path)
        robot_full.model.rotorInertia[:] = cls.motor_inertia
        robot_full.model.rotorGearRatio[:] = cls.motor_gear_ration
        controlled_joints_ids = []
        for joint_name in controlled_joints:
            controlled_joints_ids.append(robot_full.model.getJointId(joint_name))
        # Joint names to lock
        uncontrolled_joints = [] # 27 = 34 - 6 (controlled) - 1(universe)
        for joint_name in robot_full.model.names[1:]:
            if(joint_name not in controlled_joints):
                uncontrolled_joints.append(joint_name)
        locked_joints_ids = [robot_full.model.getJointId(joint_name) for joint_name in uncontrolled_joints]
        qref = se3.neutral(robot_full.model)
        reduced_model, [visual_model, collision_model] = se3.buildReducedModel(robot_full.model, 
                                                                              [robot_full.visual_model, robot_full.collision_model], 
                                                                              locked_joints_ids, 
                                                                              qref)      
        robot = se3.robot_wrapper.RobotWrapper(reduced_model, collision_model, visual_model)  
        return robot



# class Lwr4Config(KukaAbstract):
#     '''
#     Config class for the KUKA LWR 4
#     '''
#     robot_family = "kuka"   
#     robot_name = "lwr4"

#     paths = find_paths(robot_name)
#     meshes_path = paths["resources"]
#     yaml_path = paths["dgm_yaml"]
#     urdf_path = paths["urdf"]

#     # The inertia of a single blmc_motor.
#     motor_inertia = 0.0000045
#     # The motor gear ratio.
#     motor_gear_ration = 9.0
    
#     # Pinocchio model.
#     robot_model = se3.buildModelFromUrdf(urdf_path)
#     robot_model.rotorInertia[:] = motor_inertia
#     robot_model.rotorGearRatio[:] = motor_gear_ration

#     mass = np.sum([i.mass for i in robot_model.inertias])

#     base_name = robot_model.frames[2].name

#     # The number of motors, here they are the same as there are only revolute
#     # joints.
#     nb_joints = robot_model.nv

#     joint_names = [ "A1",
#                     "A2",
#                     "A3",
#                     "A4",
#                     "A5",
#                     "A6",
#                     "A7" ]

#     # Mapping between the ctrl vector in the device and the urdf indexes.
#     urdf_to_dgm = tuple(range(robot_model.nv))

#     map_joint_name_to_id = {}
#     map_joint_limits = {}
#     for i, (name, lb, ub) in enumerate(
#         zip(
#             robot_model.names[1:],
#             robot_model.lowerPositionLimit,
#             robot_model.upperPositionLimit,
#         )
#     ):
#         map_joint_name_to_id[name] = i
#         map_joint_limits[i] = [float(lb), float(ub)]

#     # Define the initial state.
#     initial_configuration = [0.]*robot_model.nq
#     initial_velocity = [0.]*robot_model.nv

#     q0 = zero(robot_model.nq)
#     q0[:] = initial_configuration
#     v0 = zero(robot_model.nv)
#     a0 = zero(robot_model.nv)