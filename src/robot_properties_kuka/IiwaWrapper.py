import numpy as np
import time
import os
import pybullet 
from py_pinocchio_bullet.wrapper import PinBulletWrapper
from robot_properties_kuka.config import IiwaConfig

dt = 1e-3

class IiwaRobot(PinBulletWrapper):

    def __init__(self, pos=None, orn=None, with_gui=True, physicsClient=None):

        # Load the robot
        if pos is None:
            pos = [0.0, 0, 0.0]
        if orn is None:
            orn = pybullet.getQuaternionFromEuler([0, 0, 0])

        if physicsClient is None:
            self.physicsClient = self.initPhysicsClient(with_gui)

        self.urdf_path = IiwaConfig.urdf_path
        self.robotId = pybullet.loadURDF(
            self.urdf_path,
            pos, orn,
            flags=pybullet.URDF_USE_INERTIA_FROM_FILE,
            useFixedBase=True,
        )
        # pybullet.getBasePositionAndOrientation(self.robotId)



        # Create the robot wrapper in pinocchio.
        self.pin_robot = IiwaConfig.buildRobotWrapper()

        # # Load the plain.
        # plain_urdf = (rospkg.RosPack().get_path("robot_properties_iiwa") +
        #               "/urdf/plane_with_restitution.urdf")
        # self.planeId = p.loadURDF(plain_urdf)

        # # Create the robot wrapper in pinocchio.
        # package_dirs = [os.path.dirname(os.path.dirname(self.urdf_path)) + '/urdf']
        # self.pin_robot, self.collision_model, self.visual_model = IiwaConfig.buildRobotWrapper() #

        # Query all the joints.
        num_joints = pybullet.getNumJoints(self.robotId)

        for ji in range(num_joints):
            pybullet.changeDynamics(self.robotId, 
                                    ji, 
                                    linearDamping=.04,
                                    angularDamping=0.04, 
                                    restitution=0.0, 
                                    lateralFriction=0.5)

        self.base_link_name = "base_link"
        self.end_eff_ids = []
        controlled_joints = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7']
        self.joint_names = controlled_joints

        # Creates the wrapper by calling the super.__init__.
        super(IiwaRobot, self).__init__(
            self.robotId, 
            self.pin_robot,
            controlled_joints,
            ['END'])
            
    @staticmethod
    def initPhysicsClient(with_gui=True):
        if with_gui:
            physicsClient = pybullet.connect(pybullet.GUI)
        else:
            physicsClient = pybullet.connect(pybullet.DIRECT)
        pybullet.setGravity(0,0, -9.81)
        pybullet.setPhysicsEngineParameter(fixedTimeStep=dt, numSubSteps=1)
        return physicsClient

    def forward_robot(self, q=None, dq=None):
        if not q:
            q, dq = self.get_state()
        elif not dq:
            raise ValueError("Need to provide q and dq or non of them.")

        self.pin_robot.forwardKinematics(q, dq)
        self.pin_robot.computeJointJacobians(q)
        self.pin_robot.framesForwardKinematics(q)
        self.pin_robot.centroidalMomentum(q, dq)

    def start_recording(self, file_name):
        self.file_name = file_name
        pybullet.startStateLogging(pybullet.STATE_LOGGING_VIDEO_MP4, self.file_name)

    def stop_recording(self):
        pybullet.stopStateLogging(pybullet.STATE_LOGGING_VIDEO_MP4, self.file_name)


# class IiwaRobot(PinBulletWrapper):

#     def __init__(self, physicsClient=None, with_gui=True):
#         if physicsClient is None:
#             self.physicsClient = self.initPhysicsClient(with_gui)

#         # Load the plain.
#         plain_urdf = (rospkg.RosPack().get_path("robot_properties_iiwa") +
#                       "/urdf/plane_with_restitution.urdf")
#         self.planeId = p.loadURDF(plain_urdf)

#         # Load the robot
#         robotStartPos = [0., 0, 0.0]
#         robotStartOrientation = p.getQuaternionFromEuler([0,0,0])

#         self.config = IiwaConfig()
#         self.urdf_path = self.config.urdf_path
#         self.robotId = p.loadURDF(self.urdf_path, robotStartPos,
#             robotStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE,
#             useFixedBase=True) 
#         p.getBasePositionAndOrientation(self.robotId)

#         # Create the robot wrapper in pinocchio.
#         package_dirs = [os.path.dirname(os.path.dirname(self.urdf_path)) + '/urdf']
#         self.pin_robot, self.collision_model, self.visual_model = IiwaConfig.buildRobotWrapper() #

#         # Query all the joints.
#         num_joints = p.getNumJoints(self.robotId)

#         for ji in range(num_joints):
#             p.changeDynamics(self.robotId, ji, linearDamping=.04,
#                 angularDamping=0.04, restitution=0.0, lateralFriction=0.5)

#         self.base_link_name = "base_link"
#         controlled_joints = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7']
#         self.joint_names = controlled_joints

#         # Creates the wrapper by calling the super.__init__.
#         super(IiwaRobot, self).__init__(self.robotId, self.pin_robot,
#             controlled_joints,
#             ['END'],
#             useFixedBase=True)
            
#     @staticmethod
#     def initPhysicsClient(with_gui=True):
#         if with_gui:
#             physicsClient = p.connect(p.GUI)
#         else:
#             physicsClient = p.connect(p.DIRECT)
#         p.setGravity(0,0, -9.81)
#         p.setPhysicsEngineParameter(fixedTimeStep=dt, numSubSteps=1)
#         return physicsClient

        
#     def forward_robot(self, q=None, dq=None):
#         if q is None:
#             q, dq = self.get_state()
#         elif dq is None:
#             raise ValueError('Need to provide q and dq or non of them.')
        
#         self.pin_robot.forwardKinematics(q, dq)
#         self.pin_robot.computeJointJacobians(q)
#         self.pin_robot.framesForwardKinematics(q)
#         self.pin_robot.centroidalMomentum(q, dq)