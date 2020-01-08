import numpy as np
import time
import os
import rospkg
import pybullet as p
import pinocchio as se3
from py_pinocchio_bullet.wrapper import PinBulletWrapper
from py_robot_properties_iiwa.config import IiwaConfig


dt = 1e-3

class IiwaRobot(PinBulletWrapper):
    @staticmethod
    def initPhysicsClient():
        physicsClient = p.connect(p.GUI)
        p.setGravity(0,0, -9.81)
        p.setPhysicsEngineParameter(fixedTimeStep=dt, numSubSteps=1)
        return physicsClient
    
    def __init__(self, physicsClient=None):
        if physicsClient is None:
            self.physicsClient = self.initPhysicsClient()

        # Load the plain.
        plain_urdf = (rospkg.RosPack().get_path("robot_properties_iiwa") +
                      "/urdf/plane_with_restitution.urdf")
        self.planeId = p.loadURDF(plain_urdf)

        # Load the robot
        robotStartPos = [0., 0, 0.0]
        robotStartOrientation = p.getQuaternionFromEuler([0,0,0])

        self.urdf_path = IiwaConfig.urdf_path
        self.robotId = p.loadURDF(self.urdf_path, robotStartPos,
            robotStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE,
            useFixedBase=True) # why not true?
        p.getBasePositionAndOrientation(self.robotId)

        # Create the robot wrapper in pinocchio.
        package_dirs = [os.path.dirname(os.path.dirname(self.urdf_path)) + '/urdf']
        self.pin_robot = IiwaConfig.buildRobotWrapper()

        # Query all the joints.
        num_joints = p.getNumJoints(self.robotId)

        for ji in range(num_joints):
            p.changeDynamics(self.robotId, ji, linearDamping=.04,
                angularDamping=0.04, restitution=0.0, lateralFriction=0.5)

        self.base_link_name = "base_link"
        controlled_joints = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7']
        self.joint_names = controlled_joints

        # Creates the wrapper by calling the super.__init__.
        super(IiwaRobot, self).__init__(self.robotId, self.pin_robot,
            controlled_joints,
            ['END'],
            useFixedBase=True
            )

        
    def forward_robot(self, q=None, dq=None):
        if q is None:
            q, dq = self.get_state()
        elif dq is None:
            raise ValueError('Need to provide q and dq or non of them.')
        
        self.pin_robot.forwardKinematics(q, dq)
        self.pin_robot.computeJointJacobians(q)
        self.pin_robot.framesForwardKinematics(q)
        self.pin_robot.centroidalMomentum(q, dq)