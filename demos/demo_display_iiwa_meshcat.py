#!/usr/bin/env python

"""demo_display_iiwa_meshcat
Basic loading and visualization for the iiwa robot using meshcat.
"""

from robot_properties_kuka.config import IiwaConfig
from pinocchio.visualize import MeshcatVisualizer

robot = IiwaConfig.buildRobotWrapper()

viz = MeshcatVisualizer(robot.model, robot.visual_model, robot.visual_model)
viz.initViewer(open=True)
viz.loadViewerModel()
viz.display(robot.q0)
