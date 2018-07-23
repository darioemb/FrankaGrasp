#!/usr/bin/env python

from __future__ import with_statement # for python 2.5
__author__ = ''

import roslib; roslib.load_manifest('pr2_fridge')
import rospy
from itertools import izip
import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

import actionlib
import time
from pr2_controllers_msgs.msg import *
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, Pr2GripperCommandGoal, Pr2GripperCommandAction, PointHeadAction, PointHeadGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from pr2_gripper_reactive_approach import controller_manager
from std_srvs.srv import Empty
from geometry_msgs.msg import PointStamped 
#import orrosplanning.srv 

SimOnly = False 

def waitrobot(robot):
    """busy wait for robot completion"""
    robot.WaitForController(10)
    while not robot.GetController().IsDone():
        time.sleep(0.01)


def open_gripper():
    taskprob.ReleaseFingers()
    waitrobot(robot)
    if (SimOnly):
        return
    goal = Pr2GripperCommandGoal()
    goal.command.position = .08
    goal.command.max_effort = -1.0
    gripper_client.send_goal_and_wait(goal)

#cm = controller_manager.ControllerManager('r', using_slip_controller = use_slip_detection)
#cm.command_gripper(.087,-1)

def close_gripper():
    taskprob.CloseFingers()
    waitrobot(robot)
    if (SimOnly):
        return
    goal = Pr2GripperCommandGoal()
    goal.command.position = 0.0
    goal.command.max_effort = 50.0
    gripper_client.send_goal_and_wait(goal)


from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

def isnum(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


if __name__ == "__main__":

    args=None
    rospy.init_node('controller_manager')

#    use_slip_detection = rospy.get_param('/reactive_grasp_node_right/use_slip_controller')
#    rospy.loginfo("use_slip_detection:"+str(use_slip_detection))
#
#    rospy.loginfo("waiting for compliant_close and grasp_adjustment services")
#    rospy.wait_for_service("r_reactive_grasp/compliant_close")
#    rospy.wait_for_service("r_reactive_grasp/grasp_adjustment")
#    rospy.loginfo("service found")
#    cc_srv = rospy.ServiceProxy("r_reactive_grasp/compliant_close", Empty)
#    ga_srv = rospy.ServiceProxy("r_reactive_grasp/grasp_adjustment", Empty)

    gripper_client = actionlib.SimpleActionClient('r_gripper_controller/gripper_action',Pr2GripperCommandAction)
    gripper_client.wait_for_server()
    left_gripper_client = actionlib.SimpleActionClient('l_gripper_controller/gripper_action',Pr2GripperCommandAction)
    left_gripper_client.wait_for_server()

    raw_input('open the gripper')
    open_gripper()
    raw_input('close the gripper')
    close_gripper()

