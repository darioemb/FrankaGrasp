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

SimOnly = True 

def waitrobot(robot):
    """busy wait for robot completion"""
    robot.WaitForController(10)
    while not robot.GetController().IsDone():
        time.sleep(0.01)

right_jointnames = ["r_shoulder_pan_joint",
	        "r_shoulder_lift_joint",
		"r_upper_arm_roll_joint",
		"r_elbow_flex_joint",
		"r_forearm_roll_joint",
		"r_wrist_flex_joint",
		"r_wrist_roll_joint"]
left_jointnames = ["l_shoulder_pan_joint",
	        "l_shoulder_lift_joint",
		"l_upper_arm_roll_joint",
		"l_elbow_flex_joint",
		"l_forearm_roll_joint",
		"l_wrist_flex_joint",
		"l_wrist_roll_joint"]
        
def move_arm(openrave_traj):    
    goal = JointTrajectoryGoal()

    traj = RaveCreateTrajectory(env,'')
    traj.deserialize(openrave_traj)
    robot.GetController().SetPath(traj)
    robot.WaitForController(0)        
    if (SimOnly):
        return
    spec = traj.GetConfigurationSpecification()
    goal.trajectory.joint_names = [str(j.GetName()) for j in robot.GetJoints(manip.GetArmIndices())]
    starttime = 0.0
    for i in range(traj.GetNumWaypoints()):
        pt=trajectory_msgs.msg.JointTrajectoryPoint()
        data = traj.GetWaypoint(i)
        pt.positions = spec.ExtractJointValues(data,robot,manip.GetArmIndices(),0)
        starttime += spec.ExtractDeltaTime(data)*2
        pt.time_from_start = rospy.Duration(starttime)
        goal.trajectory.points.append(pt)
    joint_action_client.send_goal_and_wait(goal)            

def move_arm_simple(joint_action_client, jointnames, traj):        
    if (SimOnly):
        return
    goal = JointTrajectoryGoal()
    goal.trajectory.joint_names = jointnames
    dur = dur0 = 5.
    for joints in traj:
        pt=trajectory_msgs.msg.JointTrajectoryPoint()
        for j in joints:
            pt.positions.append(j)
        pt.time_from_start = rospy.Duration(dur)
        dur = dur + dur0
        goal.trajectory.points.append(pt)
    joint_action_client.send_goal_and_wait(goal)            
           

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

def look_at(x,y,z):
    if (SimOnly):
        return
    goal = PointHeadGoal()        
    point = PointStamped()
    point.header.frame_id = 'base_link'
    point.point.x = x
    point.point.y = y
    point.point.z = z
    goal.target = point        
    goal.pointing_frame = "high_def_frame";
#goal.min_duration = .5;
    goal.max_velocity = .5;    
    point_head_client.send_goal_and_wait(goal)

def look_ahead():
    look_at(5.0,0.0,1.2)

def look_right():
    look_at(5.0,-10.0,1.2)


def feasible(options,base):
    global robot, env, manip, taskprob, basemanip
    global joint_action_client, left_joint_action_client
    global gripper_client, left_gripper_client
    args=None
    parser = OptionParser(description='', usage='')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--planner',action="store",type='string',dest='planner',default=None,
                      help='the planner to use')
    parser.add_option('--target', action="store",type='string',dest='target',default='1',
                      help='')
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)

    "Main example code."
    env.Load('exp.env.xml')

    robot = env.GetRobots()[0]

    manip = robot.SetActiveManipulator('rightarm') 
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()

    # create the interface for basic manipulation programs
    basemanip = interfaces.BaseManipulation(robot,plannername=options.planner,maxvelmult=5)
    taskprob = interfaces.TaskManipulation(robot,plannername=options.planner)

    defaultarm = [[0,1.29023451,0,-2.32099996,0,-0.69800004,0]]
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint','torso_lift_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996,0.31])
    waitrobot(robot)
    move_arm_simple(joint_action_client, right_jointnames, defaultarm)
    move_arm_simple(left_joint_action_client, left_jointnames, defaultarm)
    
    look_ahead()
    raw_input('moved to default')
    open_gripper()
    raw_input('opened the gripper')
    close_gripper()
    raw_input('closed the gripper')

    #basemanip.MoveActiveJoints(goal=[1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996,0.31],maxiter=5000,steplength=0.15,maxtries=10)

    bodies = [b for b in env.GetBodies() if not b.IsRobot() and linalg.norm(b.ComputeAABB().extents()) < 1] #0.2
    for b in bodies:
        env.RemoveKinBody(b)
    # find a location for PR2
    movex = movey = 0.0    
    placeorder = [2,1,4]#,3,0]
#placeorder = [6, 7]
    idx = 0 
    for i in range(len(placeorder)):
        if (placeorder[i]==6):
            move_robot(-.1, 1.3, math.pi) 
            raw_input('moved the robot')
            manip = robot.SetActiveManipulator('leftarm')
            basemanip = interfaces.BaseManipulation(robot,plannername=options.planner,maxvelmult=5)
            taskprob = interfaces.TaskManipulation(robot,plannername=options.planner)
            joint_action_client = left_joint_action_client
            gripper_client = left_gripper_client
        if (placeorder[i]<6):
            traj = load_traj(idx)
            idx = idx+1
            look_right()
            move_arm(traj)
        open_gripper()
        raw_input('feed the object [ENTER]...')
        close_gripper()
#robot.Grab(b)
        waitrobot(robot)
        raw_input('placing the object...')
        look_ahead()
        traj = load_traj(idx)
        idx = idx+1
        move_arm(traj)
        raw_input('release it [ENTER]...')
        open_gripper()
#robot.Release(b)
        waitrobot(robot)
        raw_input('move the arm back...')
        traj = load_traj(idx)
        idx = idx+1
        move_arm(traj)
        close_gripper()
        raw_input('done [ENTER]')
    
def move_robot(x, y, rot):
    global robot, basemanip
    robot.SetActiveDOFs([],DOFAffine.RotationAxis,[0,0,1])
    basemanip.MoveActiveJoints(goal=[rot], maxiter=5000, steplength=0.15,maxtries=2)
    #robot.SetActiveDOFValues(goal)
    waitrobot(robot)
    robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y)
    basemanip.MoveActiveJoints(goal=[x, y], maxiter=5000, steplength=0.15,maxtries=2)
    waitrobot(robot)
            
def load_traj(idx):
    f = open('constraint_traj/%d.traj' %idx,'r')
    traj = f.read()
    f.close()
    return traj 

def findvalidgrasps(options, base, target):
    global robot, env, manip, taskprob
    # move robot to the target
    robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
    with env:
        goal = [base[0], base[1], -1.58] 
        robot.SetActiveDOFValues(goal)
    waitrobot(robot)
    gmodel = databases.grasping.GraspingModel(robot,target)
    if not gmodel.load():
        print 'generating grasping model (one time computation)'
        gmodel.init(friction=0.4,avoidlinks=[])
        gmodel.generate(approachrays=gmodel.computeBoxApproachRays(delta=0.02,normalanglerange=0),forceclosurethreshold=1e-3)
        gmodel.save()
        print 'generated %d grasps' %len(gmodel.grasps)
    validgrasps,validindices = gmodel.computeValidGrasps(returnnum=20)
    return gmodel, validgrasps



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
    parser = OptionParser(description='', usage='')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--planner',action="store",type='string',dest='planner',default=None,
                      help='the planner to use')
    parser.add_option('--target', action="store",type='string',dest='target',default='1',
                      help='')
    (options, leftargs) = parser.parse_args(args=args)

    basepos = [0.6, .8]
    rospy.init_node('controller_manager')

    goal = JointTrajectoryGoal()

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

    joint_trajectory_action_name = 'l_arm_controller/joint_trajectory_action'
    left_joint_action_client = actionlib.SimpleActionClient(joint_trajectory_action_name, JointTrajectoryAction)
    left_joint_action_client.wait_for_server()
    print "got left arm server"

    joint_trajectory_action_name = 'r_arm_controller/joint_trajectory_action'
    joint_action_client = actionlib.SimpleActionClient(joint_trajectory_action_name, JointTrajectoryAction)
    joint_action_client.wait_for_server()
    print "got right arm server"

    point_head_client = actionlib.SimpleActionClient('head_traj_controller/point_head_action', PointHeadAction)
    point_head_client.wait_for_server()
    print 'got head server'

    res = feasible(options,basepos)

