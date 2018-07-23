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

import time
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
    traj = RaveCreateTrajectory(env,'')
    traj.deserialize(openrave_traj)
    robot.GetController().SetPath(traj)
    robot.WaitForController(0)        

def move_arm_simple(joint_action_client, jointnames, traj):        
    if (SimOnly):
        return

def open_gripper():
    taskprob.ReleaseFingers()
    waitrobot(robot)

#cm = controller_manager.ControllerManager('r', using_slip_controller = use_slip_detection)
#cm.command_gripper(.087,-1)

def close_gripper():
    taskprob.CloseFingers()
    waitrobot(robot)

def look_at(x,y,z):
    if (SimOnly):
        return

def look_ahead():
    look_at(5.0,0.0,1.2)

def look_right():
    look_at(5.0,-10.0,1.2)


def feasible(options,base):
    global robot, env, manip, taskprob, basemanip
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
#move_arm_simple(joint_action_client, right_jointnames, defaultarm)
#move_arm_simple(left_joint_action_client, left_jointnames, defaultarm)
    look_ahead()
    raw_input('moved to default')
    open_gripper()
    raw_input('opened the gripper')
#    close_gripper()
#    raw_input('closed the gripper')

    #basemanip.MoveActiveJoints(goal=[1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996,0.31],maxiter=5000,steplength=0.15,maxtries=10)

    bodies = [b for b in env.GetBodies() if not b.IsRobot() and linalg.norm(b.ComputeAABB().extents()) < 1] #0.2
    for b in bodies:
        env.Remove(b)
    # find a location for PR2
    movex = movey = 0.0    
    placeorder = [2,1,4, 6, 7]#,3,0]
#placeorder = [4]#,3,0]
#placeorder = [3,0]
    #for i,b in enumerate(bodies):
    idx = 0 
    for i in range(len(placeorder)):
        if (placeorder[i]==6):
            move_robot(-.1, 1.3, math.pi) 
            raw_input('moved the robot')
            manip = robot.SetActiveManipulator('leftarm')
            basemanip = interfaces.BaseManipulation(robot,plannername=options.planner,maxvelmult=5)
            taskprob = interfaces.TaskManipulation(robot,plannername=options.planner)
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
    res = feasible(options,basepos)

