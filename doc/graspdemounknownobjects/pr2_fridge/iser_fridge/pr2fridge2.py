#!/usr/bin/env python

from __future__ import with_statement # for python 2.5
__author__ = ''

from itertools import izip
import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def feasible(options,base):
    global robot, env, manip, taskprob
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

    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint','torso_lift_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996,0.31])
    waitrobot(robot)

    bodies = [b for b in env.GetBodies() if not b.IsRobot() and linalg.norm(b.ComputeAABB().extents()) < 1] #0.2
    for b in bodies:
        env.RemoveKinBody(b)
    # find a location for PR2
    movex = movey = 0.0    
    while True:
        #print 'Current location: %.3f %.3f' %(base[0]+movex, base[1]+movey)
        graspable = True
        for i,b in enumerate(bodies):
            if (i==0 or i==2 or i==3):
                continue 
            env.AddKinBody(b)
            try:
                graspable =grasp(options, base, b)
                if (not graspable):
                    break
            except:
                graspable = False
                break        
            #raw_input('press ENTER to continue...')
        if (not graspable):    
            # move robots
            movey = random.random()*.4-0.2
            movex = random.random()*.2-0.1
            robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
            with env:
                goal = [base[0]+movex, base[1]+movey, -1.58] 
                robot.SetActiveDOFValues(goal)
            waitrobot(robot)
        #print 'cur loc: %.3f, %.3f' %robot.GetActiveDOFValues()xx,yy)

def grasp(options, base, target):
    global robot, env, manip, taskprob
    #print target

    #basemanip = interfaces.BaseManipulation(robot,plannername=options.planner,maxvelmult=5)

    taskprob.ReleaseFingers()
    waitrobot(robot)

#    if (base[1]+.75 < 1.36):
#        sety = 1.36
#    else:
#        sety = base[1]+.75
#
    robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
    with env:
        goal = [base[0], base[1], -1.58] 
        robot.SetActiveDOFValues(goal)
    waitrobot(robot)
	    
    gmodel = databases.grasping.GraspingModel(robot,target)
    if not gmodel.load():
        print 'generating grasping model (one time computation)'
        gmodel.init(friction=0.4,avoidlinks=[])
        gmodel.generate(approachrays=gmodel.computeBoxApproachRays(delta=0.02,normalanglerange=0))
        gmodel.save()

    #print 'generated %d grasps' %len(gmodel.grasps)
    returnnum = 20 

    #print 'computing first %d valid grasps'%returnnum    
    validgrasps,validindices = gmodel.computeValidGrasps(returnnum=returnnum)
    if (len(validgrasps)>0):
        return True
    else:
        return False        
    print 'found %d valid grasps' %len(validgrasps)


    #print 'choosing a random grasp and move to its preshape'
    basemanip = openravepy.interfaces.BaseManipulation(robot)
    with env:
        initialvalues = robot.GetDOFValues(gmodel.manip.GetArmIndices())
    for validgrasp in random.permutation(validgrasps):
        try:
            gmodel.moveToPreshape(validgrasp)
            print 'move robot arm to grasp'
            Tgrasp = gmodel.getGlobalGraspTransform(validgrasp,collisionfree=True)
            basemanip.MoveToHandPosition(matrices=[Tgrasp],seedik=16)
            robot.WaitForController(10)
            taskprob.CloseFingers()
            waitrobot(robot)
            return True
        except planning_error,e:
            print 'try again: ',e
    return False


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

