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

def main(env,options):
    "Main example code."
    env.Load('pr2office5.env.xml')
    time.sleep(1)

    robot = env.GetRobots()[0]

    manip = robot.SetActiveManipulator('leftarm') 
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()

    # create the interface for basic manipulation programs
    basemanip = interfaces.BaseManipulation(robot,plannername=options.planner,maxvelmult=5)
    taskprob = interfaces.TaskManipulation(robot,plannername=options.planner)

    print 'move arms'
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint','torso_lift_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996,0.31])
    waitrobot(robot)

    print 'move robot base to target'

    # get x position of the object
    dat = open('office5-openrave-test.dat','r')
    lines = dat.readlines()
    line = lines[int(options.target)+3]
    w = line.split('\t')
    basex = float(w[4])/1000
    #env.Remove(bodies[int(options.target)+1])

    bodies = [b for b in env.GetBodies() if not b.IsRobot() and linalg.norm(b.ComputeAABB().extents()) < 1] #0.2
    target = bodies[0]
    robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])

    with env:
        goal = [1.4, 1.36, -1.58] 
        robot.SetActiveDOFValues(goal)
    waitrobot(robot)

    basemanip = interfaces.BaseManipulation(robot,plannername=options.planner,maxvelmult=5)

    taskprob.ReleaseFingers()
    waitrobot(robot)

    bodies = [b for b in env.GetBodies() if not b.IsRobot() and linalg.norm(b.ComputeAABB().extents()) < 1] #0.2
    robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
    target = bodies[int(options.target)]
    print 'choosing target %s'%target

    gmodel = databases.grasping.GraspingModel(robot,target)
    if not gmodel.load():
        print 'generating grasping model (one time computation)'
        gmodel.init(friction=0.4,avoidlinks=[])
        gmodel.generate(approachrays=gmodel.computeBoxApproachRays(delta=0.02,normalanglerange=0))
        gmodel.save()

    returnnum = 5

    print 'computing first %d valid grasps'%returnnum
    validgrasps,validindices = gmodel.computeValidGrasps(returnnum=returnnum)
    #for validgrasp in validgrasps:
        #gmodel.showgrasp(validgrasp)

    print len(validgrasps)

    print 'choosing a random grasp and move to its preshape'
    basemanip = openravepy.interfaces.BaseManipulation(robot)
    with env:
        initialvalues = robot.GetDOFValues(gmodel.manip.GetArmIndices())
    for validgrasp in random.permutation(validgrasps):
        try:
            gmodel.moveToPreshape(validgrasp)
            print 'move robot arm to grasp'
            Tgrasp = gmodel.getGlobalGraspTransform(validgrasp,collisionfree=True)
            basemanip.MoveToHandPosition(matrices=[Tgrasp],seedik=16)
            break
        except planning_error,e:
            print 'try again: ',e
    robot.WaitForController(10)
    taskprob.CloseFingers()
    waitrobot(robot)
    with env:
        robot.Grab(target)
    	print 'initial values'
    	basemanip.MoveManipulator(initialvalues,jitter=None)
    	waitrobot(robot)
    	raw_input('press any key')

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Explicitly specify goals to get a simple navigation and manipulation demo.', usage='openrave.py --example hanoi [options]')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--planner',action="store",type='string',dest='planner',default=None,
                      help='the planner to use')
    parser.add_option('--target', action="store",type='string',dest='target',default='1',
                      help='')
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    main(env,options)


if __name__ == "__main__":
    run()  
