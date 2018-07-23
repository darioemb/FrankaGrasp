#!/usr/bin/env python

from __future__ import with_statement # for python 2.5
__author__ = ''

from itertools import izip
import time
import openravepy
import math
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

       
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

    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint','torso_lift_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996,0.31])
    defaultarm=robot.GetDOFValues(range(15,22))       
    waitrobot(robot)

    move_robot(-.1, 1.3, math.pi) 
    raw_input('pause')
    ############################# place book############################
    manip = robot.SetActiveManipulator('leftarm')
    basemanip = interfaces.BaseManipulation(robot,plannername=options.planner,maxvelmult=5)
    taskprob = interfaces.TaskManipulation(robot,plannername=options.planner)
#    taskprob.ReleaseFinger()
#    waitrobot(robot)
#    raw_input('feed the book')
#    taskprob.CloseFinger()
#    waitrobot(robot)
#    robot.SetActiveDOFs(manip.GetArmIndices())
#    traj = basemanip.MoveToHandPosition(goal=[],maxiter=5000,steplength=0.15,maxtries=2, outputtraj = True)    
#    waitrobot(robot)
#    raw_input('pause')


    bodies = [b for b in env.GetBodies() if not b.IsRobot() and linalg.norm(b.ComputeAABB().extents()) < 1] #0.2
    for b in bodies:
        env.RemoveKinBody(b)        
    movex = movey = 0.0    
    placeorder = [2,1,4]#,3,0]
    placeorder = [6, 7]#,3,0]
    needconstraint=[0, 0]
    alltraj = []
    #for i,b in enumerate(bodies):
    for i in range(len(placeorder)):
        b = bodies[placeorder[i]]
        env.AddKinBody(b)
        print b
        T0 = b.GetTransform()
        while True:
            #print 'Current location: %.3f %.3f' %(base[0]+movex, base[1]+movey)
            try:
                gmodel, validgrasps =findvalidgrasps(options, base, b)                
                if len(validgrasps)>0:
                    break
            except:
                validgrasps=[]   
            if (len(validgrasps)==0):    
                # move the object
                movey = random.random()*.2-0.1
                movex = random.random()*.6-0.3                
                goal = [[1,0,0,movex], [0, 1, 0, movey], [0, 0, 1, 0], [0, 0, 0, 1]]
                b.SetTransform(numpy.dot(goal, T0))
#            if (len(validgrasps)==0):    
#                # move robots
#                movey = random.random()*.4-0.2
#                movex = random.random()*.6-0.3
#                robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
#                goal = [base[0]+movex, base[1]+movey, -1.58] 
#                robot.SetActiveDOFValues(goal)
#                waitrobot(robot)
        T = b.GetTransform()
        print 'Current obj location: %.3f %.3f' %(T[0,3], T[1,3])
        #print 'Current location: %.3f %.3f' %(base[0]+movex, base[1]+movey)
        #gmodel = databases.grasping.GraspingModel(robot,b)
        #basemanip = openravepy.interfaces.BaseManipulation(robot)
        for validgrasp in validgrasps:
          try:  
            print 'grasp ', b.GetName() 
            gmodel.moveToPreshape(validgrasp)
            Tplace = gmodel.getGlobalGraspTransform(validgrasp,collisionfree=True)
            Tplaceobj = b.GetTransform()
            sol = manip.FindIKSolution(Tplace, IkFilterOptions.CheckEnvCollisions) # get collision-free solution
            #print sol
            # move arm to side with the same hand position
              
            sol[0] = -3.14/2
            robot.SetActiveDOFs(manip.GetArmIndices())
            #move to grasping position
            traj = basemanip.MoveActiveJoints(goal=sol,maxiter=5000,steplength=0.15,maxtries=2, outputtraj = True)    
            alltraj.append(traj)
            #taskprob.ReleaseFingers()
            waitrobot(robot)
            taskprob.ReleaseFingers()
            waitrobot(robot)
            Tgrasp = manip.GetEndEffectorTransform()
            Tgraspobj = numpy.dot(Tgrasp, numpy.dot(linalg.inv(Tplace),Tplaceobj))
            b.SetTransform(Tgraspobj)
            print('feed the object [ENTER]...')
            taskprob.CloseFingers()
            waitrobot(robot)
            print 'placing the object...'
            robot.Grab(b)
            if (not needconstraint[i]):
                traj = basemanip.MoveToHandPosition(matrices=[Tplace],seedik=16, outputtraj = True)
            else:    
                traj = moveWithConstraint(Tgrasp, Tgraspobj, Tplace, Tplaceobj, b)
            alltraj.append(traj)
            robot.WaitForController(10)
            waitrobot(robot)
            print('release it [ENTER]...')
            taskprob.ReleaseFingers()
            waitrobot(robot)
            robot.Release(b)
            waitrobot(robot)
            print 'move the arm back...'
            with env:
                #jointnames = ['r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint','torso_lift_joint']
                #robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
                robot.SetActiveDOFs(manip.GetArmIndices())
                traj = basemanip.MoveActiveJoints(goal=defaultarm,maxiter=5000,steplength=0.15,maxtries=10, outputtraj = True)
                alltraj.append(traj)
                #robot.SetActiveDOFValues(robot.GetDOFValues(range(15,22)))
            waitrobot(robot)
            print('done [ENTER]')
            break
            #basemanip.MoveToHandPosition(matrices=[Tgrasp],seedik=16)
          except:
            robot.ReleaseAllGrabbed()
            continue

    for i, traj in enumerate(alltraj):
        f = open( '%d.traj' %i, 'w')
        f.write(traj)
        f.close()
#f.write(
#    pickle.dump(alltraj,'alltraj.dat')
            
def move_robot(x, y, rot):
    global robot, basemanip
    robot.SetActiveDOFs([],DOFAffine.RotationAxis,[0,0,1])
    basemanip.MoveActiveJoints(goal=[rot], maxiter=5000, steplength=0.15,maxtries=2)
    #robot.SetActiveDOFValues(goal)
    waitrobot(robot)
    robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y)
    basemanip.MoveActiveJoints(goal=[x, y], maxiter=5000, steplength=0.15,maxtries=2)
    waitrobot(robot)

def moveWithConstraint(Tgrasp, Tgraspobj, Tplace, Tplaceobj, target):
    global env, basemanip
    # find z axis
    localrotaxis = dot(linalg.inv(target.GetTransform()[0:3,0:3]),[0,0,1])
    constraintfreedoms = ones(6) # rotation xyz, translation xyz
    constraintfreedoms[3:] = 0
    index = argmax(abs(localrotaxis))
    constraintfreedoms[index] = 0
    localrotaxis = zeros(3)
    localrotaxis[index] = 1
    print 'planning with freedoms: %s, local rot axis: %s '%(constraintfreedoms,localrotaxis)

    #constraintfreedoms = [0, 0, 0, 0, 0, 0]
    constrainterrorthresh = 0.2
    constrainttaskmatrix=dot(linalg.inv(Tgrasp),Tgraspobj)
    constraintmatrix = linalg.inv(Tgraspobj)
    traj = basemanip.MoveToHandPosition(matrices=[Tplace],maxiter=3000,maxtries=10,seedik=40,constraintfreedoms=constraintfreedoms,\
           constraintmatrix=constraintmatrix, constrainttaskmatrix=constrainttaskmatrix,constrainterrorthresh=constrainterrorthresh,\
           steplength=0.002, outputtraj = True)
    return traj

                
def findvalidgrasps(options, base, target):
    global robot, env, manip, taskprob
    # move robot to the target
#    robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
#    with env:
#        goal = [base[0], base[1], -1.58] 
#        robot.SetActiveDOFValues(goal)
#    waitrobot(robot)
    gmodel = databases.grasping.GraspingModel(robot,target)
    if not gmodel.load():
        print 'generating grasping model (one time computation)'
        gmodel.init(friction=0.4,avoidlinks=[])
        gmodel.generate(approachrays=gmodel.computeBoxApproachRays(delta=0.02,normalanglerange=0),forceclosurethreshold=1e-3)
        gmodel.save()
        print 'generated %d grasps' %len(gmodel.grasps)
    validgrasps,validindices = gmodel.computeValidGrasps(returnnum=20)
    return gmodel, validgrasps

    basemanip = openravepy.interfaces.BaseManipulation(robot)
    with env:
        initialvalues = robot.GetDOFValues(gmodel.manip.GetArmIndices())
    for validgrasp in random.permutation(validgrasps):
        try:
            #print validgrasp
            gmodel.moveToPreshape(validgrasp)
            print 'move robot arm to grasp'
            Tgrasp = gmodel.getGlobalGraspTransform(validgrasp,collisionfree=True)
            print Tgrasp
            basemanip.MoveToHandPosition(matrices=[Tgrasp],seedik=16)
            robot.WaitForController(10)
            taskprob.CloseFingers()
            waitrobot(robot)
            #return True
        except planning_error,e:
            print 'try again: ',e
    #return False
    return validgrasps

def grasp(options, base, target):
    global robot, env, manip, taskprob
    #print target

    #basemanip = interfaces.BaseManipulation(robot,plannername=options.planner,maxvelmult=5)

    taskprob.ReleaseFingers()
    waitrobot(robot)    
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
    returnnum = 20 

    #print 'computing first %d valid grasps'%returnnum    
    validgrasps,validindices = gmodel.computeValidGrasps(returnnum=returnnum)
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

