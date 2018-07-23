#!/usr/bin/python

import roslib; roslib.load_manifest('graspingDemoTUWUObjects')

from openravepy import *
from numpy import *
import rospy, time, string
#import geometry_msgs.msg
from graspplanningTUW import *



class SceneUpdater:

    def __init__(self, env):  
        time.sleep(0.5)
        self.env = env          
        self.robot = self.env.GetRobots()[0]
        self.manip = self.robot.GetManipulators()[0]
        jointnamesArm = ' '.join(self.robot.GetJoints()[j].GetName() for j in self.manip.GetArmIndices())
        jointnamesHand = ' '.join(self.robot.GetJoints()[j].GetName() for j in self.manip.GetGripperJoints())
        jointnames = jointnamesArm + ' ' + jointnamesHand
           
            # Creating the ros controller
        print 'Setting controller: trajectoryservice /controller_session joints '+jointnames
        controller = RaveCreateController(self.env, 'ROSOpenRAVE trajectoryservice /controller_session '+jointnames)
        print self.robot.GetDOF()
        self.robot.SetController(controller,range(self.robot.GetDOF()),0)
     
        print 'Setting initial joint values/positions' 
        #Initial position
        PI = 3.1415926535
        jointValues =  [0.0, 10*PI/180, 0.0, 90*PI/180, 90*PI/180, 50*PI/180, 90*PI/180, -1.0, -1.0]
    
        self.robot.SetJointValues(jointValues)
        self.robot.GetController().SetDesired(jointValues)
        self.robot.WaitForController(0)
       
    
    
    def move_arm(self):
        robot = self.robot
        robot.SetActiveManipulator(self.manip)
        with self.env: 
            ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
            if not ikmodel.load():
                ikmodel.autogenerate()
            basemanip = interfaces.BaseManipulation(robot)
            taskmanip = interfaces.TaskManipulation(robot)
            Tstart = ikmodel.manip.GetEndEffectorTransform()
            #Tstart = array([[  1,   0,   0,   0.9], 
            #                [  0,   0,   -1,  -0.2], 
            #                [  0,   -1,   0,   0.3], 
            #                [  0,   0,   0,   1  ]])
            sol = ikmodel.manip.FindIKSolution(Tstart,IkFilterOptions.CheckEnvCollisions)
            print ikmodel.manip.GetArmIndices()
            robot.SetDOFValues(sol,ikmodel.manip.GetArmIndices())
            
            basemanip.MoveToHandPosition([Tstart],maxiter=1000,maxtries=1,seedik=4)
            robot.WaitForController(0)
        
        stop = False
        max_steps = 1000
        updir = array((0,0,1))
        while (not stop):
        
            success = basemanip.MoveHandStraight(direction=updir,stepsize=0.001,minsteps=1,maxsteps=max_steps)
            robot.WaitForController(0)
            input = raw_input("press enter to move arm straight \n press q to quit ")
                     
            updir = -updir   
            if (input == ""):
                continue         
            if (input == "q"):
                stop = True
            #set x/y/z coordinate for direction vector resp. maxsteps
            if (input[0] == "x"):
                updir[0] = string.atof(input[1:])
            if (input[0] == "y"):
                updir[1] = string.atof(input[1:])
            if (input[0] == "z"):
                updir[2] = string.atof(input[1:])
            if (input[0] == "m"):
                max_steps = string.atoi(input[1:])    


if __name__ == '__main__':
   
    sceneName = 'data/TUWlab.kintest.env.xml'    #Amtec/OttoBock
    env = Environment()
    env.SetViewer('qtcoin')
    env.Load(sceneName)
    self = SceneUpdater(env)
    raw_input("press enter to move arm")
    self.move_arm()
    env.Destroy()
