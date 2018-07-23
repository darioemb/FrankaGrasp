#!/usr/bin/python

import roslib; roslib.load_manifest('graspingDemoTUWUObjects')

#from __future__ import with_statement # for python 2.5

from openravepy import *
from numpy import *
import rospy, time, tf
from std_msgs.msg import String
import geometry_msgs.msg
from graspplanningTUW import *
import copy, string


class SceneUpdater:

    def __init__(self, env):  
        self.listener = tf.TransformListener()
        time.sleep(0.5) #needed, otherwise tf error
        self.lastObject = False  #indicates if for all other objects a grasp has allready been tested => new SS
        self.manip = None
        self.TknownObject = None
        self.demo = None
        self.env = env
        self.approachvector = None
        
        self.TknownObject1 = None
        
        # Get the robot in the scene
        self.robot = self.env.GetRobots()[0] # get the first robot
        
        self.manip = self.robot.GetManipulators()[0]
        jointnamesArm = ' '.join(self.robot.GetJoints()[j].GetName() for j in self.manip.GetArmIndices()) #df GetArmJoints=>GetArmIndices
        jointnamesHand = ' '.join(self.robot.GetJoints()[j].GetName() for j in self.manip.GetGripperJoints())
        jointnames = jointnamesArm + ' ' + jointnamesHand
           
    
        # positioning robot arm
        self.listener.waitForTransform("world",self.robot.GetLinks()[0].GetName(), rospy.Time(), rospy.Duration(3))
        (robot_trans,robot_rot) = self.listener.lookupTransform("world", self.robot.GetLinks()[0].GetName(), rospy.Time(0))
        Trobot = matrixFromQuat([robot_rot[3],robot_rot[0],robot_rot[1],robot_rot[2]])
        Trobot[0:3,3] = robot_trans
        self.robot.SetTransform(Trobot)
        
        # Creating the ros controller
        print('Creating the ros controller')
        print 'Setting controller: trajectoryservice /controller_session joints '+jointnames
        #david 24.4.2012 controller = RaveCreateController(self.env, 'ROSOpenRAVE trajectoryservice /controller_session '+jointnames)
        print self.robot.GetDOF()
        #david 24.4.2012 self.robot.SetController(controller,range(self.robot.GetDOF()),0)
     
        print 'Setting initial joint values/positions' 
        #Initial position
        PI = 3.1415926535
        jointValues =  [0.0, 10*PI/180, 0.0, 90*PI/180, 90*PI/180, 50*PI/180, 90*PI/180, -0.78, -0.78]
    
    
        with self.robot: # save the robot state, check if robot is in collision and set joint values 
            self.robot.SetJointValues(jointValues)
            if self.robot.CheckSelfCollision():
                print 'Robot in self collision! '
            if env.CheckCollision(self.robot):
                print 'Env in collision! '
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
            Tstart = array([[  0,   1,   0,   0.9], 
                            [  1,   0.5,   0,  -0.2], 
                            [  0,   0,   1,   0.3], 
                            [  0,   0,   0,   1  ]])
            #Tstart = array([[  1,   0,   0,   0.9], 
            #                [  0,   0,   -1,  -0.2], 
            #                [  0,   -1,   0,   0.3], 
            #                [  0,   0,   0,   1  ]])
            sol = ikmodel.manip.FindIKSolution(Tstart,IkFilterOptions.CheckEnvCollisions)
            print sol
            print "==="
            print ikmodel.manip.GetArmIndices()
            #robot.SetDOFValues(sol,ikmodel.manip.GetArmIndices())
        #basemanip.MoveToHandPosition([Tstart],maxiter=1000,maxtries=1,seedik=4)
        #robot.WaitForController(0)
        
        stop = False
        max_steps = 800
        updir = array((0,0,1))
        while (not stop):
        
            success = basemanip.MoveHandStraight(direction=updir,stepsize=0.001,minsteps=1,maxsteps=max_steps)
            if (success):
                print "success:"
            else:
                print "no success"
            robot.WaitForController(0)
            input = raw_input("press enter to move arm straight \n press q to quit, or x:val or y:val or z:val with val equal new updir direction for given coordinate: ")
            print "stop: ",stop

            updir = -updir            
            if (input == "q"):
                stop = True
            if (input == ""):
                continue
            if (input[0] == "x"):
                print "x was given"
                updir[0] = string.atof(input[1:])
            if (input[0] == "y"):
                print "y was given"
                updir[1] = string.atof(input[1:])
            if (input[0] == "z"):
                print "z was given"
                updir[2] = string.atof(input[1:])
            if (input[0] == "m"):
                print "m was given (max_steps)"
                max_steps = string.atoi(input[1:])

            print "updir: ", updir



if __name__ == '__main__':
   
    # Node name
    rospy.init_node('sceneUpdater')
    sceneName = '/opt/ros/graspdemounknownobjects/graspingDemoTUWUObjects/data/TUWlab.kintest.env.xml'    #Amtec/OttoBock
    env = Environment()
    env.SetViewer('qtcoin')
    env.Load(sceneName)
    
    self = SceneUpdater(env)
    raw_input("press enter to move arm")
    self.move_arm()
    env.Destroy()
