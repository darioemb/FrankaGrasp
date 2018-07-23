#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Combine the power of grasp sets and randomized planners to get any robot arm picking up objects from a table and putting them in a dish rack. 

.. image:: ../../images/examples/graspplanning.jpg
  :width: 640

**Running the Example**::

  openrave.py --example graspplanning

Description
-----------

The example uses the powerful TaskManipulation problem interface, which takes advantage of many OpenRAVE features. It performs:

* Pick grasps and validate them with the grasper planner
* Move to the appropriate grasp preshape while avoiding obstacles
* Use an RRT and Jacobian-based gradient descent methods to safely move close to an obstacle
* Use CloseFingers to grasp the object while checking for collisions with other unwanted objects
* Use body grabbing to grasp the object and move it to its destination
* Lower the object until collision and then release and move away from it. 

"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

import time
from itertools import izip
import openravepy
from openravepy.interfaces import BaseManipulation, TaskManipulation
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

try:
    from multiprocessing import cpu_count
except:
    def cpu_count(): return 1
#david neu ende
import rospy
from std_msgs.msg import String, Float32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pr2_controllers_msgs.msg import JointTrajectoryGoal, JointTrajectoryActionGoal
import graspingTUW
from openravepy.interfaces import BaseManipulation, TaskManipulation
from openravepy.databases import convexdecomposition,inversekinematics

"""

from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = '2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

import graspingTUW
from openravepy.interfaces import BaseManipulation, TaskManipulation
from openravepy.databases import convexdecomposition,inversekinematics
import numpy,time,traceback,cmath
from numpy import matrix, linalg
from optparse import OptionParser


#from openravepy import __build_doc__
#if not __build_doc__:
from numpy import *
from openravepy import *
#else:
#    from openravepy import with_destroy, metaclass
"""





class GraspPlanning(metaclass.AutoReloader):
    def __init__(self,robot, targetname, TknownObject, graspPoint1,graspPoint2, approachvector, demo_interactive ,lastObject = False, emptybb=False, or_exec_traj=False, randomize=False,dests=None,nodestinations=True,switchpatterns=None):

        self.or_exec_traj = or_exec_traj    #None or True


        # set time for sleeping (with actual arm execution)
#        self.sleep_while_gps_shown = 2.0
#        self.sleep_bef_hand_last_step = 12.0
#        self.sleep_bef_close_fingers = 12.0
#        self.sleep_bef_hand_moves_up = 3.0
#        self.sleep_while_hand_moves_up = 5.0
#        self.sleep_while_hand_above_basket_fjv = 5 #when robot changes from postion above basket to same position but with defined joint values
#        self.sleep_while_hand_delivers_obj = 16.0
#        self.sleep_while_hand_opens = 4.0
#        self.sleep_while_hand_to_initial_pos = 6

        # set time for sleeping (without actual arm execution - only simulation)
        self.sleep_while_gps_shown = 2.0
        self.sleep_bef_hand_last_step = 1.0
        self.sleep_bef_close_fingers = 1.0
        self.sleep_bef_hand_moves_up = 1.0
        self.sleep_while_hand_moves_up = 1.0
        self.sleep_while_hand_above_basket_fjv = 1 #when robot changes from postion above basket to same position but with defined joint values
        self.sleep_while_hand_delivers_obj = 1.0
        self.sleep_while_hand_opens = 1.0
        self.sleep_while_hand_to_initial_pos = 1


        self.obh_close_pub = rospy.Publisher("/obh_gripper_close_command", Float32)
        self.obh_open_pub = rospy.Publisher('/obh_gripper_open_command',Float32)
        self.jointValues_init =  [0.0, 10*pi/180, 0.0, 90*pi/180, 90*pi/180, 50*pi/180, 90*pi/180, -0.78, -0.78]
        
        #new david 7.2.2012
        self.publish_joint_traj_AG = rospy.Publisher('/arm_controller/joint_trajectory_action/goal',JointTrajectoryActionGoal)

        #set parameters
        height_th = 0.05 # if grasp point are lower, the standoffs are adapted to make grasp possible (otherwise sometimes no grasps because hand goes into table)
        standoffs = [0.001]
        maxGoodGrasps = 4 #number of good grasps to search in gernerate2()
        self.demo_interactive = demo_interactive
        self.envreal = robot.GetEnv()
        self.graspPoint1 = graspPoint1
        self.graspPoint2 = graspPoint2
        self.emptybb = emptybb
        
        self.jointTrajAG_seq = 0
        self.ag_goal_id = 0
        self.goal_id = 0
        
        
        print self.graspPoint1
        if ( (self.graspPoint1[2]+self.graspPoint2[2])/2 < height_th): #if hand would collide with basket/desk when grasping because object is so flat
            standoffs = [0.001,0.01,0.02,0.03,0.04,0.05,0.06] 
            maxGoodGrasps = 28
            
        self.approachvector = approachvector
        self.lastObject = lastObject
        print "graspPoints: ", self.graspPoint1, "  ", self.graspPoint2
        self.printGraspPoints()


        #setGraspingPoints
        self.pointsDir = ((self.graspPoint2[0]-self.graspPoint1[0]),(self.graspPoint2[1]-self.graspPoint1[1]),(self.graspPoint2[2]-self.graspPoint1[2]))
        #Normalize
        v=self.pointsDir
        len = (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]) ** 0.5
        self.pointsDir = (v[0] / len, v[1] / len, v[2] / len)
        
        self.makeNewSS_pub = rospy.Publisher('SS/doSingleShot', String)
        self.robot = robot
        self.targetname = targetname
        self.TknownObject = TknownObject
        self.nodestinations = nodestinations
        self.ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
        
        if not self.ikmodel.load():
            self.ikmodel.autogenerate()
        self.switchpatterns = switchpatterns
        with self.envreal:
            self.basemanip = interfaces.BaseManipulation(self.robot)
            self.taskmanip = None
            self.updir = self.approachvector
            print "updir: ", self.updir

            print 'attempting to generate a grasp table'
            self.gmodel = graspingTUW.GraspingModel(robot=self.robot,target=self.envreal.GetKinBody(self.targetname), demo_interactive=self.demo_interactive)
            self.gmodel.init(friction=0.4,avoidlinks=[])
            if not self.gmodel.load() or True: #df
                newapproachrays = zeros((0,6))
                                                        
                # insert some vertical approachvectors
                tar=self.envreal.GetKinBody(self.targetname)
                
                print "output basket.gettransform in graspplanning1", tar.GetTransform()
                
                pkt = numpy.hstack(([(self.graspPoint1[0]+self.graspPoint2[0])/2,
                                     (self.graspPoint1[1]+self.graspPoint2[1])/2,
                                     (self.graspPoint1[2]+self.graspPoint2[2])/2]  ,1)) 
                TknownObjectTr = TknownObject.transpose()

                #self.envreal.drawarrow(p1=pkt, p2=pkt+[0,0,1],linewidth = 0.05) 
                TknownObjectInv = matrix(TknownObject).I
                ar_new = numpy.hstack((dot(array(TknownObjectInv),pkt+[0,0,0,0])[0:3],dot(array(TknownObjectInv[0:3,0:3]),[self.approachvector[0],self.approachvector[1],self.approachvector[2]])))
                approachray = ar_new
                print "approachray: ", approachray
                
                #calculate different pitch-angels
                TknownObjectInv = eye(4)
                newapproachrays = numpy.vstack((newapproachrays, self.calculate_av_new_pitch(TknownObjectInv,nr_av=10,max_diff_angle=pi/10)))
                
                
                #Calculate correct roll
                vtargetdirection =-approachray[3:6]
                print "vtargetdirection: ",vtargetdirection
                                    
                alfazerovec = cross(vtargetdirection,self.gmodel.manip.GetDirection())
                print "self.pointsDir",self.pointsDir
                alfa = arccos(dot(self.pointsDir,alfazerovec)/linalg.norm(self.pointsDir)/linalg.norm(alfazerovec))
                               
                #find correct plane half for alfa
                if ( (cross(alfazerovec, self.pointsDir))[2] > 0):
                    alfa = 2*pi-alfa
                #quick and dirty direction change if hand has opposite direction
                if alfa < 55/180*pi:
                    alfa = alfa + pi
                elif alfa > (55+180)/180*pi:
                    alfa = alfa - pi      
               
                rolls = self.rollvariation(alfa, angel_diff=pi/1145, nr_rolls=8)
                
                # initial preshape for robot is the released fingers
                with self.gmodel.target:
                    self.gmodel.target.Enable(False)
                    taskmanip = TaskManipulation(self.robot)
                    final,traj = taskmanip.ReleaseFingers(execute=False,outputfinal=True)
                    preshapes = array([final])
                try:
                    #print "output basket.gettransform in graspplanning (2)", tar.GetTransform()
                    self.gmodel.generate2(maxGoodGrasps=maxGoodGrasps, preshapes=preshapes, standoffs=standoffs, rolls=rolls, approachrays=newapproachrays,graspingnoise=None,forceclosure=False,checkgraspfn=self.checkgraspfn)
                    
                    self.graspables = []
                    self.graspables.append([self.gmodel, dests]) 
                except:
                    print "Unexpected error"
                    raise 


    def printGraspPoints(self):
    
        handles = []
        handles.append(self.envreal.plot3(points=array(((self.graspPoint1[0],self.graspPoint1[1],self.graspPoint1[2]),(self.graspPoint2[0],self.graspPoint2[1],self.graspPoint2[2]))),
                                   pointsize=5.0,
                                   colors=array(((0,0,1),(0,0,1)))))

        self.envreal.UpdatePublishedBodies()
        time.sleep(0.1) # give time for environment to update
        if (self.demo_interactive):
            raw_input("Grasp points are shown. Press enter to continue")
        else:
            print "Grasp points are shown for (in sec): ", self.sleep_while_gps_shown
            time.sleep(self.sleep_while_gps_shown)
	raw_input('grasppunkte wurden schon gezeigt, press enter')
            
        

    #calculate approachrays: new version, starting from AV (0,0,1)
    def calculate_av_new_pitch(self, TknownObjectInv, nr_av=40, max_diff_angle=pi/4):                
        approachrayspitch = zeros((0,6))
        for i in range(nr_av/2):
            ap_pnt = numpy.hstack((add(self.graspPoint1,self.graspPoint2)/2,1))
            
            #vary pitch angle
            pitmat = matrixFromAxisAngle(cross(self.updir,self.pointsDir), i*max_diff_angle/(nr_av/2))
            pitmat2= matrixFromAxisAngle(cross(self.updir,self.pointsDir),-i*max_diff_angle/(nr_av/2))
            av_new_pitch = dot(pitmat,hstack((self.updir,0)))
            av_new_pitch2= dot(pitmat2,hstack((self.updir,0)))
            av_new_pitch_transformed = numpy.hstack((dot(array(TknownObjectInv),ap_pnt+[0,0,0,0])[0:3],dot(array(TknownObjectInv[0:3,0:3]),av_new_pitch[0:3])))
            av_new_pitch_transformed2= numpy.hstack((dot(array(TknownObjectInv),ap_pnt+[0,0,0,0])[0:3],dot(array(TknownObjectInv[0:3,0:3]),av_new_pitch2[0:3])))
            approachrayspitch = numpy.vstack((approachrayspitch, av_new_pitch_transformed,av_new_pitch_transformed2))
  
        return approachrayspitch


    def rollvariation(self, alfa, angel_diff=pi/45, nr_rolls=6):
        rolls = array([alfa])
        for i in range(1,1+nr_rolls/2):
            rolls = append(rolls,[alfa-i*angel_diff])
            rolls = append(rolls,[alfa+i*angel_diff])
            if (alfa > pi ):
                rolls = append(rolls,[alfa-pi+i*angel_diff]) #opposite hand direction
            else:
                rolls = append(rolls,[alfa+pi+i*angel_diff])
            
        return rolls


    def setGraspingPoints(self, graspPoint1X, graspPoint1Y,graspPoint1Z,graspPoint2X,graspPoint2Y,graspPoint2Z):
        self.graspPoint1 = (graspPoint1X, graspPoint1Y , graspPoint1Z)
        self.graspPoint2 = (graspPoint2X, graspPoint2Y , graspPoint2Z)    
        
        self.pointsDir = ((self.graspPoint2[0]-self.graspPoint1[0]),(self.graspPoint2[1]-self.graspPoint1[1]),(self.graspPoint2[2]-self.graspPoint1[2]))
        #Normalize
        v=self.pointsDir
        len = (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]) ** 0.5
        self.pointsDir = (v[0] / len, v[1] / len, v[2] / len)
        
        #Visualize grasp points
        self.handleGraspPoints = self.env.plot3(points=array((self.graspPoint1,self.graspPoint2)), pointsize=16, colors=array(((0,0,1),(0,0,1))))
        self.env.UpdatePublishedBodies() 


    def checkgraspfn(self, contacts,finalconfig,grasp,info):
        # check if grasp can be reached by robot
        Tglobalgrasp = self.gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
        print Tglobalgrasp
        # have to set the preshape since the current robot is at the final grasp!
        self.gmodel.setPreshape(grasp)
        sol = self.gmodel.manip.FindIKSolution(Tglobalgrasp,True)
        if sol is not None:
            print "\n checkgraspfn result: True"
            return True
        print "\n checkgraspfn result: False"
        return False
 
    def getGraspables(self,dests=None):
        graspables = []
        print 'searching for graspable objects (robot=%s)...'%(self.robot.GetRobotStructureHash())
        #currently: only one object is being grasped
        target = self.envreal.GetKinBody(self.targetname)
        self.gmodel = graspingTUW.GraspingModel(robot=self.robot,target=target, demo_interactive=self.demo_interactive)
        if self.gmodel.load():
            print '%s is graspable'%target.GetName()
            graspables.append([self.gmodel,dests])
        return graspables
                  

    def waitrobot(self,robot=None):
        """busy wait for robot completion"""
        if robot is None:
            robot = self.robot
        while not robot.GetController().IsDone():
            time.sleep(0.01)
            
            
    def graspAndPlaceObject(self,gmodel,dests,waitforkey=True,**kwargs):
        """grasps an object and places it in one of the destinations. If no destination is specified, will just grasp it"""
        env = self.envreal#.CloneSelf(CloningOptions.Bodies)
        env.SetDebugLevel(DebugLevel.Debug) #david 23.1.2012
        
        robot = self.robot
        
        
        #POS_1: arm in initial position over basket 
        if self.or_exec_traj:
            self.publishTrajAG_fromJoints(self.jointValues_init)
        if (self.demo_interactive):
            raw_input('Press enter when arm in in initial position.')
        else:
            print "Wait until hand moves to final grasp position for (in sec): ", self.sleep_bef_hand_last_step
            time.sleep(self.sleep_while_hand_to_initial_pos)
  
        
        with env:
            self.taskmanip = TaskManipulation(self.robot,graspername=gmodel.grasper.plannername)
            print "\n gmodel.grasper.plannername: ",gmodel.grasper.plannername
            
            
            
            print "graspAndPlaceObject, plannername: ", gmodel.grasper.plannername
            robot.SetActiveManipulator(gmodel.manip)
            robot.SetActiveDOFs(gmodel.manip.GetArmIndices())
            
        """    
        raw_input("1 Rosen: press enter to ReleaseFingers()")
        self.taskmanip.ReleaseFingers() 
        self.waitrobot(robot)           
        raw_input("2 Rosen: press enter to close fingers")
        self.taskmanip.CloseFingers() 
        self.waitrobot(robot)           
        raw_input("3 Rosen: press enter to release fingers")    
        """ 
            
        istartgrasp = 0
        approachoffset = 0.065
        target = gmodel.target
        stepsize = 0.001
              
              
              
        raw_input("befor graspplanning")
              
        
        while istartgrasp < len(gmodel.grasps):
            print "len(gmodel.grasps) and istartgrasp: ",len(gmodel.grasps)," ",istartgrasp
            #next line executes moving of manipulator near target
            goals,graspindex,searchtime,trajdata = self.taskmanip.GraspPlanning(graspindices=gmodel.graspindices,grasps=gmodel.grasps[istartgrasp:],
                                                                                target=target,approachoffset=approachoffset,destposes=dests,
                                                                                seedgrasps = 3,seedik=5,maxiter=1000,
                                                                                randomgrasps=False,randomdests=False,outputtraj=self.or_exec_traj)
                        
            if self.or_exec_traj:
                self.publishTrajAG(trajdata)
                
            
            #POS_2: bring arm ~6 cm above object to grasp
            #new 8.2.2012
            istartgrasp = graspindex+1
            Tglobalgrasp = gmodel.getGlobalGraspTransform(gmodel.grasps[graspindex],collisionfree=True)

            print 'grasp %d initial planning time: %f'%(graspindex,searchtime)
            self.waitrobot(robot)
            
            if (self.demo_interactive):
                raw_input('Press enter to move hand to final grasp position.')
            else:
                print "Wait until hand moves to final grasp position for (in sec): ", self.sleep_bef_hand_last_step
                time.sleep(self.sleep_bef_hand_last_step)
                
            expectedsteps = floor(approachoffset/stepsize)
         
            try:
                traj_mhs = self.basemanip.MoveHandStraight(direction=dot(gmodel.manip.GetEndEffectorTransform()[0:3,0:3],gmodel.manip.GetDirection()),
                                                      ignorefirstcollision=False,stepsize=stepsize,minsteps=expectedsteps,maxsteps=expectedsteps,outputtraj=self.or_exec_traj, maxdeviationangle=10)
              
                self.waitrobot(robot)
                
                if (self.demo_interactive):
                    raw_input('Press enter after movehandstraight down')
                
                #POS_3: hand at object
                if self.or_exec_traj:
                    self.publishTrajAG(traj_mhs)
                    
            except planning_error:
                # use a planner to move the rest of the way
                raw_input("ACHTUNG ACHTUNG: MoveHandStraight hat keine Loesung gefunden!!!")
                print "planning error in graspplanningTUW!!"
                try:
                    traj_mhs2 = self.basemanip.MoveToHandPosition(matrices=[Tglobalgrasp],maxiter=1000,maxtries=4,seedik=4,outputtraj=self.or_exec_traj)
                    self.waitrobot(robot)
                    if self.or_exec_traj:
                        self.publishTrajAG(traj_mhs2)
            
                except planning_error,e:
                    print 'failed to reach grasp',e
                    continue
            
            if (self.demo_interactive):
                raw_input('Press enter to close fingers')
            else:
                print "Wait until finger are closing for (in sec): ", self.sleep_bef_close_fingers
                time.sleep(self.sleep_bef_close_fingers)
            
            robot.Grab(target)

            self.obh_close_pub.publish(Float32(21)) #close (mechanical) fingers using ros
            #self.taskmanip.CloseFingers()
            #self.waitrobot(robot)
        


            if self.demo_interactive:
                raw_input('Press enter to move hand up')
            else:
                print "Wait until hand is moved up for (in sec): ",self.sleep_bef_hand_moves_up 
                time.sleep(self.sleep_bef_hand_moves_up)
            try:
                traj_mhs3 = self.basemanip.MoveHandStraight(direction=self.updir,ignorefirstcollision=True,stepsize=0.003,minsteps=20,maxsteps=90,outputtraj=self.or_exec_traj)
                #POS_4: Hand straight above object after grasping
                if self.or_exec_traj:
                    self.publishTrajAG(traj_mhs3)
            
            except:
                print 'failed to move hand up'
            self.waitrobot(robot)
            if self.demo_interactive:
                raw_input("Press enter move to init pos above basket (with fixed jv)")
            else:
                print "Hand should have moved up in straight line, wait for (in sec): ", self.sleep_while_hand_moves_up
                time.sleep(self.sleep_while_hand_moves_up)
                
            # set robot position to default position
            jointValues =  robot.GetDOFValues()
            PI = 3.1415926535
            
            #empty basket brutal way
            if (self.emptybb):
                jointValues[0:7] =  [0.0, 10*PI/180, 0.0, 110*PI/180, 100*PI/180, -70*PI/180, 0*PI/180]
                robot.GetController().SetDesired(jointValues)
                self.publishTrajAG_fromJoints(jointValues)  #new 8.2.2012 david
                robot.WaitForController(0)
                return 1
            
            
            #POS_5: hand above object but controlled by joint values
            if self.or_exec_traj:
                self.publishTrajAG_fromJoints(self.jointValues_init)
            with self.robot: # save the robot state 
                self.robot.SetJointValues(self.jointValues_init)
            self.robot.GetController().SetDesired(self.jointValues_init)
            self.robot.WaitForController(0)
            
            if self.demo_interactive:
                raw_input('Press enter to bring hand in default hand over position')
            else:
                print "Wait until hand is in default position above basket (in sec): ",self.sleep_while_hand_above_basket_fjv 
                time.sleep(self.sleep_while_hand_above_basket_fjv)
                        
            # set robot to hand over position
            #jointValues[0:7] = [1.4242000579833984, 0.9, -0.023394305258987856, 0.98, -1.4819355010986333, -0.4, -1.4457063674926705]
           
            
            #POS_6: hand over position
            jointValues_handover = [2.5] + self.jointValues_init[1:]
            if self.or_exec_traj:
                self.publishTrajAG_fromJoints(jointValues_handover)
            with self.robot: # save the robot state 
                self.robot.SetJointValues(jointValues_handover)
            self.robot.GetController().SetDesired(jointValues_handover)
            self.robot.WaitForController(0)
            
            if self.or_exec_traj:
                self.publishTrajAG(testtraj)
            
            
            if self.demo_interactive:
                raw_input('Press enter to release fingers and bring robot in initial position (fixed jv)')
            else:
                print "Wait while robot moves to hand over position"
                time.sleep(self.sleep_while_hand_delivers_obj)
                
            self.obh_open_pub.publish(Float32(22))
            #res = self.taskmanip.ReleaseFingers(target=target)

            
            #if res is None:
            res = None
            if True:
                print 'problems releasing, releasing target first'
                with env:
                    robot.ReleaseAllGrabbed()
                    try:
                        res = self.taskmanip.ReleaseFingers(target=target)
                    except planning_error:
                        res = None
                if res is None:
                    print 'forcing fingers'
                    with env:
                        robot.SetDOFValues(gmodel.grasps[graspindex][gmodel.graspindices['igrasppreshape']],manip.GetGripperIndices())
            
            
            self.waitrobot(robot)
            with env:
                robot.ReleaseAllGrabbed()
            if env.CheckCollision(robot):
                print 'robot in collision, moving back a little'    #!!! not done for real robot!!
                try:
                    self.basemanip.MoveHandStraight(direction=-dot(gmodel.manip.GetEndEffectorTransform()[0:3,0:3],gmodel.manip.GetDirection()),
                                                    stepsize=stepsize,minsteps=1,maxsteps=10)
                    self.waitrobot(robot)
                except planning_error,e:
                    pass
                if env.CheckCollision(robot):
                    try:
                        self.taskmanip.ReleaseFingers(target=target)
                    except planning_error:
                        res = None
                    #raise ValueError('robot still in collision?')

            time.sleep(self.sleep_while_hand_opens)
            # bring robot in default position with pathplanning
            #Tposdefault = array([[ -0.866025404,  0.50, -0.0, 0.94838], [ 0.0, -0.0,  -1.000,  -0.17750],[ -0.50, -0.866025404, -0.0, 0.474018027], [  0.00, 0.00, 0.0, 1.0]])
            #self.makeNewSS_pub.publish(String("makeitso"))
            #traj_mhtp = self.basemanip.MoveToHandPosition(matrices=[Tposdefault],maxiter=1000,maxtries=2,seedik=5,outputtraj=self.or_exec_traj)
            
            
             #POS_7: hand above object but controlled by joint values
            if self.or_exec_traj:
                self.publishTrajAG_fromJoints(self.jointValues_init)
            with self.robot: # save the robot state 
                self.robot.SetJointValues(self.jointValues_init)
            self.robot.GetController().SetDesired(self.jointValues_init)
                      
            if self.or_exec_traj:
                self.publishTrajAG(self.jointValues_init)
            
            robot.WaitForController(0)
            
            if self.demo_interactive:
                pass #raw_input("Press enter if arm is in initial position above basket")
            else:
                print "Wait until arm is in initial position above basket (for sec): ",self.sleep_while_hand_to_initial_pos
                time.sleep(self.sleep_while_hand_to_initial_pos)
            

            #delete grasped/delivered object
            objInBox = env.GetKinBody(self.targetname)
            env.Remove(objInBox)
            
            return graspindex # return successful grasp index
        # exhausted all grasps
        return -1


#david new start 7.2.2012

    # parse trajectory data into the ROS structure
    def publishTrajAG(self, trajdata):
        
        #build trajectory: trajectory_msgs.msg.JointTrajectory
        traj = JointTrajectory()
        tokens = trajdata.split()
        numpoints = int(tokens[0])
        dof = int(tokens[1])
        trajoptions = int(tokens[2])
        numvalues = dof
        offset = 0
        if trajoptions & 4:
            numvalues += 1
            offset += 1
        if trajoptions & 8:
            numvalues += 7
        if trajoptions & 16:
            numvalues += dof
        if trajoptions & 32:
            numvalues += dof
        traj.joint_names = [j.GetName() for j in self.robot.GetJoints(self.gmodel.manip.GetArmIndices())]
        for i in range(numpoints):
            start = 3+numvalues*i
            pt=JointTrajectoryPoint()
            #correct joint values for our skewed Amtec robot (and mirror axes 2 and 4)
            for j in self.robot.GetJoints(self.gmodel.manip.GetArmIndices()):
                jointCorrection = (-0.08,-0.1,-0.09,0.17,0.16,0.028,0.0)
                if (j.GetDOFIndex() == 0):                                
                    pt.positions.append(jointCorrection[j.GetDOFIndex()]+float(tokens[start+offset+j.GetDOFIndex()]))
                elif (j.GetDOFIndex() == 1):                          
                    #mirrored!      
                    pt.positions.append(jointCorrection[j.GetDOFIndex()]-float(tokens[start+offset+j.GetDOFIndex()]))
                elif (j.GetDOFIndex() == 2):                                
                    pt.positions.append(jointCorrection[j.GetDOFIndex()]+float(tokens[start+offset+j.GetDOFIndex()]))
                elif (j.GetDOFIndex() == 3):                                
                    #mirrored!
                    pt.positions.append(jointCorrection[j.GetDOFIndex()]-float(tokens[start+offset+j.GetDOFIndex()]))
                elif (j.GetDOFIndex() == 4):                                
                    pt.positions.append(jointCorrection[j.GetDOFIndex()]+float(tokens[start+offset+j.GetDOFIndex()]))
                elif (j.GetDOFIndex() == 5):                                
                    pt.positions.append(jointCorrection[j.GetDOFIndex()]+float(tokens[start+offset+j.GetDOFIndex()]))
                elif (j.GetDOFIndex() == 6):                                
                    pt.positions.append(jointCorrection[j.GetDOFIndex()]+float(tokens[start+offset+j.GetDOFIndex()]))

            
            if trajoptions & 4:
                pt.time_from_start = rospy.Duration(float(tokens[start]))
            traj.points.append(pt)
        
        traj.header.stamp = rospy.Time.now()
        
        #build trajectoryActionGoal: pr2_controllers_msgs/JointTrajectoryActionGoal
        jointTrajAG = JointTrajectoryActionGoal()
        jointTrajAG.header.stamp = traj.header.stamp
        self.jointTrajAG_seq = self.jointTrajAG_seq+1
        jointTrajAG.header.seq = self.jointTrajAG_seq
        self.goal_id = self.goal_id+1
        jointTrajAG.goal_id.id = str(self.goal_id)
        jointTrajAG.goal.trajectory = traj
        
        #publish JointTrajectoryActionGoal
        self.publish_joint_traj_AG.publish(jointTrajAG)    
        

    # parse trajectory data into the ROS structure
    def publishTrajAG_fromJoints(self, jointvalues):
        
        #build trajectory: trajectory_msgs.msg.JointTrajectory
        traj = JointTrajectory()
        traj.joint_names = [j.GetName() for j in self.robot.GetJoints(self.gmodel.manip.GetArmIndices())]
        pt=JointTrajectoryPoint()
        #correct joint values for our skewed Amtec robot (and mirror axes 2 and 4)
        jointCorrection = (-0.08,-0.1,-0.09,0.17,0.16,0.028,0.0)
        for j in self.robot.GetJoints(self.gmodel.manip.GetArmIndices()):
            if (j.GetDOFIndex() == 0):                                
                pt.positions.append(jointCorrection[j.GetDOFIndex()]+jointvalues[j.GetDOFIndex()])
            elif (j.GetDOFIndex() == 1):                          
                #mirrored!      
                pt.positions.append(jointCorrection[j.GetDOFIndex()]-jointvalues[j.GetDOFIndex()])
            elif (j.GetDOFIndex() == 2):                                
                pt.positions.append(jointCorrection[j.GetDOFIndex()]+jointvalues[j.GetDOFIndex()])
            elif (j.GetDOFIndex() == 3):                                
                #mirrored!
                pt.positions.append(jointCorrection[j.GetDOFIndex()]-jointvalues[j.GetDOFIndex()])
            elif (j.GetDOFIndex() == 4):                                
                pt.positions.append(jointCorrection[j.GetDOFIndex()]+jointvalues[j.GetDOFIndex()])
            elif (j.GetDOFIndex() == 5):                                
                pt.positions.append(jointCorrection[j.GetDOFIndex()]+jointvalues[j.GetDOFIndex()])
            elif (j.GetDOFIndex() == 6):                                
                pt.positions.append(jointCorrection[j.GetDOFIndex()]+jointvalues[j.GetDOFIndex()])

        pt.time_from_start = rospy.Duration(5.0)
        traj.points.append(pt)
        
        traj.header.stamp = rospy.Time.now()
        
        #build trajectoryActionGoal: pr2_controllers_msgs/JointTrajectoryActionGoal
        jointTrajAG = JointTrajectoryActionGoal()
        self.jointTrajAG_seq = self.jointTrajAG_seq+1
        jointTrajAG.header.seq = self.jointTrajAG_seq
        jointTrajAG.header.stamp = traj.header.stamp
        self.goal_id = self.goal_id+1
        jointTrajAG.goal_id.id = str(self.goal_id)
        jointTrajAG.goal.trajectory = traj
        
        #publish JointTrajectoryActionGoal
        self.publish_joint_traj_AG.publish(jointTrajAG)    
           
#david new




    def performGraspPlanning(self,withreplacement=False,**kwargs):
        print 'starting to pick and place objects'
        graspables = self.graspables
        cnt = 0
        while (True and cnt<10):
            cnt = cnt+1
            if len(graspables) == 0:
                    break
            i = 0
            try:
                print 'grasping object %s'%graspables[i][0].target.GetName()
                with self.envreal:
                    self.robot.ReleaseAllGrabbed()
                success = self.graspAndPlaceObject(graspables[i][0],graspables[i][1],**kwargs)
                print 'success: ',success
                if (self.lastObject and success >-1 and not self.emptybb):       #david: make new SS if last object was grasped
                #when mesh segmentation is done, this must be changed s.t. SS is done after each grasping try!!!!
                    if self.demo_interactive:
                        raw_input("Press enter for next SingleShot of scene")
                    else:
                        print "Next SingleShot of scene is done!"
                        
                    self.makeNewSS_pub.publish(String("makeitso"))
                graspables.pop(i)       #???????????????????????????????????????
                if (success == -1):   #david 29.11.2011
                    return False
                else:
                    return True
            except planning_error, e:
                print 'failed to grasp object %s'%graspables[i][0].target.GetName()
                print e
                return False

