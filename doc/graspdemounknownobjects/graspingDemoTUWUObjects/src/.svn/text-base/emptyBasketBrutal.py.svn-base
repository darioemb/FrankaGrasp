#!/usr/bin/python

import roslib; roslib.load_manifest('graspingDemoTUWUObjects')

#from __future__ import with_statement # for python 2.5
from manage_grasphypothesis.srv import *
from openravepy import *
from numpy import *
import rospy, time, tf
from std_msgs.msg import String
import geometry_msgs.msg
from graspplanningTUW import *
import copy
from math import pi

class SceneUpdater:

    def __init__(self, env):  
        
        self.demo_interactive = False    #define if user has to press return or everything is done using wait function
        
        self.makeNewSS_pub = rospy.Publisher('SS/doSingleShot', String)
        self.listener = tf.TransformListener()
        time.sleep(0.5) #needed, otherwise tf error
        self.nrOfCurObj = 0 #number of current objects respectivly GPs
        self.lastObject = True  #indicates if for all other objects a grasp has allready been tested => new SS
        self.manip = None
        self.TknownObject = numpy.eye(4)
        self.demo = None
        self.env = env
        self.targetname = 'mesh_1'
        self.graspPoint1 = None
        self.graspPoint2 = None
        self.approachvector = None
        self.basket_center_sub = rospy.Subscriber("/SS/basket_position", String, self.moveBasket)
        self.iv_model_sub = rospy.Subscriber("/pc_to_iv/generated_ivfilename", String, self.insert_iv_object)
        self.delete_objects_sub = rospy.Subscriber("/SS/doSingleShot", String, self.delete_objects)
        self.newGraspPointsIn_pub = rospy.Publisher("/manage_grasphypothesis/newGraspPointsIn", String)
        
        self.twocams = False        #indicates if two cameras are used
        self.mesh_in = False        #makes sure that GPs are only accepted/saved after mesh was included in scene (the first time)
        self.grasp_cnt = 0          #count for current grasp hypothesis
        self.grasp_hypothesis_store = []  #list of all grasp hypothesis
        
        # Get the robot in the scene
        robot = self.env.GetRobots()[0] # get the first robot
        self.robot = robot
        
        self.manip = robot.GetManipulators()[0]
        jointnamesArm = ' '.join(robot.GetJoints()[j].GetName() for j in self.manip.GetArmIndices()) #df GetArmJoints=>GetArmIndices
        jointnamesHand = ' '.join(robot.GetJoints()[j].GetName() for j in self.manip.GetGripperJoints())
        jointnames = jointnamesArm + ' ' + jointnamesHand
           
    
        # positioning robot arm
        self.listener.waitForTransform("world",robot.GetLinks()[0].GetName(), rospy.Time(), rospy.Duration(3))
        (robot_trans,robot_rot) = self.listener.lookupTransform("world", robot.GetLinks()[0].GetName(), rospy.Time(0))
        Trobot = matrixFromQuat([robot_rot[3],robot_rot[0],robot_rot[1],robot_rot[2]])
        Trobot[0:3,3] = robot_trans
        robot.SetTransform(Trobot)
        
        
        # positioning of kinect camera
        self.listener.waitForTransform("world","/openni_camera", rospy.Time(), rospy.Duration(3))
        (camera_trans,camera_rot) = self.listener.lookupTransform("world", "/openni_camera", rospy.Time(0))
        Tcamera = matrixFromQuat([camera_rot[3],camera_rot[0],camera_rot[1],camera_rot[2]])
        Tcamera[0:3,3] = camera_trans
        kinectCamera = env.GetKinBody('kinect')
        kinectCamera.SetTransform(Tcamera)
        kinectTripod = env.GetKinBody('kinecttripod')
        Tcameratrans= eye(4)
        Tcameratrans[0:2,3] = camera_trans[0:2]
        kinectTripod.SetTransform(Tcameratrans)

        if (self.twocams):
            #positioning of kinect2 camera
            self.listener.waitForTransform("world","/openni_camera2", rospy.Time(), rospy.Duration(3))
            (camera_trans2,camera_rot2) = self.listener.lookupTransform("world", "/openni_camera2", rospy.Time(0))
            Tcamera2 = matrixFromQuat([camera_rot2[3],camera_rot2[0],camera_rot2[1],camera_rot2[2]])
            Tcamera2[0:3,3] = camera_trans2
            kinectCamera2 = env.GetKinBody('kinect2')
            kinectCamera2.SetTransform(Tcamera2)
            kinectTripod2 = env.GetKinBody('kinecttripod2')
            Tcameratrans2 = eye(4)
            Tcameratrans2[0:2,3] = camera_trans2[0:2]
            kinectTripod2.SetTransform(Tcameratrans2)
    
        # Creating the ros controller
        print('Creating the ros controller')
        print 'Setting controller: trajectoryservice /controller_session joints '+jointnames
        controller = RaveCreateController(self.env, 'ROSOpenRAVE trajectoryservice /controller_session '+jointnames)
        print robot.GetDOF()
        robot.SetController(controller,range(robot.GetDOF()),0)
     
        print 'Setting initial joint values/positions' 
        #Initial position
        jointValues =  [0.0, 10*pi/180, 0.0, 90*pi/180, 90*pi/180, 50*pi/180, 90*pi/180, -1.0, -1.0]
    
    
        with robot: # save the robot state, check if robot is in collision and set joint values 
            robot.SetJointValues(jointValues)
            if robot.CheckSelfCollision():
                print 'Robot in self collision! '
            if env.CheckCollision(robot):
                print 'Env in collision! '
        robot.GetController().SetDesired(jointValues)
        robot.WaitForController(0)
        
        time.sleep(1)
        self.makeNewSS_pub.publish(String("SS_for_emptybb"))
       
    
    def insert_iv_object(self, data):
             
        self.mesh_in = True
        iv_filename = str(data.data)
        print 'insert_iv_object ', iv_filename
        self.env.UpdatePublishedBodies()
        time.sleep(0.1) # give time for environment to update
        unknown = self.env.ReadKinBodyXMLFile(iv_filename)
        bodyName = unknown.GetName()
        objectInEnv = self.env.GetKinBody(bodyName)
        if objectInEnv is not None:   
            print 'Removing Object'
            self.env.Remove(objectInEnv)
            self.env.UpdatePublishedBodies()
            time.sleep(0.1) # give time for environment to update
    
        self.env.UpdatePublishedBodies()
        self.env.AddKinBody(unknown)   
        self.nrOfCurObj = self.nrOfCurObj+1
        self.env.UpdatePublishedBodies()

        
    def delete_objects(self, data):
        #new singleShot was executed
        print 'delete_objects'    
        self.env.UpdatePublishedBodies()
        time.sleep(0.1) # give time for environment to update
        allBodies = self.env.GetBodies();
        for i in range(allBodies.__len__()):
            if allBodies[i].GetName().find('mesh_') == 0:  #object is iv model
                print "delete body ", allBodies[i].GetName()
                obj = self.env.GetKinBody(allBodies[i].GetName())
                self.env.Remove(obj)
            
        self.env.UpdatePublishedBodies()
        self.nrOfCurObj = 0
        time.sleep(0.1) # give time for environment to update
    
            

    # move basket
    def moveBasket(self, data):
        
        factor = 1000.0 #factor for mm to m
        strdata = str(data.data)
        endpos = strdata.find(" ")
        x_center = float(strdata[0:endpos])
        strdata = strdata[endpos+1:]
        endpos = strdata.find(" ")
        y_center = float(strdata[0:endpos])
        alpha = float(strdata[endpos+1:])

        print "               N E U     : "
        print "x_center: ",x_center
        print "y_center: ",y_center
        print "alpha_d : ",alpha*180/3.14159265
          
        # positioning of basket
        basket = env.GetKinBody('basket')
        Tbasket = basket.GetTransform()
        Tbasket[0,3] = x_center
        Tbasket[1,3] = y_center  
        Tbasket[0,0] = cos(alpha);
        Tbasket[0,1] = -sin(alpha);
        Tbasket[1,0] = sin(alpha);
        Tbasket[1,1] = cos(alpha);   
        
        self.TknownObject = Tbasket
        basket.SetTransform(Tbasket)       
        
        self.emptyBoxBrutal(data, Tbasket)
        

# if nothing works, empty the box straight forward
    def emptyBoxBrutal(self,data,Tbasket): 
        
        basket = self.env.GetKinBody('basket')       
        Ttmp = basket.GetTransform()
        basket.SetTransform(eye(4))
        with basket:
            ab = basket.ComputeAABB()
            print "AABB", ab
            abext = ab.extents()
        
        basket.SetTransform(Ttmp)
        
        #object coordinate system!!!
        self.graspPoint1 = (-abext[0]-0.03, 0.0, 0.07)
        self.graspPoint2 = (-abext[0]+0.03, 0.0, 0.07)
        time.sleep(3)   #wait for something iv-insert?
        
        #set robot in good initial position for grasping basket
        jointValues =  [0.0, 10*pi/180, 0.0, 90*pi/180, 90*pi/180, 50*pi/180, 90*pi/180, -1.0, -1.0]
    
        with self.robot: # save the robot state, check if robot is in collision and set joint values 
            self.robot.SetJointValues(jointValues)
        self.robot.GetController().SetDesired(jointValues)
        self.robot.WaitForController(0)
        
        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=self.robot,iktype=IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()

        basemanip = interfaces.BaseManipulation(self.robot)
        Tstart = ikmodel.manip.GetEndEffectorTransform()
        
        sol = ikmodel.manip.FindIKSolution(Tstart,IkFilterOptions.CheckEnvCollisions)
        print ikmodel.manip.GetArmIndices()
        self.robot.SetDOFValues(sol,ikmodel.manip.GetArmIndices())
        self.targetname = 'basket'
        self.approachvector = ((0,0,1))
        
        self.target = self.env.GetKinBody(self.targetname)
        self.robot.WaitForController(0)
        self.emptyboxbrutal = GraspPlanning(self.env.GetRobots()[0], self.targetname, self.TknownObject, self.graspPoint1, self.graspPoint2, self.approachvector, self.demo_interactive, True, True)
        if (self.emptyboxbrutal.performGraspPlanning() == False):
            self.graspPoint1 = (0.0, abext[1]+0.03, 0.07)
            self.graspPoint2 = (0.0, abext[1]-0.03, 0.07)
            self.emptyboxbrutal = GraspPlanning(self.env.GetRobots()[0], self.targetname, self.TknownObject, self.graspPoint1, self.graspPoint2, self.approachvector, self.demo_interactive, True)
            if (self.emptyboxbrutal.performGraspPlanning() == False):
                self.graspPoint1 = (abext[0]-0.03, 0.0, 0.07)
                self.graspPoint2 = (abext[0]+0.03, 0.0, 0.07)
                self.emptyboxbrutal = GraspPlanning(self.env.GetRobots()[0], self.targetname, self.TknownObject, self.graspPoint1, self.graspPoint2, self.approachvector, self.demo_interactive, True)
                if (self.emptyboxbrutal.performGraspPlanning() == False):
                    print "empty basket brutal way did not succeed!!"
                    exit
                
        
        raw_input("end of emptyBoxBrutal - please press enter")

    


if __name__ == '__main__':
   
    # Node name
    rospy.init_node('sceneUpdater')
    sceneName = '/opt/ros/graspdemo/rospackages/graspingDemoTUW/graspingDemoTUWUObjects/data/TUWlab.env.xml'           #Amtec/OttoBock
    env = Environment()
    env.SetViewer('qtcoin')
    env.Load(sceneName)
    
    self = SceneUpdater(env)
          
    rospy.spin()
    env.Destroy()
