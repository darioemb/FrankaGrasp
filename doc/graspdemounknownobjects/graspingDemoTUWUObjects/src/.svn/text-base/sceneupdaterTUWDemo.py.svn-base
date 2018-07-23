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

    def __init__(self, env, twocams,with_box):  
        
        self.demo_interactive = False    #define if user has to press return or everything is done using wait function
        self.or_exec_traj=False          #define if OR should controll arm directly (send trajectories to care-o-bot nodes)
	self.with_box= with_box		 #define if basket is used
        
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
        self.newGraspPointsIn_sub = rospy.Subscriber("/manage_grasphypothesis/newGraspPointsIn", String, self.useGraspPoints)
        self.basket_center_sub = rospy.Subscriber("/SS/basket_position", String, self.moveBasket)
        self.iv_model_sub = rospy.Subscriber("/pc_to_iv/generated_ivfilename", String, self.insert_iv_object)
        self.delete_objects_sub = rospy.Subscriber("/SS/doSingleShot", String, self.delete_objects)
        #self.max_objects_sub = rospy.Subscriber("/pc_merge/nr_segmented_pcs", String, self.set_max_objects)
        self.newGraspPointsIn_pub = rospy.Publisher("/manage_grasphypothesis/newGraspPointsIn", String)
        
        self.twocams = twocams        #indicates if two cameras are used
        
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
        #david 24.4.2012 controller = RaveCreateController(self.env, 'ROSOpenRAVE trajectoryservice /controller_session '+jointnames)
        #david 24.4.2012 robot.SetController(controller,range(robot.GetDOF()),0)
     
        print 'Setting initial joint values/positions' 
        #Initial position
        jointValues =  [0.0, 10*pi/180, 0.0, 90*pi/180, 90*pi/180, 50*pi/180, 90*pi/180, -0.5, -0.5]
    
    
        with robot: # save the robot state, check if robot is in collision and set joint values 
            robot.SetJointValues(jointValues)
            if robot.CheckSelfCollision():
                print 'Robot in self collision! '
            if env.CheckCollision(robot):
                print 'Env in collision! '
        robot.GetController().SetDesired(jointValues)
        robot.WaitForController(0)
       
    
    def insert_iv_object(self, data):
        
#        #david only temp for karthik-gh testing
#        allBodies = self.env.GetBodies();
#        for i in range(allBodies.__len__()):
#            if allBodies[i].GetName().find('mesh_') == 0:  #object is iv model
#                obj = self.env.GetKinBody(allBodies[i].GetName())
#                self.env.Remove(obj)
#        
#        #end tmp
        
        
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
    

    def useGraspPoints(self, data):
        
        grasp_successful = False
        cnt = 0
        rospy.wait_for_service('manage_grasphypothesis/get_best_grasphypothesis')
        get_best_gh = rospy.ServiceProxy('manage_grasphypothesis/get_best_grasphypothesis', BestGraspHypothesis)

        while (grasp_successful == False):
            cnt = cnt+1
            print "useGraspPoints - Try number: ", cnt    
            strdata = ""
            
            if (self.env.GetKinBody('mesh_'+ str(self.nrOfCurObj)) == None):
                print "\n wait for object mesh to arrive\n"
                time.sleep(1)
                self.newGraspPointsIn_pub.publish(String("mesh_not_in_yet"))
                return
                
            try:
                best_gh_response = get_best_gh("getitnow")
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    
            strdata = best_gh_response.best_grasp
            
            #if no grasp hypothesis are available in manage_grasphypothesis, the trigger new scene shot and exit
            if (strdata == "nodata"):
                print "strdata: ", strdata
                fail_cnt = 0
                print "cnt: ", cnt
                while (fail_cnt < 10 and strdata == "nodata"):
                    time.sleep(1)
                    fail_cnt = fail_cnt+1
                    print "No grasp hypothesis available for the ", fail_cnt, " time \n"
                    try:
                        best_gh_response = get_best_gh("getitnow")
                        strdata = best_gh_response.best_grasp
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    
                #even after waiting for 10 seconds no gh was available => trigger new shot
                if (strdata == "nodata"):
                    self.makeNewSS_pub.publish(String("foundnogh"))
                    print "\n No grasp hypothesis available - triggered new scene shot!! \n"
                    return
            
            print "sceneupdaterTUWDemo.py: GRASPHYPOTHESIS received by service call: ", strdata
            ar = zeros(9)
            print "\n GRASP POINTS received!"
            for i in range(9):
                endpos = strdata.find(" ")
                ar[i] = float(strdata[0:endpos])
                strdata = strdata[endpos+1:]
    
            self.graspPoint1 = (ar[0],ar[1],ar[2])
            self.graspPoint2 = (ar[3],ar[4],ar[5])
            self.approachvector = (ar[6],ar[7],ar[8])
          
            self.targetname = 'mesh_' + str(self.nrOfCurObj)
            
            if self.checkObjectsAndGP():
                #if run was successful delete stored grasp hypothesis and set counter to 0, else try next gh
                if (self.run()):
                    grasp_successful = True
   
 
            

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
        
	if (self.with_box):
	        self.TknownObject = Tbasket
	        basket.SetTransform(Tbasket)       
	else:
		self.env.Remove(basket)
        
        #self.emptyBoxBrutal(data, Tbasket)
        

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
        raw_input("temp")
        
        #set robot in good initial position for grasping basket
        jointValues =  [0.0, 10*pi/180, 0.0, 90*pi/180, 90*pi/180, 50*pi/180, 90*pi/180, -0.5, -0.5]
    
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
        self.emptyboxbrutal = GraspPlanning(self.env.GetRobots()[0], self.targetname, self.TknownObject, self.graspPoint1, self.graspPoint2, self.approachvector, self.demo_interactive, True, or_exec_traj = self.or_exec_traj)
        if (self.emptyboxbrutal.performGraspPlanning() == False):
            self.graspPoint1 = (0.0, abext[1]+0.03, 0.07)
            self.graspPoint2 = (0.0, abext[1]-0.03, 0.07)
            self.emptyboxbrutal = GraspPlanning(self.env.GetRobots()[0], self.targetname, self.TknownObject, self.graspPoint1, self.graspPoint2, self.approachvector, self.demo_interactive, True, or_exec_traj = self.or_exec_traj)
            if (self.emptyboxbrutal.performGraspPlanning() == False):
                self.graspPoint1 = (abext[0]-0.03, 0.0, 0.07)
                self.graspPoint2 = (abext[0]+0.03, 0.0, 0.07)
                self.emptyboxbrutal = GraspPlanning(self.env.GetRobots()[0], self.targetname, self.TknownObject, self.graspPoint1, self.graspPoint2, self.approachvector, self.demo_interactive, True, or_exec_traj = self.or_exec_traj)
                if (self.emptyboxbrutal.performGraspPlanning() == False):
                    print "empty basket brutal way did not succeed!!"
                    exit
                
        raw_input("wait before movehandstraight")
        updir=((0,0,-1))
        max_steps = 900
        success = basemanip.MoveHandStraight(direction=updir,stepsize=0.001,minsteps=1,maxsteps=max_steps)
        self.robot.WaitForController(0)
        
        raw_input("end of emptyBoxBrutal - please press enter")


    
    def checkObjectsAndGP(self):
        treshhold_extends_val = 21.15 # !!!!!! david 0.15 is normal 1.15<=> dont use this criteria!, for identifying box (object length equals 2x extent)
        obj = self.env.GetKinBody('mesh_'+ str(self.nrOfCurObj))
        print "obj: ", obj
        if (obj == None):
            print "checkObjectsAndGP in sceneupdaterTUWDemo.py: no object available!!!"
            return False
        ab = obj.ComputeAABB()
        #print "max x,y extent for object mesh_", self.nrOfCurObj, " ", max(ab.extents()[0], ab.extents()[1])
        if (max(ab.extents()[0], ab.extents()[1]) > treshhold_extends_val):
            #raw_input("Return of checkObjectsAndGP: False => pc is part of the box - press enter")
            return False
        else:
            #raw_input("return of checkObjectsAndGP: True - press enter to start grapoint")
            return True
                
       
    def printGraspPoints(self):
        
        handles = []
        handles.append(self.env.plot3(points=array(((self.graspPoint1[0],self.graspPoint1[1],self.graspPoint1[2]),
                                                    (self.graspPoint2[0],self.graspPoint2[1],self.graspPoint2[2]))),
                                       pointsize=5.0,
                                       colors=array(((1,0,1),(1,0,1)))))
        self.env.UpdatePublishedBodies()
        time.sleep(0.1) # give time for environment to update
        raw_input("test wait david")
            
    
    def run(self):

        #print "define object for GraspPlanningDemo"
        #define GraspPlanning object
        self.demo = GraspPlanning(self.env.GetRobots()[0], self.targetname, self.TknownObject, self.graspPoint1, self.graspPoint2, self.approachvector, self.demo_interactive, self.lastObject, or_exec_traj = self.or_exec_traj)
        if (self.demo_interactive):
            raw_input("Press enter to start GraspPlanning")
        else:
            print "Grasp planning is started"
        return self.demo.performGraspPlanning()



if __name__ == '__main__':
   
    #Set PARAMETER if two cameras are used
    twocams = False
    fast_robot = True  #fast_robot <=> only Simulation, no real robot used
    with_box = False

    if (twocams):
        if (fast_robot):
            #fast or_robot and two cams
            sceneName = '/opt/ros/graspdemounknownobjects/graspingDemoTUWUObjects/data/TUWlab2camsfast.env.xml'           #Amtec/OttoBock
        else:
            #slow or_robot => for real demo
            sceneName = '/opt/ros/graspdemounknownobjects/graspingDemoTUWUObjects/data/TUWlab2cams.env.xml'           #Amtec/OttoBock
    else:
        sceneName = '/opt/ros/graspdemounknownobjects/graspingDemoTUWUObjects/data/TUWlab.env.xml'           #Amtec/OttoBock

    # Node name
    rospy.init_node('sceneUpdater')
    env = Environment()
    env.SetViewer('qtcoin')
    env.Load(sceneName)
    
    self = SceneUpdater(env, twocams, with_box)
          
    rospy.spin()
    env.Destroy()
