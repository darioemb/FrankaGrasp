#!/usr/bin/env python

from __future__ import with_statement # for python 2.5
__author__ = ''

import roslib; roslib.load_manifest('openrave_ctrl')
import rospy
from itertools import izip
import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

import actionlib
import time, tf
from pr2_controllers_msgs.msg import *
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, Pr2GripperCommandGoal, Pr2GripperCommandAction, PointHeadAction, PointHeadGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from pr2_gripper_reactive_approach import controller_manager
from std_srvs.srv import Empty
from geometry_msgs.msg import PointStamped 
from std_msgs.msg import String
#import orrosplanning.srv 

#graspmethod values: 
# 1 ... OpenRAVE
# 2 ... Yun's Rectangle presentation and David's HAF method
graspmethod = 2
slowfactor = 8
moverightarm = False
SimOnly = False 
initialtrue = True

def waitrobot(robot):
    """busy wait for robot completion"""
    robot.WaitForController(10)
    while not robot.GetController().IsDone():
        time.sleep(0.01)

#right_jointnames have joint indices (in same order): 27-33 (robot.GetJoint("r_shoulder_pan_joint").GetDOFIndex()) == 27
right_jointnames = ["r_shoulder_pan_joint",     
	        "r_shoulder_lift_joint",
		"r_upper_arm_roll_joint",
		"r_elbow_flex_joint",
		"r_forearm_roll_joint",
		"r_wrist_flex_joint",
		"r_wrist_roll_joint"]
#left_jointnames have indices 15-21
left_jointnames = ["l_shoulder_pan_joint",
	        "l_shoulder_lift_joint",
		"l_upper_arm_roll_joint",
		"l_elbow_flex_joint",
		"l_forearm_roll_joint",
		"l_wrist_flex_joint",
		"l_wrist_roll_joint"]
#torso_jointnames index: 12
torso_jointnames = ["torso_lift_joint"]



'''def lift_torso_simple(jointnames, traj):        
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
'''

#!!! only for left arm now!!!!!!!!!!!!!!!!!!!!!!!!!
#def move_arm(openrave_traj):
def move_arm(openrave_traj):    
    global manip
    
    goal = JointTrajectoryGoal()
    traj = RaveCreateTrajectory(env,'')
    traj.deserialize(openrave_traj)
    robot.GetController().SetPath(traj)
    robot.WaitForController(0)        
    
    if (SimOnly):
        return

    spec = traj.GetConfigurationSpecification()
    goal.trajectory.joint_names = left_jointnames
    starttime = 0.0
    dur = dur0 = 5.
    for i in range(traj.GetNumWaypoints()):
        pt=trajectory_msgs.msg.JointTrajectoryPoint()
        data = traj.GetWaypoint(i)
        pt.positions = spec.ExtractJointValues(data,robot,manip.GetArmIndices(),0)
        #pt.time_from_start = rospy.Duration(dur)
        #dur = dur + dur0
        starttime += spec.ExtractDeltaTime(data)*slowfactor
        pt.time_from_start = rospy.Duration(starttime)       
        goal.trajectory.points.append(pt)
    left_joint_action_client.send_goal_and_wait(goal)       
    

def move_arm_simple(joint_action_client, jointnames, traj):
    #move arms in simulation (david)
    with env:
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        traj2 = basemanip.MoveActiveJoints(goal=traj[-1],maxiter=5000,steplength=0.15,maxtries=10, outputtraj = True) #new david
    waitrobot(robot)
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
           
           
#!!!!!! only for left gripper !!!!!!!!!!!!
def open_gripper():
    
    #open left gripper
    v = robot.GetDOFValues()
    print v
    v[robot.GetJoint('l_gripper_l_finger_joint').GetDOFIndex()] = 1 # l gripper
    robot.SetDOFValues(v) 
    #taskprob.ReleaseFingers()
    waitrobot(robot)
    if (SimOnly):    #robot.GetController().SetPath(traj)
        return
    
    goal = Pr2GripperCommandGoal()
    goal.command.position = .08
    goal.command.max_effort = -1.0
    left_gripper_client.send_goal_and_wait(goal)


#!!!!!! only for left gripper !!!!!!!!!!!!
def close_gripper():
    taskprob.CloseFingers()
    waitrobot(robot)
    if (SimOnly):
        return
    goal = Pr2GripperCommandGoal()
    goal.command.position = 0.0
    goal.command.max_effort = 50.0
    left_gripper_client.send_goal_and_wait(goal)

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
    #goal.pointing_frame = "base_footprint"; #david
#goal.min_duration = .5;
    goal.max_velocity = .5;    
    point_head_client.send_goal_and_wait(goal)

def look_ahead():
    look_at(5.0,0.0,1.2)

def look_right():
    look_at(5.0,-10.0,1.2)

def robot_move_to_perception_position():
    #move head to objects (not in simulation due to implementation of look_at)
    global initialtrue
    
    if (initialtrue):
        look_ahead()           
        waitrobot(robot)
        look_at(1.2,0.0,-0.7)
    # david raw_input("Head should now point to objects.")
    #move arms sidewards s.t. they are not blocking the view to objects    
    if (moverightarm or initialtrue):
        arm_to_right = [[-1.5,1.29023451,0,-2.32099996,0,-0.69800004,0]]
        move_arm_simple(right_joint_action_client, right_jointnames, arm_to_right)
    # david raw_input('Right arm should now be out of camera view')
    arm_to_left = [[1.5,1.29023451,0,-2.32099996,0,-0.69800004,0]]
    move_arm_simple(left_joint_action_client, left_jointnames, arm_to_left)    
    # davidraw_input('Left arm should now be out of camera view')
    initialtrue = False


def feasible(options):
    global robot, env, manip, taskprob, basemanip
    global right_joint_action_client, left_joint_action_client
    global right_gripper_client, left_gripper_client,initialtrue
    parser = OptionParser(description='', usage='')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--planner',action="store",type='string',dest='planner',default=None,
                      help='the planner to use')
    parser.add_option('--target', action="store",type='string',dest='target',default='1',
                      help='')
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)

    "Main example code."
    #env.Load('env/exp.env.xml')
    env.Load('/opt/ros/graspdemounknownobjects/openrave_ctrl/env/exp.env.xml')

    robot = env.GetRobots()[0]

    manip = robot.SetActiveManipulator('leftarm') 
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
    defaulttorso = [[0.3]]
    #lift_torso_simple(torso_jointnames, defaulttorso)
    #waitrobot(robot)
    
    arm_to_right = [[-1.5,1.29023451,0,-2.32099996,0,-0.69800004,0]]
    
    if (moverightarm or initialtrue):
        move_arm_simple(right_joint_action_client, right_jointnames, arm_to_right) #defaultarm
    move_arm_simple(left_joint_action_client, left_jointnames, defaultarm)
    
    robot_move_to_perception_position()
    print('Waiting for grasps...')



    
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
    f = open('traj/%d.traj' %idx,'r')
    traj = f.read()
    f.close()
    return traj 


def execute_grasps(msg):
    global manip,gmodel,slowfactor
    env.SetDebugLevel(DebugLevel.Debug)
    print 'grasps received!!!'
    #move left arm to general pre-grasp position
    arm_to_left = [[0.099262998150585813, -0.29549006640386022, 1.2621132860279687, -0.72527447086973229, 1.5078421684533101, -1.9995581195786034, 0.845]]
    move_arm_simple(left_joint_action_client, left_jointnames, arm_to_left)   
    open_gripper() 
    strdata_list = (str(msg.data)).split()
    data_list = map(float,strdata_list)
    print 'data_list: ', data_list
    gpc = (data_list[0], data_list[1], data_list[2])
    #alpha = float(strdata[endpos+1:])
    alpha = data_list[3]
    print 'gpc_x: %f' %gpc[0]
    print 'gpc_y: %f' %gpc[1]
    print 'gpc_z: %f' %gpc[2]
    print 'alpha: ', alpha
    #showGraspPoints(gpc)
 
    manip = robot.SetActiveManipulator('leftarm')
    basemanip = interfaces.BaseManipulation(robot,plannername=options.planner,maxvelmult=3)
    taskprob = interfaces.TaskManipulation(robot,plannername=options.planner)

    gmodel, validgrasps = findvalidgrasps(options, unknown, gpc, alpha)  
              
    if len(validgrasps)>0:
        print 'valid grasp found (findvalidgrasps)'
    else:
       print '!!!!!!!!!!!!!!!!!!!!!! NO GRASP SOLUTION FOUND !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! send new points' 
       return

    for validgrasp in validgrasps:
      try:  
        manip = robot.SetActiveManipulator('leftarm')
        gmodel.moveToPreshape(validgrasp)
        #Tglobalgrasp = gmodel.getGlobalGraspTransform(gmodel.grasps[graspindex],collisionfree=True)
        Tplace = gmodel.getGlobalGraspTransform(validgrasp,collisionfree=True)
        #Tplaceobj = unknown.GetTransform()
        # get collision-free solution
        #sol = manip.FindIKSolution(Tplace, IkFilterOptions.CheckEnvCollisions) 
        #robot.SetActiveDOFs(manip.GetArmIndices())
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in left_jointnames])
        #move to grasping position
                
        #traj_moveactivejoints = basemanip.MoveActiveJoints(goal=sol,maxiter=5000,steplength=0.15,maxtries=20, execute = False, outputtraj = True)    
        
        approachoffset = 0.1
        #taskmanip = interfaces.TaskManipulation(robot,graspername=gmodel.grasper.plannername)
        goals,graspindex,searchtime,traj_go_to_grasp_pos = taskprob.GraspPlanning(gmodel=gmodel,approachoffset=approachoffset,destposes=None,
                                                                    seedgrasps = 3,seedik=5,maxiter=1000,
                                                                    randomgrasps=False,randomdests=False,execute=False,outputtraj=True)
        #print "traj_moveactivejoints: \n",traj_moveactivejoints
        move_arm(traj_go_to_grasp_pos)
        waitrobot(robot)
        raw_input('Press enter to move hand further down')
        #move hand up after grasping object
        slowfactor = 15 #slow down movement near table
        execute_successful = movehandstraight([0,0,-1],9,11)
        if (not execute_successful):
            try:
                raw_input("Press return to use MoveToHandPosition after movehandstraight failed")
                traj_mhdown = basemanip.MoveToHandPosition(matrices=[Tplace],maxiter=1000,maxtries=4,seedik=4,outputtraj=self.or_exec_traj)
                move_arm(traj_mhdown)
                waitrobot(robot)
            except:
                raw_input("MoveToHandPosition had an exception!!!!")
        slowfactor = 5
        raw_input('Press Enter to close fingers')
        close_gripper()
        waitrobot(robot)
        robot.Grab(unknown)    
        arm_to_deliverypos = [[0.662, 0.021, 1.262, -0.689, 1.754, -1.804, -0.447]]
        raw_input('Press enter to move hand up')
        if (not movehandstraight([0,0,1],20,30)): #move hand up after grasping object
            print "move hand up failed, therefore go to delivery position by using jointvalues"
            move_arm_simple(left_joint_action_client, left_jointnames, arm_to_deliverypos)
        else:
            raw_input('Press enter to move hand to side (minus y)')
            if (not movehandstraight([0,-1,0],40,60)): #move hand to side (for delivery)
                print "move hand to side to deliverypos failed, hence go to deliverypos using joint values"
                raw_input("zur seite gehen zum abladen hat nicht gefunkt")
                move_arm_simple(left_joint_action_client, left_jointnames, arm_to_deliverypos)
        waitrobot(robot)
        raw_input('Press enter to move hand down')
        movehandstraight([0,0,-1],0,20)
        raw_input('Press enter to open gripper')
        open_gripper()
        raw_input('After release gripper. Press enter to move hand up')
        robot.Release(unknown)
        movehandstraight([0,0,1.01],20,30) #move hand up after grasping object  
        raw_input('Press enter to move robot in perception position')
        robot_move_to_perception_position()

      except:
        robot.ReleaseAllGrabbed()
        continue

#==================================================================================

def execute_grasps_openrave():
    global manip
    env.SetDebugLevel(DebugLevel.Debug)
    print 'Object iv was received, OpenRAVE default grasping method was started'
    #move left arm to general pre-grasp position
    arm_to_left = [[0.099262998150585813, -0.29549006640386022, 1.2621132860279687, -0.72527447086973229, 1.5078421684533101, -1.9995581195786034, -1.1374764885447735]]
    move_arm_simple(left_joint_action_client, left_jointnames, arm_to_left)   
    open_gripper() 
  
    manip = robot.SetActiveManipulator('leftarm')
    basemanip = interfaces.BaseManipulation(robot,plannername=options.planner,maxvelmult=5)
    taskprob = interfaces.TaskManipulation(robot,plannername=options.planner)

    gmodel, validgrasps = findvalidgrasps(options, unknown)                
    if len(validgrasps)>0:
        print 'valid grasp found (findvalidgrasps)'
    else:
       print '!!!!!!!!!!!!!!!!!!!!!! NO GRASP SOLUTION FOUND !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!' 
       return

    for validgrasp in validgrasps:
      try:  
        manip = robot.SetActiveManipulator('leftarm')
        gmodel.moveToPreshape(validgrasp)
        #Tplace = gmodel.getGlobalGraspTransform(validgrasp,collisionfree=True)
        #Tplaceobj = unknown.GetTransform()
        # get collision-free solution
        #sol = manip.FindIKSolution(Tplace, IkFilterOptions.CheckEnvCollisions) 
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in left_jointnames])
        #move to grasping position
        #traj_go_to_grasp_pos = basemanip.MoveActiveJoints(goal=sol,maxiter=5000,steplength=0.15,maxtries=20, execute = False, outputtraj = True)    
        #move_arm(traj_go_to_grasp_pos)
 
        #new begin
        approachoffset = 0.1
        goals,graspindex,searchtime,traj_go_to_grasp_pos = taskprob.GraspPlanning(gmodel=gmodel,approachoffset=approachoffset,destposes=None,
                                                                    seedgrasps = 3,seedik=5,maxiter=1000,
                                                                    randomgrasps=False,randomdests=False,execute=False,outputtraj=True)
        move_arm(traj_go_to_grasp_pos)
        #new end
     
        
        waitrobot(robot)
        raw_input('Press enter to move hand further down (bad offset')
        #move hand up after grasping object
        slowfactor = 15
        movehandstraight([0,0,-1],7,10)    
        
        raw_input('Press Enter to close fingers')
        close_gripper()
        waitrobot(robot)
        robot.Grab(unknown)
        raw_input('Press enter to move hand up')
        movehandstraight([0,0,1],20,30) #move hand up after grasping object   
        #raw_input('Press enter to move hand to side (minus y)')
        #movehandstraight([0,-1,0],5,60) #move hand to side (for delivery)
        slowfactor = 7
        raw_input('Press enter to move hand down')
        movehandstraight([0,0,-1],0,20)
        raw_input('Press enter to open gripper')
        open_gripper()
        raw_input('After release gripper. Press enter to move hand up')
        robot.Release(unknown)
        movehandstraight([0,0,1.01],20,30) #move hand up after grasping object  
        raw_input('Press enter to move robot in perception position')
        robot_move_to_perception_position()

      except:
        robot.ReleaseAllGrabbed()
        continue


def rollvariation(alfa, angle_diff=pi/45, nr_rolls=6):
    rolls = array([alfa])
    for i in range(1,1+nr_rolls/2):
        rolls = append(rolls,[alfa-i*angle_diff])
        rolls = append(rolls,[alfa+i*angle_diff])
        if (alfa > pi ):
            rolls = append(rolls,[alfa-pi+i*angle_diff]) #opposite hand direction
        else:
            rolls = append(rolls,[alfa+pi+i*angle_diff])
        
    return rolls



def showGraspPoints(graspPoint1):

    print "Tmesh in showGraspPoints",Tmesh
    graspPoint1 = (graspPoint1[0],graspPoint1[1],graspPoint1[2],1)
    graspPoint1 = dot(Tmesh,graspPoint1)
    handles = []
    handles.append(env.plot3(points=array(((graspPoint1[0],graspPoint1[1],graspPoint1[2]),(graspPoint1[0],graspPoint1[1],graspPoint1[2]))),
                               pointsize=5.0,
                               colors=array(((0,0,1),(0,0,1)))))

    env.UpdatePublishedBodies()
    time.sleep(0.1) # give time for environment to update
    raw_input("Grasp points are shown. Press enter to continue")
    

def movehandstraight(dir,minstep=5,maxstep=30):
    try:
        traj_movehandup = basemanip.MoveHandStraight(direction=dir,maxdeviationangle=pi/30,ignorefirstcollision=True,stepsize=0.01,minsteps=minstep,maxsteps=maxstep, execute = False, outputtraj=True)
        move_arm(traj_movehandup)
        waitrobot(robot)
        return True
    except:
        print 'Failed to move straight hand in direction: ',dir  
        dir2 = [0,0,1]
        if (dir == dir2):   
            try:
                arm_to_pregrasp = [[0.099262998150585813, -0.29549006640386022, 1.2621132860279687, -0.72527447086973229, 1.5078421684533101, -1.9995581195786034, -1.1374764885447735]]
                move_arm_simple(left_joint_action_client, left_jointnames, arm_to_pregrasp)
            except:
                return False
        return False  


def findvalidgrasps(options, target, gpc = None, alpha = 0):
    global robot, env, manip, taskprob

    gmodel = databases.grasping.GraspingModel(robot,target)
    if not gmodel.load():
        print 'generating grasping model (one time computation)'
        standoffs = [0.001,0.01,0.02,0.03,0.04,0.05,0.06]
        gmodel.init(friction=0.4,avoidlinks=[])
        if (graspmethod == 1):
            newarfromtop = zeros((0,6))
            newapproachraysor = zeros((0,6))
            zdirmin = 0.9   #treshhold to define the z value of the approachdirection
            newapproachraysor = gmodel.computeBoxApproachRays(delta=0.02,normalanglerange=0)
            for x in range (0,newapproachraysor.size/6-1):
                if (newapproachraysor[x][5] > zdirmin ):
                    newarfromtop = numpy.vstack((newarfromtop, newapproachraysor[x]))
            gmodel.generate(approachrays=newarfromtop,standoffs=standoffs,forceclosure=True)
        elif (graspmethod == 2):   
            with gmodel.target:
                gmodel.target.Enable(False)
                taskmanip = interfaces.TaskManipulation(robot)
                final,traj = taskmanip.ReleaseFingers(execute=False,outputfinal=True) 
                preshapes = array([final])
                alpha -= pi/2
                rolls = rollvariation(alpha, angle_diff=pi/1000, nr_rolls=8) 
                #rolls = array([pi/2 + alpha]) #pi/2 => grasp is perpendicular to robot and table length
                newapproachrays = zeros((0,6))
                ar_new = (gpc[0],gpc[1],gpc[2],0,0,1)   #!!!!!!!!!!!david offset!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                newapproachrays = numpy.vstack((newapproachrays, ar_new))
        
            def checkgraspfn(contacts,finalconfig,grasp,info):
                # check if grasp can be reached by robot
                Tglobalgrasp = gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
                print Tglobalgrasp
                # have to set the preshape since the current robot is at the final grasp!
                gmodel.setPreshape(grasp)
                sol = gmodel.manip.FindIKSolution(Tglobalgrasp,True)
                if sol is not None:
                    print "\n checkgraspfn result: True"
                    return True
                print "\n checkgraspfn result: False"
                return False

                 
            gmodel.generate2(maxGoodGrasps=20, preshapes=preshapes, standoffs=standoffs, rolls=rolls, approachrays=newapproachrays,graspingnoise=None,forceclosure=False,checkgraspfn=checkgraspfn)
            #raw_input('nachdem gmodel mit generate2() generiert wurde')
        #gmodel.generate(approachrays=[0,0,1],forceclosure=False)
        gmodel.save()
        print 'generated %d grasps' %len(gmodel.grasps)
    validgrasps,validindices = gmodel.computeValidGrasps(returnnum=1)
    return gmodel, validgrasps


 


def insert_iv_object(data):
    global unknown, Tmesh
    
    iv_filename = str(data.data)
    print 'insert_iv_object ', iv_filename
    env.UpdatePublishedBodies()
    time.sleep(0.1) # give time for environment to update
    unknown = env.ReadKinBodyXMLFile(iv_filename)
    bodyName = unknown.GetName()
    objectInEnv = env.GetKinBody(bodyName)
    if objectInEnv is not None:   
        print 'Removing Object'
        env.Remove(objectInEnv)
        env.UpdatePublishedBodies()
        time.sleep(0.1) # give time for environment to update

    env.UpdatePublishedBodies()
    env.AddKinBody(unknown)   
    env.UpdatePublishedBodies()

    # positioning of object iv mesh
    listener.waitForTransform("world_or_frame","/table_center_frame", rospy.Time(), rospy.Duration(3))
    (objects_trans,objects_rot) = listener.lookupTransform("world_or_frame", "/table_center_frame", rospy.Time(0))
    Tmesh = matrixFromQuat([objects_rot[3],objects_rot[0],objects_rot[1],objects_rot[2]])
    Tmesh[0:3,3] = objects_trans
    unknown.SetTransform(Tmesh)
    env.UpdatePublishedBodies()
    if (graspmethod == 1):
        execute_grasps_openrave()


from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

def isnum(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


if __name__ == "__main__":
    global listener
    #rospy.init_node('execute_grasp', anonymous=False)
    args=None
    parser = OptionParser(description='', usage='')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--planner',action="store",type='string',dest='planner',default=None,
                      help='the planner to use')
    parser.add_option('--target', action="store",type='string',dest='target',default='1',
                      help='')
    (options, leftargs) = parser.parse_args(args=args)

    rospy.init_node('controller_manager')

    grasps_yun_sub = rospy.Subscriber("/grasps_yun_tcf", String, execute_grasps, queue_size=1)    #subsribes to topics with grasp position in table center frame
    grasps_haf_sub = rospy.Subscriber("/SVM/grasp_hypothesis_eval", String, execute_grasps, queue_size=1)    #subsribes to topics with grasp position in table center frame
    iv_model_sub = rospy.Subscriber("/pc_to_iv/generated_ivfilename", String, insert_iv_object)
    listener = tf.TransformListener()
    
    goal = JointTrajectoryGoal()


    right_gripper_client = actionlib.SimpleActionClient('r_gripper_controller/gripper_action',Pr2GripperCommandAction)
    right_gripper_client.wait_for_server()
    left_gripper_client = actionlib.SimpleActionClient('l_gripper_controller/gripper_action',Pr2GripperCommandAction)
    left_gripper_client.wait_for_server()

    left_joint_trajectory_action_name = 'l_arm_controller/joint_trajectory_action'
    left_joint_action_client = actionlib.SimpleActionClient(left_joint_trajectory_action_name, JointTrajectoryAction)
    left_joint_action_client.wait_for_server()
    print "got left arm server"

    right_joint_trajectory_action_name = 'r_arm_controller/joint_trajectory_action'
    right_joint_action_client = actionlib.SimpleActionClient(right_joint_trajectory_action_name, JointTrajectoryAction)
    right_joint_action_client.wait_for_server()
    print "got right arm server"

    point_head_client = actionlib.SimpleActionClient('head_traj_controller/point_head_action', PointHeadAction)
    point_head_client.wait_for_server()
    print 'got head server'

    res = feasible(options)

    rospy.spin()

