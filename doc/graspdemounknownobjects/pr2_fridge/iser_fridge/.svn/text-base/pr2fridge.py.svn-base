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

import actionlib
import time
from pr2_controllers_msgs.msg import *
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, Pr2GripperCommandGoal, Pr2GripperCommandAction
from trajectory_msgs.msg import JointTrajectoryPoint
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from geometry_msgs.msg import PointStamped 
from pr2_gripper_reactive_approach import controller_manager
from std_srvs.srv import Empty
#import orrosplanning.srv 

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
    goal = JointTrajectoryGoal()

    traj = RaveCreateTrajectory(env,'')
    traj.deserialize(openrave_traj)
    spec = traj.GetConfigurationSpecification()
    goal.trajectory.joint_names = [str(j.GetName()) for j in robot.GetJoints(manip.GetArmIndices())]
    print goal.trajectory.joint_names
    starttime = 0.0
    for i in range(traj.GetNumWaypoints()):
        pt=trajectory_msgs.msg.JointTrajectoryPoint()
        data = traj.GetWaypoint(i)
        pt.positions = spec.ExtractJointValues(data,robot,manip.GetArmIndices(),0)
        starttime += spec.ExtractDeltaTime(data)*3
        pt.time_from_start = rospy.Duration(starttime)
        goal.trajectory.points.append(pt)
    joint_action_client.send_goal_and_wait(goal)            

def move_arm_simple(jointnames, traj):        
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
           

def open_gripper():
    goal = Pr2GripperCommandGoal()
    goal.command.position = .08
    goal.command.max_effort = -1.0
    gripper_client.send_goal_and_wait(goal)
#cm = controller_manager.ControllerManager('r', using_slip_controller = use_slip_detection)
#cm.command_gripper(.087,-1)

def close_gripper():
    goal = Pr2GripperCommandGoal()
    goal.command.position = 0.0
    goal.command.max_effort = 50.0
    gripper_client.send_goal_and_wait(goal)
    	

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

    defaultarm = [[0,1.29023451,0,-2.32099996,0,-0.69800004,0]]
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint','torso_lift_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996,0.31])
    waitrobot(robot)
#move_arm(left_jointnames, defaultarm)
    move_arm_simple(right_jointnames, defaultarm)
    raw_input('moved to default')
    open_gripper()
    raw_input('opened the gripper')
#    close_gripper()
#    raw_input('closed the gripper')

    #basemanip.MoveActiveJoints(goal=[1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996,0.31],maxiter=5000,steplength=0.15,maxtries=10)

    bodies = [b for b in env.GetBodies() if not b.IsRobot() and linalg.norm(b.ComputeAABB().extents()) < 1] #0.2
    for b in bodies:
        env.RemoveKinBody(b)
    # find a location for PR2
    movex = movey = 0.0    
    placeorder = [2,1,4,5]#,3,0]
#placeorder = [3,0]
    #for i,b in enumerate(bodies):
    for i in range(len(placeorder)):
        b = bodies[placeorder[i]]
        env.AddKinBody(b)
        print b
        while True:
            #print 'Current location: %.3f %.3f' %(base[0]+movex, base[1]+movey)
            try:
                gmodel, validgrasps =findvalidgrasps(options, base, b)                
                if len(validgrasps)>0:
                    break
            except:
                validgrasps=[]   
            if (len(validgrasps)==0):    
                # move robots
                movey = random.random()*.4-0.2
                movex = random.random()*.6-0.3
                robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
                goal = [base[0]+movex, base[1]+movey, -1.58] 
                robot.SetActiveDOFValues(goal)
                waitrobot(robot)
        print 'Current location: %.3f %.3f' %(base[0]+movex, base[1]+movey)
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
            #robot.SetActiveDOFValues(sol)
            traj = basemanip.MoveActiveJoints(goal=sol,maxiter=5000,steplength=0.15,maxtries=2, outputtraj = True)    
            move_arm(traj)
            #taskprob.ReleaseFingers()
            waitrobot(robot)
            taskprob.ReleaseFingers()
            waitrobot(robot)
            Tgrasp = manip.GetEndEffectorTransform()
            Tgraspobj = numpy.dot(Tgrasp, numpy.dot(linalg.inv(Tplace),Tplaceobj))
            b.SetTransform(Tgraspobj)
            open_gripper()
            raw_input('feed the object [ENTER]...')
            close_gripper()
            taskprob.CloseFingers()
            waitrobot(robot)
            raw_input('placing the object...')
            with env:
                robot.Grab(b)
                traj = basemanip.MoveToHandPosition(matrices=[Tplace],seedik=16, outputtraj = True)
            waitrobot(robot)
            move_arm(traj)
            raw_input('release it [ENTER]...')
            open_gripper()
            taskprob.ReleaseFingers()
            waitrobot(robot)
            robot.Release(b)
            waitrobot(robot)
            raw_input('move the arm back...')
            with env:
                #jointnames = ['r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint','torso_lift_joint']
                #robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
                robot.SetActiveDOFs(manip.GetArmIndices())
                traj = basemanip.MoveActiveJoints(goal=robot.GetDOFValues(range(15,22)),maxiter=5000,steplength=0.15,maxtries=10, outputtraj = True)
                #robot.SetActiveDOFValues(robot.GetDOFValues(range(15,22)))
            waitrobot(robot)
            move_arm(traj)
            raw_input('done [ENTER]')
            break
            #basemanip.MoveToHandPosition(matrices=[Tgrasp],seedik=16)
          except:
            continue
    
            

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
    rospy.init_node('controller_manager')

    goal = JointTrajectoryGoal()

#    use_slip_detection = rospy.get_param('/reactive_grasp_node_right/use_slip_controller')
#    rospy.loginfo("use_slip_detection:"+str(use_slip_detection))
#
#    rospy.loginfo("waiting for compliant_close and grasp_adjustment services")
#    rospy.wait_for_service("r_reactive_grasp/compliant_close")
#    rospy.wait_for_service("r_reactive_grasp/grasp_adjustment")
#    rospy.loginfo("service found")
#    cc_srv = rospy.ServiceProxy("r_reactive_grasp/compliant_close", Empty)
#    ga_srv = rospy.ServiceProxy("r_reactive_grasp/grasp_adjustment", Empty)

    gripper_client = actionlib.SimpleActionClient('r_gripper_controller/gripper_action',Pr2GripperCommandAction)
    gripper_client.wait_for_server()

    joint_trajectory_action_name = 'r_arm_controller/joint_trajectory_action'
    joint_action_client = actionlib.SimpleActionClient(joint_trajectory_action_name, JointTrajectoryAction)
    print "waiting for server..."
    joint_action_client.wait_for_server()
    print "got server"

#    joint_trajectory_action_name = 'l_arm_controller/joint_trajectory_action'
#    joint_action_client = actionlib.SimpleActionClient(joint_trajectory_action_name, JointTrajectoryAction)
#    print "waiting for server..."
#    joint_action_client.wait_for_server()
#    print "got server"

    res = feasible(options,basepos)

