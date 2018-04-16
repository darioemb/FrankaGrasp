#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

class stateNode():

    def __init__(self):
        #    pub = rospy.Publisher('state', PointStamped, queue_size=10)
        markerPub = rospy.Publisher('/find_grasps/grasp_hypotheses_visual', MarkerArray, queue_size=10)


        rospy.init_node('stateNode', anonymous=True)

        rate = rospy.Rate(10)
        #self.state = PointStamped()

        # initial starting location I might want to move to the param list

        self.robotMarker = Marker()
        self.robotMarker.header.frame_id = "/camera_link"
        self.robotMarker.ns = "robot"
        self.robotMarker.id = 0
        self.robotMarker.type = 0 # sphere
        self.robotMarker.action = 0
        self.robotMarker.pose.position.x = 0.62092
        self.robotMarker.pose.position.y = -0.383219
        self.robotMarker.pose.position.z = -0.676303
        self.robotMarker.pose.orientation.x = -0.458083
        self.robotMarker.pose.orientation.y = 0.885676
        self.robotMarker.pose.orientation.z = 0.0757526
        self.robotMarker.pose.orientation.w = 1.0
        self.robotMarker.scale.x = 0.1
        self.robotMarker.scale.y = 0.1
        self.robotMarker.scale.z = 0.1

        self.robotMarker.color.r = 0.5
        self.robotMarker.color.g = 1.0
        self.robotMarker.color.b = 0.0
        self.robotMarker.color.a = 1.0

        self.robotMarker.lifetime = rospy.Duration(1000)
        array = MarkerArray()
        array.markers.append(self.robotMarker)

	while not rospy.is_shutdown():
	      markerPub.publish(array)
	      rate.sleep()


stateNode()

class stateNode():

    def __init__(self):
        #    pub = rospy.Publisher('state', PointStamped, queue_size=10)
        markerPub = rospy.Publisher('/find_grasps/grasp_hypotheses_visual', MarkerArray, queue_size=10)


        rospy.init_node('stateNode', anonymous=True)

        rate = rospy.Rate(10)
        #self.state = PointStamped()

        # initial starting location I might want to move to the param list

        self.robotMarker = Marker()
        self.robotMarker.header.frame_id = "/base_link"
        self.robotMarker.ns = "robot"
        self.robotMarker.id = 0
        self.robotMarker.type = 0 # sphere
        self.robotMarker.action = 0
        self.robotMarker.pose.position.x = 0.00330322
        self.robotMarker.pose.position.y = -0.567077
        self.robotMarker.pose.position.z = 0.823658
        self.robotMarker.pose.orientation.x = 0.380023
        self.robotMarker.pose.orientation.y = 0.76258
        self.robotMarker.pose.orientation.z = 0.523502
        self.robotMarker.pose.orientation.w = 1.0
        self.robotMarker.scale.x = 1.0
        self.robotMarker.scale.y = 0.1
        self.robotMarker.scale.z = 0.1

        self.robotMarker.color.r = 0.5
        self.robotMarker.color.g = 1.0
        self.robotMarker.color.b = 0.0
        self.robotMarker.color.a = 1.0

        self.robotMarker.lifetime = rospy.Duration(1000)
        array = MarkerArray()
        array.markers.append(self.robotMarker)

	while not rospy.is_shutdown():
	      markerPub.publish(array)
	      rate.sleep()


stateNode()
