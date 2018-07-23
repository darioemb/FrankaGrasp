#!/usr/bin/python

## David Fischinger, Walter Wohlkinger
## Vienna University of Technology
## Automation and Control Institute
## First Version: 27.04.2011
# 
# !!! IMPORTANT !!!  works only if triggered not more than once in 5 seconds !!!!!!!!!!
# (rostopic pub /SS/doSingleShot std_msgs/String "asdf" -r 0.2) for cyclic triggering
#
# subscribes to topics of openni_camera and publishes a single image or pointcloud if String comes in
# works for usage of 2 kinect cameras, were the perception of point cloud should not be done at the same time (laser pattern)


PKG = 'Trigger'
import roslib; roslib.load_manifest(PKG)
import rospy
import subprocess
import time, sys
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


class Trigger():
    def __init__(self, parent=None):

        #Subscriber
        ss_sub = rospy.Subscriber("/SS/doSingleShot", String, self.start_shot, queue_size=1)
        self.pc_sub = None
        self.pc_rgb_sub = None
        self.rgb_sub= None
        self.pc2_sub = None
        #Publisher
        self.pc_pub = rospy.Publisher("/SS/camera/depth/points", PointCloud2 )
        self.pc_rgb_pub = rospy.Publisher("/SS/camera/rgb/points", PointCloud2 )  
        self.pc2_pub = rospy.Publisher("/SS/camera2/depth/points", PointCloud2 )
        self.rgb_pub = rospy.Publisher("/SS/camera/rgb/image_color", Image)
        self.depth_image_pub = rospy.Publisher("/SS/camera/depth/image", Image)
        self.pc_ = None
        self.pc_rgb_ = None
        self.pc2_ = None
        self.image = None
        self.depth_image = None
        t = None
    
    #triggers the process for publishing 
    def start_shot(self, msg):
        print "start shot"
        self.t = None
        #start subscriber
        self.pc_sub = rospy.Subscriber("/camera/depth/points", PointCloud2, self.pc_callback, queue_size=1)
        self.pc_rgb_sub = rospy.Subscriber("/camera/rgb/points", PointCloud2, self.pc_rgb_callback, queue_size=1)
        self.rgb_sub= rospy.Subscriber("/camera/rgb/image_color", Image, self.rgb_callback, queue_size=1)
        self.depth_image_sub= rospy.Subscriber("/camera/depth/image", Image, self.depth_image_callback, queue_size=1)
 
    
    #starts method to publish point cloud for camera1; unregisters subscriber camera1
    def pc_callback(self,msg):
        self.pc_ = msg
        self.pc_sub.unregister()
        self.do_publish_cam1()
        
    #starts method to publish point cloud for camera1; unregisters subscriber camera1
    def pc_rgb_callback(self,msg):
        self.pc_rgb_ = msg
        self.pc_rgb_sub.unregister()
        self.do_publish_cam1_pc_rgb()

    #starts method to publish point cloud for camera2; unregisters subscriber camera2
    def pc2_callback(self,msg):
        self.pc2_ = msg
        self.pc2_sub.unregister()
        self.do_publish_cam2()
    
    def rgb_callback(self,msg):
        print "rgb_callback"
        self.image = msg
        #time.sleep(1)
        self.rgb_sub.unregister()
        self.rgb_pub.publish(self.image)

    def depth_image_callback(self,msg):
        print "depth_image_callback"
        self.depth_image = msg
        self.depth_image_sub.unregister()
        self.depth_image_pub.publish(self.depth_image)

    #publishes pc for cam1 and starts subscriber to camera2
    def do_publish_cam1(self):
        #print "publish single shot for camera1"
        self.t = rospy.Time.now()
        if self.pc_ == None:
            return
        self.pc_.header.stamp = self.t
        self.pc_pub.publish(self.pc_)
        self.subscribe_cam2()


    #publishes rgb-pc for cam1 and DOES NOT start subscriber to camera2    new 15.August 2012
    def do_publish_cam1_pc_rgb(self):
        #print "publish single shot rgb point cloud for camera1"
        self.t = rospy.Time.now()
        if self.pc_rgb_ == None:
            return
        self.pc_rgb_.header.stamp = self.t
        self.pc_rgb_pub.publish(self.pc_rgb_)
 
    #publish camera2 point cloud
    def do_publish_cam2(self):
        #print "publish single shot for camera2"
        if self.pc2_ == None:
            return
        self.pc2_.header.stamp = self.t
        self.pc2_pub.publish(self.pc2_)

    #if only one camera is used, this subscriber is subscribing to a non existing topic, pff
    def subscribe_cam2(self):
        self.pc2_sub = rospy.Subscriber("/camera2/depth/points", PointCloud2, self.pc2_callback, queue_size=1) #second cam
      
   
   

def main(args):        
    rospy.init_node('Trigger', anonymous=False)

    trig = Trigger()
    rospy.spin()
    
if __name__ == "__main__":        
    main(sys.argv)



   

    
