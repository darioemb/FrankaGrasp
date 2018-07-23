#!/usr/bin/env python
import roslib; roslib.load_manifest('openrave_ctrl')
import rospy
import numpy
import math
import sys
import tf
from std_msgs.msg import String


from openravepy import *
from openravepy.misc import OpenRAVEGlobalArguments



global listener

def get_top_grasp_global_coords(msg):
    (ximage,yimage) = (0.0,0.0) 
    (xglobal,yglobal,zglobal) = (0.0,0.0,0.0)
    (xcam,ycam,zcam) = (0.0,0.0,0.0)
    #j1,j2,j3 = (0.0,0.0,0.0)
    p0 = (0.0,0.0)
    p1 = (0.0,0.0)
    p2 = (0.0,0.0)
    p3 = (0.0,0.0)    
      
    strdata = str(msg.data)
    print "rectangle input: ", strdata
    grasplist = strdata.split(" ")
    print "grasplist after split", grasplist
    pcd = grasplist[-1]
    print "pcd path: ",pcd
    graspslist = grasplist[0:-1]  
    graspslist = map(float, graspslist)
    print "grasplist: ", graspslist
    ximage = round((graspslist[0]+graspslist[2]+graspslist[4]+graspslist[6])/4.0)
    yimage = round((graspslist[1]+graspslist[3]+graspslist[5]+graspslist[7])/4.0)
    
    p0 = graspslist[0],graspslist[1]
    p1 = graspslist[2],graspslist[3]
    p2 = graspslist[4],graspslist[5]
    p3 = graspslist[6],graspslist[7]
    alpha = graspslist[9]
    alpha = alpha + 90
    if (alpha > 180):
        alpha = alpha - 180
    
    print "\n \n alpha: ", alpha
    with open(pcd, 'r') as pointCloud:
        pointsList = pointCloud.read().split("\n")
        pointsList = pointsList[11:-1]
        tempList = map(lambda entry : entry.split(" "), pointsList)
        minDist = 1e100
        minpos = -1
        (xdata,ydata) = (-1,-1)
        for i in range(0,len(tempList)):
            #print "templist[i]", tempList[i]
            fl = float(tempList[i][4])
            y = math.floor(fl/640.0)
            x = fl % 640.0
            dist = (ximage - x) * (ximage - x) + (yimage - y) * (yimage - y)
            if dist < minDist:
                (xdata,ydata) = x,y
                minpos = i
                minDist = dist
        (xcam,ycam,zcam) = float(tempList[minpos][0]),float(tempList[minpos][1]),float(tempList[minpos][2])
        print "Target pixel " + str((ximage,yimage))
        print "Nearest pixel " + str((xdata,ydata))
        print "Distance to pixel " + str(math.sqrt(minDist))
        print "Cam position "+str((xcam, ycam, zcam))
        # transformation to /table_center_frame (same frame like pointcloud iv mesh has) coordinate system
        listener.waitForTransform("table_center_frame", "/openni_rgb_optical_frame", rospy.Time(), rospy.Duration(3))
        (camera_trans,camera_rot) = listener.lookupTransform("table_center_frame", "/openni_rgb_optical_frame", rospy.Time(0))
        Tcamera = matrixFromQuat([camera_rot[3],camera_rot[0],camera_rot[1],camera_rot[2]])
        Tcamera[0:3,3] = camera_trans
        #print "Tcamera: ", Tcamera
        rec_center_pnt = [xcam,ycam,zcam,1]
        rec_center_pnt_table_cs = numpy.dot(Tcamera,rec_center_pnt)
        print "Tcam * cam_pos => coordinates in table_center_frame", rec_center_pnt_table_cs
           
        #output in camera coordinate system
        #str_output_grasp_ccs = "" + str(xcam) + " " + str(ycam)  + " " + str(zcam)
        #graspcenter_yun_ccs_pub.publish(String(str_output_grasp_ccs))
        
        alpha_rad = alpha*numpy.pi/180
        #output in table_cnter_frame coordinate system
        str_output_grasp_table_center_frame = "" + str(rec_center_pnt_table_cs[0]) + " " + str(rec_center_pnt_table_cs[1])  + " " + str(rec_center_pnt_table_cs[2]) + " " + str(alpha_rad)
        graspcenter_yun_tcf_pub.publish(String(str_output_grasp_table_center_frame))
        
 

if __name__ == "__main__":
    #    print "python image_to_grasp.py pcd list_of_rects joint_angles"
    rospy.init_node('image_to_grasp')   #actually rectangle to grasp
    rectangle_grasps_sub = rospy.Subscriber("/grasps_rectangle_format", String, get_top_grasp_global_coords, queue_size=1)    #subsribes to topics with grasp position
    graspcenter_yun_ccs_pub = rospy.Publisher("/grasps_yun_ccs", String)
    graspcenter_yun_tcf_pub = rospy.Publisher("/grasps_yun_tcf", String)    #table center frame
    listener = tf.TransformListener()
    rospy.spin()
 



