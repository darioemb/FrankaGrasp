#!/usr/bin/env python
import roslib; roslib.load_manifest('arm_grasping')
import rospy
import numpy
import math
import TMatrices
from TMatrices import makeBBToGlobal, gripperYprFromMatrix, BBPtToGlobal, makeKinectToGlobal, KinectPtToGlobal
import sys
import grasp

def get_top_grasp_global_coords(pcd,li,jointangles):
    (ximage,yimage) = (0.0,0.0) 
    (xglobal,yglobal,zglobal) = (0.0,0.0,0.0)
    (xcam,ycam,zcam) = (0.0,0.0,0.0)
    j1,j2,j3 = (0.0,0.0,0.0)
    p0 = (0.0,0.0)
    p1 = (0.0,0.0)
    p2 = (0.0,0.0)
    p3 = (0.0,0.0)    
    # load toGlobal matrix from the file directly
    with open(jointangles, 'r') as ja:
        jalist = ja.read().split("\n")
        jalist = jalist[0]
        jalist = jalist.split(" ")
        (j1, j2, j3, j4) = map(lambda a : float(a), jalist[0:4])
	print (j1,j2,j3,j4)
        #togl = numpy.zeros((3,4))
        #for i in range(0,3):
        #    for j in range(0,4):
        #        togl[i,j] = jalist[i*4+j]	
    with open(li, 'r') as grasps:
        graspslist = grasps.read().split("\n")
        graspslist = graspslist[0].split(" ")
        graspslist = map(float, graspslist)
        ximage = round((graspslist[0]+graspslist[2]+graspslist[4]+graspslist[6])/4.0)
        yimage = round((graspslist[1]+graspslist[3]+graspslist[5]+graspslist[7])/4.0)
        p0 = graspslist[0],graspslist[1]
        p1 = graspslist[2],graspslist[3]
        p2 = graspslist[4],graspslist[5]
        p3 = graspslist[6],graspslist[7]
    with open(pcd, 'r') as pointCloud:
        pointsList = pointCloud.read().split("\n")
        pointsList = pointsList[10:-1]
        tempList = map(lambda entry : entry.split(" "), pointsList)
        minDist = 1e100
        minpos = -1
        (xdata,ydata) = (-1,-1)
        for i in range(0,len(tempList)):
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
    togl = TMatrices.makeKinectToGlobal((j1,j2,j3,j4))
    print togl
    toglobal = numpy.append(togl, [[0.0,0.0,0.0,1.0]], axis=0)
    pointarray = numpy.array([[xcam],[ycam],[zcam],[1.0]])
    pointarray = numpy.dot(toglobal,pointarray)
    xglobal,yglobal,zglobal = float(pointarray[0]/pointarray[3]),float(pointarray[1]/pointarray[3]),float(pointarray[2]/pointarray[3])
    (Xg, Yg, Zg) = KinectPtToGlobal(xcam,ycam,zcam,j1,j2,j3,j4)
    print (Xg,Yg,Zg)
    print (xglobal,yglobal,zglobal)
    print (xcam,ycam,zcam)
    return ((xglobal,yglobal,zglobal),p0,p1,p2,p3,toglobal)
    

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print "python image_to_grasp.py pcd list_of_rects joint_angles"
        sys.exit(1)
    globalgrasp,p0,p1,p2,p3,toglobal = get_top_grasp_global_coords(sys.argv[1],sys.argv[2],sys.argv[3])
    print str(len(sys.argv))
    if len(sys.argv) == 4:
        grasp.grasp(globalgrasp,p0,p1,p2,p3,toglobal)
    else:
        grasp.grasp(globalgrasp,p0,p1,p2,p3,toglobal,sys.argv[4])



