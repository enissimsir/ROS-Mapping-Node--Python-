#!/usr/bin/env python3

import rospy
import math
import numpy as np
from geometry_msgs.msg import Pose2D
import cv2 as cv
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
resim=np.zeros((360,360),dtype=np.uint8)
mypose=Pose2D()


def callback(data):
	print(data.pose.pose.position.x)
def callback_odom(data):

	mypose.x=data.pose.pose.position.x
	mypose.y=data.pose.pose.position.y
	t3 = +2.0 * (data.pose.pose.orientation.w* data.pose.pose.orientation.z + data.pose.pose.orientation.x * data.pose.pose.orientation.y)
	t4 = +1.0 - 2.0 * (data.pose.pose.orientation.y * data.pose.pose.orientation.y + data.pose.pose.orientation.z * data.pose.pose.orientation.z)
	mypose.theta = math.atan2(t3, t4)
#	mypose.theta=(math.asin(data.pose.pose.orientation.z))*2
#	print(mypose.theta)
	
def cb_scan(data):
	dizi=[]
	for i,cor in enumerate(data.ranges):
		if not math.isinf(cor):
			dizi.append(kartezyen(cor,i))
	dizi=[(round((x+data.range_max)*31),round((y+data.range_max)*31)) for x,y in dizi]
	for x,y in dizi:
		resim[x][y]=255
	im=cv.cvtColor(resim,cv.COLOR_GRAY2BGR)
	cv.imshow("Naber",im)
	cv.waitKey(30)
	
def listener():

	rospy.init_node('listener' , anonymous=False)
	sub=rospy.Subscriber("/scan",LaserScan,cb_scan)
	sub_odom = rospy.Subscriber('/odom',Odometry, callback_odom)
	rospy.spin()

def kartezyen(uzunluk,aci):
	x=uzunluk*math.cos(aci/180*math.pi+mypose.theta)+mypose.x
	y=uzunluk*math.sin(aci/180*math.pi+mypose.theta)+mypose.y
	return x,y

if __name__ == '__main__':
	listener()
