#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2

def createPointcloud2(points, colours, stamp = None, frame_id = None, seq = None):
	N = len(points)

	#msg = rosmessage('sensor_msgs/PointCloud2')
	msg = PointCloud2()
	xyzrgb = np.asarray(np.hstack([points, colours]), dtype = np.float32)
	#N points with x,y,z,r,g,b attributes
	msg.height = 1
	msg.width = N
	#This message has multiple properties. We set values for the properties below
	if stamp:
		msg.header.stamp = stamp
	if frame_id:
		msg.header.frame_id = frame_id
	if seq: 
		msg.header.seq = seq
	#The header message contains the MessageType, stamp: timestamp; frame_id: the frame ID and sequence: seq.
	msg.is_bigendian = False
	#Stores as a ittle endian sequence: the least significant byte in the smallest address.
	msg.point_step = 24
	#length of the point in bytes (3 * 8)
	msg.row_step = msg.point_step * N
	#Row length in bytes: msg.point_step times the width of the message - N (as pointcloud2 is a 1D array)
	msg.is_dense = True 
	#As we are using an unorganised pointcloud2 datatype (1*N), we take our message to be dense
	#https://answers.ros.org/question/234455/pointcloud2-and-pointfield/
	msg.data = xyzrgb.tostring()

	return msg

def getLines(file):
	with open(file, "r") as f:
 		for i, l in enumerate(f):
			pass
			#There must be no header for the ply file	
	return i + 1

def toPublish():
	rospy.init_node('pc2Publisher', anonymous = True)
	#Name of the node, anonymous is set to true to avoid conflicts between nodes with the same name
	publisher = rospy.Publisher('cloud_in', PointCloud2, queue_size = 10)
	#topic name the message is published to, message type, size of the queue
	rate = rospy.Rate(10)
	#Message rate in Hertz

	#pointCloud = np.loadtxt("./finalRegistration.ply",delimiter=' ')
	pointCloud = open("finalRegistration.ply", "r")
	pointCloudArray = []
	lines = getLines("finalRegistration.ply")
	for i in range(0,lines):
		line = pointCloud.readline()
		line = line.split()
		pointCloudArray.append(line)
	pointCloud = np.asarray(pointCloudArray).reshape(-1,6)
	
	while not rospy.is_shutdown():
		msg = createPointcloud2(pointCloud[: , 0:3], pointCloud[: , 3:6], frame_id = "stereo_camera")
		publisher.publish(msg)
		#Published message of the String type
		#rospy.loginfo(msg)
		rate.sleep()
		#Ensures that the publisher doesn't publish a message at a frequency higher than that specified

if __name__ == '__main__':
	try:
		toPublish()
	except rospy.ROSInterruptException:
		pass