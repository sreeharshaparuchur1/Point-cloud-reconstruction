#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def toPublish():
	rospy.init_node('topicPublisher', anonymous = True)
	#Name of the node, anonymous is set to true to avoid conflicts between nodes with the same name
	publisher = rospy.Publisher('pracPublish', String, queue_size = 10)
	#topic name the message is published to, message type, size of the queue
	rate = rospy.Rate(10)
	#Message rate in Hertz

	while not rospy.is_shutdown():
		msg = "Lorem Ipsum published at %s" % rospy.get_time()
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