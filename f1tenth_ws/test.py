#!/usr/bin/env python
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

def callback(msg=None):
    # steer 0.34 -0.34
    if msg is not None:
	#rospy.loginfo('test:')
	#rospy.loginfo('hilo: %.2f',np.mean(msg.ranges[500:580]))
        if np.mean(msg.ranges[500:580]) < 1.0:
	    #print 'low hihi'
	    actuate(0)
        else:
	    actuate(0)
	    #ack_msg.drive.steering_angle = -math.pi/2.0 
        #ack_msg.drive.steering_angle = math.pi/2.0

def actuate(speed, angle=None,frameid=None):
    ack_msg = AckermannDriveStamped()
    ack_msg.header.stamp = rospy.Time.now()
    if frameid is not None:
        ack_msg.header.frame_id = frameid
    if angle is not None:
        ack_msg.drive.sterring_angle = angle
    ack_msg.drive.speed = speed
    pub.publish(ack_msg)

def verify(speedmsg):
    if (speedmsg.drive.speed > 3):
        rospy.loginfo("Speed lowered to 3 from: %.2f",speedmsg.drive.speed)
        actuate(3)

def body():
    rospy.init_node('test_node')
    rate = rospy.Rate(5)	

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try: 
	pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop',AckermannDriveStamped,queue_size=1)
        sub = rospy.Subscriber('/scan',LaserScan,callback)
        sub2 = rospy.Subscriber('/vesc/low_level/ackermann_cmd_mux/output',AckermannDriveStamped,verify)
        body()
    except rospy.ROSInterruptException:
        pass
