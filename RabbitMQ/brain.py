#!/usr/bin/env python
from xml.etree.ElementInclude import LimitedRecursiveIncludeError
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class Brain():
    def __init__(self):
        
        rospy.init_node('LidarMQ', anonymous=True)
        self.rate = rospy.Rate(100) # 10hz

        #SPEED LIMITER
        self.speed_limit = 3
        self.EMERGENCY_STOP = False

        #Subscribing to lidar data from scan topic
        self.sub = rospy.Subscriber('/scan', LaserScan, self.emergency_brake)

        #Subscribing to desired speed and steering angle data recieved from DT
        self.sub2=rospy.Subscriber('/DTdata', AckermannDriveStamped, self.DT)   

        #Publishing to Teleop to control the car
        self.pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop',AckermannDriveStamped,queue_size=1)
        

    def emergency_brake(self, scan):  
        self.ranges = scan.ranges
        rospy.loginfo("Afstand: %.2f", np.mean(self.ranges[500:580]))

        # TIME TO COLLISION SHOULD BE IMPLEMENTED HERE
        if (np.mean(self.ranges[500:580]) < 1.5):
            self.drive(0, 0)
            self.EMERGENCY_STOP = True
        else:
            self.EMERGENCY_STOP = False
    
    def drive(self, desired_speed, desired_steering_angle):
        # SPEED LIMITER
        if (desired_speed > self.speed_limit):
            self.speed = self.speed_limit
        else:
            self.speed = desired_speed

        self.steering_angle = desired_steering_angle

        rospy.loginfo("Published drive data: speed: %.2f \t angle: %.2f",self.speed, self.steering_angle)

        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.header.frame_id = "frame_id"
        ack_msg.drive.speed = self.speed
        ack_msg.drive.steering_angle = self.steering_angle
        self.pub.publish(ack_msg)

    def DT(self, data):
        #rospy.loginfo("DT DATA: speed: %.2f \t angle: %.2f",data.drive.speed, data.drive.steering_angle)
        if self.EMERGENCY_STOP == False:
            self.drive(data.drive.speed, data.drive.steering_angle)

if __name__ == '__main__': 
    try: 
        Brain()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
        

        
