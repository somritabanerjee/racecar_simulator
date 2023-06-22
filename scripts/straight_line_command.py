#!/usr/bin/env python

import rospy
import sys
import rosbag

from sensor_msgs.msg import JointState
from ackermann_msgs.msg import AckermannDrive

speed_limit = 0.250
angle_limit = 0.325
car_name = str(sys.argv[1])
multiplexer_pub = rospy.Publisher('/{}/multiplexer/command'.format(car_name), AckermannDrive, queue_size = 1)
bag = rosbag.Bag('mu_1e-2.bag','w')
start = 0.0

def publish_forward():
    speed = 0.5
    angle = 0.0 
    command                = AckermannDrive()
    command.speed          = speed * speed_limit
    command.steering_angle = angle * angle_limit
    multiplexer_pub.publish(command)

def publish_stop():
    speed = 0.0
    angle = 0.0 
    command                = AckermannDrive()
    command.speed          = speed * speed_limit
    command.steering_angle = angle * angle_limit
    multiplexer_pub.publish(command)

def joint_state_callback(data):
    if rospy.get_rostime().secs < start + 15.0:
        bag.write('/{}/joint_states'.format(car_name), data)
    

if __name__== '__main__':
    rospy.init_node('straight_line_command', anonymous = True)
    try:
        published_forward = False
        published_stop = False

        start = rospy.get_rostime().secs
        while start == 0:
            start = rospy.get_rostime().secs
        print("Start: ",start)

        rospy.Subscriber('/{}/joint_states'.format(car_name), JointState, joint_state_callback)

        while rospy.get_rostime().secs < start + 15.0:
            if rospy.get_rostime().secs > start + 1.0 and not published_forward:
                publish_forward()
                published_forward = True
            if rospy.get_rostime().secs > start + 11.0 and not published_stop:
                publish_stop() 
                published_stop = True
        rospy.sleep(1.0)
        bag.close()
        print("Rosbag closed.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass