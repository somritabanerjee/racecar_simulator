#!/usr/bin/env python

import rospy
import rosbag
import sys

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

bag = rosbag.Bag('effort_controller2.bag','w')

# vehicle name

car_name = str(sys.argv[1])

# control topics

LRW_effort_topic   = '/{}/left_rear_wheel_effort_controller/command'.format(car_name)
RRW_effort_topic   = '/{}/right_rear_wheel_effort_controller/command'.format(car_name)
LFW_effort_topic   = '/{}/left_front_wheel_effort_controller/command'.format(car_name)
RFW_effort_topic   = '/{}/right_front_wheel_effort_controller/command'.format(car_name)

LSH_topic   = '/{}/left_steering_hinge_position_controller/command'.format(car_name)
RSH_topic   = '/{}/right_steering_hinge_position_controller/command'.format(car_name)

# control publishers

pub_eff_LRW = rospy.Publisher(LRW_effort_topic, Float64, queue_size = 1)
pub_eff_RRW = rospy.Publisher(RRW_effort_topic, Float64, queue_size = 1)
pub_eff_LFW = rospy.Publisher(LFW_effort_topic, Float64, queue_size = 1)
pub_eff_RFW = rospy.Publisher(RFW_effort_topic, Float64, queue_size = 1)
pub_pos_LSH = rospy.Publisher(LSH_topic, Float64, queue_size = 1)
pub_pos_RSH = rospy.Publisher(RSH_topic, Float64, queue_size = 1)

# command callback

def publish_all_same_torque(torque):
    pub_eff_LRW.publish(torque)
    pub_eff_RRW.publish(torque)
    pub_eff_LFW.publish(torque)
    pub_eff_RFW.publish(torque)

    # pub_pos_LSH.publish(steering_angle)
    # pub_pos_RSH.publish(steering_angle)

def publish_two_side_torques(torque_right, torque_left):
    pub_eff_LRW.publish(torque_left)
    pub_eff_RRW.publish(torque_right)
    pub_eff_LFW.publish(torque_left)
    pub_eff_RFW.publish(torque_right)

    # pub_pos_LSH.publish(steering_angle)
    # pub_pos_RSH.publish(steering_angle)

def joint_state_callback(data):
    if rospy.get_rostime().secs < start + 15.0:
        bag.write('/{}/joint_states'.format(car_name), data)
        
        bag.write('/{}/left_rear_wheel_effort_controller/command'.format(car_name), data)
        bag.write('/{}/left_front_wheel_effort_controller/command'.format(car_name), data)
        bag.write('/{}/right_rear_wheel_effort_controller/command'.format(car_name), data)
        bag.write('/{}/right_front_wheel_effort_controller/command'.format(car_name), data)

        bag.write('/{}/left_rear_wheel_effort_controller/state/command'.format(car_name), data)
        bag.write('/{}/left_front_wheel_effort_controller/state/command'.format(car_name), data)
        bag.write('/{}/right_rear_wheel_effort_controller/state/command'.format(car_name), data)
        bag.write('/{}/right_front_wheel_effort_controller/state/command'.format(car_name), data)

if __name__== '__main__':
    rospy.init_node('direct_torque_command', anonymous = True)
    try:
        published_forward = False
        published_neg_torque = False
        published_stop = False

        start = rospy.get_rostime().secs
        while start == 0:
            start = rospy.get_rostime().secs
        print("Start: ",start)

        rospy.Subscriber('/{}/joint_states'.format(car_name), JointState, joint_state_callback)

        while rospy.get_rostime().secs < start + 15.0:
            if rospy.get_rostime().secs > start + 1.0 and not published_forward:
                torque = Float64()
                torque.data = 0.01
                publish_all_same_torque(torque)
                published_forward = True
            if rospy.get_rostime().secs > start + 3.0 and not published_neg_torque:
                torque = Float64()
                torque.data = 0.0
                publish_all_same_torque(torque)
                published_neg_torque = True
            if rospy.get_rostime().secs > start + 11.0 and not published_stop:
                torque = Float64()
                torque = 0.0
                publish_all_same_torque(torque)
                published_stop = True
        rospy.sleep(1.0)
        bag.close()
        print("Rosbag closed.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass