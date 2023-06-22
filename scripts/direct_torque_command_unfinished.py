#!/usr/bin/env python

import rospy
import sys
import rosbag
import rosservice

from sensor_msgs.msg import JointState
# from ackermann_msgs.msg import AckermannDrive
from gazebo_msgs.srv import ApplyJointEffort
from gazebo_msgs.srv import GetJointProperties

speed_limit = 0.250
angle_limit = 0.325
car_name = str(sys.argv[1])
multiplexer_pub = rospy.Publisher('/{}/multiplexer/command'.format(car_name), AckermannDrive, queue_size = 1)
bag = rosbag.Bag('mu_1e-2.bag','w')
start = 0.0

# def publish_forward():
#     speed = 0.5
#     angle = 0.0 
#     command                = AckermannDrive()
#     command.speed          = speed * speed_limit
#     command.steering_angle = angle * angle_limit
#     multiplexer_pub.publish(command)

# def publish_stop():
#     speed = 0.0
#     angle = 0.0 
#     command                = AckermannDrive()
#     command.speed          = speed * speed_limit
#     command.steering_angle = angle * angle_limit
#     multiplexer_pub.publish(command)

# def joint_state_callback(data):
#     if rospy.get_rostime().secs < start + 15.0:
#         bag.write('/{}/joint_states'.format(car_name), data)
    

if __name__== '__main__':
    rospy.init_node('direct_torque_command', anonymous = True)
    try:
        # published_forward = False
        # published_stop = False

        start = rospy.get_rostime().secs
        while start == 0:
            start = rospy.get_rostime().secs
        print("Start: ",start)

        service_list = rosservice.get_service_list() 
        print("List of services: ", service_list)
        service_ready = False
        while (not service_ready) {
            service_list = rosservice.get_service_list() 
            service_ready = "/gazebo/apply_joint_effort" in service_list
            rospy.loginfo("waiting for apply_joint_effort service")
            rospy.sleep(0.5);
        }
        rospy.loginfo("apply_joint_effort service exists")

        rospy.wait_for_service('/gazebo/apply_joint_effort')
        rospy.wait_for_service('/gazebo/get_joint_properties')

        set_trq_client = rospy.ServiceProxy('/gazebo/apply_joint_effort',ApplyJointEffort)
        get_jnt_state_client = rospy.ServiceProxy('/gazebo/get_joint_properties',GetJointProperties)

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

    # ros::init(argc, argv, "example_rviz_marker");
    # ros::NodeHandle nh;
    # ros::Duration half_sec(0.5);
    
    // make sure service is available before attempting to proceed, else node will crash
    bool service_ready = false;
    while (!service_ready) {
      service_ready = ros::service::exists("/gazebo/apply_joint_effort",true);
      ROS_INFO("waiting for apply_joint_effort service");
      half_sec.sleep();
    }
    ROS_INFO("apply_joint_effort service exists");

    ros::ServiceClient set_trq_client = 
       nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
    
    service_ready = false;
    while (!service_ready) {
      service_ready = ros::service::exists("/gazebo/get_joint_properties",true);
      ROS_INFO("waiting for /gazebo/get_joint_properties service");
      half_sec.sleep();
    }
    ROS_INFO("/gazebo/get_joint_properties service exists");
    
    ros::ServiceClient get_jnt_state_client = 
       nh.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");

    gazebo_msgs::ApplyJointEffort effort_cmd_srv_msg;
    gazebo_msgs::GetJointProperties get_joint_state_srv_msg;
    
    ros::Publisher trq_publisher = nh.advertise<std_msgs::Float64>("jnt_trq", 1); 
    ros::Publisher vel_publisher = nh.advertise<std_msgs::Float64>("jnt_vel", 1);     
    ros::Publisher pos_publisher = nh.advertise<std_msgs::Float64>("jnt_pos", 1);  
    ros::Publisher joint_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_states", 1); 

    ros::Subscriber pos_cmd_subscriber = nh.subscribe("pos_cmd",1,posCmdCB); 
     
    std_msgs::Float64 trq_msg;
    std_msgs::Float64 q1_msg,q1dot_msg;
    sensor_msgs::JointState joint_state_msg;

    double q1, q1dot;
    double dt = 0.01;
    ros::Duration duration(dt);
    ros::Rate rate_timer(1/dt);
    
    effort_cmd_srv_msg.request.joint_name = "joint1";
    effort_cmd_srv_msg.request.effort = 0.0;
    effort_cmd_srv_msg.request.duration= duration;

    get_joint_state_srv_msg.request.joint_name = "joint1";
    //double q1_des = 1.0;
    double q1_err;
    double Kp = 10.0;
    double Kv = 3;
    double trq_cmd;

    // set up the joint_state_msg fields to define a single joint,
    // called joint1, and initial position and vel values of 0
	joint_state_msg.header.stamp = ros::Time::now();
	joint_state_msg.name.push_back("joint1");
        joint_state_msg.position.push_back(0.0);
        joint_state_msg.velocity.push_back(0.0);
    while(ros::ok()) {    
        get_jnt_state_client.call(get_joint_state_srv_msg);
        q1 = get_joint_state_srv_msg.response.position[0];
        q1_msg.data = q1;
        pos_publisher.publish(q1_msg);
        
        q1dot = get_joint_state_srv_msg.response.rate[0];
        q1dot_msg.data = q1dot;
        vel_publisher.publish(q1dot_msg);

	joint_state_msg.header.stamp = ros::Time::now();
        joint_state_msg.position[0] = q1; 
        joint_state_msg.velocity[0] = q1dot;

	joint_state_publisher.publish(joint_state_msg);
        
        //ROS_INFO("q1 = %f;  q1dot = %f",q1,q1dot);
        //watch for periodicity
        q1_err= g_pos_cmd-q1;
        if (q1_err>M_PI) {
            q1_err -= 2*M_PI;
        }
        if (q1_err< -M_PI) {
            q1_err += 2*M_PI;
        }        
            
        trq_cmd = Kp*(q1_err)-Kv*q1dot;
        //trq_cmd = sat(trq_cmd, 10.0); //saturate at 1 N-m
        trq_msg.data = trq_cmd;
        trq_publisher.publish(trq_msg);
        // send torque command to Gazebo
        effort_cmd_srv_msg.request.effort = trq_cmd;
        set_trq_client.call(effort_cmd_srv_msg);
        //make sure service call was successful
        bool result = effort_cmd_srv_msg.response.success;
        if (!result)
            ROS_WARN("service call to apply_joint_effort failed!");
        ros::spinOnce();
	rate_timer.sleep();
  }