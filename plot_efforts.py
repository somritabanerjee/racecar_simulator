#!/usr/bin/env python
import matplotlib.pyplot as plt

mu_1eneg2_filepath = '/home/somrita/.ros/mu_1e-2.bag'
mu_1eneg1_filepath = '/home/somrita/.ros/mu_1e-1.bag'
mu_1_filepath = '/home/somrita/.ros/mu_1.bag'
mu_2_filepath = '/home/somrita/.ros/mu_2.bag'
mu_100_filepath = '/home/somrita/.ros/mu_100.bag'

import rosbag

time = {}
position = {}
velocity = {}
effort = {}
for filepath in [mu_100_filepath, mu_2_filepath, mu_1_filepath, mu_1eneg1_filepath, mu_1eneg2_filepath]:
    if filepath == mu_100_filepath:
        key = "mu_100"
    elif filepath == mu_2_filepath:
        key = "mu_2"
    elif filepath == mu_1_filepath:
        key = "mu_1"
    elif filepath == mu_1eneg1_filepath:
        key = "mu_1eneg1"
    elif filepath == mu_1eneg2_filepath:
        key = "mu_1eneg2"
    else:
        key = ""
    print("Opening bag.")
    bag = rosbag.Bag(filepath)

    time[key] = []
    position[key] = [] 
    velocity[key] = []
    effort[key] = []
    for topic, msg, t in bag.read_messages(topics=['/car_1/joint_states']):
        time[key].append(msg.header.seq)
        position[key].append(list(msg.position))
        velocity[key].append(list(msg.velocity))
        effort[key].append(list(msg.effort))
        # print(msg.name)
    bag.close()
    print("Closed bag.")

# print("Shape of position:",len(position["mu_100"]), " x ", len(position["mu_100"][0]))

fig = plt.figure()
keys_to_plot = ["mu_1eneg2","mu_1eneg1","mu_1"]
for key in keys_to_plot:
    plt.plot(time[key], [effort[key][i][0] for i in range(len(effort[key]))])
plt.legend(keys_to_plot)
plt.ylabel('Effort')
plt.xlabel('Time')
plt.grid('on')
plt.show()

fig = plt.figure()
keys_to_plot = ["mu_1eneg2","mu_1eneg1","mu_1"]
for key in keys_to_plot:
    plt.plot(time[key], [velocity[key][i][0] for i in range(len(velocity[key]))])
plt.legend(keys_to_plot)
plt.ylabel('Velocity')
plt.xlabel('Time')
plt.grid('on')
plt.show()

# Using bagpy
# import bagpy
# from bagpy import bagreader
# import pandas as pd

# b = bagreader(bag_filepath)

# # replace the topic name as per your need
# JS = b.message_by_topic('/car_1/joint_states')
# print(JS)
# df_laser = pd.read_csv(LASER_MSG)
# print(df_laser) # prints laser data in the form of pandas dataframe