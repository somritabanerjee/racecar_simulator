#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
from numpy import cos, sin, tan

plt.rcParams.update({'font.size': 14})
plt.rc('legend', fontsize=12)

### READ ROSBAG
import rosbag
mu_100_filepath = '/home/somrita/.ros/from_jugg_torque1e-2.bag'
time = {}
position = {}
velocity = {}
effort = {}
for filepath in [mu_100_filepath]:
    if filepath == mu_100_filepath:
        key = "mu_100"
    print("Opening bag.")
    bag = rosbag.Bag(filepath)
    time[key] = []
    position[key] = [] 
    velocity[key] = []
    effort[key] = []
    for topic, msg, t in bag.read_messages(topics=['/car_1/joint_states']):
        time[key].append(msg.header.stamp.to_time())
        position[key].append(list(msg.position))
        velocity[key].append(list(msg.velocity))
        effort[key].append(list(msg.effort))
    bag.close()
    print("Closed bag.")

print("Shape of time:",len(time["mu_100"]))
print("Shape of velocity:",len(velocity["mu_100"])," x ",len(velocity["mu_100"][0]))
print("First time:",time["mu_100"][0])

# rescale time
time["mu_100"][:] = [(t - time["mu_100"][0]) for t in time["mu_100"]]

N = 1500
fig = plt.figure()
keys_to_plot = ["mu_100"]
for key in keys_to_plot:
    plt.plot(time[key][0:N], [velocity[key][i][4] for i in range(N)], label = "Simulation mu = 100")

# plt.legend(keys_to_plot)
# plt.ylabel('Velocity')
# plt.xlabel('Time')
# plt.grid('on')

### Plot nominal dynamics model
m_chassis = 4
m_wheel = 0.34055
m_steering_hinge = 0.1
m_hokuyo = 0.130
mass_car = m_chassis + 4*m_wheel + 2*m_steering_hinge + m_hokuyo # 5.6992
wt_of_car = mass_car*9.8
I_wheel = 0.00041226 # matches m_wheel*(r_wheel**2)/2
r_wheel = 0.05
L = 0.390

dt = 0.01

# state = [x, y, theta, s] action = [u_torque_1, u_torque_2, u_torque_3, u_torque_4, u_phi] 
# x,y is location, theta is heading, s is speed, u_torque_i is the ith wheel torque, u_phi is steering angle
def dynamics(state, action, constants):
  verbose = False
  mu = constants['mu']; wt_of_car = constants['wt_of_car']; I_wheel = constants['I_wheel']; r_wheel = constants['r_wheel']; L = constants['L']; 
  [x, y, theta, v] = state 
  [u_torque_1, u_torque_2, u_torque_3, u_torque_4, u_phi] = action
  # L = 0.5; dt = 0.01; wt_of_car = 7; mu = 0.5; I_wheel = 0.1; r_wheel = 0.1
  traction_torque = mu*wt_of_car*r_wheel
  if verbose: print("Traction torque: ", traction_torque)
  # drive_torque = u_torque_1 + u_torque_2 + u_torque_3 + u_torque_4
  drive_torque = u_torque_1 + u_torque_2 # rear wheel drive
  if verbose: print("Drive torque: ", drive_torque)
  if drive_torque > traction_torque:
    net_torque = drive_torque - traction_torque
  elif drive_torque < -traction_torque:
    net_torque = drive_torque + traction_torque 
  else:
    net_torque = 0   
  # ang_acc_wheels = net_torque/(4*I_wheel)
  ang_acc_wheels = net_torque/(I_wheel)
  acc = ang_acc_wheels * r_wheel
  if verbose: print("Net forward acceleration: ",acc)
  stdot = np.array([v*cos(theta), v*sin(theta), v/L * tan(u_phi), acc])
  state_new = state + stdot*dt
  return state_new

def get_action(t):
  if t>=14.50 and t<=16.50:
    torque = 0.1
    steering_angle = 0.0
  else:
    torque = 0.0
    steering_angle = 0.0
  return np.array([torque, torque, torque, torque, steering_angle])

N = (int)(30/dt)
state_dim = 4
action_dim = 5
action_vector = np.zeros((action_dim, N))
state_vector = np.zeros((state_dim,N+1))
time_vector = np.zeros((N+1))
# Initial state 
state_vector[:,0] = np.array([0,0,0.0,0])

constants = {'mu': 0.0640, 'wt_of_car':55.78356, 'I_wheel':0.00041226,'r_wheel':0.05,'L':0.390}
# 0.0644 to be exact

for k in range(N):
  t = k*dt 
  time_vector[k] = t
  action = get_action(t)
  state_vector[:,k+1] = dynamics(state_vector[:,k], action, constants)
time_vector[N] = N*dt

plt.plot(time_vector,state_vector[3,:], label = 'Nominal model mu = 0.064')
plt.xlabel('Time [s]')
plt.ylabel('X velocity [m/s]')
plt.grid('true')
# plt.legend()
plt.legend(loc='center right')
plt.title('Apply torque for 2 seconds, then coast')

plt.show()