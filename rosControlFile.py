
###################################### NOT FINISHED ######################################

#!/usr/bin/env python

import sys
import rospy

from crazyflie_driver.msg import GenericLogData
from crazyflie_driver.msg import Position

import math

from paretoConstPath import findPaths

# Cube around the movement of the crazyflie
max_x = 1.0
max_y = 1.0
max_z = 1.6

# Position in Z for each planar movements
z_plan = 1.4

############################ Define some trajectories ####################################
# Define a circular trajectory. Each point returned should be send at the frequency freq
def circle_trajectory(freq, duration,r):
	x = list()
	y = list()
	t = list()
	nb_point = freq * duration
	for i in range(nb_point):
		t0 = 2 * math.pi * (1.0/freq)  * i
		x.append(r * math.cos(t0/duration))
		y.append(r * math.sin(t0/duration))
		t.append((1.0/freq)  * i)
	return (t,x,y)

#########################################################################################

# local position callback [if need]
def local_position_callback(msg):
	global current_position
	current_position.x = msg.values[0]
	current_position.y = msg.values[1]
	current_position.z = msg.values[2]
	current_position.header = msg.header

if __name__ == '__main__':
    rospy.init_node('fancy_traj_node', anonymous=True)

    cf_prefix = "/cf1"

    # SUbscribe to get the local position of the crazyflie with prefix cf_prefix
    rospy.Subscriber(cf_prefix + "/local_position" , GenericLogData , local_position_callback)

    # The publisher to be able to control in position
    pubSetpointPos = rospy.Publisher(cf_prefix +"/cmd_position", Position , queue_size=10)

    # Frequency  between each point in the waypoint list
    freq_pub = 50
    rate = rospy.Rate(freq_pub)

    # INitialize global variable for actual position of crazyflie
    current_position = Position()

    # message used to store pos info to send
    next_pos = Position()


	# Takeoff first -> go to 0, 0 , 1
    while ((0-current_position.x)**2 + (0 - current_position.y)**2 + (1 - current_position.z)**2) > 1e-2 : # 10cm 
        next_pos.x = 0
        next_pos.y = 0
        next_pos.z = 1
        next_pos.yaw = 0.0
        next_pos.header.seq += 1
        next_pos.header.stamp = rospy.Time.now()
        pubSetpointPos.publish(next_pos)
        rate.sleep()


    nmap = [[[0,0,0],
             [0,0,0],
             [0,0,0],
             [0,0,0],
             [0,0,0],
             [0,0,0]]]

    starts = [(0,0,0), (2,5,0)]
    goals = [(0,5,0),(1,0,0)]

    paths = findPaths(starts, goals, nmap)
    
    bounds = (len(map[0][0]), len(map[0]), len(map))

    changeBasisConstants = setConstants(bounds, (3,6,3))

    x, y, z = changeBasis(starts[0])
    gx, gy, gz = (goals[0])
    
    # first go to the first point of the trajectory
    while ((x-current_position.x)**2 + (y - current_position.y)**2 + (z - current_position.z)**2) > 1e-2 : # 10cm 
        next_pos.x = x
        next_pos.y = y
        next_pos.z = z
        next_pos.yaw = 0.0
        next_pos.header.seq += 1
        next_pos.header.stamp = rospy.Time.now()
        pubSetpointPos.publish(next_pos)
        rate.sleep()

    i = 0
    # Then go to goal
    while ((gx-current_position.x)**2 + (gy - current_position.y)**2 + (gz - current_position.z)**2) > 1e-2 : # 10cm 
        i+=1
        if i<len(paths[0]):
            x, y, z = paths[0][i][::-1]
        else:
            x, y, z = goals[0]
        next_pos.x = x
        next_pos.y = y
        next_pos.z = z
        next_pos.yaw = 0.0
        next_pos.header.seq += 1
        next_pos.header.stamp = rospy.Time.now()
        pubSetpointPos.publish(next_pos)
        rate.sleep()

    # Sleep for some time
    for i in range(100):
        rate.sleep()

    # Land -> go to 0 0 0.1
    while ((0-current_position.x)**2 + (0 - current_position.y)**2 + (0.1 - current_position.z)**2) > 1e-2 : # 10cm 
        next_pos.x = 0
        next_pos.y = 0
        next_pos.z = 0.1
        next_pos.yaw = 0.0
        next_pos.header.seq += 1
        next_pos.header.stamp = rospy.Time.now()
        pubSetpointPos.publish(next_pos)
        rate.sleep()
