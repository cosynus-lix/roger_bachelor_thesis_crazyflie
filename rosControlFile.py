#!/usr/bin/env python

import sys
import rospy

from crazyflie_driver.msg import GenericLogData
from crazyflie_driver.msg import Position

import math

from paretoConstPath import findPaths
from helpers import *

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
             [1,0,0],
             [1,0,0],
             [1,0,0],
             [1,0,0],
             [0,0,0],
             [0,0,0]]]

    starts = [(1,3,0), (2,6,0)]
    goals = [(0,5,0),(1,0,0)]

    paths = findPaths(starts, goals, nmap)
    bounds = (len(nmap[0][0]), len(nmap[0]), len(nmap))

    changeBasisConstants = setConstants(bounds, (2.7,5.7,2.7))

    x, y, z = changeBasis(starts[0], changeBasisConstants)
    gx, gy, gz = changeBasis(goals[0], changeBasisConstants)
    
    # first go to the first point of the trajectory
    while d((x,y,z), current_position) > 1e-2: # 10cm (as no root, rather 3cm?) 
        next_pos.x, next_pos.y, next_pos.z = x, y, z
        next_pos.yaw = 0.0
        next_pos.header.seq += 1
        next_pos.header.stamp = rospy.Time.now()
        pubSetpointPos.publish(next_pos)
        rate.sleep()

    print("Starting path")
    # Then go to goal
    for i, point in enumerate(paths[0]):
        gx, gy, gz = changeBasis(point, changeBasisConstants)
        while d((gx,gy,gz), current_position) > 1e-2: # 10cm (as no root, rather 3cm?)
            next_pos.x, next_pos.y, next_pos.z = gx, gy, gz
            next_pos.yaw = 0.0
            next_pos.header.seq += 1
            next_pos.header.stamp = rospy.Time.now()
            pubSetpointPos.publish(next_pos)
            rate.sleep()
    print("Done path")
    # Sleep for some time
    for i in range(100):
        rate.sleep()

    # Land -> go to 0 0 0.1
    while d((0,0,0.1), current_position) > 1e-2: # 10cm (as no root, rather 3cm?)
        next_pos.x, next_pos.y, next_pos.z = 0, 0, 0.1
        next_pos.yaw = 0.0
        next_pos.header.seq += 1
        next_pos.header.stamp = rospy.Time.now()
        pubSetpointPos.publish(next_pos)
        rate.sleep()
