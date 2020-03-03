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
    global current_positions
    i = int(msg.header.frame_id[2])-1
    current_positions[i].x = msg.values[0]
    current_positions[i].y = msg.values[1]
    current_positions[i].z = msg.values[2]
    current_positions[i].header = msg.header

def launchDrone(i, path, changeBasisConstants):
    global current_positions

    def goto(current, goal):
        while d(goal, current) > 1e-2 : # 10cm (as no root, rather 3cm?) 
            next_pos.x, next_pos.y, next_pos.z = goal
            next_pos.yaw = 0.0
            next_pos.header.seq += 1
            next_pos.header.stamp = rospy.Time.now()
            pubSetpointPos.publish(next_pos)
            rate.sleep()

    def rise(current, h):
        gx, gy, gz = current.x, current.y, h
        goto(current, (gx, gy, gz))
    
    def planar(current, pos):
        gx, gy, gx = pos[0], pos[1], current.z
        goto(current, (gx, gy, gz))


    rospy.init_node('fancy_traj_node', anonymous=True)

    cf_prefix = "/cf"+str(i+1)

    # SUbscribe to get the local position of the crazyflie with prefix cf_prefix
    rospy.Subscriber(cf_prefix + "/local_position" , GenericLogData , local_position_callback)

    # The publisher to be able to control in position
    pubSetpointPos = rospy.Publisher(cf_prefix +"/cmd_position", Position , queue_size=10)

    # Frequency  between each point in the waypoint list
    freq_pub = 50
    rate = rospy.Rate(freq_pub)

    # Initialize global variable for actual position of crazyflie
    current_positions[i] = Position()

    # message used to store pos info to send
    next_pos = Position()
    next_pos = current_positions[i]

	# Takeoff first -> go to 0, 0 , 1
    goto(current_positions[i], (0,0,1))

    x, y, z = changeBasis(path[0], changeBasisConstants)
    
    # first go to the first point of the trajectory
    goto(current_positions[i], (x,y,z))
    
    # Then go to goal following path
    print("Starting path")
    for j, point in enumerate(path):
        gx, gy, gz = changeBasis(point, changeBasisConstants)
        goto(current_positions[i], (gx,gy,gz))
    print("Done path")

    # Sleep for some time
    for j in range(100):
        rate.sleep()

    # Land -> go to 0 0 0.1
    goto(current_positions[i], (0,0,0.1))
    return

if __name__ == '__main__':
    global current_positions
    nmap = [[[0,0,0],
             [1,0,0],
             [1,0,0],
             [1,0,0],
             [1,0,0],
             [1,0,0],
             [0,0,0]]]

    starts = [(0,0,0), (2,6,0)]
    goals = [(0,6,0),(0,0,0)]

    paths = findPaths(starts, goals, nmap)
    bounds = (len(nmap[0][0]), len(nmap[0]), len(nmap))

    changeBasisConstants = setConstants(bounds, (2.7,5.7,2.7))

    current_positions = [Position() for _ in starts]
# to hard combine
    for i, path in enumerate(paths):
        print(len(path))
        #launchDrone(i, path, changeBasisConstants)