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

def launchDrones(paths, changeBasisConstants):
    global current_positions

    def wait(i):
        # hold position
        pass

    def goto(i, goal):
        while d(goal, current_positions[i]) > 1e-2 : # 10cm (as no root, rather 3cm?) 
            next_pos[i].x, next_pos[i].y, next_pos[i].z = goal
            next_pos[i].yaw = 0.0
            next_pos[i].header.seq += 1
            next_pos[i].header.stamp = rospy.Time.now()
            pubSetpointPos[i].publish(next_pos[i])
            rate.sleep()

    def rise(i, h):
        gx, gy, gz = current_positions[i].x, current_positions[i].y, h
        goto(i, (gx, gy, gz))
    
    def planar(i, pos):
        gx, gy, gx = pos[0], pos[1], current_positions[i].z
        goto(i, (gx, gy, gz))

    maxD = len(paths)

    rospy.init_node('fancy_traj_node', anonymous=True)
    pubSetpointPos = []

    for i in range(maxD):
        cf_prefix = "/cf"+str(i+1)
        rospy.Subscriber(cf_prefix + "/local_position" , GenericLogData , local_position_callback)
        pubSetpointPos.append(rospy.Publisher(cf_prefix +"/cmd_position", Position , queue_size=10))

    # Frequency  between each point in the waypoint list
    freq_pub = 50
    rate = rospy.Rate(freq_pub)

    # message used to store pos info to send
    next_pos = [Position() for _ in range(maxD)]

    for i in range(maxD):
        next_pos[i].x, next_pos[i].y, next_pos[i].z = current_positions[i].x, current_positions[i].y, current_positions[i].z
    
    # takeoff and go above map
    for i in range(maxD):
        rise(i, 3+0.5*i)

    # go above start
    for i in range(maxD):
        gx, gy, gz = changeBasis(paths[i][0], changeBasisConstants)
        planar(i, (gx,gy))

    # lower to start altitude
    for i in range(maxD):
        gx, gy, gz = changeBasis(paths[i][0], changeBasisConstants)
        rise(i, gz)
    
    # ajust position
    for i in range(maxD):
        gx, gy, gz = changeBasis(paths[i][0], changeBasisConstants)
        goto(i, (gx,gy,gz))

    # Have all drones follow their path step by step
    for j in range(len(paths[0])):
        for i in range(maxD):
            gx, gy, gz = changeBasis(paths[i][j], changeBasisConstants)
            goto(i, (gx,gy,gz))

    # Sleep for some time
    for j in range(100):
        rate.sleep()

    # land    
    for i in range(maxD):
        rise(i, 0.1)

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

    launchDrones(paths, changeBasisConstants)