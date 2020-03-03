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

    def goto(goals):
        while len([1 for g, c in zip(goals, current_positions) if d(g, c)>0.1])>0 : # root, rather 3cm 
            for nextP, setP, goal in zip(next_pos, pubSetpointPos, goals):
                nextP.x, nextP.y, nextP.z = goal
                nextP.yaw = 0.0
                nextP.header.seq += 1
                nextP.header.stamp = rospy.Time.now()
                setP.publish(nextP)
            rate.sleep()

    def rise(pos):
        goals = [(c.x, c.y, p[2]) for p, c in zip(pos, current_positions)]
        goto(goals)

    def planar(pos):
        goals = [(p[0], p[1], c.z) for p, c in zip(pos, current_positions)]
        goto(goals)

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
    alts = [(0,0,3+0.5*i) for i in range(maxD)]
    rise(alts)

    # go above start
    goals = [changeBasis(paths[i][0], changeBasisConstants) for i in range(maxD)]
    planar(goals)

    # lower to start altitude
    rise(goals)
    
    # ajust position
    goto(goals)

    # Have all drones follow their path step by step
    for j in range(len(paths[0])):
        goals = [changeBasis(paths[i][j], changeBasisConstants) for i in range(maxD)]
        goto(goals)

    # Sleep for some time
    for j in range(100):
        rate.sleep()

    # land
    land = [(0,0,0.1) for _ in range(maxD)]
    rise(land)

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

    changeBasisConstants = setConstants(bounds, (2.5,5.5,2.5))

    current_positions = [Position() for _ in starts]

    launchDrones(paths, changeBasisConstants)