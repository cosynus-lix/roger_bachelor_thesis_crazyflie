#!/usr/bin/env python

import sys
import rospy

from crazyflie_driver.msg import GenericLogData
from crazyflie_driver.msg import Position

import math

from paretoConstPath_useMultiPaths import findPaths
from helpers import *
from drone3D import Drone

#########################################################################################

# local position callback [if need]
def local_position_callback(msg):
    global current_positions
    i = int(msg.header.frame_id[2])-1
    current_positions[i].x = msg.values[0]
    current_positions[i].y = msg.values[1]
    current_positions[i].z = msg.values[2]
    current_positions[i].header = msg.header

def launchDrones(starts, goalsF, nmap, changeBasisConstants):
    global current_positions

    def goto(goals, precision=0.1):
        while len([1 for g, c in zip(goals, current_positions) if d(g, c)>precision])>0 : # no root, rather 3cm 
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

    def delay(n=100):
        for j in range(n):
            rate.sleep()


    drones = [Drone(s, g, nmap) for s,g in zip(starts,goalsF)]
    maxD = len(drones)

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
    next_pos = [c for c in current_positions]

    # takeoff and go above map
    alts = [(0,0,0.5) for i in range(maxD)]
    rise(alts)

    # go above start
    goals = [changeBasis(s, changeBasisConstants) for s in starts]
    planar(goals)
    
    # lower to start altitude
    rise(goals)
    
    # ajust position
    #goto(goals)
    
    print("Ready")
    delay()

    # Have all drones follow the flow
    while len([1 for g, c in zip(goalsF, current_positions) if d(g, c)>0.1])>0:
        for drone in drones:
            drone.update(0.1, [(c.x, c.y, c.z) for c in current_positions])
        goto([changeBasis(drone.pos, changeBasisConstants) for drone in drones])


    # Sleep for some time
    delay()

    # land
    land = [(0,0,0.1) for _ in range(maxD)]
    rise(land)

    return

if __name__ == '__main__':
    global current_positions

    nmap = loadMap("maps/maze.map")

    starts = [(4,1,1), (4,10,1), (4,3,1)]
    goals = [(4,10,1), (4,1,1), (3,10,1)]
    bounds = (len(nmap[0][0]), len(nmap[0]), len(nmap))

    changeBasisConstants = setConstants(bounds, (3,6,1))
    current_positions = [Position() for _ in starts]

    launchDrones(starts, goals, nmap, changeBasisConstants)