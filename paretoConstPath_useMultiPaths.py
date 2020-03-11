from drone3D import Drone
from nDimAstar import astar
import numpy as np
from time import sleep
from monotoneLavalle import monotoneLavalle
from helpers import *

# 0: empty
# 1: obstacle
# 2: path
# 3: drones

def findPaths(starts, goals, nmap):
    def findConfigSpaceAndPath(paths):
        if len(paths) == 2:
            p1 = paths[0]
            p2 = paths[1]
        if len(paths) == 3:
            p3 = paths[2]

        method = "monotone"

        lengths = [len(p) for p in paths[::-1]]
        configSpace = np.zeros(lengths[::-1])
        
        configSpace = config2D(p1, p2)

        start = np.zeros(amount, dtype=int)
        end = np.array(lengths)-np.ones(amount, dtype=int)
        
        possiblePath = astar(start, end, configSpace, lengths)

        if method == "astar":
            return possiblePath
        elif method == "monotone":
            #only implemented in 2D for now
            return monotoneLavalle(start, end, configSpace, lengths, possiblePath)

    amount = len(starts)
    if amount != len(goals):
        print("Incorrect amount of start or goal positions")
        return
    for i, s in enumerate(starts):
        if nmap[s[2]][s[1]][s[0]] == 1:
            print("Start",i+1,"is in an obstacle.")
            return 
    for i, g in enumerate(goals):
        if nmap[g[2]][g[1]][g[0]] == 1:
            print("Goal",i+1,"is in an obstacle.")
            return

    amountPathsExplore = 3

    drones = [Drone(s, g, nmap, amountPaths=amountPathsExplore) for s,g in zip(starts,goals)]
    paths = [d.path for d in drones]

    if amountPathsExplore==1:
        paretoEfficient = findConfigSpaceAndPath(paths)
        pathsToFollow = [[] for _ in range(amount)]
        
        for p in paretoEfficient:
            for i,j in enumerate(p[::-1]):
                pathsToFollow[i].append(paths[i][int(j)])
        
        return pathsToFollow

    bestLen = np.inf
    whichPaths = [0,0]

    for i, p1 in enumerate(paths[0]):
        for j,p2 in enumerate(paths[1]):
            tmp = findConfigSpaceAndPath((p1, p2))
            if len(tmp)<=1:
                continue
            if len(tmp)<bestLen:
                bestLen = len(tmp)
                whichPaths = [i,j]

    i,j = whichPaths
    paretoEfficient = findConfigSpaceAndPath((paths[0][i], paths[1][j]))

    pathsToFollow = [[] for _ in range(amount)]
    
    for p in paretoEfficient:
        p1, p2 = p
        pathsToFollow[0].append(paths[0][i][int(p2)])
        pathsToFollow[1].append(paths[1][j][int(p1)])
        
    return pathsToFollow

