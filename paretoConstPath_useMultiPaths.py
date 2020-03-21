from drone3D import Drone
from nDimAstar import astar
import numpy as np
from time import sleep
from monotoneLavalle import monotoneLavalle
from helpers import *
from itertools import product
from copy import deepcopy

# 0: empty
# 1: obstacle
# 2: path
# 3: drones

def findPaths(starts, goals, nmap):
    def findConfigSpaceAndPath(paths):
        #if len(paths) == 2:
        #    p1 = paths[0]
        #    p2 = paths[1]
        #if len(paths) == 3:
        #    p3 = paths[2]

        method = "monotone"

        lengths = [len(p) for p in paths[::-1]]
        configSpaces = []

        for i,p1 in enumerate(paths):
            for j,p2 in enumerate(paths[i+1:]):
                j += 1 + i
                #print(i,j+i+1)

                configSpace = np.zeros((lengths[j], lengths[i]))
        
                configSpace = config2D(p1, p2)

                configSpaces.append(np.array(configSpace))

        start = np.zeros(len(lengths), dtype=int)
        end = np.array(lengths)-np.ones(len(lengths), dtype=int)
        
        possiblePath = astar(start, end, configSpaces, lengths, projections=True)

        if method == "astar":
            return possiblePath
        elif method == "monotone":
            #only implemented in 2D for now
            return monotoneLavalle(start, end, configSpaces, lengths, possiblePath)

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

    amountPathsExplore = 2

    drones = [Drone(s, g, nmap, amountPaths=amountPathsExplore) for s,g in zip(starts,goals)]
    paths = [d.path for d in drones]

#    if amountPathsExplore == 1:
#        paretoEfficient = findConfigSpaceAndPath(paths)
#        pathsToFollow = [[] for _ in range(amount)]
#        for p in paretoEfficient:
#            for i,j in enumerate(p[::-1]):
#                pathsToFollow[i].append(paths[i][int(j)])
#        
#        return pathsToFollow
        
    bestLen = np.inf
    bestPath = []
    bestPaths = []

    for possiblePaths in list(product(*paths)):
        a = int(np.sqrt(sum([len(p)**2 for p in possiblePaths])))
        if a >= bestLen:
            continue
        tmpPath = findConfigSpaceAndPath(possiblePaths)
        tmpLen = len(tmpPath)
        if tmpLen <= 1:
            continue
        if tmpLen < bestLen:
            bestLen = tmpLen
            bestPath = deepcopy(tmpPath)
            bestPaths = deepcopy(possiblePaths)
            
    pathsToFollow = [[] for _ in range(amount)]
    for p in bestPath:
        for i,j in enumerate(p[::-1]):
            pathsToFollow[i].append(bestPaths[i][int(j)])
    
    return pathsToFollow
