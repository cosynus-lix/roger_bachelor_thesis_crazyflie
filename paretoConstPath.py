from drone3D import Drone
from nDimAstar import astar
import numpy as np
from time import sleep
from monotoneLavalle import monotoneLavalle

# 0: empty
# 1: obstacle
# 2: path
# 3: drones

def findPaths(starts, goals, nmap):
    amount = len(starts)
    if amount != len(goals):
        print("Incorrect amount of start or goal positions")
        return
    
    drones = [Drone(s, g, nmap) for s,g in zip(starts,goals)]
    paths = [d.path for d in drones]
    lengths = [len(p) for p in paths]


    configSpace = np.zeros(lengths)
    
    # review for n drones
    # create config space for 2 drones
    toCheck = []
    for i, point in enumerate(paths[0]):
        for j, point2 in enumerate(paths[1]):
            if point == point2:
                configSpace[(i,j)] = 1
                toCheck.append((i,j))

    # convexify it
    while len(toCheck)!=0:
        p = toCheck.pop()
        i, j = p
        if i!=lengths[0] and j!=0:
            if configSpace[(i+1,j-1)] == 1:
                if configSpace[(i,j-1)] != 1:
                    configSpace[(i,j-1)] = 1
                    toCheck.append((i,j-1))

    start = np.zeros(amount, dtype=int)
    end = np.array(lengths)-np.ones(amount, dtype=int)

    method = "monotone"

    possiblePath = astar(start, end, configSpace, lengths)

    if method == "astar":
        paretoEfficient = possiblePath
    elif method == "monotone":
        #only implemented in 2D for now
        paretoEfficient = monotoneLavalle(start, end, configSpace, lengths, possiblePath)


    pathsToFollow = [[] for _ in range(amount)]

    for i in range(len(paretoEfficient)):
        p1, p2 = paretoEfficient[i]
        
        pathsToFollow[0].append(drones[0].path[int(p1)])
        pathsToFollow[1].append(drones[1].path[int(p2)])
    
    return pathsToFollow