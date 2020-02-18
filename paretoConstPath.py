from drone3D import Drone
from nDimAstar import astar
import numpy as np
from time import sleep

# 0: empty
# 1: obstacle
# 2: path
# 3: drones
nmap = [[[0,1,0],
         [0,0,0],
         [0,1,0]]]

start = (0,0,0)
end = (2,2,0)
d1 = Drone(start, end, nmap)

start = (2,0,0)
end = (0,2,0)
d2 = Drone(start, end, nmap)

drones = [d1, d2]
paths = [d.path for d in drones]
lengths = [len(p) for p in paths]
amount = len(drones)

configSpace = np.zeros(lengths)

# review for n drones
for i, point in enumerate(paths[0]):
    for j, point2 in enumerate(paths[1]):
        if point == point2:
            configSpace[(i,j)] = 1

start = np.zeros(amount)
end = np.array(lengths)-np.ones(amount)
paretoEfficient = astar(start, end, configSpace, lengths)

# DEMO
for i in range(len(paretoEfficient)):
    p1, p2 = paretoEfficient[i]
    
    p1 = d1.path[int(p1)]
    p2 = d2.path[int(p2)]

    old1 = nmap[0][int(p1[1])][int(p1[0])]
    nmap[0][int(p1[1])][int(p1[0])] = 3

    old2 = nmap[0][int(p2[1])][int(p2[0])]
    nmap[0][int(p2[1])][int(p2[0])] = 4
    
    for line in nmap[0]:
        print(line)
    print("\n\n")
    sleep(0.5)
    nmap[0][int(p1[1])][int(p1[0])] = old1 
    nmap[0][int(p2[1])][int(p2[0])] = old2
