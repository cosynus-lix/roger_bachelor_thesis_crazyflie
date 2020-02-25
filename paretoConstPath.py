from drone3D import Drone
from nDimAstar import astar
import numpy as np
from time import sleep
from monotoneLavalle import monotoneLavalle

# 0: empty
# 1: obstacle
# 2: path
# 3: drones
nmap = [[[0,1,1,0],
         [0,0,0,0],
         [0,1,1,0]]]

start = (0,0,0)
end = (3,2,0)
d1 = Drone(start, end, nmap)

start = (3,0,0)
end = (0,2,0)
d2 = Drone(start, end, nmap)

drones = [d1, d2]
paths = [d.path for d in drones]
lengths = [len(p) for p in paths]
amount = len(drones)

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



start = np.zeros(amount)
end = np.array(lengths)-np.ones(amount)

method = "monotone"

possiblePath = astar(start, end, configSpace, lengths)

if method == "astar":
    paretoEfficient = possiblePath
elif method == "monotone":
    #only implemented in 2D for now
    paretoEfficient = monotoneLavalle(start, end, configSpace, lengths, possiblePath)

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


for line in configSpace[::-1]:
    print(line)
