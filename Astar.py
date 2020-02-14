# an A* implementation
from heapq import *
import numpy as np
from copy import deepcopy
from math import atan2, sqrt, cos
from time import sleep

def norm(a, b):
    return (a[0]-b[0])**2 + (a[1]-b[1])**2 #monotone => no need for sqrt

def normV(v):
    return sqrt(v[0]**2 + v[1]**2)

def inBounds(point, bounds):
    xm, xM, ym, yM = bounds
    x, y = point
    return (xm <= x < xM) and (ym <= y < yM)

def astar(array, start, goal, bounds):
    if not(inBounds(start, bounds)):                
        print("Start point out of bounds")
        return []
    if not(inBounds(goal, bounds)):                
        print("End point out of bounds")
        return []

    if start[0]==goal[0] and start[1]==goal[1]:                
        print("Start and goal are the same!")
        return []
    
    x, y = start[0], start[1]
    start = (x,y)
    close_set = set()
    came_from = {}
    gscore = {start:0}
    oheap = []

    heappush(oheap, (norm(start, goal), start))
    
    while oheap:

        current = heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(start)
            data.reverse()
            return data

        close_set.add(current)
        for n in range(9):
            if n==4: continue
            neighbor = (current[0] + (n%3)-1, current[1] + (n//3)-1)           
            neighbor_gscore = gscore[current] + norm(current, neighbor)**2

            if not(inBounds(neighbor, bounds)):                
                continue
                
            if array[neighbor[0]][neighbor[1]] == 1:
                continue
                
            if neighbor in close_set: # already got there
                if neighbor_gscore >= gscore[neighbor]: # and in a better way
                    continue
                
            if  neighbor_gscore < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = neighbor_gscore
                heappush(oheap, (neighbor_gscore + norm(neighbor, goal), neighbor))
                
    return []


class Drone:
    def __init__(self, start, goal, map, bounds, size=1):
        self.pos = np.array(list(start))
        self.speed = np.array([0,0])
        self.goal = goal
        self.path = []
        self.size = size
        self.map = map # carefull map will update with others
        self.bounds = bounds
        self.findPath(map, bounds)
        self.maxSpeed = 2

    def findPath(self, map, bounds):
        self.path = astar(map, self.pos, self.goal, bounds)
        # do only if deepcopy
        for x,y in self.path:
            self.map[x][y] = 2

    def update(self, t):
        a = np.array([0,0])

        # search in 5*size around:
        scan = 2*self.size

        #constants for inpact on acceleration
        c = [5, 1, 3, 1]

        x, y = self.pos

        if (int(round(x)), int(round(y))) == self.goal:
            return 1

        possible = [p for p in self.path if (x-scan < p[0] < x+scan and y-scan < p[1] < y+scan)]
        closest = [norm(self.pos, p) for p in possible]
        near = closest.index(min(closest))
       
        # a += path vector
        if possible[near] != self.goal:
            a = a + (np.array(self.path[self.path.index(possible[near])+1]) - np.array(possible[near])) * c[0] 
        #print(a)

        # a += to path vector
        a = a + (np.array(possible[near]) - self.pos) * c[1] * norm (possible[near], self.pos)
        #print(a)

        # a += obsacles
        proxy = []
        tmpa = np.array([0,0])
        angle = atan2(a[1], a[0])
        norma = normV(a)
        for i in range(max(0, int(y-scan)), min(len(self.map), int(y+scan))):
            for j in range(max(0, int(x-scan)), min(len(self.map[0]), int(x+scan))):
                if self.map[i][j] == 1:
                    f = np.array([i,j]) - self.pos
                    n = normV(f)
                    tmp = atan2(f[1], f[0]) - angle
                    if -0.2 < tmp < 0.2:
                        tmpa = tmpa - f / (n**2)
                    elif -1 < tmp < 1:
                        tmpa = tmpa + (a / n / norma * cos(tmp)) - f / (n**2)

        a = a + tmpa * c[2]
        #print(a)

        # a += drones
        #for drone in otherDrones:
        #    tmp = self.pos-drone.pos
        #    if normV(tmp) < scan:
        #        a = a + tmp / (normV(tmp)**2) * c[3]
        #print(a)


        self.speed = t*a #self.speed + t*a
        self.speed[0] = max(-self.maxSpeed, min(self.maxSpeed, self.speed[0]))
        self.speed[1] = max(-self.maxSpeed, min(self.maxSpeed, self.speed[1]))


        self.pos = self.pos + t*self.speed
        self.pos[0] = max(self.bounds[0], min(self.bounds[1]-1, self.pos[0]))
        self.pos[1] = max(self.bounds[2], min(self.bounds[3]-1, self.pos[1]))
        x, y = self.pos
        x, y = int(round(x)), int(round(y))
        
        if self.map[x][y] == 1:
            self.speed = 0.5*t*a

            self.pos = self.pos - t*self.speed
            self.pos[0] = max(self.bounds[0], min(self.bounds[1]-1, self.pos[0]))
            self.pos[1] = max(self.bounds[2], min(self.bounds[3]-1, self.pos[1]))
            x, y = self.pos
            x, y = int(round(x)), int(round(y))
        
        return 0




# 0: empty
# 1: obstacle
# 2: path
# 3: other drones
nmap = [[0,0,0,0,0,0,0,0,0,0,0,0,0],
        [1,0,1,1,1,1,1,1,1,1,1,1,1],
        [0,0,0,0,0,0,0,0,1,0,0,0,0],
        [1,1,1,1,1,0,1,1,1,1,1,1,0],
        [0,0,0,0,0,0,0,0,1,0,0,0,0],
        [1,0,1,1,1,1,1,1,1,0,1,1,1],
        [0,0,0,0,0,0,0,0,0,0,0,0,0],
        [1,1,1,1,1,1,1,1,1,1,1,1,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0]]

bounds = [0,len(nmap),0,len(nmap[0])]
start = (0,0)
end = (8,0)

#path = astar(nmap, start, end, bounds)
#for x,y in path:
#    nmap[x][y]=2
#for line in nmap:
#    print(line)

# convert path to maximum piecewise linear?

d = Drone(start, end, nmap, bounds)
while True:
    if d.update(0.1):
        break
    x, y = d.pos
    old = nmap[int(round(x))][int(round(y))]
    nmap[int(round(x))][int(round(y))] = 3
    for line in nmap:
        print(line)
    print("\n\n\n")
    print(d.speed)
    sleep(0.03)
    nmap[int(round(x))][int(round(y))] = old