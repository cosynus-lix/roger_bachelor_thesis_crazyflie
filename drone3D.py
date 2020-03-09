from heapq import *
import numpy as np
from nDimAstar import astar, norm, normV
from math import atan2, sqrt, cos

class Drone:
    def __init__(self, start, goal, nmap, size=1, amountPaths=1):
        self.pos = np.array(list(start))
        self.speed = np.array([0,0,0])
        self.goal = goal
        self.path = []
        self.size = size
        self.amountPaths = amountPaths
        self.map = nmap # carefull map will update with others
        self.bounds = (len(nmap[0][0]), len(nmap[0]), len(nmap))
        self.findPath()
        self.maxSpeed = 2
        
    def findPath(self):
        self.path = astar(self.pos, self.goal, self.map, self.bounds, False, self.amountPaths)
        # do only if deepcopy
        #for x,y,z in self.path:
        #    self.map[z][y][x] = 2

    def update(self, t, otherDronesPos):
        a = np.zeros(3)

        # search in n*size around:
        scan = 2*self.size

        #constants for inpact on acceleration
        c = [10, 5, 5, 3]

        x, y, z = self.pos

        if (int(round(x)), int(round(y)), int(round(z))) == self.goal:
            return self.goal

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
        tmpa = np.zeros(3)
        angle = atan2(a[1], a[0])
        norma = normV(a)
        for i in range(max(0, int(z-scan)), min(len(self.map), int(z+scan))):
            for j in range(max(0, int(y-scan)), min(len(self.map[0]), int(y+scan))):
                for k in range(max(0, int(x-scan)), min(len(self.map[0][0]), int(x+scan))):

                    if self.map[i][j][k] == 1:
                        f = np.array([k,j,i]) - self.pos
                        n = normV(f)
                        tmp = atan2(f[1], f[0]) - angle
                        if -0.2 < tmp < 0.2:
                            tmpa = tmpa - f / (n**2)
                        elif -1 < tmp < 1:
                            tmpa = tmpa + (a / n / norma * cos(tmp)) - f / (n**2)

        a = a + tmpa * c[2]
        #print(a)

        # a += drones
        tmpa = np.zeros(3)
        angle = atan2(a[1], a[0])
        norma = normV(a)
        for drone in otherDronesPos:
            f = np.array(drone) - self.pos
            if normV(f) < scan:
                n = normV(f)
                tmp = atan2(f[1], f[0]) - angle
                if -0.2 < tmp < 0.2:
                    tmpa = tmpa + (a / n / norma * cos(tmp)) - f / (n**2)
                elif -1 < tmp < 1:
                    tmpa = tmpa - f / (n**2)
                    
        a = a + tmpa * c[3]

            #tmp = self.pos-np.array(drone)
            #a = a + tmp / (normV(tmp)**2) * c[3]
        #print(a)

        self.speed = t*a #self.speed + t*a
        self.speed[0] = max(-self.maxSpeed, min(self.maxSpeed, self.speed[0]))
        self.speed[1] = max(-self.maxSpeed, min(self.maxSpeed, self.speed[1]))
        self.speed[2] = max(-self.maxSpeed, min(self.maxSpeed, self.speed[2]))

        self.pos = self.pos + t*self.speed
        self.pos[0] = max(0, min(self.bounds[0]-1, self.pos[0]))
        self.pos[1] = max(0, min(self.bounds[1]-1, self.pos[1]))
        self.pos[2] = max(0, min(self.bounds[2]-1, self.pos[2]))
        x, y, z = self.pos
        x, y, z = int(round(x)), int(round(y)), int(round(z))

        if self.map[z][y][x] == 1:
            self.speed = 0.5*t*a

            self.pos = self.pos - t*self.speed
            self.pos[0] = max(0, min(self.bounds[0]-1, self.pos[0]))
            self.pos[1] = max(0, min(self.bounds[1]-1, self.pos[1]))
            self.pos[2] = max(0, min(self.bounds[2]-1, self.pos[2]))
            x, y, z = self.pos
            x, y, z = int(round(x)), int(round(y)), int(round(z))
            
        return self.pos