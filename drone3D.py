from heapq import *
import numpy as np
from nDimAstar import astar

class Drone:
    def __init__(self, start, goal, nmap, size=1):
        self.pos = np.array(list(start))
        self.speed = np.array([0,0,0])
        self.goal = goal
        self.path = []
        self.size = size
        self.map = nmap # carefull map will update with others
        self.bounds = (len(nmap[0][0]), len(nmap[0]), len(nmap))
        self.findPath()
        self.maxSpeed = 2
        
    def findPath(self):
        self.path = astar(self.pos, self.goal, self.map, self.bounds)
        # do only if deepcopy
        #for x,y,z in self.path:
        #    self.map[z][y][x] = 2

    def inBounds(self, point):
        xm, xM, ym, yM, zm, zM = self.bounds
        x, y, z = point
        return (xm <= x < xM) and (ym <= y < yM) and (zm <= z < zM)

    def update(self, t):
        #get stuf

        self.speed = t*a #self.speed + t*a
        self.speed[0] = max(-self.maxSpeed, min(self.maxSpeed, self.speed[0]))
        self.speed[1] = max(-self.maxSpeed, min(self.maxSpeed, self.speed[1]))
        self.speed[2] = max(-self.maxSpeed, min(self.maxSpeed, self.speed[2]))


        self.pos = self.pos + t*self.speed
        self.pos[0] = max(0, min(self.bounds[1]-1, self.pos[0]))
        self.pos[1] = max(0, min(self.bounds[3]-1, self.pos[1]))
        self.pos[2] = max(0, min(self.bounds[5]-1, self.pos[2]))
        x, y, z = self.pos
        x, y, z = int(round(x)), int(round(y)), int(round(z))
        
        
        return 0