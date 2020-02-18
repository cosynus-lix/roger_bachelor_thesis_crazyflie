from heapq import *
import numpy as np

class Drone:
    def __init__(self, start, goal, map, size=1):
        self.pos = np.array(list(start))
        self.speed = np.array([0,0,0])
        self.goal = goal
        self.path = []
        self.size = size
        self.map = map # carefull map will update with others
        self.bounds = (0, len(map[0][0]), 0, len(map[0]), 0, len(map))
        self.findPath()
        self.maxSpeed = 2

    def findPath(self):
        self.path = self.astar(self.map, self.pos, self.goal)
        # do only if deepcopy
        #for x,y,z in self.path:
        #    self.map[z][y][x] = 2

    def norm(self, a, b):
        return (a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2 #monotone => no need for sqrt

    def normV(self, v):
        return v[0]**2 + v[1]**2 + v[2]**2

    def inBounds(self, point):
        xm, xM, ym, yM, zm, zM = self.bounds
        x, y, z = point
        return (xm <= x < xM) and (ym <= y < yM) and (zm <= z < zM)

    def astar(self, array, start, goal):
        if not(self.inBounds(start)):                
            print("Start point out of bounds")
            return []
        if not(self.inBounds(goal)):
            print("End point out of bounds")
            return []

        if start[0]==goal[0] and start[1]==goal[1] and start[2]==goal[2]:                
            print("Start and goal are the same!")
            return []
        
        x, y, z = start[0], start[1], start[2]
        start = (x, y, z)
        close_set = set()
        came_from = {}
        gscore = {start:0}
        oheap = []
        heappush(oheap, (self.norm(start, goal), start))
        
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
            for n in range(27):
                if n==13: continue
                neighbor = (current[0] + (n%3)-1, current[1] + ((n%9)//3)-1, current[2] + (n//9)-1)           
                
                if not(self.inBounds(neighbor)):                
                    continue
                    
                if array[neighbor[2]][neighbor[1]][neighbor[0]] == 1:
                    continue
                    
                neighbor_gscore = gscore[current] + self.norm(current, neighbor)
                
                if neighbor in close_set: # already got there
                    if neighbor_gscore >= gscore[neighbor]: # and in a better way
                        continue
                    
                if  neighbor_gscore < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = neighbor_gscore
                    heappush(oheap, (neighbor_gscore + self.norm(neighbor, goal), neighbor))
                    
        return []

    def update(self, t):
        #get stuf

        self.speed = t*a #self.speed + t*a
        self.speed[0] = max(-self.maxSpeed, min(self.maxSpeed, self.speed[0]))
        self.speed[1] = max(-self.maxSpeed, min(self.maxSpeed, self.speed[1]))
        self.speed[2] = max(-self.maxSpeed, min(self.maxSpeed, self.speed[2]))


        self.pos = self.pos + t*self.speed
        self.pos[0] = max(self.bounds[0], min(self.bounds[1]-1, self.pos[0]))
        self.pos[1] = max(self.bounds[2], min(self.bounds[3]-1, self.pos[1]))
        self.pos[2] = max(self.bounds[4], min(self.bounds[5]-1, self.pos[2]))
        x, y, z = self.pos
        x, y, z = int(round(x)), int(round(y)), int(round(z))
        
        
        return 0