import numpy as np
from itertools import product
from heapq import *
import random as rd

def norm(a, b):
    a, b = np.array(a), np.array(b)
    return normV(a-b)

def normV(v):
    v = np.array(v)
    return v@v

def inBounds(p, bounds):
    for i in range(len(bounds)):
        if 0 > p[i] or p[i] >= bounds[i]:
            return False
    return True

def stencil(dim):
    stencils = list(product([-1,0,1], repeat=dim))
    stencils.remove(((0,) * dim))
    return stencils

def neighbours(point):
    return [tuple([int(sum(x)) for x in zip(point,s)]) for s in stencil(len(point))]

def comp(a, b):
    if len(a) != len(b):
        return False
    for a2, b2 in zip(a, b):
        if a2 != b2:
            return False
    return True

def astar(start, goal, array, bounds, checkDiag=True, amountPaths=1):
    start = tuple(start)
    goal = tuple(goal)
    array = np.array(array)

    if not(inBounds(start, bounds)):                
        print("Start point out of bounds")
        return []
    if not(inBounds(goal, bounds)):
        print("End point out of bounds")
        return []

    if norm(start, goal) == 0:                
        print("Start and goal are the same!")
        return []
    
    close_set = set()
    came_from = {}
    gscore = {start:0}
    oheap = []
    heappush(oheap, (norm(start, goal), start))
    paths = []
    
    while oheap:
        current = heappop(oheap)[1]

        if comp(current, goal):
            data = []
            current = current
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(start)
            data.reverse()
            if amountPaths==1:
                return data
            else:
                paths.append(data)
                if len(paths)==amountPaths:
                    return paths
            #clear heap and add bonus
            while oheap:
                heappop(oheap)
            heappush(oheap, (norm(start, goal), start))
            close_set = set()
            came_from = {}
            gscore = {start:0}
            continue
        close_set.add(current)
        for neighbor in neighbours(current):
            if not(inBounds(neighbor, bounds)):                
                continue

            if array[neighbor[::-1]] == 1:
                continue
            
            tmp = norm(current, neighbor)
            if not checkDiag:
                if tmp>1.01:
                    continue
                
            bonus = 0
            if len(paths)>0: 
                bonus += sum([3 for p in paths if neighbor in p])

            neighbor_gscore = gscore[current] + tmp + bonus
            
            if neighbor in close_set: # already got there
                if neighbor_gscore >= gscore[neighbor]: # and in a better way
                    continue
                
            if  (neighbor_gscore < gscore.get(neighbor, 0)) or (neighbor not in [i[1] for i in oheap]):
                came_from[neighbor] = current
                gscore[neighbor] = neighbor_gscore
                heappush(oheap, (neighbor_gscore + norm(neighbor, goal), neighbor))

    return paths