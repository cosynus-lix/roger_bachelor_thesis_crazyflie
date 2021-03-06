import numpy as np
from helpers import comp, checkConfigWall
from copy import deepcopy

def monotoneLavalle(start, end, configSpaces, lengths, possiblePath, addCrits=True):
    def addNdimConvex(point):
        point = point[::-1]
        n = len(point)
        for i, p1 in enumerate(point):
            for j, p2 in enumerate(point[i+1:]):
                tmp = n*i - (i*i+i)//2 + j
                configSpaces[tmp][(p1,p2)] = 3
        
    if addCrits:
        dim = len(lengths)
        for point in possiblePath:
            point = [int(s) for s in point[::-1]]
            for i, p1 in enumerate(point):
                for j, p2 in enumerate(point[i+1:]):
                    tmp = dim*i - (i*i+i)//2 + j
                    configSpaces[tmp][(p1,p2)] = 2
        
        for i,c in enumerate(configSpaces):
            configSpaces[i] = addCriticals(c)

    start = [int(s) for s in start]
    optiPath = [start]

    while not comp(start, end):
        oldPoint = deepcopy(start)
        for i in range(len(lengths)):
            if start[i]+1 < lengths[i]:
                tmp = [s if j!=i else s+1 for j,s in enumerate(start)]
                if not checkConfigWall(configSpaces, tmp):
                    start = [t for t in tmp]
                    
        if comp(oldPoint, start):
            #print("Not monotone path")
            if len(optiPath) < 2:
                print("None")
                return []
            optiPath.pop()
            addNdimConvex(start)
            start = deepcopy(optiPath[-1])
            continue
            #return []
        optiPath.append(tuple(start))
    return optiPath


def addCriticals(configSpace):
    for i, row in enumerate(configSpace):
        if i==0: continue
        afterPath = True
        N = len(row)
        for j in range(N)[::-1]:
            if j==0: continue
            value = row[j]
            if value==1:
                if afterPath:
                    if configSpace[i-1, j] == 0:
                        configSpace[i-1,j] = 1
                else:
                    if configSpace[i,j-1] == 0:
                        configSpace[i,j-1] = 1
            if value==2: 
                afterPath = False
    return configSpace