import numpy as np
from helpers import comp

def monotoneLavalle(start, end, configSpace, lengths, possiblePath):
    for point in possiblePath:
        point = tuple([int(s) for s in point[::-1]])
        configSpace[point] = 2
    addCriticals(configSpace, lengths)

    start = [int(s) for s in start]
    optiPath = [start]
    
    while not comp(start, end):
        for i in range(len(lengths)):
            if start[i]+1 < lengths[i]:
                tmp = [s if j!=i else s+1 for j,s in enumerate(start)]
                if configSpace[tuple(tmp[::-1])] != 1:
                    start = [t for t in tmp]
        optiPath.append(tuple(start))
    return optiPath


def addCriticals(configSpace, lengths):
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