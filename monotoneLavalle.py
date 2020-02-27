import numpy as np

def comp(a, b):
    if len(a) != len(b):
        return False
    for a2, b2 in zip(a, b):
        if a2 != b2:
            return False
    return True

def monotoneLavalle(start, end, configSpace, lengths, possiblePath):
    for point in possiblePath:
        point = tuple([int(s) for s in point[::-1]])
        configSpace[point] = 2

    addCriticals(configSpace, lengths)

    start = [int(s) for s in start]
    optiPath = [start]

    while not comp(start, end):
        i, j = start
        if i+1>=lengths[0]:
            j += 1
            if configSpace[(j,i)]==1: 
                print("non monotone path error")
                break
        elif j+1>=lengths[1]:
            i += 1
            if configSpace[(j,i)]==1: 
                print("non monotone path error")
                break
        elif configSpace[(j+1,i+1)]!=1:
            i += 1
            j += 1
        elif configSpace[(j+1,i)]!=1:
            i += 1
        elif configSpace[(j,i+1)]!=1:
            j += 1
        else:
            print("Error, got stuck or at the end")
        
        optiPath.append((i,j))
        start = (i,j)
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