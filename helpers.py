import numpy as np

def setConstants(bounds, worldSize):
    out = []
    for b, w in zip(bounds, worldSize):
        if b==1:
            a = 0
            c = 1
        else:
            a = w/(b-1)
            c = -w/2
        out.append([a,c])
    out[2][1] = 1
    return out

def changeBasis(point, constants):
    out = []
    for p, c in zip(point, constants):
        out.append(p*c[0]+c[1])
    return out

def d(point, current):
    current = (current.x, current.y, current.z)
    return sum([(p-c)**2 for p,c in zip(point, current)])

def comp(p1, p2):
    if len(p1) != len(p2):
        return False
    for i, j in zip(p1, p2):
        if i!=j:
            return False
    return True

def config2D(p1, p2):
    lengths = [len(p2), len(p1)]
    configSpace = np.zeros(lengths[::-1])
    
    # review for n drones
    # create config space for 2 drones
    toCheck = []
    for i, point in enumerate(p1):
        for j, point2 in enumerate(p2):
            if point == point2:
                configSpace[(i,j)] = 1
                toCheck.append((i,j))

    # convexify it
    while len(toCheck)!=0:
        p = toCheck.pop()
        i, j = p
        if i+1<lengths[0] and j!=0:
            if configSpace[(i+1,j-1)] == 1:
                if configSpace[(i,j-1)] != 1:
                    configSpace[(i,j-1)] = 1
                    toCheck.append((i,j-1))
        elif i == lengths[0]-1 and j!=0:
            configSpace[(i,j-1)] = 1
            toCheck.append((i,j-1))
        elif j == lengths[1]-1 and i!=0:
            configSpace[(i-1,j)] = 1
            toCheck.append((i-1,j))
    return configSpace

def loadMap(infile):
    out = [[]]
    with open(infile, 'r') as infile:
        for l in infile.readlines():
            l = l.strip()
            if l == "":
                out.append([])
                continue
            tmp = [int(i) for i in l.split(',')]
            out[-1].append(tmp)
    return out