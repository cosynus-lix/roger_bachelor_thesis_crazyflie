

def setConstants(bounds, worldSize):
    xyxConst = [i==1 for i in bounds]
    out = []
    for b, w in zip(bounds, worldSize):
        if b==1:
            a = 0
            c = 1
        else:
            a = w/b
            c = -w/2
        out.append((a,b))
    return out

def changeBasis(point, constants):
    out = []
    for p, c in zip(point, constants):
        out.append(p*c[0]+c[1])
    return out