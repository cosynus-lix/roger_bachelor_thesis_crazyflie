
def setConstants(bounds, worldSize):
    out = []
    for b, w in zip(bounds, worldSize):
        if b==1:
            a = 0
            c = 1
        else:
            a = w/(b-1)
            c = -w/2
        out.append((a,c))
    return out

def changeBasis(point, constants):
    out = []
    for p, c in zip(point, constants):
        out.append(p*c[0]+c[1])
    return out

def d(point, current):
    current = (current.x, current.y, current.z)
    return sum([(p-c)**2 for p,c in zip(point, current)])