
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

def createBolck(point, constants, name):
    x,y,z = changeBasis(point, constants)
    return """
    <include>
      <uri>model://"""+name+"""</uri>
      <pose>"""+str(x)+' '+str(y)+' '+str(z)+""" 0 0 0</pose>
    </include>
    """

def genWorld(nmap, constants, name="block50", infile="basis.world", outfile="salle_lix.world"):
    bounds = (len(nmap[0][0]), len(nmap[0]), len(nmap))
    with open(infile, 'r') as infile:
        with open(outfile, 'w') as outfile:
            infile = infile.read()
            com = "<!-- obstacles -->"
            loc = infile.find(com) + len(com)
            outfile.write(infile[:loc])
            for i in range(bounds[2]):
                for j in range(1, bounds[1]-1):
                    for k in range(1, bounds[0]-1):
                        if nmap[i][j][k] == 1:
                            outfile.write(createBolck((k, j, i), constants, name))
                outfile.flush()
            outfile.write(infile[loc:])
            outfile.flush()

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