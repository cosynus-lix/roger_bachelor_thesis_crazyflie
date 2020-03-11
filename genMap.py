from helpers import loadMap, setConstants, changeBasis
import sys

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

if __name__ == '__main__':
    if len(sys.argv)<1:
        nmap = loadMap("map.txt")
    else:
        nmap = loadMap(sys.argv[1])
    bounds = (len(nmap[0][0]), len(nmap[0]), len(nmap))
    changeBasisConstants = setConstants(bounds, (3,6,1))
    genWorld(nmap, changeBasisConstants)