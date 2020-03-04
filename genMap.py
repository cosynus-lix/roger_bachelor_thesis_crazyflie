from helpers import loadMap, setConstants, genWorld
import sys

if __name__ == '__main__':
    if len(sys.argv)<1:
        nmap = loadMap("map.txt")
    else:
        nmap = loadMap(sys.argv[1])
    bounds = (len(nmap[0][0]), len(nmap[0]), len(nmap))
    changeBasisConstants = setConstants(bounds, (3,6,1))
    genWorld(nmap, changeBasisConstants)