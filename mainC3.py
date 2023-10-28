import sys
import math
from croblink import *

BACK = 10
HIGH_LEFT = -3
LEFT = -2
SLOW_LEFT = -1
STRAIGHT = 0
SLOW_RIGHT = 1
RIGHT = 2
HIGH_RIGHT = 3

ANGLE_MAP = {
    "E"  : 0,
    "NE" : 45,
    "N"  : 90,
    "NW" : 135,
    "W"  : 180,
    "SW" : -135,
    "S"  : -90,
    "SE" : -45,
}

##################################
# Robot class
##################################
class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host, outfile):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.outfile = outfile
        self.controllerL = Controller()
        self.controllerR = Controller()
        self.mapc = {}
        self.inter = []
        self.fst = False
        self.dev = False
        self.iter = 0
        self.checkpoints = []

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()
        
        state = 'stop'
        stopped_state = 'run'

        self.readSensors()
        self.ctr = (round(self.measures.x), round(self.measures.y))
        self.next = [(self.ctr[0]+2, self.ctr[1])]
        self.nextDir = "E"
        self.mapc[self.ctr] = ({self.next[0]}, False)
        self.checkpoints.append((self.ctr,1))

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.robName + " exiting")
                self.write()
                quit()

            if self.measures.time >= int(self.simTime):
                self.finish()

            if state == 'stop' and self.measures.start:
                state = stopped_state
            
            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'
            
            if state == 'run':
                # try:
                    self.update() # update perception status
                    if self.ctr == self.next[0]: # detect deviations
                        self.detect()    
                        if self.dev: # detection ended
                            posDir = self.convertInter() # convert intersection relative direction to absolute directions
                            self.computeNeighbors(posDir) # update map
                            self.decide() # decide next cell and clean intersections list
                    self.wander()
                # except:
                #     print("Something went wrong!")
                #     self.finish()
    
    def update(self):
        compass = self.measures.compass
        x,y = self.measures.x, self.measures.y

        # Check current direction, the closest center cell
        # and if we are before or after the center of the cell
        dir="UNKNOWN"
        ctr=(int(x),int(y))
        ctr=(ctr[0]+(ctr[0]%2), ctr[1]+(ctr[1]%2))
        befctr=None
        r = 0.438
        a = r / math.sqrt(2)

        if (compass < 22.5) and (compass > -22.5): # East
            dir="E"

            ctr=(int(x+r),int(y))
            ctr=(ctr[0]+(ctr[0]%2), ctr[1]+(ctr[1]%2))

            befctr=(x+r<ctr[0]+0.125)

        elif (abs(compass) > 157.5): # West
            dir="W"

            ctr=(int(x-r),int(y))
            ctr=(ctr[0]+(ctr[0]%2), ctr[1]+(ctr[1]%2))

            befctr=(x-r>ctr[0]+0.25)

        elif (compass > 67.5) and (compass < 112.5): # North
            dir="N"

            ctr=(int(x),int(y+r))
            ctr=(ctr[0]+(ctr[0]%2), ctr[1]+(ctr[1]%2))

            befctr=(y+r<ctr[1]+0.25)
        elif (compass < -67.5) and (compass > -112.5): # South
            dir="S"

            ctr=(int(x),int(y-r))
            ctr=(ctr[0]+(ctr[0]%2), ctr[1]+(ctr[1]%2))
            
            befctr=(y-r>ctr[1]+0.5)

        elif (compass > 22.5) and (compass < 67.5): # Northeast
            dir="NE"

            ctr=(int(x+a),int(y+a))
            ctr=(ctr[0]+(ctr[0]%2), ctr[1]+(ctr[1]%2))

            befctr=(x+a<ctr[0]+0.125) and (y+a<ctr[1]+0.25)

        elif (compass < -22.5) and (compass > -67.5): # Southeast
            dir="SE"

            ctr=(int(x+a),int(y-a))
            ctr=(ctr[0]+(ctr[0]%2), ctr[1]+(ctr[1]%2))

            befctr=(x+a<ctr[0]+0.125) and (y-a>ctr[1]+0.5)

        elif (compass > 112.5) and (compass < 157.5): # Northwest
            dir="NW"

            ctr=(int(x-a),int(y+a))
            ctr=(ctr[0]+(ctr[0]%2), ctr[1]+(ctr[1]%2))

            befctr=((x-a>ctr[0]+0.25) and (y+a<ctr[1]+0.25))
        elif (compass > -157.5) and (compass < -112.5): # Southwest
            dir="SW"

            ctr=(int(x-a),int(y-a))
            ctr=(ctr[0]+(ctr[0]%2), ctr[1]+(ctr[1]%2))

            befctr=((x-a>ctr[0]+0.25) and (y-a>ctr[1]+0.5))
        
        # Store status
        self.dir=dir
        self.ctr=ctr
        self.befctr=befctr

    # Detect if there are any possible intersections
    def detect(self):
        inter = self.inter
        line = self.measures.lineSensor

        # Get distance on each side to the line, rounded
        posL, posR = getLinePos(self.measures.lineSensor)
        posL, posR = round(posL,3), round(posR,3)

        # if the detection is before the centre of the cell
        if self.befctr:
            if ((posL == 0.08)) and (HIGH_LEFT not in inter): # detect accentuated left turn
                inter.append(HIGH_LEFT)

            if ((posR == 0.08)) and (HIGH_RIGHT not in inter): # detect accentuated right turn
                inter.append(HIGH_RIGHT)

        # the detection is after the centre of the cell
        else:
            if (posL == 0.100) and (LEFT not in inter) and (HIGH_LEFT not in inter) and (SLOW_LEFT not in inter): # detect left turn
                inter.append(LEFT)
            if (posR == 0.100) and (RIGHT not in inter) and (HIGH_RIGHT not in inter) and (SLOW_RIGHT not in inter): # detect right turn
                inter.append(RIGHT)

            if (posL >= 0.053) and (posL < 0.16) and (posL != 0.100) and (SLOW_LEFT not in inter) and (HIGH_LEFT not in inter): # detect soft left turn
                inter.append(SLOW_LEFT)
            if (posR >= 0.053) and (posR < 0.16) and (posR != 0.100) and (SLOW_RIGHT not in inter) and (HIGH_RIGHT not in inter): # detect soft right turn
                inter.append(SLOW_RIGHT)
            
            if self.inCenter():
                cLine = int(line[2])+int(line[3])+int(line[4])
                if cLine > 1 and STRAIGHT not in inter: # detect straight line
                    inter.append(STRAIGHT)
                # store checkpoint
                if self.measures.ground > -1:
                    for coord,_ in self.checkpoints:
                        if coord == self.ctr:
                            break
                    else:
                        self.checkpoints.append((self.ctr, self.measures.ground))
                self.dev=True

    def inCenter(self):
        dir = self.dir
        x,y = self.measures.x, self.measures.y
        ctr = self.ctr

        if dir == "E":
            return x >= ctr[0]+0.125
        elif dir == "W":
            return x <= ctr[0]+0.25
        elif dir == "N":
            return y >= ctr[1]+0.25
        elif dir == "S":
            return y <= ctr[1]+0.5
        elif dir == "NE":
            return x >= ctr[0]+0.125 and y >= ctr[1]+0.25
        elif dir == "SE":
            return x >= ctr[0]+0.125 and y <= ctr[1]+0.5
        elif dir == "NW":
            return x <= ctr[0]+0.25 and y >= ctr[1]+0.25
        elif dir == "SW":
            return x <= ctr[0]+0.25 and y <= ctr[1]+0.5

    def convertInter(self):
        ori = ["E", "NE", "N", "NW", "W", "SW", "S", "SE"]
        posInter = [STRAIGHT, SLOW_LEFT, LEFT, HIGH_LEFT, BACK, HIGH_RIGHT, RIGHT, SLOW_RIGHT]
        inter = self.inter
        dir = self.dir
        posDir = []

        thresh=ori.index(dir) # index of robot current orientation
        for i in inter:
            idx = posInter.index(i)
            posDir.append(ori[(idx+thresh)%8]) # append corresponding direction
        
        return posDir

    def computeNeighbors(self, posDir):
        mapc = self.mapc
        ctr = self.ctr
        dir = self.dir

        # first time the cell was traversed
        if ctr not in mapc.keys():
            # compute coordinates of previous cell
            angle = ANGLE_MAP[dir]
            if angle > 0:
                nx = ctr[0] + 2*round(math.cos(math.radians(angle-180)))
                ny = ctr[1] + 2*round(math.sin(math.radians(angle-180)))
            else:
                nx = ctr[0] + 2*round(math.cos(math.radians(angle+180)))
                ny = ctr[1] + 2*round(math.sin(math.radians(angle+180)))

            # add cell to map
            mapc[ctr] = ({(nx,ny)}, False)
        # the cell is already expanded
        elif mapc[ctr][1]:
            return # neighbors already computed

        for pd in posDir:
            # neighbor info
            angle = ANGLE_MAP[pd]
            nx = ctr[0] + 2*round(math.cos(math.radians(angle)))
            ny = ctr[1] + 2*round(math.sin(math.radians(angle)))
            nei = (nx,ny)

            # add nei to ctr neighbor list
            mapc[ctr][0].add(nei)

            # check if neighbor already in map
            if nei not in mapc.keys():
                # add to map
                mapc[nei] = ({ctr}, False)
            # update nei neighbor list
            else:
                mapc[nei][0].add(ctr)
                # check if nei is already expanded
                if len(mapc[nei][0]) >= 8:
                    mapc[nei] = (mapc[nei][0], True)
            
        # signal ctr cell as expanded
        mapc[ctr] = (mapc[ctr][0], True)

    def decide(self):
        mapc = self.mapc
        ctr = self.ctr
        next = self.next

        # check for open neighbors
        for nei in mapc[ctr][0]:
            if nei in mapc.keys():
                if not mapc[nei][1]:
                    next = [nei]
                    break
            else:
                mapc[nei] = ([ctr], False)
                next = nei
                break
        else:
            # A path was already computed
            if len(next) > 1:
                next.pop(0)
            else:
                # compute new path
                for p in mapc.keys():
                    if not mapc[p][1]:
                        n = p
                        break
                else:
                    print("All points are closed")
                    self.finish()
                    return

                next = A_Star(ctr,n,mapc)
                if next:
                    next.pop(0)
                else:
                    print("Error computing path")
                    self.finish()
                    return

        # compute nextDir
        dx = next[0][0] - ctr[0]
        dy = next[0][1] - ctr[1]
        angle = math.degrees(math.atan2(dy, dx))
        nextDir = list(ANGLE_MAP.keys())[list(ANGLE_MAP.values()).index(angle)]

        self.next = next
        self.nextDir = nextDir

        # clean intersections
        self.inter = []
        self.dev = False
        self.iter = 0

    def wander(self):
        ctr = self.ctr
        dir = self.dir
        next = self.next[0]
        nextDir = self.nextDir
        mapc = self.mapc
        line = self.measures.lineSensor

        if ctr != next: # if not arrived at next yet
            posL, posR = getLinePos(line)
            if (dir != nextDir) or posL > 0.0 or posR > 0.0 or math.isnan(posL) or math.isnan(posR): # if the direction is wrong
                a = ANGLE_MAP[nextDir] - self.measures.compass
                a -= 360 if a > 180 else 0
                a += 360 if a < -180 else 0
                rot = 1 if a > 0 else -1
                self.driveMotors(-rot*0.05,rot*0.05) # rotate the robot
                
                # advance if stuck (try to correct it with the controller)
                if a < 5 and a > -5 and (not math.isnan(posL) or not math.isnan(posR)):
                    if not math.isnan(posL) and not math.isnan(posR):
                        uL = self.controllerL.compute(0.0, posL)
                        uR = self.controllerR.compute(0.0, posR)
                        self.driveMotors(0.1+uL,0.1+uR)
                    elif not math.isnan(posL):
                        uL = self.controllerL.compute(0.0, posL)
                        self.driveMotors(0.1+uL, 0.1)
                    elif not math.isnan(posR):
                        uR = self.controllerR.compute(0.0, posR)
                        self.driveMotors(0.1, 0.1+uR)
                    else:
                        self.driveMotors(0, 0)

                # correct detection
                if (dir == nextDir and math.isnan(posL) and math.isnan(posR)) or self.iter > 50:
                    for c in mapc.keys():
                        if next in mapc[c][0]:
                            mapc[c][0].remove(next)
                    mapc.pop(next)
                    self.next = []
                    self.decide()

                self.iter +=1

            else:
                if not math.isnan(posL) and not math.isnan(posR):
                    uL = self.controllerL.compute(0.0, posL)
                    uR = self.controllerR.compute(0.0, posR)
                    self.driveMotors(0.1+uL,0.1+uR)
                elif not math.isnan(posL):
                    uL = self.controllerL.compute(0.0, posL)
                    self.driveMotors(0.1+uL, 0.1)
                elif not math.isnan(posR):
                    uR = self.controllerR.compute(0.0, posR)
                    self.driveMotors(0.1, 0.1+uR)
                else:
                    self.driveMotors(0.1, 0.1)
        
        else:
            tmpLine = ["0", "0", line[2], line[3], line[4], "0", "0"]
            posL, posR = getLinePos(tmpLine)
            if not math.isnan(posL) and not math.isnan(posR):
                uL = self.controllerL.compute(0.0, posL)
                uR = self.controllerR.compute(0.0, posR)
            else:
                uL = 0
                uR = 0

            self.driveMotors(0.05+uL,0.05+uR)

    def write(self):
        outfile = self.outfile
        checkpoints = self.checkpoints
        checkpoints.sort(key=lambda x: x[1])
        print(checkpoints)
        path = [checkpoints[0][0]]
        for i in range(len(checkpoints)):
            closestPath = A_Star(checkpoints[i][0], checkpoints[(i+1)%len(checkpoints)][0], self.mapc)
            if closestPath:
                closestPath.pop(0)
                path += closestPath
        print(path)
        refX, refY = checkpoints[0][0]
        with open(outfile+".path", "w") as f:
            for xp,yp in path:
                pL = (yp-refY)
                pC = (xp-refX)
                f.write(str(pC) + " " + str(pL) + "\n")


##################################
# LINE SENSOR POSITION ESTIMATION
##################################
# compute estimate of robot position over the line
def getLinePos(line):
    # Compute for the left side of the centre of the robot
    posOverLine=0
    nActiveSensors=0
    for i in range(NUM_LINE_ELEMENTS-4):
        if line[i] == "1":
            posOverLine+=abs(i-3)
            nActiveSensors+=1    
    if line[3] == "1":
        posOverLine+=-1
        nActiveSensors+=1
    if nActiveSensors<=0: posOverLineL=math.nan
    else: posOverLineL=0.08*posOverLine/nActiveSensors
    # Compute for the right side of the centre of the robot
    posOverLine=0
    nActiveSensors=0
    for i in range(4,NUM_LINE_ELEMENTS):
        if line[i] == "1":
            posOverLine+=(i-3)
            nActiveSensors+=1
    if line[3] == "1":
        posOverLine+=-1
        nActiveSensors+=1
    if nActiveSensors<=0: posOverLineR=math.nan
    else: posOverLineR=0.08*posOverLine/nActiveSensors

    return posOverLineL, posOverLineR

##################################
# Controller class
##################################
class Controller():
    def __init__(self):
        self.h = 0.050  # sampling interval
        Ku = 5                      # ultimate gain
        self.Kp = 0.6*Ku            # proportional constant
        self.Ti = (1.2*Ku)/0.05     # integral time
        self.Td = (3*Ku*0.05)/40    # differential time
        self.max_u = 0.04 # saturation value for control signal
        # memory for error
        self.e_m1 = 0.0
        self.e_m2 = 0.0
        # memory for control signal
        self.u_m1 = 0.0

    def compute(self, r, y):
        # Simplify variable names
        h = self.h; Kp = self.Kp; Ti = self.Ti; Td = self.Td; max_u = self.max_u
        e_m1 = self.e_m1; e_m2 = self.e_m2; u_m1 = self.u_m1
        # auxiliary constants for PID controller
        K0 = Kp*(1+(h/Ti)+(Td/h))
        K1 = -Kp*(1+2*(Td/h))
        K2 = Kp*(Td/h)
        # Compute error
        e = r - y
        # Compute control signal
        u = u_m1 + K0*e + K1*e_m1 + K2*e_m2
        # Store values for next iteration
        self.e_m2 = e_m1
        self.e_m1 = e
        self.u_m1 = u
        # clip the control signal to avoid saturation
        if u > max_u:
            u = max_u
        elif u < -max_u:
            u = -max_u
        return u


########################################
# PATH FINDING
########################################
# Heuristic function for path finding, based on euclidean distance
def h(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

# path reconstruction for a* algorithm
def reconstruct_path(cameFrom, current):
    total_path = [current]
    while current in cameFrom.keys():
        current = cameFrom[current]
        total_path.insert(0, current)
    return total_path

# weight of the edge from current to neighbor
def d(current, neighbor):
    return 1    

# A* algorithm and heuristic function
def A_Star(start, goal, vertexMap):
    openSet = [start]
    cameFrom = {}
    gScore = {}
    gScore[start] = 0
    fScore = {}
    fScore[start] = h(start, goal)

    while len(openSet) > 0:
        current = openSet[0]
        if current == goal:
            return reconstruct_path(cameFrom, current)
        openSet.remove(current)
        for neighbor in vertexMap[current][0]:
            tentative_gScore = gScore.get(current, math.inf) + d(current, neighbor)
            if tentative_gScore < gScore.get(neighbor, math.inf):
                cameFrom[neighbor] = current
                gScore[neighbor] = tentative_gScore
                fScore[neighbor] = tentative_gScore + h(neighbor, goal)
                if neighbor not in openSet:
                    openSet.append(neighbor)
                    openSet.sort(key=lambda x: fScore.get(x, math.inf))
    
    return None

##################################
# Main
##################################
rob_name = "pClient"
host = "localhost"
pos = 0
outfile = "solution"

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--outfile" or sys.argv[i] == "-f") and i != len(sys.argv) - 1:
        outfile = sys.argv[i + 1]
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host,outfile)    
    rob.run()
