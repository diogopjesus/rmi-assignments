import sys
import math
import time
from croblink import *

##################################
# Robot class
##################################
class AgentC1(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.controllerL = Controller()
        self.controllerR = Controller()
        self.nextL = 0.15
        self.nextR = 0.15
        # Variables to turn around when arriving at a the wrong beacon
        self.last = 0 # last beacon detected
        self.check = False # check if the robot is at the right beacon
        self.reverse = False # start reverse function
        self.turn_around = False # turn around the robot
        self.average = 0 # average of the last 9 values
        self.counter = 0 # counter to count the number of iterations
        self.rotating = False # check if the robot is rotating

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()
        
        state = 'stop'
        stopped_state = 'run'

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.robName + " exiting")
                quit()
            
            if self.measures.time >= int(self.simTime):
                print("Simulation time reached")
                print("Score: " + str(self.measures.score))
                self.finish()

            if state == 'stop' and self.measures.start:
                state = stopped_state
            
            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'
            
            if state == 'run':
                self.wander()

    def wander(self):
        # Check line sensor
        line = self.measures.lineSensor
        line[0:4] = filterL(line[0:4], line[4:7])
        line[3:7] = filterR(line[3:7], line[0:3])
        posL, posR = getLinePos(line)
        # Check if ground sensor should be detected
        if self.measures.ground < 0 and not self.check:
            self.check = True
        if self.check and self.measures.ground >= 0: # arrived at a beacon
            self.check = False
            if self.measures.ground != ((self.last+1)%3):
                self.turn_around = True
            self.last = self.measures.ground
        # turn around if needed
        if self.turn_around:
            # Get the signal of the distance to the line (in average)
            if not self.rotating:
                self.driveMotors(0.0, 0.0)
                posR = 0.24 if math.isnan(posR) else posR
                posL = 0.24 if math.isnan(posL) else posL
                self.average += 1 if posR < posL else -1
                self.counter += 1
                if self.counter >= 9:
                    self.rotating = True
                    self.counter = 0
                return
            self.counter+=1
            if self.average > 0:
                self.driveMotors(-0.1, 0.1)
                self.nextL = -0.15
                self.nextR = 0.15
                if self.counter < 10: return
            else:
                self.driveMotors(0.1, -0.1)
                self.nextL = 0.15
                self.nextR = -0.15
                if self.counter < 10: return
            self.turn_around = False
            self.rotating = False
            self.average = 0
            self.counter = 0

        if not math.isnan(posL) and not math.isnan(posR):
            if (posL >= 0.08) or (posR >= 0.08): # verify if an accentuated turn was detected
                if (posL > posR): # turn to the left
                    self.driveMotors(0.1,0.1) # Does not correct direction to not interfere with the turn (on V turns)
                    self.nextL = -0.15
                    self.nextR = 0.15
                elif (posR > posL): # turn to the right
                    self.driveMotors(0.1,0.1) # Does not correct direction to not interfere with the turn (on V turns)
                    self.nextL = 0.15
                    self.nextR = -0.15
                else: # Same exact value for both, should not happen. TODO: check if this actually happens
                    self.driveMotors(0.1,0.1)
            else: # no accentuated turn detected
                uL = self.controllerL.compute(0.0, posL)
                uR = self.controllerR.compute(0.0, posR)
                self.driveMotors(0.15+uL,0.15+uR)
        elif math.isnan(posL) and not math.isnan(posR): # turn to the right (no left line to the left of the centre of the robot)
            self.driveMotors(0.15,-0.15)
            self.nextL = 0.15
            self.nextR = -0.15
        elif not math.isnan(posL) and math.isnan(posR): # turn to the left (no line to the right of the centre of the robot)
            self.driveMotors(-0.15,0.15)
            self.nextL = -0.15
            self.nextR = 0.15
        if math.isnan(posL) and math.isnan(posR): # no line detected. turn to the side that was previously saved
            self.driveMotors(self.nextL,self.nextR)

        #print(line, self.measures.lineSensor, self.nextL, self.nextR, posL, posR)

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
# LINE FILTERING
##################################
# filter half a line
def filterL(line,right):
    idx = []
    count = 0
    for i in range(len(line)):
        if line[i] == "1":
            count+=1
            idx.append(i)

    if count == 1 and idx[0] != 3:
        if idx[0] == 2 and line[3] == 0 and right == ["0", "1", "1", "0"]:
            line[3] = "1"
        else:
            line[idx[0]] = "0"
    
    elif count == 2 and idx[1] == 3 and idx[0] < 2:
        if idx[0] == 1 and right[1] == "0":
            line[2] = "1"
        else:
            line[0] = "0"
    
    return line

def filterR(line,left):
    idx = []
    count = 0
    for i in range(len(line)):
        if line[i] == "1":
            count+=1
            idx.append(i)

    if count == 1 and idx[0] != 0:
        if idx[0] == 1 and line[0] == 0 and left == ["0", "1", "1", "0"]:
            line[0] = "1"
        else:
            line[idx[0]] = "0"
    
    elif count == 2 and idx[0] == 0 and idx[1] > 1:
        if idx[1] == 2 and left[2] == "0":
            line[1] = "1"
        else:
            line[3] = "0"

    return line

##################################
# Controller
##################################
class Controller():
    def __init__(self):
        self.h = 0.050  # sampling interval
        Ku = 4    # ultimate gain
        self.Kp = 0.6*Ku            # proportional constant
        self.Ti = (1.2*Ku)/0.05     # integral time
        self.Td = (3*Ku*0.05)/40    # differential time
        self.max_u = 1.0 # saturation value for control signal
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
