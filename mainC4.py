import sys
import math
import numpy as np

from croblink import *


class Controller():
    def __init__(self, type, Ku=5, h=0.050):
        self.h = h # sampling interval
        
        self.Kp = 0.6*Ku # proportional constant
        self.Ti = (1.2*Ku)/h # integral time
        self.Td = (3*Ku*h)/40 # differential time
            
        self.max_u = 0.3 # saturation value for control signal
        
        # memory for error
        self.e_m1 = 0.0
        self.e_m2 = 0.0
        
        self.u_m1 = 0.0 # memory for control signal 
        
    def computeControlSignal(self, r, y):
        # auxiliary constants for the controller
        K0 = self.Kp * (1 + (self.h/self.Ti) + (self.Td/self.h))
        K1 = -self.Kp * (1 + 2 * (self.Td / self.h))
        K2 = self.Kp * (self.Td / self.h)
        
        # compute error
        e = r - y
        
        # compute control signal
        u = self.u_m1 + K0*self.e + K1*self.e_m1 + K2*self.e_m2
        
        # clamp the control signal
        if u > self.max_u:
            u = self.max_u
        elif u < -self.max_u:
            u = -self.max_u
        
        # store error values for next iteration
        self.e_m2 = self.e_m1
        self.e_m1 = e

        # store control signal for next iteration
        self.u_m1 = u
        
        return u


class MovementModel():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.outr = 0.0
        self.outl = 0.0

        self.noise = 1.0 # best case
        
        self.D = 1 # robot diameter
   
    def updateMovementModel(self, inl, inr):
        # clamp input
        inl = max(min(inl, 0.15), -0.15)
        inr = max(min(inr, 0.15), -0.15)
        
        # update effective powers applied to motors
        self.outl = (0.5*inl + 0.5*self.outl) * self.noise
        self.outr = (0.5*inr + 0.5*self.outr) * self.noise
        
        lin = (self.outr + self.outl) / 2.0
        rot = (self.outr - self.outl) / self.D
        
        # compute the robot translation
        self.x = self.x + lin * math.cos(self.theta)
        self.y = self.y + lin * math.sin(self.theta)

        # compute the robot rotation (in radians)
        self.theta += rot
        if (self.theta > math.pi):
            self.theta -= (2*math.pi)
        elif (self.theta < -math.pi):
            self.theta += (2*math.pi)
        
        
class Filter():
    def __init__(self):
        pass


class MyRob(CRobLinkAngs, MovementModel):
    def __init__(self, rob_name, rob_id, angles, host, outfile):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        
        self.outfile = outfile
        
        self.controller = Controller()
        self.mov_model = MovementModel()
    
    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            return 1
        
        state = "stop"
        stopped_state = "run"

        self.readSensors()

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.robName + " exiting")
                return 0

            if self.measures.time >= int(self.simTime):
                self.finish()
        
            if state == "stop" and self.measures.start:
                state = stopped_state

            if state != "stop" and self.measures.stop:
                stopped_state = state
                state = "stop"

            if state == "run":
                self.driveMotorsExt(0.1, 0.1)

    def driveMotorsExt(self, lPow, rPow):
        self.driveMotors(lPow, rPow)
        self.mov_model.updateMovementModel(lPow, rPow)


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