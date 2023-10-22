import sys
import math
from croblink import *

##################################
# Robot class
##################################
class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
    
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
            
            if state == 'stop' and self.measures.start:
                state = stopped_state
            
            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'
            
            if state == 'run':
                self.wander()
    
    def wander(self):
        self.driveMotors(0.1,0.1)


##################################
# Main
##################################
rob_name = "pClient"
host = "localhost"
pos = 0

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)    
    rob.run()
