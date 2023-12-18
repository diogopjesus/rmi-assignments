import sys
from agentC1 import AgentC1
from agentC2 import AgentC2
from agentC3 import AgentC3

##################################
# Main
##################################
rob_name = "pClient"
host = "localhost"
pos = 0
challenge = 0
outfile = "solution"

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--challenge" or sys.argv[i] == "-c") and i != len(sys.argv) - 1:
        challenge = sys.argv[i + 1]
    elif (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
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
    if challenge == "1":
        rob=AgentC1(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
        rob.run()
    elif challenge == "2":
        rob=AgentC2(rob_name,pos,[0.0,60.0,-60.0,180.0],host, outfile)
        rob.run()
    elif challenge == "3":
        rob=AgentC3(rob_name,pos,[0.0,60.0,-60.0,180.0],host, outfile)
        rob.run()
