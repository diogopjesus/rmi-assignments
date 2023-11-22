/* main.cpp
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <locale.h>
#include <limits.h>

#include <iostream>
#include <string>

#include "robSock/RobSock.h"

#include "agent/agentC4.h"

int main(int argc, char** argv)
{
    if (CHAR_BIT * sizeof(float) != 32)
    std::cerr << "WARNING: float size different than 32 bits! Agent might behave differently..." << std::endl;

    std::string host = "localhost";
    std::string rob_name = "cppAgent";
    int rob_id = 1;
    std::string outfile = "solution";

    // processing arguments
    while(argc > 2) // every option has a value, thus argc must be 1, 3, 5, ...
    {
        std::string opt = argv[1];
        
        if(opt == "--host" || opt == "-h")
        {
            host = argv[2];
        }
        else if(opt == "--robname" || opt == "-r")
        {
            rob_name = argv[2];
        }
        else if(opt == "--pos" || opt == "-p")
        {
            try { rob_id = std::stoi(argv[2]); }
            catch(...) { argc = 0; } // error message will be printed
        }
        else if(opt == "--outfile" || opt == "-f")
        {
            outfile = argv[2];
        }
        else
        {
            break; // the while
        } 
        argc -= 2;
        argv += 2;
    }

    if (argc != 1)
    {
        std::cerr << "Bad number of parameters\n" <<
            "SYNOPSIS: MainC4 [--host hostname] [--robname robotname] [--pos posnumber] [--outfile outfilename]" << std::endl;
      return 1;
    }

    // create application
    agent::AgentC4 agent(outfile);

    // connect Robot to simulator
    double irSensorAngles[4] = {0.0, 60.0, -60.0, 180.0};
    if(InitRobot2(const_cast<char*>(rob_name.c_str()), rob_id, irSensorAngles, const_cast<char*>(host.c_str())) != 0)
    {
        std::cerr << "Error connecting to robot" << std::endl;
        return 1;
    }
    std::cout << rob_name << " Connected" << std::endl;

    agent.run();

    return agent.write();
}
