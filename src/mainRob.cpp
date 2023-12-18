/* main.cpp
 */

#include "robSock/RobSock.h"
#include "challenges/agentC4.h"

#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    std::string host = "localhost";
    std::string rob_name = "cppAgent";
    int rob_id = 1;
    std::string outfile = "solution";

    // processing arguments
    while(argc > 2) // every option has a value, thus argc must be 1, 3, 5, ...
    {
        std::string opt = argv[1];
        
        if(opt == "--challenge" || opt == "-c")
        {
            try { rob_id = std::stoi(argv[2]); }
            catch(...) { argc = 0; } // error message will be printed
        }
        else if(opt == "--host" || opt == "-h")
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
            break;
        } 
        argc -= 2;
        argv += 2;
    }

    if (argc != 1)
    {
        std::cerr << "Bad number of parameters\n" <<
            "SYNOPSIS: mainRob [--challenge chanumber] [--host hostname] [--robname robotname] [--pos posnumber] [--outfile outfilename]" << std::endl;
      return 1;
    }

    // create application
    AgentC4 agent{outfile};

    // connect Robot to simulator
    double irSensorAngles[4] = {0.0, 60.0, -60.0, 180.0};
    if(InitRobot2(const_cast<char*>(rob_name.c_str()), rob_id, irSensorAngles, const_cast<char*>(host.c_str())) != 0)
    {
        std::cerr << "Error connecting to robot" << std::endl;
        return 1;
    }
    std::cout << rob_name << " Connected" << std::endl;

    try {
        agent.run();
    }
    catch(...)
    {
        std::cerr << "Error running agent" << std::endl;
    }

    int ret;
    try {
        ret = agent.write();
    }
    catch(...)
    {
        std::cerr << "Error writing files" << std::endl;
        ret = 1;
    }

    return ret;
}
