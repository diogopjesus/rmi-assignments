/* main.cpp
 */

#include "robSock/RobSock.h"
#include "challenges/agentC4.h"

#include <iostream>
#include <string>

int runAgentC4(const std::string& outfile)
{
    int ret;
    AgentC4 agent(outfile);

    try {
        agent.run();
    }
    catch(std::exception& e)
    {
        std::cerr << "ERROR: during agent.run(): " << e.what() << std::endl;
    }

    try {
        ret = agent.write();
    }
    catch(std::exception& e)
    {
        std::cerr << "ERROR: during agent.write(): " << e.what() << std::endl;
        ret = 1;
    }

    return ret;
}

int main(int argc, char** argv)
{
    std::string host = "localhost";
    std::string rob_name = "cppAgent";
    int rob_id = 1;
    int challenge = 0;
    std::string outfile = "solution";

    // processing arguments
    while(argc > 2) // every option has a value, thus argc must be 1, 3, 5, ...
    {
        std::string opt = argv[1];
        
        if(opt == "--challenge" || opt == "-c")
        {
            try { challenge = std::stoi(argv[2]); }
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

    // connect Robot to simulator
    double irSensorAngles[4] = {0.0, 60.0, -60.0, 180.0};
    if(InitRobot2(const_cast<char*>(rob_name.c_str()), rob_id, irSensorAngles, const_cast<char*>(host.c_str())) != 0)
    {
        std::cerr << "Error connecting to robot" << std::endl;
        return 1;
    }
    std::cout << rob_name << " Connected" << std::endl;

    // select agent
    int ret;
    switch (challenge)
    {
        case 4:
            ret = runAgentC4(outfile);
            break;
        default:
            std::cerr << "Challenge " << challenge << " not implemented" << std::endl;
            ret = 1;
    }

    return ret;
}
