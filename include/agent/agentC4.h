/* agentC4.h
 * 
 * Robot Agent
 */

#ifndef AGENT_C4_H
#define AGENT_C4_H

#include <string>
#include <iostream>
#include <vector>
#include <tuple>

#include "robSock/RobSock.h"

#include "controller.h"
#include "localmap.h"
#include "localpose.h"

namespace agent
{

#define RUN        1
#define STOP       2
#define FINISHED   3

// Agent class declaration
class AgentC4
{
public:
    AgentC4() {}
    explicit AgentC4(const std::string& outfile)
        : mOutfile(outfile)
        {
            mController = new Controller();
            mLocalPose = new LocalPose();
            mLocalMap = new LocalMap();
        }
    virtual ~AgentC4();

    int run();
    int write() const;

private:
    void driveMotorsExt(double lPow, double rPow);
    float getLinePos() const;
    std::tuple<float,float> getVel() const;
    float computeReferenceValue();
    float computeCurrentState();

    const std::string mOutfile = "solution";
    Controller* mController = nullptr;
    LocalPose* mLocalPose = nullptr;
    LocalMap* mLocalMap = nullptr;
};

}; // namespace agent

#endif // AGENT_C4_H
