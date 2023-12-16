/* agent.h
 * 
 * Abstract Robot Agent
 */

#ifndef AGENT_H
#define AGENT_H

namespace agent
{

class Agent
{
public:
    Agent() {}
    virtual ~Agent() {}

    virtual int run() = 0;
    virtual int write() = 0;

    static constexpr int INIT = 1;
    static constexpr int RUN = 2;
    static constexpr int RETURN = 3;
    static constexpr int STOP = 4;
    static constexpr int FINISHED = 5;
};

}; // namespace agent

#endif // AGENT_H