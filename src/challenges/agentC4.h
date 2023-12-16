#ifndef AGENT_C4_H
#define AGENT_C4_H

#include "agent/agent.h"
#include "agent/map.h"
#include "agent/pose.h"
#include "agent/controller.h"

class AgentC4 : public agent::Agent
{
public:
    AgentC4() {}
    explicit AgentC4(const std::string& t_outfile) : m_outfile(t_outfile) {}
    virtual ~AgentC4() = default;

    int run() override;
    int write() override;

private:
    /**
     * @brief Initialize the robot, sensors, map and controller
     * 
     * @return id of the current cell
    */
    int init();

    /**
     * @brief Compute the powers to move the robot
     * 
     * @param t_cid id of the current cell
     * @param t_nid id of the next cell
     * @return powers for the left and right motors
    */
    std::pair<double,double> move(int& t_cid, int& t_nid);
    
    /**
     * @brief Find neighbors of the current cell
     * 
     * Stores the neighbors in the vector t_neighbors
     * 
     * @param t_id id of the current cell
     * @param t_neighbors vector of neighbors
     * @return true if neighbors are found
    */
    bool findNeighbors();
    
    /**
     * @brief Drive the motors, updating the movement model
     * 
     * @param t_lPow left motor power
     * @param t_rPow right motor power
    */
    void driveMotorsExt(double t_lPow, double t_rPow);

    const std::string m_outfile{"solution"};
    agent::MovementModel m_movModel{};
    agent::PerceivedMap m_perceivedMap{};
    agent::Controller m_controller{};
    std::vector<int> m_checkpoints;
};

#endif // AGENT_C4_H