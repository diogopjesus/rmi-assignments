#include "agentC4.h"
#include "agent/utils.h"
#include "robSock/RobSock.h"

#include <iostream>

int AgentC4::run()
{
    int state=STOP,
        stoppedState=INIT;
    double lPow=0.0,
           rPow=0.0;
    int cid, nid, ground;

    cid = init(); // initialize robot, sensors, map and controller
    nid = m_perceivedMap.getNextCell(cid); // get next cell

    while(!GetFinished()) {
        ReadSensors(); //update sensors

        // check if simulation has finished
        if(GetTime() >= GetFinalTime() || state == FINISHED) {
            Finish();
        }

        if(state==STOP && GetStartButton()) {
            state=stoppedState;
        }
        if(state!=STOP && GetStopButton()) {
            stoppedState=state;
            state=STOP;
        }

        // update checkpoints
        ground = GetGroundSensor();
        if(ground >= 0) {
            if(ground+1 >= m_checkpoints.size())
                m_checkpoints.resize(ground+1);
            m_checkpoints[ground] = agent::getNearestCell(m_movModel.getX(), m_movModel.getY());
        }

        switch(state)
        {
            // expand first cell and get next cell
            case INIT:
            {
                bool any_nei = findNeighbors();

                float dir = m_movModel.getDir();
                if(dir > -(M_PI/4) && dir < 0) {
                    m_perceivedMap.expandCell(cid);
                    nid = m_perceivedMap.getNextCell(cid);
                    state = RUN;
                }
                lPow = -0.1;
                rPow = 0.1;
                break;
            }

            case RUN:
            {
                bool any_nei = findNeighbors();                
                std::pair<double,double> powers = move(cid,nid);
                lPow = powers.first;
                rPow = powers.second;

                if(m_perceivedMap.isComplete()) {
                    state = RETURN;
                }

                break;
            }

            case RETURN:
            {
                if(ground == 0) { // TODO: check if this is enough
                    state = FINISHED;
                    break;
                }

                std::pair<double,double> powers = move(cid,nid);
                lPow = powers.first;
                rPow = powers.second;
                break;
            }
        }

        // std::cout << m_perceivedMap.toString() << std::endl;

        driveMotorsExt(lPow,rPow);
    }


    return m_perceivedMap.isComplete() ? 0 : 1;
}

int AgentC4::write()
{
    try {
        m_perceivedMap.writeToFile(m_outfile, m_checkpoints);
    }
    catch(const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}

int AgentC4::init()
{
    ReadSensors(); // initialize sensors

    int cid = agent::computeCellId(0, 0); // compute cell id of the starting cell
    m_perceivedMap.addCell(cid); // add starting cell to the perceived map
    m_checkpoints.push_back(cid); // add starting cell to the checkpoints | TODO: check if this assumption is correct

    // initialize controller
    m_controller.setType(agent::P);
    m_controller.setSamplingInterval(0.050); // 50 ms. TODO: change to a computed value
    m_controller.setParameters(0.5, 0.0, 0.0); // TODO: change to Kp to a computed value
    m_controller.setSaturation(0.5); // TODO: change to a more appropriate value
    m_controller.reset();

    driveMotorsExt(0.0, 0.0);

    return cid;
}

std::pair<double,double> AgentC4::move(int& t_cid, int& t_nid)
{
    double lPow, rPow;

    // distance to nid
    agent::Position np = agent::computeCellCoordinates(t_nid);
    double dist = agent::distance(m_movModel.getX(), m_movModel.getY(), np.x, np.y);

    if(dist < 0.1) { // if robot is close to the next cell, then get the next one
        m_perceivedMap.expandCell(t_cid);
        t_cid = t_nid;
        t_nid = m_perceivedMap.getNextCell(t_cid);
        np = agent::computeCellCoordinates(t_nid);
    }

    // get direction to next cell
    double theta = atan2(np.y-m_movModel.getY(), np.x-m_movModel.getX());
    double dif = fabs(theta-m_movModel.getDir());
    if(dif > M_PI) dif = 2*M_PI - dif;
    else if (dif < -M_PI) dif = 2*M_PI + dif;

    // if difference is too big, then rotate
    if(dif > M_PI/8) {
        // subtract the two angles
        double a = theta - m_movModel.getDir();
        if (a > M_PI) a -= 2*M_PI;
        else if (a < -M_PI) a += 2*M_PI;

        // compute offset
        double offset = (fabs(a) / (M_PI/2)) * 0.15;
        if(offset < 0.02) offset = 0.02;

        // rotate to the correct direction
        int sig = (a > 0) ? -1 : 1;
        lPow = sig * offset;
        rPow = -sig * offset;
        
        return std::make_pair(lPow, rPow);
    }

    // follow the line
    double linePos = agent::getLinePos();
    if(!std::isinf(linePos)){
        double u = m_controller.computeControlSignal(0.0, linePos);
        lPow = 0.1-u;
        rPow = 0.1+u;
    }
    else {
        lPow = 0.1;
        rPow = 0.1;
    }

    return std::make_pair(lPow, rPow);
}

void AgentC4::driveMotorsExt(double t_lPow, double t_rPow)
{
    DriveMotors(t_lPow, t_rPow);
    m_movModel.update(t_lPow, t_rPow);
}

bool AgentC4::findNeighbors()
{
    using namespace agent;

    bool any_nei = false;
    
    // read line sensor
    bool line[7];
    GetLineSensor(line);

    // check if any sensor is active
    bool ret = false;
    for(int i = 0; i < 7; i++) {
        ret |= line[i];
    }
    if(!ret) return false;    

    // robot position
    double x = m_movModel.getX();
    double y = m_movModel.getY();
    double dir = m_movModel.getDir();

    // nearest cell for each sensor
    for(int i = 0; i < 7; i++) {
        if(!line[i]) continue; // skip inactive sensors

        // compute each sensor position
        Position sensorPos = getLineSensorPosition(x, y, dir, i);

        // find nearest cell
        int nearest = getNearestCell(sensorPos.x, sensorPos.y);

        MapWall wall{}, tmpw{};
        bool in_wall = false, // the sensor is inside a valid wall
             found = false;   // a wall is found (but might not be a valid one)

        // check if the sensor is inside one of the possible walls for the nearest cell
        for(int i = 0; i < 8; i++) {
            tmpw.update(nearest, (M_PI/4)*i);
            if(!tmpw.isInside(sensorPos.x, sensorPos.y)) continue;

            // check for intersections in any of the links already defined in the map
            // this code avoid a special case of when a robot is traversing diagonally and
            // one of the sensors is closer to a cell that is not a neighbor
            if(fmod(M_PI/4*i, M_PI/2) != 0) {
                std::pair<int,int> cells = tmpw.getCells();
                Position c1 = computeCellCoordinates(cells.first);
                Position c2 = computeCellCoordinates(cells.second);
                
                // rotate 90 degrees
                if(c1.x > c2.x) {
                    c1.x -= 2;
                    c2.x += 2;
                }
                else {
                    c1.x += 2;
                    c2.x -= 2;
                }

                // check if linkage exists
                int id1 = computeCellId(c1.x, c1.y);
                int id2 = computeCellId(c2.x, c2.y);
                if(m_perceivedMap.areNeighbors(id1,id2)) {
                    in_wall = false;
                    found = true;
                    break;
                }
            }

            // if the sensor is inside more than one wall, then ignore it to avoid errors
            if(in_wall) {
                in_wall = false;
                found = true;
                break;
            }

            in_wall = true;
            wall = tmpw;
        }

        if(in_wall) {
            std::pair<int,int> cells = wall.getCells();
            m_perceivedMap.addCell(cells.first);
            m_perceivedMap.addCell(cells.second);
            m_perceivedMap.linkNeighbors(cells.first, cells.second);
            found = true;
        }

        if(!found && !in_wall) { // check if wall is always found
            throw std::runtime_error("no wall found"); // this should never happen on a simulation without noise
        }

        any_nei |= found;
    }

    return any_nei;
}
