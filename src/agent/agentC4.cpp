/* agentC4.cpp
 */

#include <unistd.h>

#include "agent/agentC4.h"

namespace agent
{

AgentC4::~AgentC4()
{
    delete mController;
    delete mLocalPose;
    delete mLocalMap;
}

int AgentC4::run()
{
    int state=STOP, stoppedState=RUN;
    float r, y, u; // reference, current state and control signal

    while(!GetFinished()) // simulator has received finish or robot removed
    {
        ReadSensors();

        if(GetTime() >= GetFinalTime()) Finish();

        if(state==STOP && GetStartButton()) state=stoppedState;
        if(state!=STOP && GetStopButton())  {
            stoppedState=state;
            state=STOP; // interrupt
        }

        switch(state)
        {
            case RUN:
                r = computeReferenceValue();
                y = computeCurrentState();
                u = mController->computeControlSignal(r, y);
                driveMotorsExt(0.1-u, 0.1+u);
                usleep(500000);
        }
    }

    std::cout << "Exiting " << GetTime() << std::endl;
    return 0;
}

int AgentC4::write() const
{
    return 0;
}

void AgentC4::driveMotorsExt(double t_lPow, double t_rPow)
{
    DriveMotors(t_lPow, t_rPow);
    mLocalPose->update(t_lPow, t_rPow);
}

float AgentC4::getLinePos() const
{
    bool line[7];
    GetLineSensor(line);

    float posOverLine=0;
    int nActiveSensors=0;

    // read sensors
    for (int i = 0; i < N_LINE_ELEMENTS; i++) {
        if(line[i]){
            posOverLine += (float) (i-3);
            nActiveSensors++;
        }
    }

    // Compute the position and scale the measure for the distance between sensors.
    posOverLine = 0.08*posOverLine/nActiveSensors;

    return posOverLine;
}

std::tuple<float,float> AgentC4::getVel() const
{
    float xVel, yVel;
    int currXpos, currYpos;
    static float lastXpos=0, lastYpos=0;
    static int lastTime;
    int currTime;

    currXpos = mLocalPose->getX();
    currYpos = mLocalPose->getY();
    currTime = GetTime();

    if(currTime > lastTime){
        xVel = (currXpos - lastXpos)/(currTime - lastTime);
        yVel = (currYpos - lastYpos)/(currTime - lastTime);
    }
    else{
        xVel = 0;
        yVel = 0;
    }

    // Store for future memory...
    lastXpos = currXpos;
    lastYpos = currYpos;
    lastTime = currTime;
    
    return std::make_tuple(xVel, yVel);
}

float AgentC4::computeReferenceValue()
{
    return 0.0;
}

float AgentC4::computeCurrentState()
{
    static float oldpos = 0.0;
    float pos = getLinePos();
    if(pos!=pos) pos = oldpos; // check for NaN
    else oldpos = pos;
    return pos;
}

}; // namespace agent
