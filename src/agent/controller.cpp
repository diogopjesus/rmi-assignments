/* controller.cpp
 */

#include "agent/controller.h"

namespace agent
{

float Controller::computeControlSignal(float r, float y)
{
    float u = 0.0; // control signal

    float e = r - y;   // Compute error

    if(type == BANG)
    {
        if(e > 0) u = bangvalue;
        else u = 0.0;
    }
    else if(type == BANG2)
    {
        if(e > 0) u = bangvalue;
        else if(e < 0) u = -bangvalue;
        else u = 0.0;
    }
    else if(type == BANGH)
    {
        if(e > 0.5*deltah) u = bangvalue;
        else if(e < -0.5*deltah) u = -bangvalue;
        else u = u_m1;

        u_m1 = u;
    }
    else if(type == P)
    {
        u = Kp * e;
    }
    else if(type == PID)
    {
        // Auxiliary constants for PID controller
        float K0 = Kp * (1+(h/Ti)) + (Td/h);
        float K1 = -Kp * (1+2*(Td/h));
        float K2 = Kp * (Td/h);

        u = u_m1 + K0*e + K1*e_m1 + K2*e_m2;

        // clamp control signal
        if (u < -max_u) u = -max_u;
        else if (u > max_u) u = max_u;

        // update memory
        e_m2 = e_m1;
        e_m1 = e;
        u_m1 = u;
    }
    else // NONE
    {
        u = r;
    }

    return u;
}

void Controller::reset()
{
    e_m1 = e_m2 = 0.0;  // reset memory for error
    u_m1 = 0.0; // reset memory for control signal
}

}; // namespace agent
