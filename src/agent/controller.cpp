/* controller.cpp
 */

#include "agent/controller.h"

namespace agent
{

double Controller::computeControlSignal(double t_r, double t_y)
{
    double u = 0.0; // control signal

    double e = t_r - t_y;   // Compute error

    if(m_type == BANG)
    {
        if(e > 0) u = m_bangvalue;
        else u = 0.0;
    }
    else if(m_type == BANG2)
    {
        if(e > 0) u = m_bangvalue;
        else if(e < 0) u = -m_bangvalue;
        else u = 0.0;
    }
    else if(m_type == BANGH)
    {
        if(e > 0.5*m_deltah) u = m_bangvalue;
        else if(e < -0.5*m_deltah) u = -m_bangvalue;
        else u = m_u_m1;

        m_u_m1 = u;
    }
    else if(m_type == P)
    {
        u = m_Kp * e;
    }
    else if(m_type == PID)
    {
        // Auxiliary constants for PID controller
        double K0 = m_Kp * (1+(m_h/m_Ti)) + (m_Td/m_h);
        double K1 = -m_Kp * (1+2*(m_Td/m_h));
        double K2 = m_Kp * (m_Td/m_h);

        u = m_u_m1 + K0*e + K1*m_e_m1 + K2*m_e_m2;

        // update memory
        m_e_m2 = m_e_m1;
        m_e_m1 = e;
        m_u_m1 = u;
    }
    else // NONE
    {
        u = t_r;
    }

    // clamp control signal
    if (u < -m_max_u) u = -m_max_u;
    else if (u > m_max_u) u = m_max_u;

    return u;
}

void Controller::reset()
{
    m_e_m1 = m_e_m2 = 0.0;  // reset memory for error
    m_u_m1 = 0.0; // reset memory for control signal
}

}; // namespace agent
