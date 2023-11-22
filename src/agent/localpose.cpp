/* localpose.cpp
 */

#include "agent/localpose.h"

namespace agent
{

void LocalPose::update(float lPow, float rPow)
{
    // clamp input values
    if (lPow < -0.15) lPow = -0.15;
    else if (lPow > 0.15) lPow = 0.15;

    if (rPow < -0.15) rPow = -0.15;
    else if (rPow > 0.15) rPow = 0.15;

    m_outl = (0.5*lPow + 0.5*m_outl) * m_bnoise;
    m_outr = (0.5*rPow + 0.5*m_outr) * m_bnoise;

    float lin = (m_outr + m_outl) / 2.0;
    float rot = (m_outr - m_outl) / m_D;

    m_x += lin * cos(m_theta);
    m_y += lin * sin(m_theta);

    m_theta += rot;
    if (m_theta > M_PI) m_theta -= 2*M_PI;
    else if (m_theta < -M_PI) m_theta += 2*M_PI;
}

}; // namespace agent
