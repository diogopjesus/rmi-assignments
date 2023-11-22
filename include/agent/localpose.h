/* localPose.h
 */

#ifndef AGENT_LOCALPOSE_H
#define AGENT_LOCALPOSE_H

#include <cmath>

namespace agent
{

class LocalPose
{
public:
    LocalPose() : LocalPose(0.0, 0.0, 0.0) {}
    LocalPose(float t_x, float t_y, float t_theta)
        : m_x(t_x), m_y(t_y), m_theta(t_theta),
          m_x0(t_x), m_y0(t_y), m_theta0(t_theta)
        {}
    virtual ~LocalPose() = default;

    inline const float& getX() const { return m_x; }
    inline const float& getY() const { return m_y; }
    inline const float& getTheta() const { return m_theta; }
    inline const float getThetaInDegrees() const { return floor(m_theta*180/M_PI + 0.5); }

    void update(float t_lPow, float t_rPow);

private:
    float m_x, m_y, m_theta,        // current position and orientation
          m_outr{0.0}, m_outl{0.0}; // effective power applied to motors

    const float m_x0, m_y0, m_theta0,           // start position and orientation
                m_bnoise{1.0}, m_wnoise{1.0},   // noise level (1 for best case, 0 for worst case)
                m_D{1.0};                       // Robot diameter 
};

}; // namespace agent

#endif // AGENT_LOCALPOSE_H