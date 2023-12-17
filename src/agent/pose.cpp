/* localpose.cpp
 */

#include "agent/pose.h"
#include "agent/robot.h"

#include <iostream>
#include <cmath>

namespace agent
{

void MovementModel::update(double t_lPow, double t_rPow)
{
    // clamp input values
    if (t_lPow < -0.15) t_lPow = -0.15;
    else if (t_lPow > 0.15) t_lPow = 0.15;

    if (t_rPow < -0.15) t_rPow = -0.15;
    else if (t_rPow > 0.15) t_rPow = 0.15;

    m_outl = (0.5*t_lPow + 0.5*m_outl) * (m_noise/100.0);
    m_outr = (0.5*t_rPow + 0.5*m_outr) * (m_noise/100.0);

    m_vel = (m_outr + m_outl) / 2.0;
    double rot = (m_outr - m_outl) / robot::ROBOT_DIAMETER;

    m_x += m_vel * cos(m_dir);
    m_y += m_vel * sin(m_dir);

    m_dir += rot;
    if (m_dir > M_PI) m_dir -= 2.0*M_PI;
    else if (m_dir < -M_PI) m_dir += 2.0*M_PI;
}

void MovementModel::correct(const double& t_dir)
{
    m_dir = t_dir;
}

void MovementModel::correct(const double& t_x, const double& t_y)
{
    m_x = t_x;
    m_y = t_y;
}

void MovementModel::correct(const double& t_x, const double& t_y, const double& t_dir)
{

    m_x = t_x;
    m_y = t_y;
    m_dir = t_dir;
}

std::pair<Position,double> MovementModel::predict(double t_lPow, double t_rPow) const
{
    // clamp input values
    if (t_lPow < -0.15) t_lPow = -0.15;
    else if (t_lPow > 0.15) t_lPow = 0.15;

    if (t_rPow < -0.15) t_rPow = -0.15;
    else if (t_rPow > 0.15) t_rPow = 0.15;

    double outl = (0.5*t_lPow + 0.5*m_outl) * m_noise;
    double outr = (0.5*t_rPow + 0.5*m_outr) * m_noise;

    double vel = (outr + outl) / 2.0;
    double rot = (outr - outl) / robot::ROBOT_DIAMETER;

    Position p;
    p.x = m_x + vel * cos(m_dir);
    p.y = m_y + vel * sin(m_dir);

    double dir = m_dir + rot;
    if (dir > M_PI) dir -= 2.0*M_PI;
    else if (dir < -M_PI) dir += 2.0*M_PI;

    return std::make_pair(p, dir);
}

std::pair<double,double> MovementModel::stop()
{
    return std::make_pair(-m_outl, -m_outr);
}

bool MovementModel::stopped() const
{
    return (m_outl == 0 && m_outr == 0);
}

}; // namespace agent
