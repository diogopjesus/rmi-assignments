/* localpose.cpp
 */

#include "agent/pose.h"
#include "agent/robot.h"

#include <iostream>
#include <cmath>

namespace agent
{

void MovementModel::reset()
{
    m_x = 0.0;
    m_y = 0.0;
    m_dir = 0.0;
    m_vel = 0.0;
    m_outl = 0.0;
    m_outr = 0.0;
}

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


void CompassFilter::init(double t_deg, double t_var)
{
    m_p_deg = t_deg;
    m_p_deg_var = t_var;
}

void CompassFilter::update(double t_deg, double t_var)
{    
    // Calculate Kalman gain
    double Kn = m_p_deg_var / (m_p_deg_var + t_var);
    if(Kn != Kn) Kn = 0.0; // if the variance is 0

    // Estimate the current state (orientation)
    // using the state update equation
    double angdiff = t_deg - m_p_deg;
    if(angdiff > 180.0) angdiff -= 360.0;
    else if(angdiff < -180.0) angdiff += 360.0;
    m_deg = m_p_deg + Kn * (angdiff);
    if(m_deg > 180.0) m_deg -= 360.0;
    else if(m_deg < -180.0) m_deg += 360.0;

    // Update the current estimate uncertainty
    m_deg_var = (1.0 - Kn) * m_p_deg_var;
}

void CompassFilter::predict(double t_deg, double t_deg_var, double t_rot_var)
{
    // Predict the next state (orientation)
    // t_deg is the predicted orientation computed by the movement model.
    m_p_deg = t_deg;

    // Predict the current estimate uncertainty
    // t_deg_var is the predicted variance
    // t_rot_var is the predicted variance of the rotation (computed in movement model)
    // delta_t is 1 because the state is predicted at each time step
    m_p_deg_var = t_deg_var + 1.0 * t_rot_var;
}

void CompassFilter::reset()
{
    m_deg = 0.0;
    m_deg_var = 0.0;
    m_p_deg = 0.0;
    m_p_deg_var = 0.0;
}

}; // namespace agent
