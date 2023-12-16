/* localPose.h
 */

#ifndef AGENT_LOCALPOSE_H
#define AGENT_LOCALPOSE_H

#include "utils.h"

#include <tuple>
#include <cmath>

namespace agent
{

/**
 * @class MovementModel
 * @brief Representation of the movement model of the robot.
*/
class MovementModel
{
public:
    MovementModel() {};
    virtual ~MovementModel() = default;

    /**
     * Updates the movement model with the given power values.
     * 
     * @param t_lPow Left motor power.
     * @param t_rPow Right motor power.
    */
    void update(double t_lPow, double t_rPow);

    /**
     * Corrects the movement model with the given values.
     * 
     * Sets the current values for the position (x and y) and
     * computes the corrected values for the output power of
     * the motors.
     * 
     * The orientation must be filtered to compute the more accurate
     * values for the output power of the motors.
     * 
     * @param t_x X position.
     * @param t_y Y position.
     * @param t_theta filtered orientation.
    */
    void correct(const double& t_x, const double& t_y, const double& t_theta);

    /**
     * Predicts the next position and orientation of the robot.
     * 
     * @param t_lPow Left motor power.
     * @param t_rPow Right motor power.
     * @return a pair with the predicted position and orientation.
    */
    std::pair<Position,double> predict(double t_lPow, double t_rPow) const;

    /**
     * Get the necessary forces to stop the robot.
     * 
     * @return a pair with the forces necessary to stop the robot.
    */
    std::pair<double,double> stop();

    /**
     * Checks if the robot is stopped.
     * 
     * @return true if the robot is stopped, false otherwise.
    */
    bool stopped() const;

    /** Getters **/
    inline const double getVel() const { return m_vel; }
    inline const double getX() const { return m_x; }
    inline const double getY() const { return m_y; }
    inline const double getDir() const { return m_dir; } // in radians
    inline const double getXWithOffset() const { return floor((m_x+844.188)*10.0+0.5)/10.0; } // same value has the GPS sensor without noise
    inline const double getYWithOffset() const { return floor((m_y+404.383)*10.0+0.5)/10.0; } // same value has the GPS sensor without noise
    inline const double getDegreesWithOffset() const { return floor(m_dir*180.0/M_PI + 0.5); } // same value has the compass sensor without noise (in degrees)

    /** Setters **/
    inline void setNoise(double t_noise) { m_noise = t_noise; }

private:
    double m_vel;                               // linear velocity
    double m_x{0.0}, m_y{0.0}, m_dir{0.0},      // current position and orientation
           m_outr{0.0}, m_outl{0.0};            // effective power applied to motors
    double m_noise{100.0};                      // noise level
};

}; // namespace agent

#endif // AGENT_LOCALPOSE_H