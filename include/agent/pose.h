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
     * Resets the movement model.
    */
    void reset();

    /**
     * Updates the movement model with the given power values.
     * 
     * @param t_lPow Left motor power.
     * @param t_rPow Right motor power.
    */
    void update(double t_lPow, double t_rPow);

    /**
     * Corrects the movement model direction.
     * 
     * @param t_dir corrected direction.
    */
    void correct(const double& t_dir);

    /**
     * Corrects the movement model position.
     * 
     * @param t_x corrected X position.
     * @param t_y corrected Y position.
    */
    void correct(const double& t_x, const double& t_y);

    /**
     * Corrects the movement model position and direction.
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
    inline const double getDegrees() const { return m_dir*180.0/M_PI; } // in degrees
    inline const double getRoundedX() const { return floor(m_x*10.0+0.5)/10.0; }
    inline const double getRoundedY() const { return floor(m_y*10.0+0.5)/10.0; }
    inline const double getXWithOffset() const { return floor((m_x+844.188)*10.0+0.5)/10.0; } // same value has the GPS sensor without noise
    inline const double getYWithOffset() const { return floor((m_y+404.383)*10.0+0.5)/10.0; } // same value has the GPS sensor without noise
    inline const double getDegreesWithOffset() const { return floor(m_dir*180.0/M_PI + 0.5); } // same value has the compass sensor without noise (in degrees)

    /** Setters **/
    inline void setNoise(double t_noise) { m_noise = t_noise; }

private:
    double m_vel{0.0};                          // linear velocity
    double m_x{0.0}, m_y{0.0}, m_dir{0.0},      // current position and orientation
           m_outr{0.0}, m_outl{0.0};            // effective power applied to motors
    double m_noise{100.0};                      // noise level
};

/**
 * Kalman Filter implementation
*/
class CompassFilter
{
public:
    CompassFilter() = default;
    virtual ~CompassFilter() = default;

    /**
     * Initializes the filter.
     * 
     * @param t_deg initial orientation (in degrees).
     * @param t_var initial variance.
    */
    void init(double t_deg, double t_var);

    /**
     * Updates the filter with the measured orientation and uncertainty.
     * 
     * @param t_deg measured orientation (in degrees).
     * @param t_var measured variance.
    */
    void update(double t_deg, double t_var);

    /**
     * Stores the predicted orientation and compute the .
     * 
     * @param t_deg the predicted orientation (movement model).
     * @param t_deg_var the predicted variance.
     * @param t_rot_var the predicted variance of the rotation (computed in movement model)
    */
    void predict(double t_deg, double t_deg_var, double t_rot_var);

    /**
     * Gets the estimated orientation (in degrees).
     * 
     * @return the estimated orientation.
    */
    inline double degrees() const { return m_deg; } // in degrees
    
    /**
     * Resets the compass sensor.
    */
    void reset();

private:
    double m_deg{0.0},          // estimated orientation (in degrees)
           m_deg_var{0.0},      // estimated variance
           m_p_deg{0.0},        // predicted orientation (in degrees)
           m_p_deg_var{0.0};    // predicted variance
};

}; // namespace agent

#endif // AGENT_LOCALPOSE_H