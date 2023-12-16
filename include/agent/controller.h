/* controller.h
 */

#ifndef AGENT_CONTROLLER_H
#define AGENT_CONTROLLER_H

#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <set>
#include <map>

namespace agent
{

// Controller type
enum ControllerType {NONE, BANG, BANG2, BANGH, P, PID};

/**
 * @class Controller
 * @brief Multi-type controller.
 * Parameters essential to a specific type of controller must be set before use. 
*/
class Controller
{
public:
    Controller() : Controller(NONE) {}
    Controller(ControllerType t_type)
        : m_type(m_type)
        {}
    virtual ~Controller() = default;

    /**
     * Set the controller type.
     * 
     * @param t_type Controller type.
    */
    inline void setType(ControllerType t_type) { m_type = t_type; }

    /**
     * Set the sampling interval.
     * 
     * @param t_h Sampling interval.
    */
    inline void setSamplingInterval(double t_h) { m_h = t_h; }

    /**
     * Set the controller parameters.
     * 
     * @param t_Kp Proportional gain.
     * @param t_Ti Integral time.
     * @param t_Td Derivative time.
    */
    inline void setParameters(double t_Kp, double t_Ti, double t_Td) { m_Kp = t_Kp; m_Ti = t_Ti; m_Td = t_Td; }

    /**
     * Set the hysteresis for bang-bang controller.
     * 
     * @param t_deltah Hysteresis.
     * @param t_bangvalue Control signal value.
    */
    inline void setHysteresis(double t_deltah, double t_bangvalue) { m_deltah = t_deltah; m_bangvalue = t_bangvalue;}

    /**
     * Set the saturation value for control signal.
     * 
     * @param t_max_u Saturation value.
    */
    inline void setSaturation(double t_max_u) { m_max_u = t_max_u;}

    /**
     * Compute the control signal.
     * 
     * @param t_r reference value.
     * @param t_y measured value.
     * @return the control signal.
    */
    double computeControlSignal(double t_r, double t_y);

    /**
     * Reset the controller.
    */
    void reset();

private:
    ControllerType m_type;
    double m_h{0.0},                        // sampling interval
           m_Kp{0.0}, m_Ti{0.0}, m_Td{0.0}, // PID controller parameters
           m_deltah{0.0}, m_bangvalue{0.0}, // hysteresis for bang-bang controller
           m_max_u{0.0},                    // saturation value for control signal
           m_e_m1{0.0}, m_e_m2{0.0},        // memory for error
           m_u_m1{0.0};                     // memory for control signal
};

}; // namespace agent

#endif // AGENT_CONTROLLER_H
