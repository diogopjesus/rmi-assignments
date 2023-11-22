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
enum CONTROLLER_TYPE {NONE, BANG, BANG2, BANGH, P, PID};

// Controller class declaration
class Controller
{
public:
    Controller() : Controller(NONE, 0.0) {}
    Controller(CONTROLLER_TYPE type, float h)
        : type(type),
          h(h),
          Kp(0.0), Ti(0.0), Td(0.0),
          deltah(0.0), bangvalue(0.0),
          max_u(0.3),
          e_m1(0.0), e_m2(0.0),
          u_m1(0.0)
          {}
    virtual ~Controller() = default;

    float computeControlSignal(float r, float y);
    void reset();

private:
    CONTROLLER_TYPE type;
    float h,                   // sampling interval
          Kp, Ti, Td,          // PID controller parameters
          deltah, bangvalue,   // hysteresis for bang-bang controller
          max_u,               // saturation value for control signal
          e_m1, e_m2,          // memory for error
          u_m1;                // memory for control signal
};

}; // namespace agent

#endif // AGENT_CONTROLLER_H
