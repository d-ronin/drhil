#ifndef _INTEGRATOR_HPP
#define _INTEGRATOR_HPP

#include "BodyEnvironment.hpp"
#include "RigidBody.hpp"

namespace yasim {

//
// These objects are responsible for extracting force data from a
// BodyEnvironment object, using a RigidBody object to calculate
// accelerations, and then tying that all together into a new
// "solution" of position/orientation/etc... for the body.  The method
// used is a fourth-order Runge-Kutta integration.
//
class Integrator
{
public:
    // Sets the RigidBody that will be integrated.
    void setBody(RigidBody* body);

    // Sets the BodyEnvironment object used to calculate the second
    // derivatives.
    void setEnvironment(BodyEnvironment* env);

    // The current state, i.e. initial conditions for the next
    // integration iteration.  Note that the acceleration parameters
    // in the State object are ignored.
    State* getState();
    void setState(State* s);

    // Do a 4th order Runge-Kutta integration over one time interval.
    // This is the top level of the simulation.
    void calcNewInterval(float user_dt);

private:
    void orthonormalize(float* m);
    void rotMatrix(float* r, float dt, float* out);
    void l2gVector(float* orient, float* v, float* out);
    void extrapolatePosition(double* pos, float* v, float dt,
                             float* o1, float* o2);

    BodyEnvironment* _env;
    RigidBody* _body;

    State _s;
};

}; // namespace yasim
#endif // _INTEGRATOR_HPP
