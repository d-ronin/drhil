// airspeed_indicator.hxx - a regular VSI tied to the static port.
// Written by David Megginson, started 2002.
//
// This file is in the Public Domain and comes with no warranty.


#ifndef __INSTRUMENTS_AIRSPEED_INDICATOR_HXX
#define __INSTRUMENTS_AIRSPEED_INDICATOR_HXX 1

#ifndef __cplusplus
# error This library requires C++
#endif

#include <simgear/props/props.hxx>
#include <simgear/structure/subsystem_mgr.hxx>


/**
 * Model an airspeed indicator tied to the pitot and static ports.
 *
 * Input properties:
 *
 * /instrumentation/"name"/serviceable
 * "pitot_port"/total-pressure-inhg
 * "static_port"/pressure-inhg
 * /environment/density-slugft3
 *
 * Output properties:
 *
 * /instrumentation/"name"/indicated-speed-kt
 */
class AirspeedIndicator : public SGSubsystem
{

public:

    AirspeedIndicator ( SGPropertyNode *node );
    virtual ~AirspeedIndicator ();

    virtual void init ();
    virtual void update (double dt);

private:

    string _name;
    unsigned int _num;
    string _total_pressure;
    string _static_pressure;
    SGPropertyNode_ptr _serviceable_node;
    SGPropertyNode_ptr _total_pressure_node;
    SGPropertyNode_ptr _static_pressure_node;
    SGPropertyNode_ptr _density_node;
    SGPropertyNode_ptr _speed_node;
    
};

#endif // __INSTRUMENTS_AIRSPEED_INDICATOR_HXX