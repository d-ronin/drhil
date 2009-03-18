
#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif

#include <simgear/debug/logstream.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/misc/sg_path.hxx>
#include <simgear/scene/model/placement.hxx>
#include <simgear/xml/easyxml.hxx>

#include <Main/globals.hxx>
#include <Main/fg_props.hxx>

#include "FGFDM.hpp"
#include "Atmosphere.hpp"
#include "Math.hpp"
#include "Airplane.hpp"
#include "Model.hpp"
#include "Integrator.hpp"
#include "Glue.hpp"
#include "Gear.hpp"
#include "Hook.hpp"
#include "Launchbar.hpp"
#include "FGGround.hpp"
#include "PropEngine.hpp"
#include "PistonEngine.hpp"

#include "YASim.hxx"

using namespace yasim;

static const float YASIM_PI = 3.14159265358979323846;
static const float RAD2DEG = 180/YASIM_PI;
static const float PI2 = YASIM_PI*2;
static const float RAD2RPM = 9.54929658551;
static const float M2FT = 3.2808399;
static const float FT2M = 0.3048;
static const float MPS2KTS = 3600.0/1852.0;
static const float CM2GALS = 264.172037284; // gallons/cubic meter
static const float KG2LBS = 2.20462262185;
static const float W2HP = 1.3416e-3;
static const float INHG2PA = 3386.389;
static const float SLUG2KG = 14.59390;

YASim::YASim(double dt) :
    _simTime(0)
{
//     set_delta_t(dt);
    _fdm = new FGFDM();

    _dt = dt;

    _fdm->getAirplane()->getModel()->setGroundCallback( new FGGround(this) );
    _fdm->getAirplane()->getModel()->getIntegrator()->setInterval(_dt);
}

YASim::~YASim()
{
    delete _fdm;
}

void YASim::report()
{
    Airplane* a = _fdm->getAirplane();

    float aoa = a->getCruiseAoA() * RAD2DEG;
    float tail = -1 * a->getTailIncidence() * RAD2DEG;
    float drag = 1000 * a->getDragCoefficient();

    SG_LOG(SG_FLIGHT,SG_INFO,"YASim solution results:");
    SG_LOG(SG_FLIGHT,SG_INFO,"       Iterations: "<<a->getSolutionIterations());
    SG_LOG(SG_FLIGHT,SG_INFO," Drag Coefficient: "<< drag);
    SG_LOG(SG_FLIGHT,SG_INFO,"       Lift Ratio: "<<a->getLiftRatio());
    SG_LOG(SG_FLIGHT,SG_INFO,"       Cruise AoA: "<< aoa);
    SG_LOG(SG_FLIGHT,SG_INFO,"   Tail Incidence: "<< tail);
    SG_LOG(SG_FLIGHT,SG_INFO,"Approach Elevator: "<<a->getApproachElevator());
    

    float cg[3];
    char buf[256];
    a->getModel()->getBody()->getCG(cg);
    sprintf(buf, "            CG: %.3f, %.3f, %.3f", cg[0], cg[1], cg[2]);
    SG_LOG(SG_FLIGHT, SG_INFO, buf);

    if(a->getFailureMsg()) {
        SG_LOG(SG_FLIGHT, SG_ALERT, "YASim SOLUTION FAILURE:");
        SG_LOG(SG_FLIGHT, SG_ALERT, a->getFailureMsg());
        exit(1);
    }
}

void YASim::bind()
{
    // Run the superclass bind to set up a bunch of property ties
    FGInterface::bind();

    // Now UNtie the ones that we are going to set ourselves.
    fgUntie("/consumables/fuel/tank[0]/level-gal_us");
    fgUntie("/consumables/fuel/tank[1]/level-gal_us");

    char buf[256];
    for(int i=0; i<_fdm->getAirplane()->getModel()->numThrusters(); i++) {
	sprintf(buf, "/engines/engine[%d]/fuel-flow-gph", i);        fgUntie(buf);
	sprintf(buf, "/engines/engine[%d]/rpm", i);                  fgUntie(buf);
	sprintf(buf, "/engines/engine[%d]/mp-osi", i);               fgUntie(buf);
	sprintf(buf, "/engines/engine[%d]/egt-degf", i);             fgUntie(buf);
	sprintf(buf, "/engines/engine[%d]/oil-temperature-degf", i); fgUntie(buf);
    }
}

void YASim::init()
{
    Airplane* a = _fdm->getAirplane();
    Model* m = a->getModel();

    // Superclass hook
    common_init();

    m->setCrashed(false);

    // Figure out the initial speed type
    string speed_set = fgGetString("/sim/presets/speed-set", "UVW");
    if (speed_set == "NED")
        _speed_set = NED;
    else if (speed_set == "UVW")
        _speed_set = UVW;
    else if (speed_set == "knots")
        _speed_set = KNOTS;
    else if (speed_set == "mach")
        _speed_set = MACH;
    else {
        _speed_set = UVW;
        SG_LOG(SG_FLIGHT, SG_ALERT, "Unknown speed type " << speed_set);
    }

    // Build a filename and parse it
    SGPath f(fgGetString("/sim/aircraft-dir"));
    f.append(fgGetString("/sim/aero"));
    f.concat(".xml");
    readXML(f.str(), *_fdm);

    // Compile it into a real airplane, and tell the user what they got
    a->compile();
    report();

    _fdm->init();

    // Create some FG{Eng|Gear}Interface objects
    int i;
    for(i=0; i<a->numGear(); i++) {
        Gear* g = a->getGear(i);
	SGPropertyNode * node = fgGetNode("gear/gear", i, true);
        float pos[3];
        g->getPosition(pos);
	node->setDoubleValue("xoffset-in", pos[0] * M2FT * 12);
	node->setDoubleValue("yoffset-in", pos[1] * M2FT * 12);
	node->setDoubleValue("zoffset-in", pos[2] * M2FT * 12);
    }

    // Are we at ground level?  If so, lift the plane up so the gear
    // clear the ground.
    double runway_altitude = get_Runway_altitude();
    if(get_Altitude() - runway_altitude < 50) {
        fgSetBool("/controls/gear/gear-down", false);
	float minGearZ = 1e18;
	for(i=0; i<a->numGear(); i++) {
	    Gear* g = a->getGear(i);
	    float pos[3];
	    g->getPosition(pos);
	    if(pos[2] < minGearZ)
		minGearZ = pos[2];
	}
	_set_Altitude(runway_altitude - minGearZ*M2FT);
	fgSetBool("/controls/gear/gear-down", true);
    }

    // Blank the state, and copy in ours
    State s;
    m->setState(&s);
    copyToYASim(true);

    _fdm->getExternalInput();
    _fdm->getAirplane()->initEngines();

    set_inited(true);
}

void YASim::update(double dt)
{
    if (is_suspended())
        return;

    int iterations = _calc_multiloop(dt);

    // If we're crashed, then we don't care
    if(_fdm->getAirplane()->getModel()->isCrashed()) {
        if(!fgGetBool("/sim/crashed"))
            fgSetBool("/sim/crashed", true);
        return;
    }

    // ground.  Calculate a cartesian coordinate for the ground under
    // us, find the (geodetic) up vector normal to the ground, then
    // use that to find the final (radius) term of the plane equation.
    float v[3] = { get_uBody(), get_vBody(), get_wBody() };
    float lat = get_Latitude(); float lon = get_Longitude();
    float alt = get_Altitude() * FT2M; double xyz[3];
    sgGeodToCart(lat, lon, alt, xyz);
    // build the environment cache.
    float vr = _fdm->getVehicleRadius();
    vr += 2.0*FT2M*dt*Math::mag3(v);
    prepare_ground_cache_m( _simTime, _simTime + dt, xyz, vr );

    // Track time increments.
    FGGround* gr
      = (FGGround*)_fdm->getAirplane()->getModel()->getGroundCallback();

    int i;
    for(i=0; i<iterations; i++) {
        gr->setTimeOffset(_simTime + i*_dt);
        copyToYASim(false);
        _fdm->iterate(_dt);
        copyFromYASim();
    }

    // Increment the local sim time
    _simTime += dt;
    gr->setTimeOffset(_simTime);
}

void YASim::copyToYASim(bool copyState)
{
    // Physical state
    double lat = get_Latitude();
    double lon = get_Longitude();
    float alt = get_Altitude() * FT2M;
    float roll = get_Phi();
    float pitch = get_Theta();
    float hdg = get_Psi();

    // Environment
    float wind[3];
    wind[0] = get_V_north_airmass() * FT2M * -1.0;
    wind[1] = get_V_east_airmass() * FT2M * -1.0;
    wind[2] = get_V_down_airmass() * FT2M * -1.0;

    float pressure = fgGetFloat("/environment/pressure-inhg") * INHG2PA;
    float temp = fgGetFloat("/environment/temperature-degc") + 273.15;
    float dens = fgGetFloat("/environment/density-slugft3") 
        * SLUG2KG * M2FT*M2FT*M2FT;

    // Convert and set:
    Model* model = _fdm->getAirplane()->getModel();
    State s;
    float xyz2ned[9];
    Glue::xyz2nedMat(lat, lon, xyz2ned);

    // position
    sgGeodToCart(lat, lon, alt, s.pos);

    // orientation
    Glue::euler2orient(roll, pitch, hdg, s.orient);
    Math::mmul33(s.orient, xyz2ned, s.orient);

    // Velocity
    string speed_set = fgGetString("/sim/presets/speed-set", "UVW");
    float v[3];
    bool needCopy = false;
    switch (_speed_set) {
    case NED:
        v[0] = get_V_north() * FT2M * -1.0;
        v[1] = get_V_east() * FT2M * -1.0;
        v[2] = get_V_down() * FT2M * -1.0;
        break;
    case UVW:
        v[0] = get_uBody() * FT2M;
        v[1] = get_vBody() * FT2M;
        v[2] = get_wBody() * FT2M;
        Math::tmul33(s.orient, v, v);
        break;
    case KNOTS:
        v[0] = Atmosphere::spdFromVCAS(get_V_calibrated_kts()/MPS2KTS,
                                       pressure, temp);
        v[1] = 0;
        v[2] = 0;
        Math::tmul33(s.orient, v, v);
        needCopy = true;
        break;
    case MACH:
        v[0] = Atmosphere::spdFromMach(get_Mach_number(), temp);
        v[1] = 0;
        v[2] = 0;
        Math::tmul33(s.orient, v, v);
        needCopy = true;
        break;
    default:
        v[0] = 0;
        v[1] = 0;
        v[2] = 0;
        break;
    }
    if (!copyState)
        _speed_set = UVW;       // change to this after initial setting
    Math::set3(v, s.v);

    if(copyState || needCopy)
	model->setState(&s);

    // wind
    Math::tmul33(xyz2ned, wind, wind);
    model->setWind(wind);

    // air
    model->setAir(pressure, temp, dens);

    // Query a ground plane for each gear/hook/launchbar and
    // write that value into the corresponding class.
    _fdm->getAirplane()->getModel()->updateGround(&s);

    Launchbar* l = model->getLaunchbar();
    if (l)
        l->setLaunchCmd(0.0<fgGetFloat("/controls/gear/catapult-launch-cmd"));
}

// All the settables:
//
// These are set below:
// _set_Accels_Local
// _set_Accels_Body
// _set_Accels_CG_Body 
// _set_Accels_Pilot_Body
// _set_Accels_CG_Body_N 
// _set_Velocities_Local
// _set_Velocities_Ground
// _set_Velocities_Wind_Body
// _set_Omega_Body
// _set_Euler_Rates
// _set_Euler_Angles
// _set_V_rel_wind
// _set_V_ground_speed
// _set_V_equiv_kts
// _set_V_calibrated_kts
// _set_Alpha
// _set_Beta
// _set_Mach_number
// _set_Climb_Rate
// _set_Tank1Fuel
// _set_Tank2Fuel
// _set_Altitude_AGL
// _set_Geodetic_Position
// _set_Runway_altitude

// Ignoring these, because they're unused:
// _set_Geocentric_Position
// _set_Geocentric_Rates
// _set_Cos_phi
// _set_Cos_theta
// _set_Earth_position_angle (WTF?)
// _set_Gamma_vert_rad
// _set_Inertias
// _set_T_Local_to_Body
// _set_CG_Position
// _set_Sea_Level_Radius

// Externally set via the weather code:
// _set_Velocities_Local_Airmass
// _set_Density
// _set_Static_pressure
// _set_Static_temperature
void YASim::copyFromYASim()
{
    Airplane* airplane = _fdm->getAirplane();
    Model* model = airplane->getModel();
    State* s = model->getState();

    // position
    double lat, lon, alt;
    sgCartToGeod(s->pos, &lat, &lon, &alt);
    _set_Geodetic_Position(lat, lon, alt*M2FT);
    double groundlevel_m = get_groundlevel_m(lat, lon, alt);
    _set_Runway_altitude(groundlevel_m*SG_METER_TO_FEET);
    _set_Altitude_AGL((alt-groundlevel_m)*SG_METER_TO_FEET);

    // the smallest agl of all gears
    fgSetFloat("/position/gear-agl-m", model->getAGL());
    fgSetFloat("/position/gear-agl-ft", model->getAGL()*M2FT);

    // UNUSED
    //_set_Geocentric_Position(Glue::geod2geocLat(lat), lon, alt*M2FT);

    // useful conversion matrix
    float xyz2ned[9];
    Glue::xyz2nedMat(lat, lon, xyz2ned);

    // velocity
    float v[3];
    Math::vmul33(xyz2ned, s->v, v);
    _set_Velocities_Local(M2FT*v[0], M2FT*v[1], M2FT*v[2]);
    _set_V_ground_speed(Math::sqrt(M2FT*v[0]*M2FT*v[0] +
				   M2FT*v[1]*M2FT*v[1]));
    _set_Climb_Rate(-M2FT*v[2]);

    // The HUD uses this, but inverts down (?!)
    _set_Velocities_Ground(M2FT*v[0], M2FT*v[1], -M2FT*v[2]);

    // _set_Geocentric_Rates(M2FT*v[0], M2FT*v[1], M2FT*v[2]); // UNUSED

    // Airflow velocity.
    float wind[3];
    wind[0] = get_V_north_airmass() * FT2M * -1.0;  // Wind in NED
    wind[1] = get_V_east_airmass() * FT2M * -1.0;
    wind[2] = get_V_down_airmass() * FT2M * -1.0;
    Math::tmul33(xyz2ned, wind, wind);              // Wind in global
    Math::sub3(s->v, wind, v);                      // V - wind in global
    Math::vmul33(s->orient, v, v);               // to body coordinates
    _set_Velocities_Wind_Body(v[0]*M2FT, -v[1]*M2FT, -v[2]*M2FT);
    _set_V_rel_wind(Math::mag3(v)*M2FT); // units?

    float P = fgGetDouble("/environment/pressure-inhg") * INHG2PA;
    float T = fgGetDouble("/environment/temperature-degc") + 273.15;
    float D = fgGetFloat("/environment/density-slugft3")
        *SLUG2KG * M2FT*M2FT*M2FT;
    _set_V_equiv_kts(Atmosphere::calcVEAS(v[0], P, T, D)*MPS2KTS);
    _set_V_calibrated_kts(Atmosphere::calcVCAS(v[0], P, T)*MPS2KTS);
    _set_Mach_number(Atmosphere::calcMach(v[0], T));

    // acceleration
    Math::vmul33(xyz2ned, s->acc, v);
    _set_Accels_Local(M2FT*v[0], M2FT*v[1], M2FT*v[2]);

    Math::vmul33(s->orient, s->acc, v);
    _set_Accels_Body(M2FT*v[0], -M2FT*v[1], -M2FT*v[2]);
    _set_Accels_CG_Body(M2FT*v[0], -M2FT*v[1], -M2FT*v[2]);

    _fdm->getAirplane()->getPilotAccel(v);
    _set_Accels_Pilot_Body(-M2FT*v[0], M2FT*v[1], M2FT*v[2]);

    // There is no property for pilot G's, but I need it for a panel
    // instrument.  Hack this in here, and REMOVE IT WHEN IT FINDS A
    // REAL HOME!
    fgSetFloat("/accelerations/pilot-g", -v[2]/9.8);

    // The one appears (!) to want inverted pilot acceleration
    // numbers, in G's...
    Math::mul3(1.0/9.8, v, v);
    _set_Accels_CG_Body_N(v[0], -v[1], -v[2]);

    // orientation
    float alpha, beta;
    Glue::calcAlphaBeta(s, wind, &alpha, &beta);
    _set_Alpha(alpha);
    _set_Beta(beta);

    float tmp[9];
    Math::trans33(xyz2ned, tmp);
    Math::mmul33(s->orient, tmp, tmp);
    float roll, pitch, hdg;
    Glue::orient2euler(tmp, &roll, &pitch, &hdg);
    // make heading positive value
    if(hdg < 0.0) hdg += PI2;
    _set_Euler_Angles(roll, pitch, hdg);

    // rotation
    Math::vmul33(s->orient, s->rot, v);
    _set_Omega_Body(v[0], -v[1], -v[2]);

    Glue::calcEulerRates(s, &roll, &pitch, &hdg);
    _set_Euler_Rates(roll, pitch, hdg);

    // Fill out our engine and gear objects
    int i;
    for(i=0; i<airplane->numGear(); i++) {
        Gear* g = airplane->getGear(i);
	SGPropertyNode * node = fgGetNode("gear/gear", i, true);
	node->setBoolValue("has-brake", g->getBrake() != 0);
	node->setBoolValue("wow", g->getCompressFraction() != 0);
	node->setFloatValue("compression-norm", g->getCompressFraction());
	node->setFloatValue("compression-m", g->getCompressDist());
        node->setFloatValue("caster-angle-deg", g->getCasterAngle() * RAD2DEG);
        node->setFloatValue("rollspeed-ms", g->getRollSpeed());
        node->setBoolValue("ground-is-solid", g->getGroundIsSolid()!=0);
        node->setFloatValue("ground-friction-factor", g->getGroundFrictionFactor());
    }

    Hook* h = airplane->getHook();
    if(h) {
	SGPropertyNode * node = fgGetNode("gear/tailhook", 0, true);
	node->setFloatValue("position-norm", h->getCompressFraction());
    }

    Launchbar* l = airplane->getLaunchbar();
    if(l) {
	SGPropertyNode * node = fgGetNode("gear/launchbar", 0, true);
	node->setFloatValue("position-norm", l->getCompressFraction());
        node->setFloatValue("holdback-position-norm", l->getHoldbackCompressFraction());
        node->setStringValue("state", l->getState());
        node->setBoolValue("strop", l->getStrop());
    }

}
