#include <string>
#include <cstdio>
#include <cstring>
#include <cstdlib>

#include <simgear/misc/sg_path.hxx>
#include <simgear/props/props.hxx>
#include <simgear/xml/easyxml.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/misc/sg_path.hxx>

#include "fg_props.hxx"

#include "FGFDM.hpp"
#include "Atmosphere.hpp"
#include "Airplane.hpp"
#include "Glue.hpp"

using namespace yasim;

static const float RAD2DEG = 57.2957795131;

/* TODO: organize these into a better place */
struct command {
    uint32_t magic;

    float roll, pitch, yaw, throttle;

    float resv[8];

    bool armed;
};

struct status {
    uint32_t magic;

    double lat, lon, alt;

    float p, q, r;
    float acc[3];
    float vel[3];

    /* Provided only to "check" attitude solution */
    float roll, pitch, hdg;

    float resv[4];
};

bool readState(Airplane *a) {
    struct command frm;

    ssize_t rd = read(STDIN_FILENO, &frm, sizeof(frm));

    if (rd != sizeof(frm)) {
        return false;
    }

    if (frm.magic != 0xb33fbeef) {
        return false;
    }

    if (!frm.armed) {
        /* Before arming, hold position and run model... */
        Model *m = a->getModel();
        State *s = m->getState();
        
        float xyz2ned[9];
        Glue::xyz2nedMat(0, 0, xyz2ned);

        float alt = 100;

        sgGeodToCart(0, 0, alt, s->pos);

        Glue::euler2orient(0, 0, 0, s->orient);
        Math::mmul33(s->orient, xyz2ned, s->orient);

        /* Start off going 50 m/s forward */
        float v[3] = { 50, 0, 0 };

        Math::tmul33(s->orient, v, s->v);

        float wind[3] = { 0, 0, 0 };
    }

    fgSetFloat("/controls/flight/aileron", frm.roll);
    fgSetFloat("/controls/flight/elevator", frm.pitch);
    fgSetFloat("/controls/flight/rudder", frm.yaw);
    fgSetFloat("/controls/engines/engine[0]/throttle", frm.throttle);

    return true;
}

bool writeState(Airplane *a)
{
    Model *m = a->getModel();
    State *s = m->getState();

    struct status frm;

    memset(&frm, 0, sizeof(frm));

    frm.magic=0x00700799;

    // ------ Pilot-frame accelerations
    a->getPilotAccel(frm.acc);

    // ------ Position
    sgCartToGeod(s->pos, &frm.lat, &frm.lon, &frm.alt);

    // ------ Euler angles
    float xyz2ned[9];
    Glue::xyz2nedMat(frm.lat, frm.lon, xyz2ned);

    float tmp[9];
    Math::trans33(xyz2ned, tmp);
    Math::mmul33(s->orient, tmp, tmp);

    Glue::orient2euler(tmp, &frm.roll, &frm.pitch, &frm.hdg);

    // make heading positive value
    if (frm.hdg < 0.0) frm.hdg += 2*M_PI;

    // ------ Rotation rates
    float rot[3];

    Math::vmul33(s->orient, s->rot, rot);

    // Fix for odd coordinate system...
    frm.p = rot[0];
    frm.q = -rot[1];
    frm.r = -rot[2];

    // ------ NED velocities
    Math::vmul33(xyz2ned, s->v, frm.vel);

    // These next updates don't really fit in here, but they're more convenient
    // to factor this way.
    float wind[3] = { 0, 0, 0 };

    m->setWind(wind);

    m->setAir(Atmosphere::getStdPressure(frm.alt),
            Atmosphere::getStdTemperature(frm.alt),
            Atmosphere::getStdDensity(frm.alt));

    m->updateGround(s);

    // Send the frame.

    ssize_t wr = write(STDOUT_FILENO, &frm, sizeof(frm));

    return wr == sizeof(frm);
}

int usage()
{
    fprintf(stderr, "Usage: yasim <ac.xml>\n");
    return 1;
}

int main(int argc, char** argv)
{
    FGFDM* fdm = new FGFDM();
    Airplane* a = fdm->getAirplane();

    if(argc < 2) return usage();

    // Read
    try {
	std::string file = argv[1];
        readXML(file, *fdm);
    } catch (const sg_exception &e) {
        printf("XML parse error: %s (%s)\n",
               e.getFormattedMessage().c_str(), e.getOrigin());
	exit(1);
    }

    // ... and run
    a->compile();

    float aoa = a->getCruiseAoA() * RAD2DEG;
    float tail = -1 * a->getTailIncidence() * RAD2DEG;
    float drag = 1000 * a->getDragCoefficient();

    SG_LOG(SG_FLIGHT,SG_ALERT,"YASim solution results:");
    SG_LOG(SG_FLIGHT,SG_ALERT,"       Iterations: "<<a->getSolutionIterations());
    SG_LOG(SG_FLIGHT,SG_ALERT," Drag Coefficient: "<< drag);
    SG_LOG(SG_FLIGHT,SG_ALERT,"       Lift Ratio: "<<a->getLiftRatio());
    SG_LOG(SG_FLIGHT,SG_ALERT,"       Cruise AoA: "<< aoa);
    SG_LOG(SG_FLIGHT,SG_ALERT,"   Tail Incidence: "<< tail);
    SG_LOG(SG_FLIGHT,SG_ALERT,"Approach Elevator: "<<a->getApproachElevator());

    if(a->getFailureMsg()) {
        printf("SOLUTION FAILURE: %s\n", a->getFailureMsg());
	exit(2);
    }

    fdm->init();

    Model *m = a->getModel();
    State s;
    m->setState(&s);

    /* Initial conditions */
    fgSetFloat("/controls/engines/engine[0]/throttle", 0.5);
    fgSetFloat("/controls/engines/engine[0]/mixture", 1.0);
    fgSetFloat("/controls/engines/engine[0]/magnetos", 3.0);
    fgSetFloat("/controls/flight/elevator", -0.1);
    fgSetFloat("/controls/flight/rudder", 0.112);

    fdm->getExternalInput();
    a->initEngines();

    long double t = 0;

    while (writeState(a) && readState(a)) {
        fdm->iterate(1.0f/200.0f);
        t += 1.0/200.0L;
    }

    delete fdm;
    return 0;
}
