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
static const float DEG2RAD = 0.0174532925199;
static const float KTS2MPS = 0.514444444444;


int usage()
{
    fprintf(stderr, "Usage: yasim <ac.xml> [-g [-a alt] [-s kts]]\n");
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

    {
        State s;
        m->setState(&s);
    }

    fgSetFloat("/controls/engines/engine[0]/throttle", 0.5);
    fgSetFloat("/controls/engines/engine[0]/mixture", 1.0);
    fgSetFloat("/controls/engines/engine[0]/magnetos", 3.0);
    fgSetFloat("/controls/flight/elevator", -0.1);
    fgSetFloat("/controls/flight/rudder", 0.112);

    /* Run the sim for 5 seconds first, to spin up engines etc */
    for (int i = 0; i < 1000; i++) {
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

        m->setWind(wind);

        m->setAir(Atmosphere::getStdPressure(alt),
                Atmosphere::getStdTemperature(alt),
                Atmosphere::getStdDensity(alt));

        fdm->getExternalInput();
        a->initEngines();
        fdm->iterate(1.0f/200.0f);
    }

    long double t = 0;

    for (int i=0; i<200*300; i++) {
        State *s = m->getState();

        float acc[3];
        a->getPilotAccel(acc);

        // position
        double lat, lon, alt;
        sgCartToGeod(s->pos, &lat, &lon, &alt);

        float xyz2ned[9];
        Glue::xyz2nedMat(lat, lon, xyz2ned);

        float tmp[9];
        Math::trans33(xyz2ned, tmp);
        Math::mmul33(s->orient, tmp, tmp);

        float roll, pitch, hdg;
        Glue::orient2euler(tmp, &roll, &pitch, &hdg);
        // make heading positive value
        if(hdg < 0.0) hdg += 2*M_PI;

        float v[3];

        Math::vmul33(xyz2ned, s->v, v);

        printf("t=%f\n", (double) t);
        printf("Acc %f %f %f\n", acc[0], acc[1], acc[2]);
        printf("pos %f %f %f\n", s->pos[0], s->pos[1], s->pos[2]);
        printf("Loc %f %f %f\n", lat, lon, alt);
        printf("Vel %f %f %f\n", v[0], v[1], v[2]);
        printf("Vbb %f %f %f\n", s->v[0], s->v[1], s->v[2]);
        printf("Att %f %f %f\n", roll * RAD2DEG, pitch * RAD2DEG, hdg * RAD2DEG);

        float wind[3] = { 0, 0, 0 };

        m->setWind(wind);

        m->setAir(Atmosphere::getStdPressure(alt),
                Atmosphere::getStdTemperature(alt),
                Atmosphere::getStdDensity(alt));

        m->updateGround(s);

        fdm->iterate(1.0f/200.0f);
        t += 1.0/200.0L;
    }

    delete fdm;
    return 0;
}
