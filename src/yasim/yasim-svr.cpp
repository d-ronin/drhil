#include <string>
#include <cstdio>
#include <cstring>
#include <cstdlib>

#include <simgear/misc/sg_path.hxx>
#include <simgear/props/props.hxx>
#include <simgear/xml/easyxml.hxx>

#include "FGFDM.hpp"
#include "Atmosphere.hpp"
#include "Airplane.hpp"

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

    if(a->getFailureMsg()) {
        printf("SOLUTION FAILURE: %s\n", a->getFailureMsg());
	exit(2);
    }

    fdm->init();

    for (int i=0; i<200*300; i++) {
        fdm->iterate(1.0f/200.0f);
    }

    delete fdm;
    return 0;
}
