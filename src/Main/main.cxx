// main.cxx -- top level sim routines
//
// Written by Curtis Olson, started May 1997.
//
// Copyright (C) 1997 - 2002  Curtis L. Olson  - http://www.flightgear.org/~curt
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//
// $Id$


#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/compiler.h>

#if defined(__linux__) && defined(__i386__)
#  include <fpu_control.h>
#  include <signal.h>
#endif

#include <iostream>

#include <plib/netSocket.h>

#include <osg/Camera>
#include <osg/GraphicsContext>
#include <osgDB/Registry>

// Class references
#include <simgear/ephemeris/ephemeris.hxx>
#include <simgear/scene/model/modellib.hxx>
#include <simgear/scene/material/matlib.hxx>
#include <simgear/scene/model/animation.hxx>
#include <simgear/scene/sky/sky.hxx>
#include <simgear/structure/event_mgr.hxx>
#include <simgear/props/props.hxx>
#include <simgear/timing/sg_time.hxx>
#include <simgear/math/sg_random.h>

#include <Time/light.hxx>
#include <Include/general.hxx>
#include <Aircraft/replay.hxx>
#include <Cockpit/cockpit.hxx>
#include <Cockpit/hud.hxx>
#include <Model/panelnode.hxx>
#include <Model/modelmgr.hxx>
#include <Model/acmodel.hxx>
#include <Scenery/scenery.hxx>
#include <Scenery/tilemgr.hxx>
#include <Sound/beacon.hxx>
#include <Sound/morse.hxx>
#include <FDM/flight.hxx>
#include <ATCDCL/ATCmgr.hxx>
#include <ATCDCL/AIMgr.hxx>
#include <Time/tmp.hxx>
#include <Time/fg_timer.hxx>
#include <Environment/environment_mgr.hxx>
#include <GUI/new_gui.hxx>
#include <MultiPlayer/multiplaymgr.hxx>

#include "CameraGroup.hxx"
#include "fg_commands.hxx"
#include "fg_io.hxx"
#include "renderer.hxx"
#include "splash.hxx"
#include "main.hxx"
#include "util.hxx"
#include "fg_init.hxx"
#include "WindowSystemAdapter.hxx"


static double real_delta_time_sec = 0.0;
double delta_time_sec = 0.0;
extern float init_volume;

using namespace flightgear;

using std::cerr;

// This is a record containing a bit of global housekeeping information
FGGeneral general;

// Specify our current idle function state.  This is used to run all
// our initializations out of the idle callback so that we can get a
// splash screen up and running right away.
int idle_state = 0;
long global_multi_loop;

SGTimeStamp last_time_stamp;
SGTimeStamp current_time_stamp;

// The atexit() function handler should know when the graphical subsystem
// is initialized.
extern int _bootstrap_OSInit;



// Update internal time dependent calculations (i.e. flight model)
// FIXME: this distinction is obsolete; all subsystems now get delta
// time on update.
void fgUpdateTimeDepCalcs() {
    static bool inited = false;

    static const SGPropertyNode *replay_state
        = fgGetNode( "/sim/freeze/replay-state", true );
    static SGPropertyNode *replay_time
        = fgGetNode( "/sim/replay/time", true );
    // static const SGPropertyNode *replay_end_time
    //     = fgGetNode( "/sim/replay/end-time", true );

    //SG_LOG(SG_FLIGHT,SG_INFO, "Updating time dep calcs()");

    // Initialize the FDM here if it hasn't been and if we have a
    // scenery elevation hit.

    // cout << "cur_fdm_state->get_inited() = " << cur_fdm_state->get_inited() 
    //      << " cur_elev = " << scenery.get_cur_elev() << endl;

    if (!cur_fdm_state->get_inited()) {
        // Check for scenery around the aircraft.
        double lon = fgGetDouble("/sim/presets/longitude-deg");
        double lat = fgGetDouble("/sim/presets/latitude-deg");
        // We require just to have 50 meter scenery availabe around
        // the aircraft.
        double range = 1000.0;
        if (globals->get_scenery()->scenery_available(lat, lon, range)) {
            //SG_LOG(SG_FLIGHT, SG_INFO, "Finally initializing fdm");
            cur_fdm_state->init();
            if ( cur_fdm_state->get_bound() ) {
                cur_fdm_state->unbind();
            }
            cur_fdm_state->bind();
        }
    }

    // conceptually, the following block could be done for each fdm
    // instance ...
    if ( cur_fdm_state->get_inited() ) {
        // we have been inited, and  we are good to go ...

        if ( replay_state->getIntValue() == 0 ) {
            // replay off, run fdm
            cur_fdm_state->update( delta_time_sec );
        } else {
            FGReplay *r = (FGReplay *)(globals->get_subsystem( "replay" ));
            r->replay( replay_time->getDoubleValue() );
            if ( replay_state->getIntValue() == 1 ) {
                // normal playback
                replay_time->setDoubleValue( replay_time->getDoubleValue()
                                             + ( delta_time_sec
                                               * fgGetInt("/sim/speed-up") ) );
            } else if ( replay_state->getIntValue() == 2 ) {
                // paused playback (don't advance replay time)
            }
        }

        if ( !inited ) {
            inited = true;
            fgSetBool("/sim/signals/fdm-initialized", true);
        }

    } else {
        // do nothing, fdm isn't inited yet
    }

    globals->get_aircraft_model()->update(delta_time_sec);

    // Update solar system
    globals->get_ephem()->update( globals->get_time_params()->getMjd(),
                                  globals->get_time_params()->getLst(),
                                  cur_fdm_state->get_Latitude() );

}


void fgInitTimeDepCalcs( void ) {
    // noop for now
}


static const double alt_adjust_ft = 3.758099;
static const double alt_adjust_m = alt_adjust_ft * SG_FEET_TO_METER;


// What should we do when we have nothing else to do?  Let's get ready
// for the next move and update the display?
static void fgMainLoop( void ) {
    int model_hz = fgGetInt("/sim/model-hz");

    static SGConstPropertyNode_ptr longitude
        = fgGetNode("/position/longitude-deg");
    static SGConstPropertyNode_ptr latitude
        = fgGetNode("/position/latitude-deg");
    static SGConstPropertyNode_ptr altitude
        = fgGetNode("/position/altitude-ft");
    static SGConstPropertyNode_ptr vn_fps
        = fgGetNode("/velocities/speed-north-fps");
    static SGConstPropertyNode_ptr ve_fps
        = fgGetNode("/velocities/speed-east-fps");
    static SGConstPropertyNode_ptr vd_fps
        = fgGetNode("/velocities/speed-down-fps");
    static SGConstPropertyNode_ptr clock_freeze
        = fgGetNode("/sim/freeze/clock", true);
    static SGConstPropertyNode_ptr cur_time_override
        = fgGetNode("/sim/time/cur-time-override", true);
    static SGConstPropertyNode_ptr max_simtime_per_frame
        = fgGetNode("/sim/max-simtime-per-frame", true);
    static SGPropertyNode_ptr frame_signal
        = fgGetNode("/sim/signals/frame", true);

    frame_signal->fireValueChanged();
    SGCloudLayer::enable_bump_mapping = fgGetBool("/sim/rendering/bump-mapping");

    bool scenery_loaded = fgGetBool("sim/sceneryloaded");
    bool wait_for_scenery = !(scenery_loaded || fgGetBool("sim/sceneryloaded-override"));

    // Update the elapsed time.
    static bool first_time = true;
    if ( first_time ) {
        last_time_stamp.stamp();
        first_time = false;
    }

    double throttle_hz = fgGetDouble("/sim/frame-rate-throttle-hz", 0.0);
    if ( throttle_hz > 0.0 && !wait_for_scenery ) {
        // optionally throttle the frame rate (to get consistent frame
        // rates or reduce cpu usage.

        double frame_us = 1000000.0 / throttle_hz;

#define FG_SLEEP_BASED_TIMING 1
#if defined(FG_SLEEP_BASED_TIMING)
        // sleep based timing loop.
        //
        // Calling sleep, even usleep() on linux is less accurate than
        // we like, but it does free up the cpu for other tasks during
        // the sleep so it is desirable.  Because of the way sleep()
        // is implemented in consumer operating systems like windows
        // and linux, you almost always sleep a little longer than the
        // requested amount.
        //
        // To combat the problem of sleeping too long, we calculate the
        // desired wait time and shorten it by 2000us (2ms) to avoid
        // [hopefully] over-sleep'ing.  The 2ms value was arrived at
        // via experimentation.  We follow this up at the end with a
        // simple busy-wait loop to get the final pause timing exactly
        // right.
        //
        // Assuming we don't oversleep by more than 2000us, this
        // should be a reasonable compromise between sleep based
        // waiting, and busy waiting.

        // sleep() will always overshoot by a bit so undersleep by
        // 2000us in the hopes of never oversleeping.
        frame_us -= 2000.0;
        if ( frame_us < 0.0 ) {
            frame_us = 0.0;
        }
        current_time_stamp.stamp();
        /* Convert to ms */
        double elapsed_us = current_time_stamp - last_time_stamp;
        if ( elapsed_us < frame_us ) {
            double requested_us = frame_us - elapsed_us;
            ulMilliSecondSleep ( (int)(requested_us / 1000.0) ) ;
        }
#endif

        // busy wait timing loop.
        //
        // This yields the most accurate timing.  If the previous
        // ulMilliSecondSleep() call is omitted this will peg the cpu
        // (which is just fine if FG is the only app you care about.)
        current_time_stamp.stamp();
        while ( current_time_stamp - last_time_stamp < frame_us ) {
            current_time_stamp.stamp();
        }
    } else {
        // run as fast as the app will go
        current_time_stamp.stamp();
    }

    real_delta_time_sec
        = double(current_time_stamp - last_time_stamp) / 1000000.0;

    // Limit the time we need to spend in simulation loops
    // That means, if the /sim/max-simtime-per-frame value is strictly positive
    // you can limit the maximum amount of time you will do simulations for
    // one frame to display. The cpu time spent in simulations code is roughly
    // at least O(real_delta_time_sec). If this is (due to running debug
    // builds or valgrind or something different blowing up execution times)
    // larger than the real time you will no longer get any response
    // from flightgear. This limits that effect. Just set to property from
    // your .fgfsrc or commandline ...
    double dtMax = max_simtime_per_frame->getDoubleValue();
    if (0 < dtMax && dtMax < real_delta_time_sec)
        real_delta_time_sec = dtMax;

    // round the real time down to a multiple of 1/model-hz.
    // this way all systems are updated the _same_ amount of dt.
    {
        static double rem = 0.0;
        real_delta_time_sec += rem;
        double hz = model_hz;
        double nit = floor(real_delta_time_sec*hz);
        rem = real_delta_time_sec - nit/hz;
        real_delta_time_sec = nit/hz;
    }


    if (clock_freeze->getBoolValue() || wait_for_scenery) {
        delta_time_sec = 0;
    } else {
        delta_time_sec = real_delta_time_sec;
    }
    last_time_stamp = current_time_stamp;
    globals->inc_sim_time_sec( delta_time_sec );

    // These are useful, especially for Nasal scripts.
    fgSetDouble("/sim/time/delta-realtime-sec", real_delta_time_sec);
    fgSetDouble("/sim/time/delta-sec", delta_time_sec);

    static long remainder = 0;
    long elapsed;
#ifdef FANCY_FRAME_COUNTER
    int i;
    double accum;
#else
    static time_t last_time = 0;
    static int frames = 0;
#endif // FANCY_FRAME_COUNTER

    SGTime *t = globals->get_time_params();

    SG_LOG( SG_ALL, SG_DEBUG, "Running Main Loop");
    SG_LOG( SG_ALL, SG_DEBUG, "======= ==== ====");

    // Fix elevation.  I'm just sticking this here for now, it should
    // probably move eventually

    /* printf("Before - ground = %.2f  runway = %.2f  alt = %.2f\n",
           scenery.get_cur_elev(),
           cur_fdm_state->get_Runway_altitude() * SG_FEET_TO_METER,
           cur_fdm_state->get_Altitude() * SG_FEET_TO_METER); */

    /* printf("Adjustment - ground = %.2f  runway = %.2f  alt = %.2f\n",
           scenery.get_cur_elev(),
           cur_fdm_state->get_Runway_altitude() * SG_FEET_TO_METER,
           cur_fdm_state->get_Altitude() * SG_FEET_TO_METER); */

    // cout << "Warp = " << globals->get_warp() << endl;

    // update "time"
    static bool last_clock_freeze = false;

    if ( clock_freeze->getBoolValue() ) {
        // clock freeze requested
        if ( cur_time_override->getLongValue() == 0 ) {
            fgSetLong( "/sim/time/cur-time-override", t->get_cur_time() );
            globals->set_warp( 0 );
        }
    } else {
        // no clock freeze requested
        if ( last_clock_freeze == true ) {
            // clock just unfroze, let's set warp as the difference
            // between frozen time and current time so we don't get a
            // time jump (and corresponding sky object and lighting
            // jump.)
            globals->set_warp( cur_time_override->getLongValue() - time(NULL) );
            fgSetLong( "/sim/time/cur-time-override", 0 );
        }
        if ( globals->get_warp_delta() != 0 ) {
            globals->inc_warp( globals->get_warp_delta() );
        }
    }

    last_clock_freeze = clock_freeze->getBoolValue();

    t->update( longitude->getDoubleValue() * SGD_DEGREES_TO_RADIANS,
               latitude->getDoubleValue() * SGD_DEGREES_TO_RADIANS,
               cur_time_override->getLongValue(),
               globals->get_warp() );

    if (globals->get_warp_delta() != 0) {
        FGLight *l = (FGLight *)(globals->get_subsystem("lighting"));
        l->update( 0.5 );
    }

    // update magvar model
    globals->get_mag()->update( longitude->getDoubleValue()
                                * SGD_DEGREES_TO_RADIANS,
                                latitude->getDoubleValue()
                                * SGD_DEGREES_TO_RADIANS,
                                altitude->getDoubleValue() * SG_FEET_TO_METER,
                                globals->get_time_params()->getJD() );

    // Get elapsed time (in usec) for this past frame
    elapsed = fgGetTimeInterval();
    SG_LOG( SG_ALL, SG_DEBUG,
            "Elapsed time interval is = " << elapsed
            << ", previous remainder is = " << remainder );

    // Calculate frame rate average
#ifdef FANCY_FRAME_COUNTER
    /* old fps calculation */
    if ( elapsed > 0 ) {
        double tmp;
        accum = 0.0;
        for ( i = FG_FRAME_RATE_HISTORY - 2; i >= 0; i-- ) {
            tmp = general.get_frame(i);
            accum += tmp;
            // printf("frame[%d] = %.2f\n", i, g->frames[i]);
            general.set_frame(i+1,tmp);
        }
        tmp = 1000000.0 / (float)elapsed;
        general.set_frame(0,tmp);
        // printf("frame[0] = %.2f\n", general.frames[0]);
        accum += tmp;
        general.set_frame_rate(accum / (float)FG_FRAME_RATE_HISTORY);
        // printf("ave = %.2f\n", general.frame_rate);
    }
#else
    if ( (t->get_cur_time() != last_time) && (last_time > 0) ) {
        general.set_frame_rate( frames );
        fgSetInt("/sim/frame-rate", frames);
        SG_LOG( SG_ALL, SG_DEBUG,
            "--> Frame rate is = " << general.get_frame_rate() );
        frames = 0;
    }
    last_time = t->get_cur_time();
    ++frames;
#endif

    // Update any multiplayer's network queues, the AIMultiplayer
    // implementation is an AI model and depends on that
    globals->get_multiplayer_mgr()->Update();

    // Run ATC subsystem
    if (fgGetBool("/sim/atc/enabled"))
        globals->get_ATC_mgr()->update(delta_time_sec);

    // Run the AI subsystem
    // FIXME: run that also if we have multiplaying enabled since the
    // multiplayer information is interpreted by an AI model
    if (fgGetBool("/sim/ai-traffic/enabled"))
        globals->get_AI_mgr()->update(delta_time_sec);

    // Run flight model

    // Calculate model iterations needed for next frame
    elapsed += remainder;

    global_multi_loop = (long)(((double)elapsed * 0.000001) * model_hz );
    remainder = elapsed - ( (global_multi_loop*1000000) / model_hz );
    SG_LOG( SG_ALL, SG_DEBUG,
            "Model iterations needed = " << global_multi_loop
            << ", new remainder = " << remainder );

    // chop max iterations to something reasonable if the sim was
    // delayed for an excessive amount of time
    if ( global_multi_loop > 2.0 * model_hz ) {
        global_multi_loop = (int)(2.0 * model_hz );
        remainder = 0;
    }

    // flight model
    if ( global_multi_loop > 0) {
        // first run the flight model each frame until it is initialized
        // then continue running each frame only after initial scenery load is complete.
        fgUpdateTimeDepCalcs();
    } else {
        SG_LOG( SG_ALL, SG_DEBUG,
            "Elapsed time is zero ... we're zinging" );
    }

    // Run audio scheduler
#ifdef ENABLE_AUDIO_SUPPORT
    if ( globals->get_soundmgr()->is_working() ) {
        globals->get_soundmgr()->update( delta_time_sec );
    }
#endif

    globals->get_subsystem_mgr()->update(delta_time_sec);

    //
    // Tile Manager updates - see if we need to load any new scenery tiles.
    //   this code ties together the fdm, viewer and scenery classes...
    //   we may want to move this to its own class at some point
    //
    double visibility_meters = fgGetDouble("/environment/visibility-m");
    FGViewer *current_view = globals->get_current_view();

    globals->get_tile_mgr()->prep_ssg_nodes( visibility_meters );
    // update tile manager for view...
    SGLocation *view_location = globals->get_current_view()->getSGLocation();
    globals->get_tile_mgr()->update( view_location, visibility_meters );
    {
        double lon = view_location->getLongitude_deg();
        double lat = view_location->getLatitude_deg();
        double alt = view_location->getAltitudeASL_ft() * SG_FEET_TO_METER;

        // check if we can reuse the groundcache for that purpose.
        double ref_time, r;
        SGVec3d pt;
        bool valid = cur_fdm_state->is_valid_m(&ref_time, pt.sg(), &r);
        SGVec3d viewpos(globals->get_current_view()->get_view_pos());
        if (valid && distSqr(viewpos, pt) < r*r) {
            // Reuse the cache ...
            double lev
                = cur_fdm_state->get_groundlevel_m(lat*SGD_DEGREES_TO_RADIANS,
                                                   lon*SGD_DEGREES_TO_RADIANS,
                                                   alt + 2.0);
            view_location->set_cur_elev_m( lev );
        } else {
            // Do full intersection test.
            double lev;
            if (globals->get_scenery()->get_elevation_m(lat, lon, alt+2, lev, 0))
                view_location->set_cur_elev_m( lev );
            else
                view_location->set_cur_elev_m( -9999.0 );
        }
    }

    // run Nasal's settimer() loops right before the view manager
    globals->get_event_mgr()->update(delta_time_sec);

    // pick up model coordidnates that Nasal code may have set relative to the
    // aircraft's
    globals->get_model_mgr()->update(delta_time_sec);

    // update the view angle as late as possible, but before sound calculations
    globals->get_viewmgr()->update(delta_time_sec);

    // Do any I/O channel work that might need to be done (must come after viewmgr)
    globals->get_io()->update(real_delta_time_sec);

#ifdef ENABLE_AUDIO_SUPPORT
    // Right now we make a simplifying assumption that the primary
    // aircraft is the source of all sounds and that all sounds are
    // positioned in the aircraft base

    static sgdVec3 last_listener_pos = {0, 0, 0};
    static sgdVec3 last_model_pos = {0, 0, 0};

    // get the orientation
    const SGQuatd view_or = current_view->getViewOrientation();
    SGQuatd surf_or = SGQuatd::fromLonLatDeg(
        current_view->getLongitude_deg(), current_view->getLatitude_deg());
    SGQuatd model_or = SGQuatd::fromYawPitchRollDeg(
        globals->get_aircraft_model()->get3DModel()->getHeadingDeg(),
        globals->get_aircraft_model()->get3DModel()->getPitchDeg(),
        globals->get_aircraft_model()->get3DModel()->getRollDeg());

    // get the up and at vector in the aircraft base
    // (ok, the up vector is a down vector, but the coordinates
    // are finally calculated in a left hand system and openal
    // lives in a right hand system. Therefore we need to pass
    // the down vector to get correct stereo sound.)
    SGVec3d sgv_up = model_or.rotateBack(
        surf_or.rotateBack(view_or.rotate(SGVec3d(0, 1, 0))));
    sgVec3 up;
    sgSetVec3(up, sgv_up[0], sgv_up[1], sgv_up[2]);
    SGVec3d sgv_at = model_or.rotateBack(
        surf_or.rotateBack(view_or.rotate(SGVec3d(0, 0, 1))));
    sgVec3 at;
    sgSetVec3(at, sgv_at[0], sgv_at[1], sgv_at[2]);

    // get the location data for the primary FDM (now hardcoded to ac model)...
    SGLocation *acmodel_loc = NULL;
    acmodel_loc = (SGLocation *)globals->
        get_aircraft_model()->get3DModel()->getSGLocation();

    // Calculate speed of listener and model.  This code assumes the
    // listener is either tracking the model at the same speed or
    // stationary.

    sgVec3 listener_vel, model_vel;
    SGVec3d SGV3d_help;
    sgdVec3 sgdv3_help;
    sgdVec3 sgdv3_null = {0, 0, 0};

    // the aircraft velocity as reported by the fdm (this will not
    // vary or be affected by frame rates or timing jitter.)
    sgVec3 fdm_vel_vec;
    sgSetVec3( fdm_vel_vec,
	       vn_fps->getDoubleValue() * SG_FEET_TO_METER,
	       ve_fps->getDoubleValue() * SG_FEET_TO_METER,
	       vd_fps->getDoubleValue() * SG_FEET_TO_METER );
    double fdm_vel = sgLengthVec3(fdm_vel_vec);

    // compute the aircraft velocity vector and scale it to the length
    // of the fdm velocity vector.  This gives us a vector in the
    // proper coordinate system, but also with the proper time
    // invariant magnitude.
    sgdSubVec3( sgdv3_help,
                last_model_pos, acmodel_loc->get_absolute_view_pos());
    sgdAddVec3( last_model_pos, sgdv3_null, acmodel_loc->get_absolute_view_pos());
    SGV3d_help = model_or.rotateBack(
        surf_or.rotateBack(SGVec3d(sgdv3_help[0],
        sgdv3_help[1], sgdv3_help[2])));
    sgSetVec3( model_vel, SGV3d_help[0], SGV3d_help[1], SGV3d_help[2]);

    float vel = sgLengthVec3(model_vel);
    if ( fabs(vel) > 0.0001 ) {
	if ( fabs(fdm_vel / vel) > 0.0001 ) {
	    sgScaleVec3( model_vel, fdm_vel / vel );
	}
    }

    // check for moving or stationary listener (view position)
    sgdSubVec3( sgdv3_help,
                last_listener_pos, (double *)&current_view->get_view_pos());
    sgdAddVec3( last_listener_pos,
		sgdv3_null, (double *)&current_view->get_view_pos());

    if ( sgdLengthVec3(sgdv3_help) > 0.2 ) {
	sgCopyVec3( listener_vel, model_vel );
    } else {
	sgSetVec3( listener_vel, 0.0, 0.0, 0.0 );
    }

    globals->get_soundmgr()->set_listener_vel( listener_vel );

    // set positional offset for sources
    sgdVec3 dsource_pos_offset;
    sgdSubVec3( dsource_pos_offset,
                (double*) &current_view->get_view_pos(),
                acmodel_loc->get_absolute_view_pos() );
    SGVec3d sgv_dsource_pos_offset = model_or.rotateBack(
        surf_or.rotateBack(SGVec3d(dsource_pos_offset[0],
        dsource_pos_offset[1], dsource_pos_offset[2])));

    sgVec3 source_pos_offset;
    sgSetVec3(source_pos_offset, sgv_dsource_pos_offset[0],
        sgv_dsource_pos_offset[1], sgv_dsource_pos_offset[2]);

    globals->get_soundmgr()->set_source_pos_all( source_pos_offset );

    float orient[6];
    for (int i = 0; i < 3; i++) {
        orient[i] = sgv_at[i];
        orient[i + 3] = sgv_up[i];
    }
    globals->get_soundmgr()->set_listener_orientation( orient );

    // set the velocity
    // all sources are defined to be in the model
    globals->get_soundmgr()->set_source_vel_all( model_vel );

    // The listener is always positioned at the origin.
    sgVec3 listener_pos;
    sgSetVec3( listener_pos, 0.0, 0.0, 0.0 );
    globals->get_soundmgr()->set_listener_pos( listener_pos );
#endif

    // END Tile Manager udpates

    if (!scenery_loaded && globals->get_tile_mgr()->isSceneryLoaded()
        && cur_fdm_state->get_inited()) {
        fgSetBool("sim/sceneryloaded",true);
        fgSetFloat("/sim/sound/volume", init_volume);
        globals->get_soundmgr()->set_volume(init_volume);
    }

    fgRequestRedraw();

    SG_LOG( SG_ALL, SG_DEBUG, "" );
}

// Operation for querying OpenGL parameters. This must be done in a
// valid OpenGL context, potentially in another thread.
namespace
{
struct GeneralInitOperation : public GraphicsContextOperation
{
    GeneralInitOperation()
        : GraphicsContextOperation(std::string("General init"))
    {
    }
    void run(osg::GraphicsContext* gc)
    {
        general.set_glVendor( (char *)glGetString ( GL_VENDOR ) );
        general.set_glRenderer( (char *)glGetString ( GL_RENDERER ) );
        general.set_glVersion( (char *)glGetString ( GL_VERSION ) );
        SG_LOG( SG_GENERAL, SG_INFO, general.get_glRenderer() );

        GLint tmp;
        glGetIntegerv( GL_MAX_TEXTURE_SIZE, &tmp );
        general.set_glMaxTexSize( tmp );
        SG_LOG ( SG_GENERAL, SG_INFO, "Max texture size = " << tmp );

        glGetIntegerv( GL_DEPTH_BITS, &tmp );
        general.set_glDepthBits( tmp );
        SG_LOG ( SG_GENERAL, SG_INFO, "Depth buffer bits = " << tmp );
    }
};
}

// This is the top level master main function that is registered as
// our idle funciton

// The first few passes take care of initialization things (a couple
// per pass) and once everything has been initialized fgMainLoop from
// then on.

static void fgIdleFunction ( void ) {
    static osg::ref_ptr<GeneralInitOperation> genOp;
    if ( idle_state == 0 ) {
        idle_state++;
        // Pick some window on which to do queries.
        // XXX Perhaps all this graphics initialization code should be
        // moved to renderer.cxx?
        genOp = new GeneralInitOperation;
        osg::Camera* guiCamera = getGUICamera(CameraGroup::getDefault());
        WindowSystemAdapter* wsa = WindowSystemAdapter::getWSA();
        osg::GraphicsContext* gc = 0;
        if (guiCamera)
            gc = guiCamera->getGraphicsContext();
        if (gc) {
            gc->add(genOp.get());
        } else {
            wsa->windows[0]->gc->add(genOp.get());
        }
        guiStartInit(gc);
    } else if ( idle_state == 1 ) {
        if (genOp.valid()) {
            if (!genOp->isFinished())
                return;
            genOp = 0;
        }
        if (!guiFinishInit())
            return;
        idle_state++;
        fgSplashProgress("reading aircraft list");


    } else if ( idle_state == 2 ) {
        idle_state++;
        // Read the list of available aircraft
        fgReadAircraft();
        fgSplashProgress("reading airport & navigation data");


    } else if ( idle_state == 3 ) {
        idle_state++;
        fgInitNav();
        fgSplashProgress("setting up scenery");


    } else if ( idle_state == 4 ) {
        idle_state++;
        // based on the requested presets, calculate the true starting
        // lon, lat
        fgInitPosition();
        fgInitTowerLocationListener();

        SGTime *t = fgInitTime();
        globals->set_time_params( t );

        // Do some quick general initializations
        if( !fgInitGeneral()) {
            SG_LOG( SG_GENERAL, SG_ALERT,
                "General initialization failed ..." );
            exit(-1);
        }

        ////////////////////////////////////////////////////////////////////
        // Initialize the property-based built-in commands
        ////////////////////////////////////////////////////////////////////
        fgInitCommands();


        ////////////////////////////////////////////////////////////////////
        // Initialize the material manager
        ////////////////////////////////////////////////////////////////////
        globals->set_matlib( new SGMaterialLib );
        simgear::SGModelLib::init(globals->get_fg_root());


        ////////////////////////////////////////////////////////////////////
        // Initialize the TG scenery subsystem.
        ////////////////////////////////////////////////////////////////////
        globals->set_scenery( new FGScenery );
        globals->get_scenery()->init();
        globals->get_scenery()->bind();
        globals->set_tile_mgr( new FGTileMgr );


        ////////////////////////////////////////////////////////////////////
        // Initialize the general model subsystem.
        ////////////////////////////////////////////////////////////////////
        globals->set_model_mgr(new FGModelMgr);
        globals->get_model_mgr()->init();
        globals->get_model_mgr()->bind();
        fgSplashProgress("loading aircraft");


    } else if ( idle_state == 5 ) {
        idle_state++;
        ////////////////////////////////////////////////////////////////////
        // Initialize the 3D aircraft model subsystem (has a dependency on
        // the scenery subsystem.)
        ////////////////////////////////////////////////////////////////////
        globals->set_aircraft_model(new FGAircraftModel);
        globals->get_aircraft_model()->init();
        globals->get_aircraft_model()->bind();

        ////////////////////////////////////////////////////////////////////
        // Initialize the view manager subsystem.
        ////////////////////////////////////////////////////////////////////
        FGViewMgr *viewmgr = new FGViewMgr;
        globals->set_viewmgr( viewmgr );
        viewmgr->init();
        viewmgr->bind();
        fgSplashProgress("generating sky elements");


    } else if ( idle_state == 6 ) {
        idle_state++;
        // Initialize the sky
        SGPath ephem_data_path( globals->get_fg_root() );
        ephem_data_path.append( "Astro" );
        SGEphemeris *ephem = new SGEphemeris( ephem_data_path.c_str() );
        ephem->update( globals->get_time_params()->getMjd(),
                       globals->get_time_params()->getLst(),
                       0.0 );
        globals->set_ephem( ephem );

        // TODO: move to environment mgr
        thesky = new SGSky;
        SGPath texture_path(globals->get_fg_root());
        texture_path.append("Textures");
        texture_path.append("Sky");
        for (int i = 0; i < FGEnvironmentMgr::MAX_CLOUD_LAYERS; i++) {
            SGCloudLayer * layer = new SGCloudLayer(texture_path.str());
            thesky->add_cloud_layer(layer);
        }

        SGPath sky_tex_path( globals->get_fg_root() );
        sky_tex_path.append( "Textures" );
        sky_tex_path.append( "Sky" );
        thesky->texture_path( sky_tex_path.str() );

        // The sun and moon diameters are scaled down numbers of the
        // actual diameters. This was needed to fit both the sun and the
        // moon within the distance to the far clip plane.
        // Moon diameter:    3,476 kilometers
        // Sun diameter: 1,390,000 kilometers
        thesky->build( 80000.0, 80000.0,
                       463.3, 361.8,
                       globals->get_ephem()->getNumPlanets(),
                       globals->get_ephem()->getPlanets(),
                       globals->get_ephem()->getNumStars(),
                       globals->get_ephem()->getStars(),
                       fgGetNode("/environment", true));

        // Initialize MagVar model
        SGMagVar *magvar = new SGMagVar();
        globals->set_mag( magvar );


                                    // kludge to initialize mag compass
                                    // (should only be done for in-flight
                                    // startup)
        // update magvar model
        globals->get_mag()->update( fgGetDouble("/position/longitude-deg")
                                    * SGD_DEGREES_TO_RADIANS,
                                    fgGetDouble("/position/latitude-deg")
                                    * SGD_DEGREES_TO_RADIANS,
                                    fgGetDouble("/position/altitude-ft")
                                    * SG_FEET_TO_METER,
                                    globals->get_time_params()->getJD() );
        double var = globals->get_mag()->get_magvar() * SGD_RADIANS_TO_DEGREES;
        fgSetDouble("/instrumentation/heading-indicator/offset-deg", -var);
        fgSetDouble("/instrumentation/heading-indicator-fg/offset-deg", -var);


        // airport = new ssgBranch;
        // airport->setName( "Airport Lighting" );
        // lighting->addKid( airport );

        // build our custom render states
        fgSplashProgress("initializing subsystems");


    } else if ( idle_state == 7 ) {
        idle_state++;
        // Initialize audio support
#ifdef ENABLE_AUDIO_SUPPORT

        // Start the intro music
        if ( fgGetBool("/sim/startup/intro-music") ) {
            SGPath mp3file( globals->get_fg_root() );
            mp3file.append( "Sounds/intro.mp3" );

            SG_LOG( SG_GENERAL, SG_INFO,
                "Starting intro music: " << mp3file.str() );

#if defined( __CYGWIN__ )
            string command = "start /m `cygpath -w " + mp3file.str() + "`";
#elif defined( WIN32 )
            string command = "start /m " + mp3file.str();
#else
            string command = "mpg123 " + mp3file.str() + "> /dev/null 2>&1";
#endif

            system ( command.c_str() );
        }
#endif
        // This is the top level init routine which calls all the
        // other subsystem initialization routines.  If you are adding
        // a subsystem to flightgear, its initialization call should be
        // located in this routine.
        if( !fgInitSubsystems()) {
            SG_LOG( SG_GENERAL, SG_ALERT,
                "Subsystem initialization failed ..." );
            exit(-1);
        }
        fgSplashProgress("setting up time & renderer");


    } else if ( idle_state == 8 ) {
        idle_state = 1000;
        // Initialize the time offset (warp) after fgInitSubsystem
        // (which initializes the lighting interpolation tables.)
        fgInitTimeOffset();

        // setup OpenGL view parameters
        globals->get_renderer()->init();

        SG_LOG( SG_GENERAL, SG_INFO, "Panel visible = " << fgPanelVisible() );
        globals->get_renderer()->resize( fgGetInt("/sim/startup/xsize"),
                                         fgGetInt("/sim/startup/ysize") );

        fgSplashProgress("loading scenery objects");

    }

    if ( idle_state == 1000 ) {
        // We've finished all our initialization steps, from now on we
        // run the main loop.
        fgSetBool("sim/sceneryloaded", false);
        fgRegisterIdleHandler( fgMainLoop );
    }
}


static void upper_case_property(const char *name)
{
    SGPropertyNode *p = fgGetNode(name, false);
    if (!p) {
        p = fgGetNode(name, true);
        p->setStringValue("");
    } else {
        SGPropertyNode::Type t = p->getType();
        if (t == SGPropertyNode::NONE || t == SGPropertyNode::UNSPECIFIED)
            p->setStringValue("");
        else
            assert(t == SGPropertyNode::STRING);
    }
    p->addChangeListener(new FGMakeUpperCase);
}


// Main top level initialization
bool fgMainInit( int argc, char **argv ) {

    // set default log levels
    sglog().setLogLevels( SG_ALL, SG_ALERT );

    string version;
#ifdef FLIGHTGEAR_VERSION
    version = FLIGHTGEAR_VERSION;
#else
    version = "unknown version";
#endif
    SG_LOG( SG_GENERAL, SG_INFO, "FlightGear:  Version "
            << version );
    SG_LOG( SG_GENERAL, SG_INFO, "Built with " << SG_COMPILER_STR << endl );

    // Allocate global data structures.  This needs to happen before
    // we parse command line options

    globals = new FGGlobals;

    // seed the random number generator
    sg_srandom_time();

    FGControls *controls = new FGControls;
    globals->set_controls( controls );

    string_list *col = new string_list;
    globals->set_channel_options_list( col );

    fgValidatePath("", false);  // initialize static variables
    upper_case_property("/sim/presets/airport-id");
    upper_case_property("/sim/presets/runway");
    upper_case_property("/sim/tower/airport-id");
    upper_case_property("/autopilot/route-manager/input");

    // Scan the config file(s) and command line options to see if
    // fg_root was specified (ignore all other options for now)
    fgInitFGRoot(argc, argv);

    // Check for the correct base package version
    static char required_version[] = "1.9.0";
    string base_version = fgBasePackageVersion();
    if ( !(base_version == required_version) ) {
        // tell the operator how to use this application

        SG_LOG( SG_GENERAL, SG_ALERT, "" ); // To popup the console on windows
        cerr << endl << "Base package check failed ... " \
             << "Found version " << base_version << " at: " \
             << globals->get_fg_root() << endl;
        cerr << "Please upgrade to version: " << required_version << endl;
#ifdef _MSC_VER
        cerr << "Hit a key to continue..." << endl;
        cin.get();
#endif
        exit(-1);
    }

    // Load the configuration parameters.  (Command line options
    // override config file options.  Config file options override
    // defaults.)
    if ( !fgInitConfig(argc, argv) ) {
        SG_LOG( SG_GENERAL, SG_ALERT, "Config option parsing failed ..." );
        exit(-1);
    }

    // Initialize the Window/Graphics environment.
    fgOSInit(&argc, argv);
    _bootstrap_OSInit++;

    fgRegisterWindowResizeHandler( &FGRenderer::resize );
    fgRegisterIdleHandler( &fgIdleFunction );
    fgRegisterDrawHandler( &FGRenderer::update );

#ifdef FG_ENABLE_MULTIPASS_CLOUDS
    bool get_stencil_buffer = true;
#else
    bool get_stencil_buffer = false;
#endif

    // Initialize plib net interface
    netInit( &argc, argv );

    // Clouds3D requires an alpha channel
    // clouds may require stencil buffer
    fgOSOpenWindow(get_stencil_buffer);

    // Initialize the splash screen right away
    fntInit();
    fgSplashInit();

    // pass control off to the master event handler
    fgOSMainLoop();

    // we never actually get here ... but to avoid compiler warnings,
    // etc.
    return false;
}

