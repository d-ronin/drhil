// fg_fx.hxx -- Sound effect management class
//
// Started by David Megginson, October 2001
// (Reuses some code from main.cxx, probably by Curtis Olson)
//
// Copyright (C) 2001  Curtis L. Olson - http://www.flightgear.org/~curt
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

#ifndef __FGFX_HXX
#define __FGFX_HXX 1

#include <simgear/compiler.h>

#include <queue>
#include <vector>

#include <simgear/sound/sample_openal.hxx>
#include <simgear/structure/subsystem_mgr.hxx>

using std::queue;
using std::vector;

class SGXmlSound;

/**
 * Generator for FlightGear sound effects.
 *
 * This module uses FGSoundMgr to generate sound effects based
 * on current flight conditions.  The sound manager must be initialized
 * before this object is.
 *
 * Note: this module supports two separate sound mechanisms concurrently.
 *
 * 1. This module will load and play a set of sound effects defined in an
 *    xml file and tie them to various property states.
 * 2. This modules also maintains a queue of 'message' audio files.  These
 *    are played sequentially with no overlap until the queue is finished.
 *    This second mechanims is useful for things like tutorial messages or
 *    background atc chatter.
 */
class FGFX : public SGSubsystem
{

public:

    FGFX ();
    virtual ~FGFX ();

    virtual void init ();
    virtual void reinit ();
    virtual void bind ();
    virtual void unbind ();
    virtual void update (double dt);

    /**
     * add a sound sample to the message queue which is played sequentially
     * in order.
     */
    void play_message( SGSoundSample *_sample );
    void play_message( const string path, const string fname, double volume );

private:

    void update_pos_and_orientation(SGSoundMgr *smgr, double dt);
    sgdVec3 last_visitor_pos;
    sgdVec3 last_model_pos;

    vector<SGXmlSound *> _sound;
    queue<SGSoundSample *> _samplequeue;

    bool last_pause;
    double last_volume;

    SGPropertyNode_ptr _pause;
    SGPropertyNode_ptr _volume;
};


#endif

// end of fg_fx.hxx