// Wx Radar background texture
//
// Written by Harald JOHNSEN, started May 2005.
// With major amendments by Vivian MEAZZA May 2007
// Ported to OSG by Tim Moore Jun 2007
//
//
// Copyright (C) 2005  Harald JOHNSEN
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
//

#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif

#include <osg/Array>
#include <osg/Geometry>
#include <osg/Matrixf>
#include <osg/PrimitiveSet>
#include <osg/StateSet>
#include <osg/Version>
#include <osgDB/ReaderWriter>
#include <osgDB/WriteFile>

#include <simgear/constants.h>
#include <simgear/misc/sg_path.hxx>
#include <simgear/environment/visual_enviro.hxx>
#include <simgear/scene/model/model.hxx>
#include <simgear/structure/exception.hxx>
#include <simgear/misc/sg_path.hxx>
#include <simgear/math/sg_geodesy.hxx>

#include <sstream>
#include <iomanip>
using std::stringstream;
using std::endl;
using std::setprecision;
using std::fixed;
using std::setw;
using std::setfill;

#include <Main/fg_props.hxx>
#include <Main/globals.hxx>
#include <Cockpit/panel.hxx>
#include <Cockpit/hud.hxx>
#include <AIModel/AIBase.hxx>
#include <AIModel/AIManager.hxx>
#include <AIModel/AIBallistic.hxx>

#include <Include/general.hxx>
#include "instrument_mgr.hxx"
#include "od_gauge.hxx"
#include "wxradar.hxx"


typedef list <osg::ref_ptr<FGAIBase> > radar_list_type;
typedef radar_list_type::iterator radar_list_iterator;
typedef radar_list_type::const_iterator radar_list_const_iterator;


static const float UNIT = 1.0f / 8.0f;  // 8 symbols in a row/column in the texture
static const char *DEFAULT_FONT = "typewriter.txf";


wxRadarBg::wxRadarBg(SGPropertyNode *node) :
    _name(node->getStringValue("name", "radar")),
    _num(node->getIntValue("number", 0)),
    _interval(node->getDoubleValue("update-interval-sec", 1.0)),
    _time(0.0),
    _sim_init_done(false),
    _odg(0),
    _last_switchKnob("off"),
    _resultTexture(0),
    _wxEcho(0)
{
    string branch;
    branch = "/instrumentation/" + _name;
    _Instrument = fgGetNode(branch.c_str(), _num, true);

    const char *tacan_source = node->getStringValue("tacan-source", "/instrumentation/tacan");
    _Tacan = fgGetNode(tacan_source, true);

    _font_node = _Instrument->getNode("font", true);

#define INITFONT(p, val, type) if (!_font_node->hasValue(p)) _font_node->set##type##Value(p, val)
    INITFONT("name", DEFAULT_FONT, String);
    INITFONT("size", 8, Float);
    INITFONT("line-spacing", 0.25, Float);
    INITFONT("color/red", 0, Float);
    INITFONT("color/green", 0.8, Float);
    INITFONT("color/blue", 0, Float);
    INITFONT("color/alpha", 1, Float);
#undef INITFONT

    _font_node->addChangeListener(this, true);
}


wxRadarBg::~wxRadarBg ()
{
    _font_node->removeChangeListener(this);
}


void
wxRadarBg::init ()
{
    _serviceable_node = _Instrument->getNode("serviceable", true);

    // texture name to use in 2D and 3D instruments
    _texture_path = _Instrument->getStringValue("radar-texture-path",
            "Aircraft/Instruments/Textures/od_wxradar.rgb");
    _resultTexture = FGTextureManager::createTexture(_texture_path.c_str(), false);

    SGPath tpath(globals->get_fg_root());
    string path = _Instrument->getStringValue("echo-texture-path",
            "Aircraft/Instruments/Textures/wxecho.rgb");
    tpath.append(path);

    // no mipmap or else alpha will mix with pixels on the border of shapes, ruining the effect
    _wxEcho = SGLoadTexture2D(tpath, false, false);


    _Instrument->setFloatValue("trk", 0.0);
    _Instrument->setFloatValue("tilt", 0.0);
    _Instrument->setStringValue("status", "");
    // those properties are used by a radar instrument of a MFD
    // input switch = OFF | TST | STBY | ON
    // input mode = WX | WXA | MAP
    // output status = STBY | TEST | WX | WXA | MAP | blank
    // input lightning = true | false
    // input TRK = +/- n degrees
    // input TILT = +/- n degree
    // input autotilt = true | false
    // input range = n nm (20/40/80)
    // input display-mode = arc | rose | map | plan

    FGInstrumentMgr *imgr = (FGInstrumentMgr *)globals->get_subsystem("instrumentation");
    _odg = (FGODGauge *)imgr->get_subsystem("od_gauge");
    _odg->setSize(512);

    _ai = (FGAIManager*)globals->get_subsystem("ai_model");
    _ai_enabled_node = fgGetNode("/sim/ai/enabled", true);

    _user_lat_node = fgGetNode("/position/latitude-deg", true);
    _user_lon_node = fgGetNode("/position/longitude-deg", true);
    _user_alt_node = fgGetNode("/position/altitude-ft", true);

    _user_speed_east_fps_node   = fgGetNode("/velocities/speed-east-fps", true);
    _user_speed_north_fps_node  = fgGetNode("/velocities/speed-north-fps", true);

    _tacan_serviceable_node = _Tacan->getNode("serviceable", true);
    _tacan_distance_node    = _Tacan->getNode("indicated-distance-nm", true);
    _tacan_name_node        = _Tacan->getNode("name", true);
    _tacan_bearing_node     = _Tacan->getNode("indicated-bearing-true-deg", true);
    _tacan_in_range_node    = _Tacan->getNode("in-range", true);

    _radar_mode_control_node = _Instrument->getNode("mode-control", true);
    _radar_coverage_node     = _Instrument->getNode("limit-deg", true);
    _radar_ref_rng_node      = _Instrument->getNode("reference-range-nm", true);
    _radar_hdg_marker_node   = _Instrument->getNode("heading-marker", true);

    SGPropertyNode *n = _Instrument->getNode("display-controls", true);
    _radar_weather_node     = n->getNode("WX", true);
    _radar_position_node    = n->getNode("pos", true);
    _radar_data_node        = n->getNode("data", true);
    _radar_symbol_node      = n->getNode("symbol", true);
    _radar_centre_node      = n->getNode("centre", true);
    _radar_rotate_node      = n->getNode("rotate", true);

    _radar_centre_node->setBoolValue(false);
    if (!_radar_coverage_node->hasValue())
        _radar_coverage_node->setFloatValue(120);
    if (!_radar_ref_rng_node->hasValue())
        _radar_ref_rng_node->setDoubleValue(35);
    if (!_radar_hdg_marker_node->hasValue())
        _radar_hdg_marker_node->setBoolValue(true);

    _x_offset = 0;
    _y_offset = 0;

    // OSG geometry setup. The polygons for the radar returns will be
    // stored in a single Geometry. The geometry will have several
    // primitive sets so we can have different kinds of polys and
    // choose a different overall color for each set.
    _radarGeode = new osg::Geode;
    osg::StateSet *stateSet = _radarGeode->getOrCreateStateSet();
    stateSet->setTextureAttributeAndModes(0, _wxEcho.get());
    _geom = new osg::Geometry;
    _geom->setUseDisplayList(false);
    // Initially allocate space for 128 quads
    _vertices = new osg::Vec2Array;
    _vertices->setDataVariance(osg::Object::DYNAMIC);
    _vertices->reserve(128 * 4);
    _geom->setVertexArray(_vertices);
    _texCoords = new osg::Vec2Array;
    _texCoords->setDataVariance(osg::Object::DYNAMIC);
    _texCoords->reserve(128 * 4);
    _geom->setTexCoordArray(0, _texCoords);
    osg::Vec3Array *colors = new osg::Vec3Array;
    colors->push_back(osg::Vec3(1.0f, 1.0f, 1.0f)); // color of echos
    colors->push_back(osg::Vec3(1.0f, 0.0f, 0.0f)); // arc mask
    colors->push_back(osg::Vec3(0.0f, 0.0f, 0.0f)); // rest of mask
    _geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
    _geom->setColorArray(colors);
    osg::PrimitiveSet *pset = new osg::DrawArrays(osg::PrimitiveSet::QUADS);
    pset->setDataVariance(osg::Object::DYNAMIC);
    _geom->addPrimitiveSet(pset);
    pset = new osg::DrawArrays(osg::PrimitiveSet::QUADS);
    pset->setDataVariance(osg::Object::DYNAMIC);
    _geom->addPrimitiveSet(pset);
    pset = new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES);
    pset->setDataVariance(osg::Object::DYNAMIC);
    _geom->addPrimitiveSet(pset);
    _geom->setInitialBound(osg::BoundingBox(osg::Vec3f(-256.0f, -256.0f, 0.0f),
            osg::Vec3f(256.0f, 256.0f, 0.0f)));
    _radarGeode->addDrawable(_geom);
    _odg->allocRT();
    // Texture in the 2D panel system
    FGTextureManager::addTexture(_texture_path.c_str(), _odg->getTexture());

    _textGeode = new osg::Geode;

    osg::Camera *camera = _odg->getCamera();
    camera->addChild(_radarGeode.get());
    camera->addChild(_textGeode.get());
}


// Local coordinates for each echo
const osg::Vec3f echoCoords[4] = {
    osg::Vec3f(-.7f, -.7f, 0.0f), osg::Vec3f(.7f, -.7f, 0.0f),
    osg::Vec3f(.7f, .7f, 0.0f), osg::Vec3f(-.7f, .7f, 0.0f)
};


const osg::Vec2f echoTexCoords[4] = {
    osg::Vec2f(0.0f, 0.0f), osg::Vec2f(UNIT, 0.0f),
    osg::Vec2f(UNIT, UNIT), osg::Vec2f(0.0f, UNIT)
};


// helper
static void
addQuad(osg::Vec2Array *vertices, osg::Vec2Array *texCoords,
        const osg::Matrixf& transform, const osg::Vec2f& texBase)
{
    for (int i = 0; i < 4; i++) {
        const osg::Vec3f coords = transform.preMult(echoCoords[i]);
        texCoords->push_back(texBase + echoTexCoords[i]);
        vertices->push_back(osg::Vec2f(coords.x(), coords.y()));
    }
}


// Rotate by a heading value
static inline
osg::Matrixf wxRotate(float angle)
{
    return osg::Matrixf::rotate(angle, 0.0f, 0.0f, -1.0f);
}


void
wxRadarBg::update (double delta_time_sec)
{
    if (!_sim_init_done) {
        if (!fgGetBool("sim/sceneryloaded", false))
            return;

        _sim_init_done = true;
    }
    if (!_odg || !_serviceable_node->getBoolValue()) {
        _Instrument->setStringValue("status", "");
        return;
    }
    _time += delta_time_sec;
    if (_time < _interval)
        return;

    _time = 0.0;

    string mode = _Instrument->getStringValue("display-mode", "arc");
    if (mode == "map") {
        if (_display_mode != MAP) {
            _display_mode = MAP;
            center_map();
        }
    } else if (mode == "plan") {
        _display_mode = PLAN;
    } else {
        _display_mode = ARC;
    }

    string switchKnob = _Instrument->getStringValue("switch", "on");
    if (_last_switchKnob != switchKnob) {
        // since 3D models don't share textures with the rest of the world
        // we must locate them and replace their handle by hand
        // only do that when the instrument is turned on
        //if (_last_switchKnob == "off")
        //_odg->set_texture(_texture_path.c_str(), _resultTexture->getHandle());

        _last_switchKnob = switchKnob;
    }

    if (switchKnob == "off") {
        _Instrument->setStringValue("status", "");
    } else if (switchKnob == "stby") {
        _Instrument->setStringValue("status", "STBY");
    } else if (switchKnob == "tst") {
        _Instrument->setStringValue("status", "TST");
        // find something interesting to do...
    } else {
        float r = _Instrument->getFloatValue("range", 40.0);
        if (r != _range_nm) {
            center_map();
            _range_nm = r;
        }

        _radar_ref_rng = _radar_ref_rng_node->getDoubleValue();
        _view_heading = get_heading() * SG_DEGREES_TO_RADIANS;
        _centerTrans.makeTranslate(0.0f, 0.0f, 0.0f);

        _scale = 200.0 / _range_nm;
        _angle_offset = 0;

        if (_display_mode == ARC) {
            _scale = 2*200.0f / _range_nm;
            _angle_offset = -_view_heading;
            _centerTrans.makeTranslate(0.0f, -200.0f, 0.0f);

        } else if (_display_mode == MAP) {
            apply_map_offset();

            bool centre = _radar_centre_node->getBoolValue();
            if (centre) {
                center_map();
                _radar_centre_node->setBoolValue(false);
            }

            //SG_LOG(SG_GENERAL, SG_DEBUG, "Radar: displacement "
            //        << _x_offset <<", "<<_y_offset
            //        << " user_speed_east_fps * SG_FPS_TO_KT "
            //        << user_speed_east_fps * SG_FPS_TO_KT
            //        << " user_speed_north_fps * SG_FPS_TO_KT "
            //        << user_speed_north_fps * SG_FPS_TO_KT
            //        << " dt " << delta_time_sec);

            _centerTrans.makeTranslate(_x_offset, _y_offset, 0.0f);

        } else if (_display_mode == PLAN) {
            if (_radar_rotate_node->getBoolValue()) {
                _angle_offset = -_view_heading;
            }
        } else {
            // rose
        }

        _vertices->clear();
        _texCoords->clear();
        _textGeode->removeDrawables(0, _textGeode->getNumDrawables());


        update_weather();


        osg::DrawArrays *quadPSet
                = static_cast<osg::DrawArrays*>(_geom->getPrimitiveSet(0));
        quadPSet->set(osg::PrimitiveSet::QUADS, 0, _vertices->size());
        quadPSet->dirty();

        // erase what is out of sight of antenna
        /*
            |\     /|
            | \   / |
            |  \ /  |
            ---------
            |       |
            |       |
            ---------
        */

        osg::DrawArrays *maskPSet
                = static_cast<osg::DrawArrays*>(_geom->getPrimitiveSet(1));
        osg::DrawArrays *trimaskPSet
                = static_cast<osg::DrawArrays*>(_geom->getPrimitiveSet(2));

        if (_display_mode == ARC) {
            float xOffset = 256.0f;
            float yOffset = 200.0f;

            int firstQuadVert = _vertices->size();
            _texCoords->push_back(osg::Vec2f(0.5f, 0.25f));
            _vertices->push_back(osg::Vec2f(-xOffset, 0.0 + yOffset));
            _texCoords->push_back(osg::Vec2f(1.0f, 0.25f));
            _vertices->push_back(osg::Vec2f(xOffset, 0.0 + yOffset));
            _texCoords->push_back(osg::Vec2f(1.0f, 0.5f));
            _vertices->push_back(osg::Vec2f(xOffset, 256.0 + yOffset));
            _texCoords->push_back(osg::Vec2f(0.5f, 0.5f));
            _vertices->push_back(osg::Vec2f(-xOffset, 256.0 + yOffset));
            maskPSet->set(osg::PrimitiveSet::QUADS, firstQuadVert, 4);

            // The triangles aren't supposed to be textured, but there's
            // no need to set up a different Geometry, switch modes,
            // etc. I happen to know that there's a white pixel in the
            // texture at 1.0, 0.0 :)
            float centerY = tan(30 * SG_DEGREES_TO_RADIANS);
            _vertices->push_back(osg::Vec2f(0.0, 0.0));
            _vertices->push_back(osg::Vec2f(-256.0, 0.0));
            _vertices->push_back(osg::Vec2f(-256.0, 256.0 * centerY));

            _vertices->push_back(osg::Vec2f(0.0, 0.0));
            _vertices->push_back(osg::Vec2f(256.0, 0.0));
            _vertices->push_back(osg::Vec2f(256.0, 256.0 * centerY));

            _vertices->push_back(osg::Vec2f(-256, 0.0));
            _vertices->push_back(osg::Vec2f(256.0, 0.0));
            _vertices->push_back(osg::Vec2f(-256.0, -256.0));

            _vertices->push_back(osg::Vec2f(256, 0.0));
            _vertices->push_back(osg::Vec2f(256.0, -256.0));
            _vertices->push_back(osg::Vec2f(-256.0, -256.0));

            const osg::Vec2f whiteSpot(1.0f, 0.0f);
            for (int i = 0; i < 3 * 4; i++)
                _texCoords->push_back(whiteSpot);

            trimaskPSet->set(osg::PrimitiveSet::TRIANGLES, firstQuadVert + 4, 3 * 4);

        } else {
            maskPSet->set(osg::PrimitiveSet::QUADS, 0, 0);
            trimaskPSet->set(osg::PrimitiveSet::TRIANGLES, 0, 0);
        }

        maskPSet->dirty();
        trimaskPSet->dirty();

        // draw without mask
        _vertices->clear();
        _texCoords->clear();

        update_aircraft();
        update_tacan();
        update_heading_marker();

        quadPSet->set(osg::PrimitiveSet::QUADS, 0, _vertices->size());
        quadPSet->dirty();
    }
}


void
wxRadarBg::update_weather()
{
    string modeButton = _Instrument->getStringValue("mode", "wx");
    _radarEchoBuffer = *sgEnviro.get_radar_echo();

    // pretend we have a scan angle bigger then the FOV
    // TODO:check real fov, enlarge if < nn, and do clipping if > mm
    const float fovFactor = 1.45f;
    _Instrument->setStringValue("status", modeButton.c_str());

    list_of_SGWxRadarEcho *radarEcho = &_radarEchoBuffer;
    list_of_SGWxRadarEcho::iterator iradarEcho, end = radarEcho->end();
    const float LWClevel[] = { 0.1f, 0.5f, 2.1f };

    // draw the cloud radar echo
    bool drawClouds = _radar_weather_node->getBoolValue();
    if (drawClouds) {

        // we do that in 3 passes, one for each color level
        // this is to 'merge' same colors together
        for (int level = 0; level <= 2; level++) {
            float col = level * UNIT;

            for (iradarEcho = radarEcho->begin(); iradarEcho != end; ++iradarEcho) {
                int cloudId = iradarEcho->cloudId;
                bool upgrade = (cloudId >> 5) & 1;
                float lwc = iradarEcho->LWC + (upgrade ? 1.0f : 0.0f);

                // skip ns
                if (iradarEcho->LWC >= 0.5 && iradarEcho->LWC <= 0.6)
                    continue;

                if (iradarEcho->lightning || lwc < LWClevel[level])
                    continue;

                float radius = sgSqrt(iradarEcho->dist) * SG_METER_TO_NM * _scale;
                float size = iradarEcho->radius * 2.0 * SG_METER_TO_NM * _scale;

                if (radius - size > 180)
                    continue;

                float angle = (iradarEcho->heading - _angle_offset) //* fovFactor
                        + 0.5 * SG_PI;

                // Rotate echo into position, and rotate echo to have
                // a constant orientation towards the
                // airplane. Compass headings increase in clockwise
                // direction, while graphics rotations follow
                // right-hand (counter-clockwise) rule.
                const osg::Vec2f texBase(col, (UNIT * (float) (4 + (cloudId & 3))));

                osg::Matrixf m(osg::Matrixf::scale(size, size, 1.0f)
                        * osg::Matrixf::translate(0.0f, radius, 0.0f)
                        * wxRotate(angle) * _centerTrans);
                addQuad(_vertices, _texCoords, m, texBase);

                //SG_LOG(SG_GENERAL, SG_DEBUG, "Radar: drawing clouds"
                //        << " ID=" << cloudId
                //        << " x=" << x
                //        << " y="<< y
                //        << " radius=" << radius
                //        << " view_heading=" << _view_heading * SG_RADIANS_TO_DEGREES
                //        << " heading=" << iradarEcho->heading * SG_RADIANS_TO_DEGREES
                //        << " angle=" << angle * SG_RADIANS_TO_DEGREES);
            }
        }
    }

    // draw lightning echos
    bool drawLightning = _Instrument->getBoolValue("lightning", true);
    if (drawLightning) {
        const osg::Vec2f texBase(3 * UNIT, 4 * UNIT);

        for (iradarEcho = radarEcho->begin(); iradarEcho != end; ++iradarEcho) {
            if (!iradarEcho->lightning)
                continue;

            float size = UNIT * 0.5f;
            float radius = iradarEcho->dist * _scale;
            float angle = iradarEcho->heading * SG_DEGREES_TO_RADIANS
                    - _angle_offset;

            osg::Matrixf m(osg::Matrixf::scale(size, size, 1.0f)
                    * wxRotate(-angle)
                    * osg::Matrixf::translate(0.0f, radius, 0.0f)
                    * wxRotate(angle) * _centerTrans);
            addQuad(_vertices, _texCoords, m, texBase);
        }
    }
}


void
wxRadarBg::update_data(FGAIBase *ac, double radius, double bearing, bool selected)
{
    osgText::Text *callsign = new osgText::Text;
    callsign->setFont(_font.get());
    callsign->setFontResolution(12, 12);
    callsign->setCharacterSize(_font_size);
    callsign->setColor(selected ? osg::Vec4(1, 1, 1, 1) : _font_color);
    osg::Matrixf m(wxRotate(-bearing)
            * osg::Matrixf::translate(0.0f, radius, 0.0f)
            * wxRotate(bearing) * _centerTrans);

    osg::Vec3 pos = m.preMult(osg::Vec3(16, 16, 0));
    // cast to int's, otherwise text comes out ugly
    callsign->setPosition(osg::Vec3((int)pos.x(), (int)pos.y(), 0));
    callsign->setAlignment(osgText::Text::LEFT_BOTTOM_BASE_LINE);
    callsign->setLineSpacing(_font_spacing);

    stringstream text;
    text << ac->_getCallsign() << endl
            << setprecision(0) << fixed
            << setw(3) << setfill('0') << ac->_getHeading() << "\xB0 "
            << setw(0) << ac->_getAltitude() << "ft" << endl
            << ac->_getSpeed() << "kts";

    callsign->setText(text.str());
    _textGeode->addDrawable(callsign);
}


void
wxRadarBg::update_aircraft()
{
    if (!_ai_enabled_node->getBoolValue())
        return;

    bool draw_echoes = _radar_position_node->getBoolValue();
    bool draw_symbols = _radar_symbol_node->getBoolValue();
    bool draw_data = _radar_data_node->getBoolValue();
    if (!draw_echoes && !draw_symbols && !draw_data)
        return;

    radar_list_type radar_list = _ai->get_ai_list();
    //SG_LOG(SG_GENERAL, SG_DEBUG, "Radar: AI submodel list size" << radar_list.size());
    if (radar_list.empty())
        return;

    //SG_LOG(SG_GENERAL, SG_DEBUG, "Radar: Loading AI submodels ");
    const double echo_radii[] = {0, 1, 1.5, 1.5, 0.001, 0.1, 1.5, 2, 1.5, 1.5, 1.5};

    double user_lat = _user_lat_node->getDoubleValue();
    double user_lon = _user_lon_node->getDoubleValue();
    double user_alt = _user_alt_node->getDoubleValue();

    float limit = _radar_coverage_node->getFloatValue();
    if (limit > 180)
        limit = 180;
    else if (limit < 0)
        limit = 0;
    limit *= SG_DEGREES_TO_RADIANS;

    radar_list_iterator it = radar_list.begin();
    radar_list_iterator end = radar_list.end();
    FGAIBase *selected_ac = 0;
    double selected_radius = 0;
    double selected_bearing = 0;
    int selected_id = fgGetInt("/instrumentation/radar/selected-id", -1);

    for (; it != end; ++it) {
        FGAIBase *ac = (*it).get();
        int type       = ac->getType();
        double lat     = ac->_getLatitude();
        double lon     = ac->_getLongitude();
        double alt     = ac->_getAltitude();
        double heading = ac->_getHeading();

        double range, bearing;
        calcRangeBearing(user_lat, user_lon, lat, lon, range, bearing);

        //SG_LOG(SG_GENERAL, SG_DEBUG,
        /*        "Radar: ID=" << ac->getID() << "(" << radar_list.size() << ")"
                << " type=" << type
                << " view_heading=" << _view_heading * SG_RADIANS_TO_DEGREES
                << " alt=" << alt
                << " heading=" << heading
                << " range=" << range
                << " bearing=" << bearing);*/

        bool isVisible = withinRadarHorizon(user_alt, alt, range);
        //SG_LOG(SG_GENERAL, SG_DEBUG, "Radar: visible " << isVisible);
        if (!isVisible)
            continue;

        if (!inRadarRange(type, range))
            continue;

        bearing *= SG_DEGREES_TO_RADIANS;
        heading *= SG_DEGREES_TO_RADIANS;

        float radius = range * _scale;
        float angle = calcRelBearing(bearing, _view_heading);

        if (angle > limit || angle < -limit)
            continue;

        bearing += _angle_offset;
        heading += _angle_offset;

        // pos mode
        if (draw_echoes) {
            float echo_radius = echo_radii[type] * 120;
            float size = echo_radius * UNIT;

            const osg::Vec2f texBase(3 * UNIT, 3 * UNIT);
            osg::Matrixf m(osg::Matrixf::scale(size, size, 1.0f)
                    * osg::Matrixf::translate(0.0f, radius, 0.0f)
                    * wxRotate(bearing) * _centerTrans);
            addQuad(_vertices, _texCoords, m, texBase);

            //SG_LOG(SG_GENERAL, SG_DEBUG, "Radar:    drawing AI"
                //<< " ID=" << ac->getID()
                //<< " type=" << type
            //        << " radius=" << radius
            //        << " angle=" << angle * SG_RADIANS_TO_DEGREES);
        }

        // data mode
        if (draw_symbols) {
            const osg::Vec2f texBase(0, 3 * UNIT);
            float size = 600 * UNIT;
            osg::Matrixf m(osg::Matrixf::scale(size, size, 1.0f)
                    * wxRotate(heading - bearing)
                    * osg::Matrixf::translate(0.0f, radius, 0.0f)
                    * wxRotate(bearing) * _centerTrans);
            addQuad(_vertices, _texCoords, m, texBase);

            //SG_LOG(SG_GENERAL, SG_DEBUG, "Radar:    drawing data"
            //        << " x=" << x <<" y="<< y
            //        << " bearing=" << angle * SG_RADIANS_TO_DEGREES
            //        << " radius=" << radius);
        }

        if (draw_data) {
            if (ac->getID() == selected_id) {
                selected_ac = ac;
                selected_radius = radius;
                selected_bearing = bearing;
            } else {
                update_data(ac, radius, bearing, false);
            }
        }
    }
    if (selected_ac) {
        update_data(selected_ac, selected_radius, selected_bearing, true);
    }
}


void
wxRadarBg::update_tacan()
{
    // draw TACAN symbol
    int mode = _radar_mode_control_node->getIntValue();
    bool inRange = _tacan_in_range_node->getBoolValue();

    if (mode != 1 || !inRange)
        return;

    float size = 600 * UNIT;
    float radius = _tacan_distance_node->getFloatValue() * _scale;
    float angle = _tacan_bearing_node->getFloatValue() * SG_DEGREES_TO_RADIANS
            + _angle_offset;

    const osg::Vec2f texBase(1 * UNIT, 3 * UNIT);
    osg::Matrixf m(osg::Matrixf::scale(size, size, 1.0f)
            * wxRotate(-angle)
            * osg::Matrixf::translate(0.0f, radius, 0.0f)
            * wxRotate(angle) * _centerTrans);
    addQuad(_vertices, _texCoords, m, texBase);

    //SG_LOG(SG_GENERAL, SG_DEBUG, "Radar:     drawing TACAN"
    //        << " dist=" << radius
    //        << " view_heading=" << _view_heading * SG_RADIANS_TO_DEGREES
    //        << " bearing=" << angle * SG_RADIANS_TO_DEGREES
    //        << " x=" << x << " y="<< y
    //        << " size=" << size);
}


void
wxRadarBg::update_heading_marker()
{
    if (!_radar_hdg_marker_node->getBoolValue())
        return;

    const osg::Vec2f texBase(2 * UNIT, 3 * UNIT);
    float size = 600 * UNIT;
    osg::Matrixf m(osg::Matrixf::scale(size, size, 1.0f)
            * wxRotate(_view_heading + _angle_offset));

    m *= _centerTrans;
    addQuad(_vertices, _texCoords, m, texBase);

    //SG_LOG(SG_GENERAL, SG_DEBUG, "Radar:   drawing heading marker"
    //        << " x,y " << x <<","<< y
    //        << " dist" << dist
    //        << " view_heading" << _view_heading * SG_RADIANS_TO_DEGREES
    //        << " heading " << iradarEcho->heading * SG_RADIANS_TO_DEGREES
    //        << " angle " << angle * SG_RADIANS_TO_DEGREES);
}


void
wxRadarBg::center_map()
{
    _lat = _user_lat_node->getDoubleValue();
    _lon = _user_lon_node->getDoubleValue();
    _x_offset = _y_offset = 0;
}


void
wxRadarBg::apply_map_offset()
{
    double lat = _user_lat_node->getDoubleValue();
    double lon = _user_lon_node->getDoubleValue();
    double bearing, distance, az2;
    geo_inverse_wgs_84(_lat, _lon, lat, lon, &bearing, &az2, &distance);
    distance *= SG_METER_TO_NM * _scale;
    bearing *= SG_DEGREES_TO_RADIANS;
    _x_offset += sin(bearing) * distance;
    _y_offset += cos(bearing) * distance;
    _lat = lat;
    _lon = lon;
}


bool
wxRadarBg::withinRadarHorizon(double user_alt, double alt, double range_nm)
{
    // Radar Horizon  = 1.23(ht^1/2 + hr^1/2),
    //don't allow negative altitudes (an approximation - yes altitudes can be negative)

    if (user_alt < 0)
        user_alt = 0;

    if (alt < 0)
        alt = 0;

    double radarhorizon = 1.23 * (sqrt(alt) + sqrt(user_alt));
    //SG_LOG(SG_GENERAL, SG_DEBUG, "Radar: horizon " << radarhorizon);
    return radarhorizon >= range_nm;
}


bool
wxRadarBg::inRadarRange(int type, double range_nm)
{
    //The Radar Equation:
    //
    // MaxRange^4 = (TxPower * AntGain^2 * lambda^2 * sigma)/((constant) * MDS)
    //
    // Where (constant) = (4*pi)3 and MDS is the Minimum Detectable Signal power.
    //
    // For a given radar we can assume that the only variable is sigma,
    // the target radar cross section.
    //
    // Here, we will use a normalised rcs (sigma) for a standard taget and assume that this
    // will provide a maximum range of 35nm;
    //
    // TODO - make the maximum range adjustable at runtime

    const double sigma[] = {0, 1, 100, 100, 0.001, 0.1, 100, 100, 1, 1, 1};
    double constant = _radar_ref_rng;

    if (constant <= 0)
        constant = 35;

    double maxrange = constant * pow(sigma[type], 0.25);
    //SG_LOG(SG_GENERAL, SG_DEBUG, "Radar: max range " << maxrange);
    return maxrange >= range_nm;
}


void
wxRadarBg::calcRangeBearing(double lat, double lon, double lat2, double lon2,
        double &range, double &bearing) const
{
    // calculate the bearing and range of the second pos from the first
    double az2, distance;
    geo_inverse_wgs_84(lat, lon, lat2, lon2, &bearing, &az2, &distance);
    range = distance *= SG_METER_TO_NM;
}


float
wxRadarBg::calcRelBearing(float bearing, float heading)
{
    float angle = bearing - heading;

    if (angle >= SG_PI)
        angle -= 2.0 * SG_PI;

    if (angle < -SG_PI)
        angle += 2.0 * SG_PI;

    return angle;
}


void
wxRadarBg::updateFont()
{
    float red = _font_node->getFloatValue("color/red");
    float green = _font_node->getFloatValue("color/green");
    float blue = _font_node->getFloatValue("color/blue");
    float alpha = _font_node->getFloatValue("color/alpha");
    _font_color.set(red, green, blue, alpha);

    _font_size = _font_node->getFloatValue("size");
    _font_spacing = _font_size * _font_node->getFloatValue("line-spacing");
    string path = _font_node->getStringValue("name", DEFAULT_FONT);

    SGPath tpath;
    if (path[0] != '/') {
        tpath = globals->get_fg_root();
        tpath.append("Fonts");
        tpath.append(path);
    } else {
        tpath = path;
    }

#if (FG_OSG_VERSION >= 21000)
    osg::ref_ptr<osgDB::ReaderWriter::Options> fontOptions = new osgDB::ReaderWriter::Options("monochrome");
    osg::ref_ptr<osgText::Font> font = osgText::readFontFile(tpath.c_str(), fontOptions.get());
#else
    osg::ref_ptr<osgText::Font> font = osgText::readFontFile(tpath.c_str());
#endif

    if (font != 0) {
        _font = font;
        _font->setMinFilterHint(osg::Texture::NEAREST);
        _font->setMagFilterHint(osg::Texture::NEAREST);
        _font->setGlyphImageMargin(0);
        _font->setGlyphImageMarginRatio(0);
    }
}

void
wxRadarBg::valueChanged(SGPropertyNode*)
{
    updateFont();
}
