// kln89_page.hxx - a class to manage the simulation of a KLN89
//                  GPS unit.  Note that this is primarily the 
//                  simulation of the user interface and display
//                  - the core GPS calculations such as position
//                  and waypoint sequencing are done (or should 
//                  be done) by FG code. 
//
// Written by David Luff, started 2005.
//
// Copyright (C) 2005 - David C Luff - david.luff@nottingham.ac.uk
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

#ifndef _KLN89_HXX
#define _KLN89_HXX

#include <Instrumentation/dclgps.hxx>
#include "kln89_page.hxx"

const int KLN89MapScales[2][21] = {{1, 2, 3, 5, 7, 10, 12, 15, 17, 20, 25, 30, 40, 60, 80, 100, 120, 160, 240, 320, 500},
	                               {2, 4, 6, 9, 13, 18, 22, 28, 32, 37, 46, 55, 75, 110, 150, 185, 220, 300, 440, 600, 925}};

enum KLN89Mode {
	KLN89_MODE_DISP,
	KLN89_MODE_CRSR
};

/*
const char* KLN89TimeCodes[20] = { "UTC", "GST", "GDT", "ATS", "ATD", "EST", "EDT", "CST", "CDT", "MST", 
                                   "MDT", "PST", "PDT", "AKS", "AKD", "HAS", "HAD", "SST", "SDT", "LCL" };
*/

// Used for storing airport town and county mapped by ID, since currently FG does not store this
typedef map<string, string> airport_id_str_map_type;
typedef airport_id_str_map_type::iterator airport_id_str_map_iterator;

class KLN89 : public DCLGPS {
	
	friend class KLN89Page;
	friend class KLN89AptPage;
	friend class KLN89VorPage;
	friend class KLN89NDBPage;
	friend class KLN89IntPage;
	friend class KLN89UsrPage;
	friend class KLN89ActPage;
	friend class KLN89NavPage;
	friend class KLN89FplPage;
	friend class KLN89CalPage;
	friend class KLN89SetPage;
	friend class KLN89OthPage;
	friend class KLN89DirPage;
	friend class KLN89NrstPage;
	
public:
	KLN89(RenderArea2D* instrument);
	~KLN89();
	
	void bind();
	void unbind();
	void update(double dt);
	
	inline void SetTurnAnticipation(bool b) { _turnAnticipationEnabled = b; }
	inline bool GetTurnAnticipation() { return(_turnAnticipationEnabled); }

	inline void SetSuaAlertEnabled(bool b) { _suaAlertEnabled = b; }
	inline bool GetSuaAlertEnabled() { return(_suaAlertEnabled); }
	
	inline void SetAltAlertEnabled(bool b) { _altAlertEnabled = b; }
	inline bool GetAltAlertEnabled() { return(_altAlertEnabled); }
	
	inline bool GetMsgAlert() const { return(!_messageStack.empty()); }
	
	void Knob1Right1();
	void Knob1Left1();
	void Knob2Right1();
	void Knob2Left1();
	void CrsrPressed();
	void EntPressed();
	void ClrPressed();
	void DtoPressed();
	void NrstPressed();
	void AltPressed();
	void OBSPressed();
	void MsgPressed();
	
	void CreateDefaultFlightPlans();

private:
	//----------------------- Drawing functions which take CHARACTER units -------------------------
	// Render string s in display field field at position x, y
	// WHERE POSITION IS IN CHARACTER UNITS!
	// zero y at bottom?
	// invert: -1 => no inversion, 0 -> n => 1 char - s[invert] gets inverted, 99 => entire string gets inverted 
	void DrawText(const string& s, int field, int px, int py, bool bold = false, int invert = -1);
	
	void DrawLatitude(double d, int field, int px, int py);
	void DrawLongitude(double d, int field, int px, int py);

	// Draw a frequency as xxx.xx
	void DrawFreq(double d, int field, int px, int py);
	
	// Draw a time in seconds as hh:mm
	// NOTE: px is RIGHT JUSTIFIED!
	void DrawTime(double time, int field, int px, int py);

	// Draw an integer heading, where px specifies the position of the degrees sign at the RIGHT of the value.
	void DrawHeading(int h, int field, int px, int py);
	
	// Draw a distance spec'd as nm as an integer (TODO - may need 1 decimal place if < 100) where px specifies RHS of units.
	// Some uses definately don't want decimal place though (as at present), so would have to be arg.
	void DrawDist(double d, int field, int px, int py);
	
	// Draw a speed specifed in knots.  px is RHS of the units.  Can draw up to 2 decimal places.
	void DrawSpeed(double v, int field, int px, int py, int decimals = 0);
	
	void Underline(int field, int px, int py, int len);
	
	// Render a char at a given position as above (position in CHARACTER units)
	void DrawChar(char c, int field, int px, int py, bool bold = false, bool invert = false);
	void DrawSpecialChar(char c, int field, int cx, int cy, bool bold = false);
	
	// Draws the dir/dist field at the bottom of the main field
	void DrawDirDistField(double lat, double lon, int field, int px, int py, bool to_flag = true, bool cursel = false);
	//
	//--------------------------------- end char units -----------------------------------------------
	
	//----------------------- Drawing functions which take PIXEL units ------------------------------
	//
	// Takes instrument *pixel* co-ordinates NOT character units
	// Position is specified by the bottom of the *visible* portion, by default the left position unless align_right is true.
	// The return value is the pixel width of the visible portion
	int DrawSmallChar(char c, int x, int y, bool align_right = false);
	
	void DrawFreeChar(char c, int x, int y, bool draw_background = false);
	//
	//----------------------------------- end pixel unit functions -----------------------------------
	
	void DrawDivider();
	
	void DrawEnt(int field = 1, int px = 0, int py = 1);
	
	void DrawMessageAlert();
	
	void DrawKPH(int field, int cx, int cy);
	
	void DrawDTO(int field, int cx, int cy);
	
	// Draw the bar that indicates which page we're on (zero-based)
	void DrawBar(int page);
	
	void DrawCDI();
	
	void DrawLegTail(int py);
	void DrawLongLegTail(int py);
	void DrawHalfLegTail(int py);
	
	void UpdateMapHeading();
	
	// Draw the moving map
	// Apt, VOR and SUA drawing can be suspended by setting draw_avs to false, without affecting the stored drawing preference state.
	void DrawMap(bool draw_avs = true);
	
	// Set whether the display should be draw pixelated (more primatives, but might be closer to real-life)
	// or not (in which case it is assumed that pixels are square and can be merged into quads).
	bool _pixelated;
	
	// Flashing output should be hidden when blink is true 
	bool _blink;
	
	double _cum_dt;
	
	// In Crsr mode, CRSR pressed events are passed to the active page, in disp mode they change which page is active
	KLN89Mode _mode;
	// And the facility to save a mode
	KLN89Mode _lastMode;
	
	// Increment/Decrement a character in the KLN89 A-Z,0-9 scheme.  
	// Set gap to true to get a space between A and 9 when wrapping, set wrap to false to disable wrap.
	char IncChar(char c, bool gap = false, bool wrap = true);
	char DecChar(char c, bool gap = false, bool wrap = true);
	
	// Hackish
	int _entJump;	// The page to jump back to if ent is pressed.  -1 indicates no jump
	bool _entRestoreCrsr;	// Indicates that pressing ENT at this point should restore cursor mode
	
	// Misc pages
	// Direct To
	GPSPage* _dir_page;
	// Nearest
	GPSPage* _nrst_page;
	
	// Moving-map display stuff
	int _mapOrientation;	// 0 => North (true) up, 1 => DTK up, 2 => TK up, 3 => heading up (only when connected to external heading source).
	double _mapHeading;		// Degrees.  The actual map heading gets updated at a lower frequency than DrawMap() is called at, hence we need to store it.
	double _mapHeadingUpdateTimer;	// Timer to determine when to update the above.
	bool _mapScaleAuto;		// Indicates that map should autoscale when true.
	int _mapScaleIndex;		// Index into array of available map scales.
	int _mapScaleUnits;		// 0 => nm, 1 => km.
	double _mapScale;	// nm or km from aircraft position to top of map.
						// Note that aircraft position differs depending on orientation, but 'scale' retains the same meaning,
						// so the scale per pixel alters to suit the defined scale when the rendered aircraft position changes.
	bool _drawSUA;	// special user airspace
	bool _drawVOR;
	bool _drawApt;
	
	// Convert map to instrument coordinates
	void MapToInstrument(int &x, int &y);
	
	// The following map drawing functions all take MAP co-ordinates, NOT instrument co-ordinates!
	
	// Draw the diamond style of user pos
	void DrawUser1(int x, int y);

	// Draw the airplane style of user pos
	void DrawUser2(int x, int y);
	
	// Draw an airport symbol on the moving map
	void DrawApt(int x, int y);
	
	// Draw a waypoint on the moving map
	void DrawWaypoint(int x, int y);
	
	// Draw a VOR on the moving map
	void DrawVOR(int x, int y);
	
	// Draw an airport or waypoint label on the moving map
	// Specify position by the map pixel co-ordinate of the left or right, bottom, of the *visible* portion of the label.
	// The black background quad will automatically overlap this by 1 pixel.
	void DrawLabel(const string& s, int x1, int y1, bool right_align = false);
	
	int GetLabelQuadrant(double h);
	int GetLabelQuadrant(double h1, double h2);
	
	// Draw a line on the moving map
	void DrawLine(int x1, int y1, int x2, int y2);
	
	// Draw normal sized text on the moving map
	void DrawMapText(const string& s, int x, int y, bool draw_background = false);
	
	void DrawMapUpArrow(int x, int y);
	
	// Draw a Quad on the moving map
	void DrawMapQuad(int x1, int y1, int x2, int y2, bool invert = false);
	
	// Airport town and state mapped by ID, since currently FG does not store this
	airport_id_str_map_type _airportTowns;
	airport_id_str_map_type _airportStates;
	
	// NOTE - It is a deliberate decision not to have a proper message page class,
	// since button events get directed to the page that was active before the
	// message was displayed, not the message page itself.
	bool _dispMsg;	// Set true while the message page is being displayed
};

#endif  // _KLN89_HXX