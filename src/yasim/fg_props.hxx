#ifndef _FGPROPS_HXX
#define _FGPROPS_HXX

#include <simgear/props/props.hxx>

// Stubs, required to link
bool fgSetFloat (const char * name, float val);
bool fgSetBool(char const * name, bool val);
bool fgGetBool(char const * name, bool def);
bool fgSetString(char const * name, char const * str);
SGPropertyNode* fgGetNode (const char * path, bool create = false);
SGPropertyNode* fgGetNode (const char * path, int i, bool create = false);
float fgGetFloat (const char * name, float defaultValue = 0.0);
double fgGetDouble (const char * name, double defaultValue = 0.0);
bool fgSetDouble (const char * name, double defaultValue);

#endif // _FGPROPS_HXX
