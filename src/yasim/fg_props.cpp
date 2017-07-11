#include "fg_props.hxx"

static SGPropertyNode *root = new SGPropertyNode();

// Stubs, required to link
SGPropertyNode* fgGetNode (const char * path, bool create) {
       return root->getNode(path, create);
}

SGPropertyNode* fgGetNode (const char * path, int i, bool create) {
	return root->getNode(path, i, create);
}

bool fgSetFloat (const char * name, float val) { return false; }
bool fgSetBool(char const * name, bool val) { return false; }
bool fgGetBool(char const * name, bool def) { return false; }
bool fgSetString(char const * name, char const * str) { return false; }
float fgGetFloat (const char * name, float defaultValue) { return 0; }
double fgGetDouble (const char * name, double defaultValue) { return 0; }
bool fgSetDouble (const char * name, double defaultValue) { return 0; }
