#include "fg_props.hxx"

static SGPropertyNode *root = new SGPropertyNode();

// Stubs, required to link
SGPropertyNode* fgGetNode (const char * path, bool create)
{
    return root->getNode(path, create);
}

SGPropertyNode* fgGetNode (const char * path, int i, bool create)
{
    return root->getNode(path, i, create);
}

bool fgSetFloat (const char * name, float val)
{
    SGPropertyNode *n = root->getNode(name, true);

    if (!n) {
        return false;
    }

    return n->setFloatValue(val);
}

bool fgSetBool(char const * name, bool val)
{
    SGPropertyNode *n = root->getNode(name, true);

    if (!n) {
        return false;
    }

    return n->setBoolValue(val);
}

bool fgSetString(char const * name, char const * val)
{
    SGPropertyNode *n = root->getNode(name, true);

    if (!n) {
        return false;
    }

    return n->setStringValue(val);
}

bool fgSetDouble (const char * name, double val)
{
    SGPropertyNode *n = root->getNode(name, true);

    if (!n) {
        return false;
    }

    return n->setDoubleValue(val);
}

bool fgGetBool(char const * name, bool def)
{
    return root->getBoolValue(name, def);
}

float fgGetFloat (const char * name, float def)
{
    float f = root->getFloatValue(name, def);

    //printf("getting %s=%f\n", name, f);

    return f;
}

double fgGetDouble (const char * name, double def)
{
    return root->getDoubleValue(name, def);
}
