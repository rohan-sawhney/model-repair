#include "Face.h"

Face::Face(const int& index0):
index(index0),
isIsolated(false)
{
    
}

bool Face::containsVertex(const int& v)
{
    for (int i = 0; i < 3; i++) {
        if (v == vIndices[i]) return true;
    }
    
    return false;
}