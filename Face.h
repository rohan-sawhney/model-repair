#ifndef FACE_H
#define FACE_H

#include "Types.h"

class Face {
public:
    // constructor
    Face(const int& index0);
    
    // vertex indices
    Eigen::Vector3i vIndices;
    
    // uv indices
    Eigen::Vector3i uvIndices;
    
    // normal indices
    Eigen::Vector3i nIndices;
    
    // incident edges
    std::vector<EdgeIter> incidentEdges;
    
    // index
    int index;
    
    // flag for isolated face
    bool isIsolated;
};

#endif
