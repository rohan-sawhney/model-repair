#ifndef FACE_H
#define FACE_H

#include "Types.h"

class Face {
public:
    // constructor
    Face(const int& index0);
    
    // checks if face contains vertex
    bool containsVertex(const int& v);
    
    // vertex indices
    Eigen::Vector3i vIndices;
    
    // uv indices
    Eigen::Vector3i uvIndices;
    
    // normal indices
    Eigen::Vector3i nIndices;
    
    // adjacent faces
    std::vector<FaceIter> adjacentFaces;
    
    // index
    int index;
    
    // flag for isolated face
    bool isIsolated;
};

#endif
