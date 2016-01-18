#ifndef VERTEX_H
#define VERTEX_H

#include "Types.h"

class Vertex {
public:
    // constructor
    Vertex(const Eigen::Vector3d& position0, const int& index0);
    
    // position
    Eigen::Vector3d position;
    
    // adjacent faces
    std::vector<FaceIter> adjacentFaces;
    
    // index
    int index;
    
    // flag for boundary vertex
    bool isBoundary;
    
    // flag for isolated vertex
    bool isIsolated;
};

#endif
