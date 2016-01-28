#ifndef EDGE_H
#define EDGE_H

#include "Types.h"

class Edge {
public:
    // constructor
    Edge(const int& i0, const int& i1, const int& index0);
    
    // checks if edge contains vertex
    bool containsVertex(const int& v) const;
        
    // vertex indices (v1 > v0)
    int v0, v1;
    
    // adjacent faces
    std::vector<int> adjacentFaces;
    
    // index
    int index;
};

#endif
