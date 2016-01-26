#ifndef VERTEX_H
#define VERTEX_H

#include "Types.h"

class Vertex {
public:
    // constructor
    Vertex(const Eigen::Vector3d& position0, const int& index0);
    
    // removes face index from face adjacency list
    void removeFaceFromAdjacencyList(const int& f);
    
    // position
    Eigen::Vector3d position;
    
    // adjacent faces
    std::vector<int> adjacentFaces;
    
    // index
    int index;
};

#endif
