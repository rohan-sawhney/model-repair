#include "Vertex.h"

Vertex::Vertex(const Eigen::Vector3d& position0, const int& index0):
position(position0),
index(index0)
{
    
}

void Vertex::removeFaceFromAdjacencyList(const int& f)
{
    for (size_t i = 0; i < adjacentFaces.size(); i++) {
        if (adjacentFaces[i] == f) {
            adjacentFaces.erase(adjacentFaces.begin() + i);
            break;
        }
    }
}