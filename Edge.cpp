#include "Edge.h"

Edge::Edge(const int& i0, const int& i1, const int& index0):
v0(i0),
v1(i1),
index(index0)
{
    if (v1 < v0) std::swap(v0, v1);
}

bool Edge::containsVertex(const int& v) const
{
    return v == v0 || v == v1;
}

void Edge::removeFaceFromAdjacencyList(const int& f)
{
    for (size_t i = 0; i < adjacentFaces.size(); i++) {
        if (adjacentFaces[i] == f) {
            adjacentFaces.erase(adjacentFaces.begin() + i);
            break;
        }
    }
}
