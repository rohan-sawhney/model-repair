#include "Edge.h"

Edge::Edge(const int& i0, const int& i1, const int& index0):
v0(i0),
v1(i1),
index(index0),
isBoundary(false)
{
    if (v1 < v0) {
        std::swap(v0, v1);
    }
}

bool Edge::containsVertex(const int& v)
{
    return v == v0 || v == v1;
}
