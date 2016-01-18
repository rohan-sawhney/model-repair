#include "Vertex.h"

Vertex::Vertex(const Eigen::Vector3d& position0, const int& index0):
position(position0),
index(index0),
isSingular(false),
isBoundary(false),
isIsolated(false)
{
    
}