#include "Face.h"

Face::Face(const int& index0):
index(index0),
isIsolated(false)
{
    
}

Face::Face(const Eigen::Vector3i& vIndices0, const Eigen::Vector3i& uvIndices0,
           const Eigen::Vector3i& nIndices0, const Eigen::Vector3d& normal0,
           const int& index0):
vIndices(vIndices0),
uvIndices(uvIndices0),
nIndices(nIndices0),
normal(normal0),
index(index0),
isIsolated(false)
{
    
}

void Face::updateVertexIndex(const int& vOld, const int& vNew)
{
    for (int i = 0; i < 3; i++) {
        if (vIndices[i] == vOld) {
            vIndices[i] = vNew;
            break;
        }
    }
}
