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

void Face::flipOrientation(const Face& f)
{
    for (int i = 0; i < 3; i++) {
        int a = (int)f.vIndices[i];
        int b = (int)f.vIndices[(i+1)%3];
        
        for (int j = 0; j < 3; j++) {
            int nextJ = (j+1)%3;
            int c = (int)vIndices[j];
            int d = (int)vIndices[nextJ];
            
            if (a == c && b == d) {
                std::swap(vIndices[j], vIndices[nextJ]);
                std::swap(uvIndices[j], uvIndices[nextJ]);
                break;
            }
        }
    }
}
