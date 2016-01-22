#ifndef FACE_H
#define FACE_H

#include "Types.h"

class Face {
public:
    // constructor
    Face(const int& index0);
    
    // constructor
    Face(const Eigen::Vector3i& vIndices0, const Eigen::Vector3i& uvIndices0,
         const Eigen::Vector3i& nIndices0, const Eigen::Vector3d& normal0,
         const int& index0);
    
    // updates vertex index
    void updateVertexIndex(const int& vOld, const int& vNew);
    
    // flip orientation
    void flipOrientation(const Face& f);
    
    // vertex indices
    Eigen::Vector3i vIndices;
    
    // uv indices
    Eigen::Vector3i uvIndices;
    
    // normal indices
    Eigen::Vector3i nIndices;
    
    // normal
    Eigen::Vector3d normal;
    
    // incident edges
    Eigen::Vector3i incidentEdges;
    
    // index
    int index;
    
    // flag for isolated face
    bool isIsolated;
};

#endif
