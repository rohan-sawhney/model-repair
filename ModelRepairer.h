#ifndef MODEL_REPAIRER_H
#define MODEL_REPAIRER_H

#include "Types.h"

class ModelRepairer {
public:
    // constructor
    ModelRepairer(Mesh& mesh0);
    
    // repairs mesh 
    void repair();
    
private:
    // makes v, uv, n unique and reassigns face indices
    void makeMeshElementsUnique() const;
    
    // collects mesh edges
    int collectEdge(std::unordered_map<size_t, size_t>& edgeMap, const int& v1,
                    const int& v2, const size_t& vertexCount) const;

    // identifies singular edges
    void identifySingularEdges();
    
    // identifies singular vertices
    void identifySingularVertices();
    
    // cutting operation
    void cut() const;
    
    // orient consistently
    void orient() const;
    
    // snapping operation
    void snap() const;
    
    // centers mesh about origin and rescales to unit radius
    void normalize() const;
    
    // member variable
    Mesh& mesh;
    std::unordered_map<int, bool> singularEdges;
    std::unordered_map<int, bool> singularVertices;
};

#endif 
