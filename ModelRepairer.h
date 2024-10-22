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
    // centers mesh about origin and rescales to unit radius
    void normalize() const;
    
    // makes v, uv, n unique and reassigns face indices
    void makeMeshElementsUnique();
    
    // collects mesh edges
    bool collectEdge(int& e, const int& v1, const int& v2);

    // identifies singular edges
    void identifySingularEdges();
    
    // identifies singular vertices
    void identifySingularVertices();
    
    // updates edges
    void updateEdges();
    
    // cutting operation
    void cut();
    
    // orient component faces consistently
    void orient();
        
    // flips closed mesh outwards
    void flipClosedMeshOutwards() const;
    
    // calculates mesh normals
    void recalculateNormals() const;
    
    // member variable
    Mesh& mesh;
    size_t hashFactor;
    std::unordered_map<size_t, size_t> edgeMap;
    std::unordered_map<int, bool> singularVertices;
    std::unordered_map<int, bool> singularEdges;
    std::unordered_map<int, bool> isolatedVertices;
    std::vector<std::vector<Face *>> components;
};

#endif 
