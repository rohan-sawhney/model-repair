#include "ModelRepairer.h"
#include "Mesh.h"
#include <stack>

template <typename T>
inline void hashCombine(std::size_t& seed, const T& v)
{
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

size_t hash(const Eigen::Vector3d& v)
{
    size_t hash = 0;
    hashCombine(hash, v[0]);
    hashCombine(hash, v[1]);
    hashCombine(hash, v[2]);
    
    return hash;
}

ModelRepairer::ModelRepairer(Mesh& mesh0):
mesh(mesh0),
hashFactor(0)
{
    
}

void ModelRepairer::repair()
{
    makeMeshElementsUnique();
    identifySingularEdges();
    identifySingularVertices();
    cut();
    orient();
    // TODO: check correctness + boundary edge count
    stitch();
    // TODO: remove isolated vertices, edges & faces
    // TODO: correct closed object orientation
    // TODO: calculate normals
    normalize();
    
    std::cout << "EULER CHARACTERISTIC: "
              << (int)(mesh.vertices.size() - mesh.edges.size() + mesh.faces.size())
              << "\nSINGULAR VERTICES: " << (int)singularVertices.size()
              << "\nSINGULAR EDGES: " << (int)singularEdges.size()
              << "\nBOUNDARY EDGES: " << (int)boundaryEdges.size()
              << "\nISOLATED VERTICES: " << (int)isolatedVertices.size()
              << "\nISOLATED EDGES: " << (int)isolatedEdges.size()
              << "\nISOLATED FACES: " << (int)isolatedFaces.size()
              << "\nVERTICES: " << (int)mesh.vertices.size()
              << "\nEDGE: " << (int)mesh.edges.size()
              << "\nFACES: " << (int)mesh.faces.size()
              << "\n" << std::endl;
}

bool ModelRepairer::collectEdge(int& e, const int& v1, const int& v2)
{
    // generate order independent key
    size_t key = (v1*v1 + v2*v2)*hashFactor + v1 + v2;
    
    bool newEdge = false;
    if (edgeMap.find(key) == edgeMap.end()) {
        mesh.edges.push_back(Edge(v1, v2, (int)mesh.edges.size()));
        edgeMap[key] = mesh.edges.size();
        newEdge = true;
    }

    e = (int)edgeMap[key]-1;
    return newEdge;
}

void ModelRepairer::makeMeshElementsUnique()
{
    std::vector<Vertex> vertices = mesh.vertices; mesh.vertices.clear();
    std::vector<Eigen::Vector3d> uvs = mesh.uvs; mesh.uvs.clear();
    std::vector<Eigen::Vector3d> normals = mesh.normals; mesh.normals.clear();
    std::vector<Face> faces = mesh.faces; mesh.faces.clear();
    
    std::unordered_map<size_t, size_t> vertexMap;
    std::unordered_map<size_t, size_t> uvMap;
    std::unordered_map<size_t, size_t> normalMap;
    std::unordered_map<size_t, bool> faceMap;
    
    hashFactor = 2 * vertices.size();
    
    for (FaceIter f = faces.begin(); f != faces.end(); f++) {
        
        // make v, uv, n unique and reassign face indices
        for (int i = 0; i < 3; i++) {
            
            // insert unique vertex into map and return index
            size_t vHash = hash(vertices[f->vIndices[i]].position);
            if (vertexMap.find(vHash) == vertexMap.end()) {
                mesh.vertices.push_back(vertices[f->vIndices[i]]);
                vertexMap[vHash] = mesh.vertices.size();
                mesh.vertices[(int)vertexMap[vHash]-1].index = (int)vertexMap[vHash]-1;
            }
            f->vIndices[i] = (int)vertexMap[vHash]-1;
            
            if (uvs.size() > 0) {
                // insert unique uv into map and return index
                size_t uvHash = hash(uvs[f->uvIndices[i]]);
                if (uvMap.find(uvHash) == uvMap.end()) {
                    mesh.uvs.push_back(uvs[f->uvIndices[i]]);
                    uvMap[uvHash] = mesh.uvs.size();
                }
                f->uvIndices[i] = (int)uvMap[uvHash]-1;
            }
            
            if (normals.size() > 0) {
                // insert unique normal into map and return index
                size_t nHash = hash(normals[f->nIndices[i]]);
                if (normalMap.find(nHash) == normalMap.end()) {
                    mesh.normals.push_back(normals[f->nIndices[i]]);
                    normalMap[nHash] = mesh.normals.size();
                }
                f->nIndices[i] = (int)normalMap[nHash]-1;
            }
        }
        
        // add face to mesh list if it is not degenerate and not a duplicate
        const Eigen::Vector3d& v1(mesh.vertices[f->vIndices[0]].position);
        const Eigen::Vector3d& v2(mesh.vertices[f->vIndices[1]].position);
        const Eigen::Vector3d& v3(mesh.vertices[f->vIndices[2]].position);
        Eigen::Vector3d normal = (v2-v1).cross((v3-v1));
        
        if (normal.norm() != 0.0) {
            size_t fHash = hash((v1 + v2 + v3) / 3.0); // hash of face centroid
            
            if (faceMap.find(fHash) == faceMap.end()) {
                int fIdx = (int)mesh.faces.size();
                mesh.faces.push_back(Face(f->vIndices, f->uvIndices, f->nIndices, normal, fIdx));
                Face *cf = &mesh.faces[fIdx];
                
                // build vertex face adjacency
                mesh.vertices[cf->vIndices[0]].adjacentFaces.push_back(fIdx);
                mesh.vertices[cf->vIndices[1]].adjacentFaces.push_back(fIdx);
                mesh.vertices[cf->vIndices[2]].adjacentFaces.push_back(fIdx);
                
                // build edge face adjaceny
                collectEdge(cf->incidentEdges[0], cf->vIndices[0], cf->vIndices[1]);
                mesh.edges[cf->incidentEdges[0]].adjacentFaces.push_back(fIdx);
                
                collectEdge(cf->incidentEdges[1], cf->vIndices[0], cf->vIndices[2]);
                mesh.edges[cf->incidentEdges[1]].adjacentFaces.push_back(fIdx);
                
                collectEdge(cf->incidentEdges[2], cf->vIndices[1], cf->vIndices[2]);
                mesh.edges[cf->incidentEdges[2]].adjacentFaces.push_back(fIdx);
                
                faceMap[fHash] = true;
            }
        }
    }
}

void ModelRepairer::identifySingularEdges()
{
    for (EdgeIter e = mesh.edges.begin(); e != mesh.edges.end(); e++) {
        if (e->adjacentFaces.size() == 1) {
            boundaryEdges[e->index] = true;
        
        } else if (e->adjacentFaces.size() > 2) {
            singularEdges[e->index] = true;
            singularVertices[e->v0] = true;
            singularVertices[e->v1] = true;
        }
    }
}

void ModelRepairer::identifySingularVertices()
{
    for (VertexIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
        if (v->adjacentFaces.size() == 0) {
            isolatedVertices[v->index] = true;
            
        } else if (singularVertices.find(v->index) == singularVertices.end()) {
            // traverse adjacent faces on vertex
            int valence = 0;
            Face *f = &mesh.faces[v->adjacentFaces[0]];
            
            // insert arbitary face incident on vertex
            std::stack<Face *> stack;
            stack.push(f);
                
            std::unordered_map<int, bool> visitedFaceMap;
            visitedFaceMap[f->index] = true;
                
            while (!stack.empty()) {
                f = stack.top();
                stack.pop();
                valence++;
                    
                for (int i = 0; i < 3; i++) {
                    Edge *e = &mesh.edges[f->incidentEdges[i]];
                    
                    if (boundaryEdges.find(e->index) == boundaryEdges.end() &&
                        e->containsVertex(v->index)) {
                        
                        Face *af = &mesh.faces[e->adjacentFaces[0]];
                        if (f == af) af = &mesh.faces[e->adjacentFaces[1]];
                        
                        // push faces not yet visited
                        if (visitedFaceMap.find(af->index) == visitedFaceMap.end()) {
                            stack.push(af);
                            visitedFaceMap[af->index] = true;
                        }
                    }
                }
            }
        
            if (valence != (int)v->adjacentFaces.size()) {
                singularVertices[v->index] = true;
            }
        }
    }
}

bool ModelRepairer::shareSingularEdge(const Face& f1, const Face& f2)
{
    for (int i = 0; i < 3; i++) {
        if (singularEdges.find(f1.incidentEdges[i]) == singularEdges.end()) continue;
        
        for (int j = 0; j < 3; j++) {
            if (singularEdges.find(f2.incidentEdges[j]) == singularEdges.end()) continue;
            else if (f1.incidentEdges[i] == f2.incidentEdges[j]) return true;
        }
    }
    
    return false;
}

void ModelRepairer::updateAdjacencyLists()
{    
    for (auto kv : singularEdges) {
        if (kv.second) {
            // create a local copy of adjacent faces
            std::vector<int> adjacentFaces = mesh.edges[kv.first].adjacentFaces;
            
            // update edge face adjacency relations
            for (size_t i = 0; i < adjacentFaces.size(); i++) {
                Face& f(mesh.faces[adjacentFaces[i]]);
                
                // check if edge has been multiplied
                int eIdx = f.edgeIndex(kv.first);
                if (eIdx != -1) {
                    
                    int v1 = 0;
                    int v2 = 1;
                    if (eIdx == 1) v2 = 2;
                    else if (eIdx == 2) {
                        v1 = 1;
                        v2 = 2;
                    }
                
                    // mark new edges as boundary edges
                    const int index = f.incidentEdges[eIdx];
                    if (collectEdge(f.incidentEdges[eIdx], f.vIndices[v1], f.vIndices[v2])) {
                        boundaryEdges[f.incidentEdges[eIdx]] = true;
                    }
                    
                    // update face edge adjacency lists
                    if (index != f.incidentEdges[eIdx]) {
                        mesh.edges[kv.first].removeFaceFromAdjacencyList(f.index);
                        mesh.edges[f.incidentEdges[eIdx]].adjacentFaces.push_back(f.index);
                    }
                }
            }
            
            // update edge type
            if (mesh.edges[kv.first].adjacentFaces.size() == 0) isolatedEdges[kv.first] = true;
            else if (mesh.edges[kv.first].adjacentFaces.size() == 1) boundaryEdges[kv.first] = true;
        }
    }
}

void ModelRepairer::cut() 
{
    for (auto kv : singularVertices) {
        if (kv.second) {
            const int& vIdx(kv.first);
            std::vector<int> adjacentFaces = mesh.vertices[vIdx].adjacentFaces;
            
            // assign components
            int components = 0;
            const size_t faceCount = adjacentFaces.size();
            std::unordered_map<int, bool> visitedFaceMap;
            
            while (visitedFaceMap.size() != faceCount) {
                // find a face that has not been visited
                std::stack<Face *> stack;
                std::vector<Face *> componentFaces;
                
                for (int i = 0; i < faceCount; i++) {
                    Face *f = &mesh.faces[adjacentFaces[i]];
                    if (visitedFaceMap.find(f->index) == visitedFaceMap.end()) {
                        stack.push(f);
                        visitedFaceMap[f->index] = true;
                        break;
                    }
                }
                
                // isolate a component
                while (!stack.empty()) {
                    Face *f = stack.top();
                    stack.pop();
                    componentFaces.push_back(f);
                    
                    for (int i = 0; i < 3; i++) {
                        Edge *e = &mesh.edges[f->incidentEdges[i]];
                        
                        if (singularEdges.find(e->index) == singularEdges.end() &&
                            boundaryEdges.find(e->index) == boundaryEdges.end() &&
                            e->containsVertex(vIdx)) {
                            
                            Face *af = &mesh.faces[e->adjacentFaces[0]];
                            if (f == af) af = &mesh.faces[e->adjacentFaces[1]];
                            
                            // push faces not yet visited
                            if (visitedFaceMap.find(af->index) == visitedFaceMap.end()) {
                                stack.push(af);
                                visitedFaceMap[af->index] = true;
                            }
                        }
                    }
                }
                
                // check if first & last component faces share a singular edge
                if (componentFaces.size() > 1 &&
                    shareSingularEdge(*componentFaces[0], *componentFaces[componentFaces.size()-1])) {
                    visitedFaceMap.erase(componentFaces[componentFaces.size()-1]->index);
                    componentFaces.pop_back();
                }
                
                components++;
                if (components > 1) {
                    // multiply vertices
                    int vNew = (int)mesh.vertices.size();
                    mesh.vertices.push_back(Vertex(mesh.vertices[vIdx].position, vNew));

                    for (size_t i = 0; i < componentFaces.size(); i++) {
                        componentFaces[i]->updateVertexIndex(vIdx, vNew);
                        
                        // update vertex face adjacency relation
                        mesh.vertices[vIdx].removeFaceFromAdjacencyList(componentFaces[i]->index);
                        mesh.vertices[mesh.vertices.size()-1].adjacentFaces.push_back(componentFaces[i]->index);
                    }
                }
            }
        }
    }
    
    // mesh edges and edge face adjaceny relations are invalid at this point
    updateAdjacencyLists();
}

void ModelRepairer::orient()
{
    // build spanning tree
    const size_t faceCount = mesh.faces.size();
    std::unordered_map<int, bool> visitedFaceMap;
    
    while (visitedFaceMap.size() != faceCount) {
        
        // find a face that has not been visited
        std::stack<Face *> stack;
        for (int i = 0; i < faceCount; i++) {
            
            Face *f = &mesh.faces[i];
            if (visitedFaceMap.find(f->index) == visitedFaceMap.end()) {
                stack.push(f);
                visitedFaceMap[f->index] = true;
                components.push_back(std::vector<Face *>());
                break;
            }
        }
        
        // flip orientation of neighboring faces if inconsistent
        while (!stack.empty()) {
            Face *f = stack.top();
            stack.pop();
            components[components.size()-1].push_back(f);
            
            int boundaryEdgeCount = 0;
            for (int i = 0; i < 3; i++) {
                Edge *e = &mesh.edges[f->incidentEdges[i]];
                
                if (boundaryEdges.find(e->index) == boundaryEdges.end()) {
                    
                    Face *af = &mesh.faces[e->adjacentFaces[0]];
                    if (f == af) af = &mesh.faces[e->adjacentFaces[1]];
                    
                    // push faces not yet visited
                    if (visitedFaceMap.find(af->index) == visitedFaceMap.end()) {
                        af->flipOrientation(*f);
                        
                        stack.push(af);
                        visitedFaceMap[af->index] = true;
                    }
                    
                } else {
                    boundaryEdgeCount++;
                }
            }
            
            if (boundaryEdgeCount == 3) isolatedFaces[f->index] = true;
        }
    }
    
    // mesh normals are invalid at this point
}

void ModelRepairer::stitch() const
{
    // TODO: snapping, join components & flip faces if necessary
}

void ModelRepairer::normalize() const
{
    // compute center of mass
    Eigen::Vector3d cm = Eigen::Vector3d::Zero();
    for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
        cm += v->position;
    }
    cm /= (double)mesh.vertices.size();
    
    // translate to origin and determine radius
    double rMax = 0;
    for (VertexIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
        v->position -= cm;
        rMax = std::max(rMax, v->position.norm());
    }
    
    // rescale to unit sphere
    for (VertexIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
        v->position /= rMax;
    }
}
