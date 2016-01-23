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
mesh(mesh0)
{
    
}

void ModelRepairer::repair()
{
    makeMeshElementsUnique();
    identifySingularEdges();
    identifySingularVertices();
    cut();
    orient();
    stitch();
    // TODO: remove isolated vertices & faces
    // TODO: submesh
    // TODO: correct closed object orientation
    // TODO: calculate normals
    normalize();
    
    std::cout << "EULER CHARACTERISTIC: "
              << (int)(mesh.vertices.size() - mesh.edges.size() + mesh.faces.size())
              << "\nSINGULAR VERTICES: " << (int)singularVertices.size()
              << "\nSINGULAR EDGES: " << (int)singularEdges.size()
              << "\nVERTICES: " << (int)mesh.vertices.size()
              << "\nEDGE: " << (int)mesh.edges.size()
              << "\nFACES: " << (int)mesh.faces.size()
              << "\n" << std::endl;
}

int ModelRepairer::collectEdge(std::unordered_map<size_t, size_t>& edgeMap, const int& v1,
                               const int& v2, const size_t& vertexCount) const
{
    // generate order independent key
    size_t key = (v1*v1 + v2*v2)*vertexCount + v1 + v2;
    
    if (edgeMap.find(key) == edgeMap.end()) {
        mesh.edges.push_back(Edge(v1, v2, (int)mesh.edges.size()));
        edgeMap[key] = mesh.edges.size();
    }
    
    return (int)edgeMap[key]-1;
}

void ModelRepairer::makeMeshElementsUnique() const
{
    std::vector<Vertex> vertices = mesh.vertices; mesh.vertices.clear();
    std::vector<Eigen::Vector3d> uvs = mesh.uvs; mesh.uvs.clear();
    std::vector<Eigen::Vector3d> normals = mesh.normals; mesh.normals.clear();
    std::vector<Face> faces = mesh.faces; mesh.faces.clear();
    
    std::unordered_map<size_t, size_t> vertexMap;
    std::unordered_map<size_t, size_t> uvMap;
    std::unordered_map<size_t, size_t> normalMap;
    std::unordered_map<size_t, bool> faceMap;
    
    size_t v = 2 * vertices.size();
    std::unordered_map<size_t, size_t> edgeMap;

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
                int e1 = collectEdge(edgeMap, cf->vIndices[0], cf->vIndices[1], v);
                cf->incidentEdges[0] = e1;
                mesh.edges[e1].adjacentFaces.push_back(fIdx);
                
                int e2 = collectEdge(edgeMap, cf->vIndices[0], cf->vIndices[2], v);
                cf->incidentEdges[1] = e2;
                mesh.edges[e2].adjacentFaces.push_back(fIdx);
                
                int e3 = collectEdge(edgeMap, cf->vIndices[1], cf->vIndices[2], v);
                cf->incidentEdges[2] = e3;
                mesh.edges[e3].adjacentFaces.push_back(fIdx);
                
                faceMap[fHash] = true;
            }
        }
    }
}

void ModelRepairer::identifySingularEdges()
{
    for (EdgeIter e = mesh.edges.begin(); e != mesh.edges.end(); e++) {
        if (e->adjacentFaces.size() == 1) {
            e->isBoundary = true;
            mesh.vertices[e->v0].isBoundary = true;
            mesh.vertices[e->v1].isBoundary = true;
        
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
            v->isIsolated = true;
            
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
                    
                    if (!e->isBoundary && e->containsVertex(v->index)) {
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

void ModelRepairer::cut() const
{
    for (auto kv : singularVertices) {
        // assign components
        int components = 0;
        const size_t faceCount = mesh.vertices[kv.first].adjacentFaces.size();
        std::unordered_map<int, bool> visitedFaceMap;

        while (visitedFaceMap.size() != faceCount) {
            // find a face that has not been visited
            std::stack<Face *> stack;
            std::vector<Face *> componentFaces;

            for (int i = 0; i < faceCount; i++) {
                Face *f = &mesh.faces[mesh.vertices[kv.first].adjacentFaces[i]];
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
                       !e->isBoundary && e->containsVertex(mesh.vertices[kv.first].index)) {
                        
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

            components++;
            if (components > 1) {
                // multiply vertices
                int vNew = (int)mesh.vertices.size();
                mesh.vertices.push_back(Vertex(mesh.vertices[kv.first].position, vNew));
                
                for (size_t i = 0; i < componentFaces.size(); i++) {
                    componentFaces[i]->updateVertexIndex(mesh.vertices[kv.first].index, vNew);
                }
            }
        }
    }
    
    // NOTE: mesh edges and adjaceny relations are invalid at this point
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
            
            for (int i = 0; i < 3; i++) {
                Edge *e = &mesh.edges[f->incidentEdges[i]];
                
                if (singularEdges.find(e->index) == singularEdges.end() && !e->isBoundary) {
                    Face *af = &mesh.faces[e->adjacentFaces[0]];
                    if (f == af) af = &mesh.faces[e->adjacentFaces[1]];
                    
                    // push faces not yet visited
                    if (visitedFaceMap.find(af->index) == visitedFaceMap.end()) {
                        af->flipOrientation(*f);
                        
                        stack.push(af);
                        visitedFaceMap[af->index] = true;
                    }
                }
            }
        }
    }
    
    // NOTE: mesh normals are invalid at this point
}

void ModelRepairer::stitch() const
{
    // TODO
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
