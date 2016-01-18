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
    hashCombine(hash, v.x());
    hashCombine(hash, v.y());
    hashCombine(hash, v.z());
    
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
    snap();
    // TODO: remove isolated vertices and faces, and degenerate faces
    normalize();
    
    std::cout << "EULER CHARACTERISTIC: "
              << (int)(mesh.vertices.size() - mesh.edges.size() + mesh.faces.size())
              << "\nSINGULAR VERTICES: " << (int)singularVertices.size()
              << "\nSINGULAR EDGES: " << (int)singularEdges.size()
              << "\n" << std::endl;
}

void ModelRepairer::collectEdge(std::unordered_map<size_t, size_t>& edgeMap, const int& v1,
                                const int& v2, const size_t& vertexCount, FaceIter f) const
{
    // generate order independent key
    size_t key = (v1*v1 + v2*v2)*vertexCount + v1 + v2;
    if (edgeMap.find(key) == edgeMap.end()) {
        mesh.edges.push_back(Edge(v1, v2, (int)mesh.edges.size()));
        edgeMap[key] = mesh.edges.size();
    }
    
    mesh.edges[edgeMap[key]-1].adjacentFaces.push_back(f);
}

void ModelRepairer::makeMeshElementsUnique() const
{
    std::vector<Vertex> vertices = mesh.vertices; mesh.vertices.clear();
    std::vector<Eigen::Vector3d> uvs = mesh.uvs; mesh.uvs.clear();
    std::vector<Eigen::Vector3d> normals = mesh.normals; mesh.normals.clear();

    std::unordered_map<size_t, size_t> vertexMap;
    std::unordered_map<size_t, size_t> uvMap;
    std::unordered_map<size_t, size_t> normalMap;
    
    size_t vertexCount = 2 * vertices.size();
    std::unordered_map<size_t, size_t> edgeMap;
    
    for (FaceIter f = mesh.faces.begin(); f != mesh.faces.end(); f++) {
        // make v, uv, n unique and reassign face indices
        for (int i = 0; i < 3; i++) {
            // insert unique vertex into map and return index
            size_t v = hash(vertices[f->vIndices[i]].position);
            if (vertexMap.find(v) == vertexMap.end()) {
                mesh.vertices.push_back(vertices[f->vIndices[i]]);
                vertexMap[v] = mesh.vertices.size();
                mesh.vertices[(int)vertexMap[v]-1].index = (int)vertexMap[v]-1;
            }
            f->vIndices[i] = (int)vertexMap[v]-1;
            mesh.vertices[f->vIndices[i]].adjacentFaces.push_back(f);
            
            if (uvs.size() > 0) {
                // insert unique uv into map and return index
                size_t uv = hash(uvs[f->uvIndices[i]]);
                if (uvMap.find(uv) == uvMap.end()) {
                    mesh.uvs.push_back(uvs[f->uvIndices[i]]);
                    uvMap[uv] = mesh.uvs.size();
                }
                f->uvIndices[i] = (int)uvMap[uv]-1;
            }
            
            if (normals.size() > 0) {
                // insert unique normal into map and return index
                size_t n = hash(normals[f->nIndices[i]]);
                if (normalMap.find(n) == normalMap.end()) {
                    mesh.normals.push_back(normals[f->nIndices[i]]);
                    normalMap[n] = mesh.normals.size();
                }
                f->nIndices[i] = (int)normalMap[n]-1;
            }
        }
        
        // collect edges
        collectEdge(edgeMap, f->vIndices[0], f->vIndices[1], vertexCount, f);
        collectEdge(edgeMap, f->vIndices[0], f->vIndices[2], vertexCount, f);
        collectEdge(edgeMap, f->vIndices[1], f->vIndices[2], vertexCount, f);
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
        
        // build face adjacency relation
        for (size_t i = 0; i < e->adjacentFaces.size()-1; i++) {
            for (size_t j = i+1; j < e->adjacentFaces.size(); j++) {
                e->adjacentFaces[i]->adjacentFaces.push_back(e->adjacentFaces[j]);
                e->adjacentFaces[j]->adjacentFaces.push_back(e->adjacentFaces[i]);
            }
        }
    }
}

void ModelRepairer::identifySingularVertices()
{
    for (VertexIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
        
        if (v->adjacentFaces.size() > 0) {
            if (singularVertices.find(v->index) == singularVertices.end()) {
                
                // traverse adjacent faces on vertex
                int valence = 0;
                FaceIter f = v->adjacentFaces[0];
                
                std::stack<FaceIter> stack;
                stack.push(f);
                
                std::unordered_map<int, bool> visitedFaceMap;
                visitedFaceMap[f->index] = true;
                
                while (!stack.empty()) {
                    f = stack.top();
                    stack.pop();
                    valence++;
                    
                    for (size_t i = 0; i < f->adjacentFaces.size(); i++) {
                        FaceIter af = f->adjacentFaces[i];
                        
                        if (af->containsVertex(v->index) &&
                            visitedFaceMap.find(af->index) == visitedFaceMap.end()) {
                            stack.push(af);
                            visitedFaceMap[af->index] = true;
                        }
                    }
                }

                if (valence != (int)v->adjacentFaces.size()) {
                    singularVertices[v->index] = true;
                }
            }
            
        } else {
            v->isIsolated = true;
        }
    }
}

void ModelRepairer::cut() const
{
    for (auto kv : singularVertices) {
        VertexIter v = mesh.vertices.begin() + kv.first;
        // TODO
    }
}

void ModelRepairer::orient() const
{
    // TODO
}

void ModelRepairer::snap() const
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
