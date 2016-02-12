#include "ModelRepairer.h"
#include "Mesh.h"
#include <stack>
#define EPSILON 1e-6

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
    flipClosedMeshOutwards();
    recalculateNormals();
    normalize();
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
    std::vector<Face> faces = mesh.faces; mesh.faces.clear();
    
    std::unordered_map<size_t, size_t> vertexMap;
    std::unordered_map<size_t, size_t> uvMap;
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
        }
        
        // add face to mesh list if it is not degenerate and not a duplicate
        const Eigen::Vector3d& v1(mesh.vertices[f->vIndices[0]].position);
        const Eigen::Vector3d& v2(mesh.vertices[f->vIndices[1]].position);
        const Eigen::Vector3d& v3(mesh.vertices[f->vIndices[2]].position);
        Eigen::Vector3d normal = (v2-v1).cross((v3-v1));
        
        if (normal.norm() != 0.0) {
            Eigen::Vector3d centroid = (v1 + v2 + v3) / 3.0;
            size_t fHash = hash(centroid); // hash of face centroid
            
            if (faceMap.find(fHash) == faceMap.end()) {
                int fIdx = (int)mesh.faces.size();
                mesh.faces.push_back(Face(f->vIndices, f->uvIndices, f->nIndices,
                                          normal.normalized(), centroid, fIdx));
                Face *cf = &mesh.faces[fIdx];
                
                // build vertex face adjacency
                mesh.vertices[cf->vIndices[0]].adjacentFaces.push_back(fIdx);
                mesh.vertices[cf->vIndices[1]].adjacentFaces.push_back(fIdx);
                mesh.vertices[cf->vIndices[2]].adjacentFaces.push_back(fIdx);
                
                // build edge face adjaceny
                collectEdge(cf->incidentEdges[0], cf->vIndices[0], cf->vIndices[1]);
                mesh.edges[cf->incidentEdges[0]].adjacentFaces.push_back(fIdx);
                
                collectEdge(cf->incidentEdges[1], cf->vIndices[1], cf->vIndices[2]);
                mesh.edges[cf->incidentEdges[1]].adjacentFaces.push_back(fIdx);
                
                collectEdge(cf->incidentEdges[2], cf->vIndices[2], cf->vIndices[0]);
                mesh.edges[cf->incidentEdges[2]].adjacentFaces.push_back(fIdx);
                
                faceMap[fHash] = true;
            }
        }
    }
}

void ModelRepairer::identifySingularEdges()
{
    for (EdgeIter e = mesh.edges.begin(); e != mesh.edges.end(); e++) {
        if (e->adjacentFaces.size() > 2) {
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
                    
                    if (!e->isBoundary() && e->containsVertex(v->index)) {
                        
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

void ModelRepairer::updateEdges()
{
    hashFactor = 2 * mesh.vertices.size();
    edgeMap.clear();
    mesh.edges.clear();
    
    // collect edges
    for (FaceIter f = mesh.faces.begin(); f != mesh.faces.end(); f++) {
        
        // build edge face adjaceny
        collectEdge(f->incidentEdges[0], f->vIndices[0], f->vIndices[1]);
        mesh.edges[f->incidentEdges[0]].adjacentFaces.push_back(f->index);
        
        collectEdge(f->incidentEdges[1], f->vIndices[1], f->vIndices[2]);
        mesh.edges[f->incidentEdges[1]].adjacentFaces.push_back(f->index);
        
        collectEdge(f->incidentEdges[2], f->vIndices[2], f->vIndices[0]);
        mesh.edges[f->incidentEdges[2]].adjacentFaces.push_back(f->index);
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
                            !e->isBoundary() && e->containsVertex(vIdx)) {
                            
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
    
    updateEdges();
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
                
                if (!e->isBoundary()) {
                    
                    Face *af = &mesh.faces[e->adjacentFaces[0]];
                    if (f == af) af = &mesh.faces[e->adjacentFaces[1]];
                    
                    // push faces not yet visited
                    if (visitedFaceMap.find(af->index) == visitedFaceMap.end()) {
                        af->flipOrientation(*f);
                        
                        stack.push(af);
                        visitedFaceMap[af->index] = true;
                    }
                    
                } else {
                    mesh.closed = false;
                }
            }
        }
    }
}

bool intersectFaceRay(const Mesh& mesh, const Face& f,
                      const Eigen::Vector3d& origin, const Eigen::Vector3d& direction)
{
    // Möller–Trumbore intersection algorithm
    const Eigen::Vector3d& p1(mesh.vertices[f.vIndices[0]].position);
    const Eigen::Vector3d& p2(mesh.vertices[f.vIndices[1]].position);
    const Eigen::Vector3d& p3(mesh.vertices[f.vIndices[2]].position);
    
    Eigen::Vector3d e1 = p2 - p1;
    Eigen::Vector3d e2 = p3 - p1;
    Eigen::Vector3d n = direction.cross(e2);
    
    double det = e1.dot(n);
    
    // ray does not lie in the plane
    if (fabs(det) < EPSILON) {
        return false;
    }
    
    double invDet = 1.0 / det;
    Eigen::Vector3d t = origin - p1;
    double u = t.dot(n) * invDet;
    
    // ray lies outside triangle
    if (u < 0.0 || u > 1.0) {
        return false;
    }
    
    Eigen::Vector3d q = t.cross(e1);
    double v = direction.dot(q) * invDet;
    
    // ray lies outside the triangle
    if (v < 0.0 || v + u > 1.0) {
        return false;
    }
    
    // check for intersection
    if (e2.dot(q) * invDet > 0) {
        return true;
    }
    
    // no hit
    return false;
}

void ModelRepairer::flipClosedMeshOutwards() const
{
    if (mesh.closed) {
        for (size_t i = 0; i < components.size(); i++) {
            
            Eigen::Vector3d n = components[i][0]->normal;
            Eigen::Vector3d p = components[i][0]->centroid + EPSILON * n;
            
            // count ray face intersections
            int intersections = 0;
            for (size_t j = 0; j < components[i].size(); j++) {
                if (intersectFaceRay(mesh, *components[i][j], p, -n)) intersections++;
            }
            
            // check if p is inside mesh
            if (intersections % 2 == 1) {
                for (size_t j = 0; j < components[i].size(); j++) components[i][j]->flip(0, 1);
            }
        }
    }
}

bool hardAngle(const int& f1, const int& f2, const Mesh& mesh)
{
    const Eigen::Vector3d& n1(mesh.faces[f1].normal);
    const Eigen::Vector3d& n2(mesh.faces[f2].normal);
    
    return fabs(n1.dot(n2)) < EPSILON;
}

double pivotAngle(const int& vp, const int& fIdx, const Mesh& mesh)
{
    const Face& f(mesh.faces[fIdx]);
    
    int v0 = vp-1;
    if (v0 < 0) v0 = 2;
    
    int v1 = vp+1;
    if (v1 > 2) v1 = 0;
    
    Eigen::Vector3d e1 = mesh.vertices[f.vIndices[v0]].position - mesh.vertices[f.vIndices[vp]].position;
    Eigen::Vector3d e2 = mesh.vertices[f.vIndices[v1]].position - mesh.vertices[f.vIndices[vp]].position;
    
    double angle = e1.dot(e2) / sqrt(e1.dot(e1) * e2.dot(e2));
    if (angle < -1.0) angle = -1.0;
    else if (angle >  1.0) angle = 1.0;
    return acos(angle);
}

void ModelRepairer::recalculateNormals() const
{
    mesh.normals.clear();
    std::unordered_map<size_t, size_t> normalMap;
    
    for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
        if (isolatedVertices.find(v->index) == isolatedVertices.end()) {
            // bucket faces based on angles between them
            std::unordered_map<int, std::vector<int>> faceMap;
            for (size_t i = 0; i < v->adjacentFaces.size(); i++) {
                int j = 0;
                for (auto kv : faceMap) {
                    if (hardAngle(v->adjacentFaces[i], kv.second[0], mesh)) j++;
                }
                faceMap[j].push_back(v->adjacentFaces[i]);
            }
            
            // calculate vertex normal with faces from each bucket
            for (auto kv : faceMap) {
                Eigen::Vector3d n = Eigen::Vector3d::Zero();
                for (size_t i = 0; i < kv.second.size(); i++) {
                    int index = mesh.faces[kv.second[i]].vertexIndex(v->index);
                    if (index != -1) n += pivotAngle(index, kv.second[i], mesh) *
                        mesh.faces[kv.second[i]].normal;
                }
                n.normalize();
                
                // insert unique normal into map and return index
                size_t nHash = hash(n);
                if (normalMap.find(nHash) == normalMap.end()) {
                    mesh.normals.push_back(n);
                    normalMap[nHash] = mesh.normals.size();
                }
                
                // set normal index for bucket faces
                for (size_t i = 0; i < kv.second.size(); i++) {
                    int index = mesh.faces[kv.second[i]].vertexIndex(v->index);
                    if (index != -1) mesh.faces[kv.second[i]].nIndices[index] = (int)normalMap[nHash]-1;
                }
            }
        }
    }
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
