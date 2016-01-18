#include "MeshIO.h"
#include "Mesh.h"

class Index {
public:
    Index() {}
    
    Index(int v, int vt, int vn): position(v), uv(vt), normal(vn) {}
    
    bool operator<(const Index& i) const {
        if (position < i.position) return true;
        if (position > i.position) return false;
        if (uv < i.uv) return true;
        if (uv > i.uv) return false;
        if (normal < i.normal) return true;
        if (normal > i.normal) return false;
        
        return false;
    }
    
    int position;
    int uv;
    int normal;
};

Index parseFaceIndex(const std::string& token)
{
    std::stringstream in(token);
    std::string indexString;
    int indices[3] = {-1, -1, -1};
    
    int i = 0;
    while (getline(in, indexString, '/')) {
        std::stringstream ss(indexString);
        ss >> indices[i++];
    }
    
    // decrement since indices in OBJ files are 1-based
    return Index(indices[0]-1,
                 indices[1]-1,
                 indices[2]-1);
}

bool MeshIO::read(std::ifstream& in, Mesh& mesh)
{
    mesh.vertices.clear();
    mesh.uvs.clear();
    mesh.normals.clear();
    mesh.edges.clear();
    mesh.faces.clear();
    
    // parse obj format
    std::string line;
    while(getline(in, line)) {
        std::stringstream ss(line);
        std::string token;
        
        ss >> token;
        
        if (token == "v") {
            double x, y, z;
            ss >> x >> y >> z;
            
            mesh.vertices.push_back(Vertex(Eigen::Vector3d(x, y, z), (int)mesh.vertices.size()));
            
        } else if (token == "vt") {
            double u, v;
            ss >> u >> v;
            
            mesh.uvs.push_back(Eigen::Vector3d(u,v,0));
            
        } else if (token == "vn") {
            double x, y, z;
            ss >> x >> y >> z;
            
            mesh.normals.push_back(Eigen::Vector3d(x, y, z));
            
        } else if (token == "f") {
            int f = (int)mesh.faces.size();
            mesh.faces.push_back(Face(f));
            
            // assumes face is a triangle
            int i = 0;
            while (ss >> token && i < 3) {
                Index index = parseFaceIndex(token);
                mesh.faces[f].vIndices[i] = index.position;
                mesh.faces[f].uvIndices[i] = index.uv;
                mesh.faces[f].nIndices[i] = index.normal;
                i++;
            }
        }
    }
    
    return true;
}
