#include "Mesh.h"
#include "MeshIO.h"
#include "ModelRepairer.h"

Mesh::Mesh():
closed(true)
{
    
}

Mesh::Mesh(const Mesh& mesh)
{
    *this = mesh;
}

bool Mesh::read(const std::string& fileName)
{
    std::ifstream in(fileName.c_str());

    if (!in.is_open()) {
        std::cerr << "Error: Could not open file for reading" << std::endl;
        return false;
    }
    
    bool readSuccessful = false;
    if ((readSuccessful = MeshIO::read(in, *this))) {
        ModelRepairer modelRepairer(*this);
        modelRepairer.repair();
    }
    
    return readSuccessful;
}

bool Mesh::write(const std::string& fileName) const
{
    std::ofstream out(fileName.c_str());
    
    if (!out.is_open()) {
        std::cerr << "Error: Could not open file for writing" << std::endl;
        return false;
    }
    
    bool writeSuccessful = false;
    if ((writeSuccessful = MeshIO::write(out, *this))) {};
    
    return writeSuccessful;
}

void Mesh::reset()
{
    vertices.clear();
    uvs.clear();
    normals.clear();
    edges.clear();
    faces.clear();
    closed = true;
}

