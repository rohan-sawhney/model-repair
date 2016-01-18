#include "Mesh.h"
#include "MeshIO.h"
#include "ModelRepairer.h"

Mesh::Mesh()
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

