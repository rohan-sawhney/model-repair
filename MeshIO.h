#ifndef MESH_IO_H
#define MESH_IO_H

#include "Types.h"
#include <fstream>

class MeshIO {
public:
    // reads data from obj file
    static bool read(std::ifstream& in, Mesh& mesh);
};

#endif
