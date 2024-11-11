#pragma once

#include <stdint.h>

#ifdef WIN32
    #define CIGL_API extern "C" __declspec(dllexport)
#else
    #define CIGL_API extern "C"
#endif

struct cigl_CylinderInfo {
    float posX, posY, posZ;
    float dirX, dirY, dirZ;
    float radius;
    float halfHeight;
};

struct cigl_InputMesh {
    uint32_t numVertices;
    uint32_t numTriangles;
    const float* vertices;
    const uint32_t* indices;
};

struct cigl_OutputMesh {
    uint32_t numVertices;
    uint32_t numTriangles;
    float* vertices;
    uint32_t* indices;
};

CIGL_API cigl_OutputMesh cigl_substract_mesh_cylinders(cigl_InputMesh inMesh, uint32_t numCylinders, const cigl_CylinderInfo* cylinderInfos);
//void add_mesh_mesh(float* meshA_positions, uint32_t* meshA_indices, float* meshB_positions, uint32_t* meshB_inidices);


//CIGL_API void cigl_release_output_mesh(cigl_OutputMesh mesh);