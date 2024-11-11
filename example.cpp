#include <igl/opengl/glfw/Viewer.h>
#include "cigl.h"

typedef uint32_t u32;

int main(int argc, char* argv[])
{
    const float cubeVerts[] = {
        0, 0, 0,
        0, 0, 1,
        0, 1, 0,
        0, 1, 1,
        1, 0, 0,
        1, 0, 1,
        1, 1, 0,
        1, 1, 1,
    };

    const u32 cubeInds[] = {
        0, 6, 4,
        0, 2, 6,
        0, 3, 2,
        0, 1, 3,
        2, 7, 6,
        2, 3, 7,
        4, 6, 7,
        4, 7, 5,
        0, 4, 5,
        0, 5, 1,
        1, 5, 7,
        1, 7, 3,
    };

    const cigl_CylinderInfo cylinders[] = {
        {
            .posX = 0.5, .posY = 0.5, .posZ = 0.5,
            .dirX = 0, .dirY = 1, .dirZ = 0,
            .radius = 0.1f,
            .halfHeight = 4,
        },
        {
            .posX = 0.25, .posY = 0.5, .posZ = 0.25,
            .dirX = 0, .dirY = 1, .dirZ = 0,
            .radius = 0.1f,
            .halfHeight = 4,
        },
        {
            .posX = 0.75, .posY = 0.5, .posZ = 0.25,
            .dirX = 0, .dirY = 1, .dirZ = 0,
            .radius = 0.1f,
            .halfHeight = 4,
        },
        {
            .posX = 0.25, .posY = 0.5, .posZ = 0.75,
            .dirX = 0, .dirY = 1, .dirZ = 0,
            .radius = 0.1f,
            .halfHeight = 4,
        },
        {
            .posX = 0.75, .posY = 0.5, .posZ = 0.75,
            .dirX = 0, .dirY = 1, .dirZ = 0,
            .radius = 0.1f,
            .halfHeight = 4,
        },
    };

    // Plot the mesh
    igl::opengl::glfw::Viewer viewer;

    Eigen::MatrixXf verts;
    Eigen::MatrixXi faces;
    
    const cigl_InputMesh inputMesh = {
        .numVertices = std::size(cubeVerts) / 3,
        .numTriangles = std::size(cubeInds) / 3,
        .vertices = cubeVerts,
        .indices = cubeInds,
    };

    auto outMesh = cigl_substract_mesh_cylinders(inputMesh, std::size(cylinders), cylinders);

    Eigen::MatrixXd outVerts(outMesh.numVertices, 3);
    for (int i = 0; i < outVerts.rows(); i++) {
        for (int dimI = 0; dimI < 3; dimI++)
            outVerts(i, dimI) = double(outMesh.vertices[3 * i + dimI]);
    }

    Eigen::MatrixXi outTris(outMesh.numTriangles, 3);
    for (int i = 0; i < outTris.rows(); i++) {
        for (int j = 0; j < outTris.cols(); j++)
            outTris(i, j) = u32(outMesh.indices[3*i + j]);
    }

    viewer.data().clear();
    viewer.data().set_mesh(outVerts, outTris);
    //viewer.data().set_colors(C);
    viewer.data().set_face_based(true);
    viewer.launch();
}