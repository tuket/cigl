#include "cigl.h"

#include <algorithm>
#include <stdio.h>
#include <igl/copyleft/cgal/mesh_boolean.h>
#include <igl/per_vertex_normals.h>

typedef uint32_t u32;
typedef Eigen::Vector3f Vec3;

constexpr float PI = 3.14159265f;
constexpr u32 cylinderResolution = 32;


template <typename TF, typename TI>
static void makeCylinder(Eigen::MatrixX<TF>& vertices, Eigen::MatrixX<TI>& triangles,
    Vec3 center,
    Vec3 basisX, Vec3 basisY, Vec3 basisZ,
    float radius, float halfHeight,
    u32 resolution)
{
    vertices.resize(2 * resolution, 3);
    const float deltaAlpha = 2 * PI / resolution;
    for (TI i = 0; i < resolution; i++) {
        const float alpha = i * deltaAlpha;
        float z = radius * cos(alpha);
        float x = radius * sin(alpha);

        vertices(i, 0) = x;
        vertices(i, 1) = -halfHeight;
        vertices(i, 2) = z;
    }

    for (TI i = 0; i < resolution; i++) {
        vertices(resolution + i, 0) = vertices(i, 0);
        vertices(resolution + i, 1) = -vertices(i, 1);
        vertices(resolution + i, 2) = vertices(i, 2);
    }

    for (int i = 0; i < vertices.rows(); i++) {
        auto& v = vertices.row(i);
        v = center + v[0] * basisX + v[1] * basisY + v[2] * basisZ;
    }

    triangles.resize(2 * (resolution - 2) + 2 * resolution, 3);
    TI triI = 0;
    for (TI i = 0; i < resolution - 2; i++) {
        triangles(triI, 0) = i + 1;
        triangles(triI, 1) = 0;
        triangles(triI, 2) = i + 2;
        triI++;
    }
    for (TI i = 0; i < resolution - 2; i++) {
        triangles(triI, 0) = resolution;
        triangles(triI, 1) = resolution + i + 1;
        triangles(triI, 2) = resolution + i + 2;
        triI++;
    }
    for (TI i = 0; i < resolution; i++) {
        const TI i00 = i;
        const TI i01 = (i + 1) % resolution;
        const TI i10 = i + resolution;
        const TI i11 = i01 + resolution;
        triangles(triI, 0) = i11;
        triangles(triI, 1) = i10;
        triangles(triI, 2) = i00;
        triI++;
        triangles(triI, 0) = i11;
        triangles(triI, 1) = i00;
        triangles(triI, 2) = i01;
        triI++;
    }
}

CIGL_API cigl_OutputMesh cigl_substract_mesh_cylinders(cigl_InputMesh inMesh, uint32_t numCylinders, const cigl_CylinderInfo* cylinderInfos)
{
	Eigen::MatrixXf verts(inMesh.numVertices, 3);
	for (u32 i = 0; i < inMesh.numVertices; i++) {
		for (int dim = 0; dim < 3; dim++)
			verts(i, dim) = inMesh.vertices[3 * i + dim];
	}

	Eigen::MatrixXi tris(inMesh.numTriangles, 3);
	for (u32 i = 0; i < inMesh.numTriangles; i++) {
		for (int j = 0; j < 3; j++)
            tris(i, j) = inMesh.indices[3 * i + j];
	}
    igl::writeOBJ("input.obj", verts, tris);

    Eigen::MatrixXf cylinderVertices;
    Eigen::MatrixXi cylinderTriangles;
    Eigen::MatrixXf resultVerts;
    Eigen::MatrixXi resultTris;
    Eigen::VectorXi J;
    for (u32 cylinderI = 0; cylinderI < numCylinders; cylinderI++) {
        const auto& cylinderInfo = cylinderInfos[cylinderI];
        const Vec3 basisY(cylinderInfo.dirX, cylinderInfo.dirY, cylinderInfo.dirZ);
        const Vec3 basisX = basisY.cross(fabs(basisY.z()) > 0.1 ? Vec3(0, 0, 1) : Vec3(1, 0, 0)).normalized();
        const Vec3 basisZ = basisX.cross(basisY);

        makeCylinder(cylinderVertices, cylinderTriangles,
            { cylinderInfo.posX, cylinderInfo.posY, cylinderInfo.posZ },
            basisX, basisY, basisZ,
            cylinderInfo.radius, cylinderInfo.halfHeight,
            cylinderResolution
        );
        
        char fileName[64];
        snprintf(fileName, std::size(fileName), "cylinder_%d.obj", cylinderI);
        igl::writeOBJ(fileName, cylinderVertices, cylinderTriangles);

        igl::copyleft::cgal::mesh_boolean(
            verts, tris,
            cylinderVertices, cylinderTriangles,
            igl::MeshBooleanType(igl::MESH_BOOLEAN_TYPE_MINUS),
            resultVerts, resultTris, J);

        std::swap(verts, resultVerts);
        std::swap(tris, resultTris);
    }

    Eigen::MatrixXf normals;
    igl::per_vertex_normals(verts, tris, normals);
    
    float* verticesData = new float[3 * verts.rows()];
    for (int i = 0; i < verts.rows(); i++) {
        for (int dimI = 0; dimI < 3; dimI++)
            verticesData[3 * i + dimI] = float(verts(i, dimI));
    }

    float* normalsData = new float[3 * normals.rows()];
    for (int i = 0; i < normals.rows(); i++) {
        for (int dimI = 0; dimI < 3; dimI++)
            normalsData[3 * i + dimI] = float(normals(i, dimI));
    }

    u32* indicesData = new u32[3 * tris.rows()];
    for (int i = 0; i < tris.rows(); i++) {
        for (int j = 0; j < 3; j++)
            indicesData[3 * i + j] = u32(tris(i, j));
    }

    igl::writeOBJ("output.obj", verts, tris);

    return cigl_OutputMesh {
        u32(verts.rows()),
        u32(tris.rows()),
        verticesData,
        normalsData,
        indicesData,
    };
    return {};
}

CIGL_API void cigl_release_output_mesh(cigl_OutputMesh mesh)
{
    delete[] mesh.vertices;
    delete[] mesh.normals;
    delete[] mesh.indices;
}