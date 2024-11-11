#include "cigl.h"

#include <algorithm>
#include <stdio.h>
#include <igl/copyleft/cgal/mesh_boolean.h>

typedef uint32_t u32;
typedef Eigen::Vector3f Vec3;

constexpr float PI = 3.14159265f;
constexpr u32 cylinderResolution = 16;


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

    Eigen::MatrixXf cylinderVertices;
    Eigen::MatrixXi cylinderTriangles;
    Eigen::MatrixXf resultVerts;
    Eigen::MatrixXi resultTris;
    Eigen::VectorXi J;
    for (u32 cylinderI = 0; cylinderI < numCylinders; cylinderI++) {
        const auto& cylinderInfo = cylinderInfos[cylinderI];
        Vec3 basisX = { 1, 0, 0 };
        Vec3 basisY = { 0, 1, 0 };
        Vec3 basisZ = { 0, 0, 1 };
        makeCylinder(cylinderVertices, cylinderTriangles,
            { cylinderInfo.posX, cylinderInfo.posY, cylinderInfo.posZ },
            basisX, basisY, basisZ,
            cylinderInfo.radius, cylinderInfo.halfHeight,
            cylinderResolution
        );
        igl::copyleft::cgal::mesh_boolean(
            verts, tris,
            cylinderVertices, cylinderTriangles,
            igl::MeshBooleanType(igl::MESH_BOOLEAN_TYPE_MINUS),
            resultVerts, resultTris, J);
            std::swap(verts, resultVerts);
            std::swap(tris, resultTris
        );
    }
    
    float* verticesData = new float[3 * verts.rows()];
    for (int i = 0; i < verts.rows(); i++) {
        for (int dimI = 0; dimI < 3; dimI++)
            verticesData[3 * i + dimI] = float(verts(i, dimI));
    }

    u32* indicesData = new u32[3 * tris.rows()];
    for (int i = 0; i < tris.rows(); i++) {
        for (int j = 0; j < 3; j++)
            indicesData[3 * i + j] = u32(tris(i, j));
    }

    return cigl_OutputMesh {
        u32(verts.rows()),
        u32(tris.rows()),
        verticesData,
        indicesData,
    };
    return {};
}