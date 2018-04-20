/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_smoothing.h"


#ifdef CG3_LIBIGL_DEFINED
#include <cg3/libigl/mesh_distance.h>
#include <cg3/libigl/vertex_adjacencies.h>
#endif

#include <cg3/geometry/triangle.h>

#define BINARY_SEARCH_ITERATIONS 20

namespace FourAxisFabrication {


/* ----- INTERNAL FUNCTION DECLARATION ----- */

namespace internal {

bool validateMove(
        const cg3::EigenMesh& mesh,
        const Data& data,
        const int vId,
        const cg3::Pointd& newPoint,
        const std::vector<std::vector<int>>& vertexFaceAdjacencies);

std::vector<cg3::Vec3> computeDifferentialCoordinates(
        const cg3::EigenMesh& mesh,
        const std::vector<std::vector<int>>& vertexAdjacencies);

}

/* ----- RESTORE FREQUENCIES ----- */

/**
 * @brief Restore frequencies of a smoothed mesh.
 * Note that the meshes must have the same number of vertices and faces.
 * @param[in] originalMesh Original detailed mesh
 * @param[in] data Four axis fabrication data
 * @param[in] iterations Number of iterations of the algorithm
 * @param[out] targetMesh Target mesh
 */
void restoreFrequencies(
        const cg3::EigenMesh& mesh,
        const Data& data,
        const int iterations,
        cg3::EigenMesh& targetMesh)
{
    assert(mesh.getNumberVertices() == targetMesh.getNumberVertices());
    assert(mesh.getNumberFaces() == targetMesh.getNumberFaces());

    //Get vertex adjacencies
    const std::vector<std::vector<int>> vertexAdjacencies =
            cg3::libigl::getVertexAdjacencies(mesh);
    assert(vertexAdjacencies.size() == mesh.getNumberVertices());

    const std::vector<std::vector<int>> vertexFaceAdjacencies =
            cg3::libigl::getVertexFaceAdjacencies(mesh);
    assert(vertexFaceAdjacencies.size() == mesh.getNumberVertices());

    const std::vector<cg3::Vec3> differentialCoordinates =
            internal::computeDifferentialCoordinates(mesh, vertexAdjacencies);

    for(int i = 0; i < iterations; ++i) {
        for(unsigned int vId = 0; vId < targetMesh.getNumberVertices(); ++vId) {
            cg3::Pointd delta(0,0,0);

            const std::vector<int>& neighbors = vertexAdjacencies.at(vId);
            cg3::Pointd currentPoint = targetMesh.getVertex(vId);

            for(const int& neighborId : neighbors) {
                delta += targetMesh.getVertex(neighborId);
            }
            delta /= neighbors.size();

            cg3::Pointd newPoint = differentialCoordinates.at(vId) + delta;

            //Do binary search until the face normals do not violate the height-field condition
            int count = 0;
            while (!internal::validateMove(targetMesh, data, vId, newPoint, vertexFaceAdjacencies) &&
                   count < BINARY_SEARCH_ITERATIONS)
            {
                newPoint = 0.5 * (newPoint + currentPoint);

                count++;
            }

            if (count < BINARY_SEARCH_ITERATIONS) {
                targetMesh.setVertex(vId, newPoint);
            }
        }
    }

    targetMesh.updateFacesAndVerticesNormals();
    targetMesh.updateBoundingBox();
}


/* ----- INTERNAL FUNCTION DEFINITION ----- */

namespace internal {

/**
 * @brief Compute differential coordinates for the vertices of a mesh
 * @param[in] mesh Input mesh
 * @param[in] vertexAdjacencies Vertex-vertex adjacencies
 * @return differentialCoordinates Vector of differential coordinates for each vertex
 */
std::vector<cg3::Vec3> computeDifferentialCoordinates(
        const cg3::EigenMesh& mesh,
        const std::vector<std::vector<int>>& vertexAdjacencies)
{
    //Resulting vector
    std::vector<cg3::Vec3> differentialCoordinates;
    differentialCoordinates.resize(mesh.getNumberVertices());

    #pragma omp parallel for
    for(unsigned int vId = 0; vId < mesh.getNumberVertices(); ++vId) {
        //Calculate differential coordinates for each point
        cg3::Pointd currentPoint = mesh.getVertex(vId);
        cg3::Vec3 delta(0,0,0);

        const std::vector<int>& neighbors = vertexAdjacencies.at(vId);
        for(const int& neighborId : neighbors) {
            delta += currentPoint - mesh.getVertex(neighborId);
        }

        delta /= neighbors.size();

        differentialCoordinates[vId] = delta;
    }

    return differentialCoordinates;
}

/**
 * @brief Validate move of a vertex.
 * @param[in] mesh Input mesh
 * @param[in] data Four axis fabrication data
 * @param[in] vId Vertex id
 * @param[in] newPos New position
 * @param[in] vertexFaceAdjacencies Vertex-face adjacencies of the mesh
 * @return True if the move is valid, false otherwise
 */
bool validateMove(
        const cg3::EigenMesh& mesh,
        const Data& data,
        const int vId,
        const cg3::Pointd& newPoint,
        const std::vector<std::vector<int>>& vertexFaceAdjacencies)
{
    //Check if the triangle has an angle less than 90° with the given direction
    const std::vector<int>& faces = vertexFaceAdjacencies.at(vId);
    for (const int& fId : faces) {
        const cg3::Pointi face = mesh.getFace(fId);

        cg3::Pointd p1, p2, p3;

        if (face.x() == vId)
            p1 = newPoint;
        else
            p1 = mesh.getVertex(face.x());

        if (face.y() == vId)
            p2 = newPoint;
        else
            p2 = mesh.getVertex(face.y());

        if (face.z() == vId)
            p3 = newPoint;
        else
            p3 = mesh.getVertex(face.z());

        cg3::Triangle3Dd triangle(p1, p2, p3);
        cg3::Vec3 faceNormal = triangle.normal();

        const cg3::Vec3& associatedDirection = data.directions[data.association[fId]];
        if (faceNormal.dot(associatedDirection) < 0)
            return false;
    }
    return true;
}

}

}

