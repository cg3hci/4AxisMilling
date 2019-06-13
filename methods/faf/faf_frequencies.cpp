/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_frequencies.h"

#include <set>
#include <unordered_set>
#include <utility>

#ifdef CG3_LIBIGL_DEFINED
#include <cg3/libigl/mesh_adjacencies.h>
#endif

#include <cg3/geometry/triangle.h>

#include <cg3/geometry/2d/triangle2d_utils.h>
#include <cg3/geometry/utils3d.h>

#include <cg3/geometry/transformations.h>

#include "faf_visibilitycheck.h"
#include "faf_association.h"

#define BINARY_SEARCH_ITERATIONS 20

namespace FourAxisFabrication {


/* ----- INTERNAL FUNCTION DECLARATION ----- */

namespace internal {

bool restoreFrequenciesValidHeightfields(
        cg3::EigenMesh& mesh,
        const std::vector<cg3::Vec3>& differentialCoordinates,
        const std::vector<std::vector<int>>& vertexVertexAdjacencies,
        const std::vector<std::vector<int>>& vertexFaceAdjacencies,
        const Data& data,
        const double heightfieldAngle);

std::vector<cg3::Vec3> computeDifferentialCoordinates(
        const cg3::EigenMesh& mesh,
        const std::vector<std::vector<int>>& vertexVertexAdjacencies);

cg3::Pointd getTargetPoint(const cg3::EigenMesh& mesh,
        const std::vector<cg3::Vec3>& differentialCoordinates,
        const unsigned int vId,
        const std::vector<std::vector<int>>& vertexVertexAdjacencies);

bool isMoveValid(const cg3::EigenMesh& mesh,
        const Data& data,
        const unsigned int vId,
        const cg3::Pointd& newPoint,
        const std::vector<std::vector<int>>& vertexFaceAdjacencies,
        const double heightfieldAngle,
        const double normalAngle);

}

/* ----- RESTORE FREQUENCIES ----- */

/**
 * @brief Restore frequencies of a smoothed mesh.
 * Note that the meshes must have the same number of vertices and faces.
 * @param[in] iterations Number of iterations of the algorithm
 * @param[in] heightfieldAngle Limit angle with triangles normal in order to be a heightfield
 * @param[in] originalMesh Original detailed mesh
 * @param[out] smoothedMesh mesh Target smoothed mesh
 * @param[out] data Four axis fabrication data
 */
void restoreFrequencies(
        const unsigned int iterations,
        const double heightfieldAngle,
        const cg3::EigenMesh& originalMesh,
        cg3::EigenMesh& smoothedMesh,
        Data& data)
{
    assert(originalMesh.numberVertices() <= smoothedMesh.numberVertices());
    assert(originalMesh.numberFaces() <= smoothedMesh.numberFaces());

    //Get vertex-vertex adjacencies
    const std::vector<std::vector<int>> vvAdjOriginal =
            cg3::libigl::vertexToVertexAdjacencies(originalMesh);
    assert(vvAdjOriginal.size() == originalMesh.numberVertices());
    const std::vector<std::vector<int>> vvAdjSmoothed =
            cg3::libigl::vertexToVertexAdjacencies(smoothedMesh);
    assert(vvAdjSmoothed.size() == smoothedMesh.numberVertices());

    //Get differential coordinates
    const std::vector<cg3::Vec3> differentialCoordinatesOriginal =
            internal::computeDifferentialCoordinates(originalMesh, vvAdjOriginal);
    const std::vector<cg3::Vec3> differentialCoordinatesSmoothed =
            internal::computeDifferentialCoordinates(smoothedMesh, vvAdjSmoothed);


    //Update differential coordinates with the new faces and vertices
    std::vector<cg3::Vec3> differentialCoordinates(smoothedMesh.numberVertices());
    std::vector<std::vector<int>> vvAdj(smoothedMesh.numberVertices());

    for (size_t i = 0; i < smoothedMesh.numberVertices(); i++) {
        if (i < differentialCoordinatesOriginal.size()) {
            differentialCoordinates[i] = differentialCoordinatesOriginal[i];
            vvAdj[i] = vvAdjOriginal[i];
        }
        else {
            differentialCoordinates[i] = differentialCoordinatesSmoothed[i];
            vvAdj[i] = vvAdjSmoothed[i];
        }
    }

    //Get vertex-face adjacencies of the smoothed mesh
    const std::vector<std::vector<int>> vfAdjSmoothed =
            cg3::libigl::vertexToFaceIncidences(smoothedMesh);
    assert(vfAdjSmoothed.size() == smoothedMesh.numberVertices());

    //Copy the target mesh
    cg3::EigenMesh& restoredMesh = data.restoredMesh;
    restoredMesh = smoothedMesh;

    for (unsigned int i = 0; i < iterations; ++i) {
        internal::restoreFrequenciesValidHeightfields(
                    restoredMesh,
                    differentialCoordinates,
                    vvAdj,
                    vfAdjSmoothed,
                    data,
                    heightfieldAngle);
    }

    //Needed because they changed
    restoredMesh.updateFacesAndVerticesNormals();
    restoredMesh.updateBoundingBox();

    data.restoredMeshAssociation = data.association;
}



/**
 * @brief Recheck visibility of each face of the mesh from the associated direction.
 * It is implemented by a ray casting algorithm or checking the intersections
 * in a 2D projection from a given direction.
 * If assign is true, it tries to solve the visibility problem, assigning triangles
 * to adjacent directions from which it is visible
 * @param[out] data Four axis fabrication data
 * @param[in] resolution Resolution for the rendering
 * @param[in] heightfieldAngle Limit angle with triangles normal in order to be a heightfield
 * @param[in] includeXDirections Compute visibility for +x and -x directions
 * @param[in] reassign Reassign the non-visible triangles to an adjacent chart
 * @param[in] checkMode Visibility check mode
 * @returns The number of no longer visible triangles
 */
void recheckVisibilityAfterRestore(        
        const unsigned int resolution,
        const double heightfieldAngle,
        const bool includeXDirections,
        const bool reassign,
        Data& data,
        const CheckMode checkMode)
{
    const cg3::EigenMesh& restoredMesh = data.restoredMesh;

    const std::vector<cg3::Vec3>& directions = data.directions;

    const std::vector<unsigned int>& minExtremes = data.minExtremes;
    const std::vector<unsigned int>& maxExtremes = data.maxExtremes;

    std::vector<int>& restoredMeshAssociation = data.restoredMeshAssociation;
    std::vector<unsigned int>& restoredMeshNonVisibleFaces = data.restoredMeshNonVisibleFaces;
    cg3::Array2D<int>& restoredMeshVisibility = data.restoredMeshVisibility;

    const unsigned int nDirections = static_cast<unsigned int>(directions.size()-2);

    //Initialize new data
    Data newData;
    newData.minExtremes = minExtremes;
    newData.maxExtremes = maxExtremes;

    //Get new visibility
    getVisibility(restoredMesh, nDirections, resolution, heightfieldAngle, includeXDirections, newData, checkMode);
    restoredMeshVisibility = newData.visibility;

    //Update association non-visible faces
    restoredMeshNonVisibleFaces.clear();
    for (unsigned int fId = 0; fId < restoredMesh.numberFaces(); fId++){
        if (restoredMeshVisibility(restoredMeshAssociation[fId], fId) == 0) {
            restoredMeshNonVisibleFaces.push_back(fId);
        }
    }

    std::cout << "Non-visible triangles after frequencies restore: " << restoredMeshNonVisibleFaces.size() << std::endl;

    if (reassign) {
        unsigned int facesReassigned = 0;

        //Get face-face adjacencies
        const std::vector<std::vector<int>> ffAdj =
                cg3::libigl::faceToFaceAdjacencies(restoredMesh);


        bool done;

        do {
            std::vector<unsigned int> newNonVisibleFaces;

            done = true;

            for (unsigned int fId : restoredMeshNonVisibleFaces) {
                cg3::Vec3 normal = restoredMesh.faceNormal(fId);
                const std::vector<int>& adjacentFaces = ffAdj.at(fId);

                //The best label for the face is one among the adjacent
                //which has the less dot product with the normal
                double maxDot = -1;
                int bestLabel = -1;

                for (const unsigned int adjId : adjacentFaces) {
                    int adjLabel = restoredMeshAssociation[adjId];

                    //If it is visible
                    if (restoredMeshVisibility(adjLabel, fId) == 1) {
                        double dot = normal.dot(directions[adjLabel]);

                        if (dot >= maxDot) {
                            maxDot = dot;
                            bestLabel = adjLabel;
                        }
                    }
                }

                if (bestLabel >= 0) {
                    restoredMeshAssociation[fId] = bestLabel;
                    facesReassigned++;

                    done = false;
                }
                else {
                    newNonVisibleFaces.push_back(fId);
                }
            }

            restoredMeshNonVisibleFaces = newNonVisibleFaces;

        } while (!done);

        std::cout << "Faces reassigned: " << facesReassigned << std::endl;
    }
}



/* ----- INTERNAL FUNCTION DEFINITION ----- */

namespace internal {


/**
 * @brief Restore frequencies with no occlusion (just heightfield validation)
 * @param[out] targetMesh Target mesh
 * @param[in] differentialCoordinates Differential coordinate of the mesh
 * @param[in] vertexVertexAdjacencies Vertex-vertex adjacencies of the mesh
 * @param[in] vertexFaceAdjacencies Vertex-Face adjacencies of the mesh
 * @param[in] data Four axis fabrication data
 * @param[in] heightfieldAngle Limit angle with triangles normal in order to be a heightfield
 * @return True if at least a vertex has been moved
 */
bool restoreFrequenciesValidHeightfields(
        cg3::EigenMesh& targetMesh,
        const std::vector<cg3::Vec3>& differentialCoordinates,
        const std::vector<std::vector<int>>& vertexVertexAdjacencies,
        const std::vector<std::vector<int>>& vertexFaceAdjacencies,
        const Data& data,
        const double heightfieldAngle)
{
    bool aVertexHasMoved = false;
    for(unsigned int vId = 0; vId < targetMesh.numberVertices(); ++vId) {
        //Get current and target point
        cg3::Pointd currentPoint = targetMesh.vertex(vId);
        cg3::Pointd targetPoint = internal::getTargetPoint(targetMesh, differentialCoordinates, vId, vertexVertexAdjacencies);

        //Do binary search until the face normals do not violate the heightfield conditions
        int count = 0;
        bool isValid = internal::isMoveValid(targetMesh, data, vId, targetPoint, vertexFaceAdjacencies, heightfieldAngle, M_PI/4);
        while (!isValid && count < BINARY_SEARCH_ITERATIONS) {
            targetPoint = (0.5 * (targetPoint - currentPoint) + currentPoint);

            isValid = internal::isMoveValid(targetMesh, data, vId, targetPoint, vertexFaceAdjacencies, heightfieldAngle, M_PI/4);

            count++;
        }

        if (isValid) {
            targetMesh.setVertex(vId, targetPoint);
            aVertexHasMoved = true;
        }
    }
    return aVertexHasMoved;
}


/**
 * @brief Compute differential coordinates for the vertices of a mesh
 * @param[in] mesh Input mesh
 * @param[in] vertexVertexAdjacencies Vertex-vertex adjacencies
 * @return differentialCoordinates Vector of differential coordinates for each vertex
 */
std::vector<cg3::Vec3> computeDifferentialCoordinates(
        const cg3::EigenMesh& mesh,
        const std::vector<std::vector<int>>& vertexVertexAdjacencies)
{
    //Resulting vector
    std::vector<cg3::Vec3> differentialCoordinates;
    differentialCoordinates.resize(mesh.numberVertices());

    #pragma omp parallel for
    for(unsigned int vId = 0; vId < mesh.numberVertices(); ++vId) {
        //Calculate differential coordinates for each point
        cg3::Pointd currentPoint = mesh.vertex(vId);
        cg3::Vec3 delta(0,0,0);

        const std::vector<int>& neighbors = vertexVertexAdjacencies.at(vId);
        for(const int& neighborId : neighbors) {
            delta += currentPoint - mesh.vertex(neighborId);
        }

        delta /= neighbors.size();

        differentialCoordinates[vId] = delta;
    }

    return differentialCoordinates;
}

/**
 * @brief Get the target point using the differential coordinates with
 * respect to the neighborhood
 * @param targetMesh Target mesh
 * @param differentialCoordinates Differential coordinate of the mesh
 * @param vId Target vertex id
 * @param vertexVertexAdjacencies Vertex-vertex adjacencies of the mesh
 * @return Target point
 */
cg3::Pointd getTargetPoint(
        const cg3::EigenMesh& targetMesh,
        const std::vector<cg3::Vec3>& differentialCoordinates,
        const unsigned int vId,
        const std::vector<std::vector<int>>& vertexVertexAdjacencies)
{
    const std::vector<int>& neighbors = vertexVertexAdjacencies.at(vId);

    cg3::Pointd delta(0,0,0);

    //Calculate delta
    for(const int& neighborId : neighbors) {
        delta += targetMesh.vertex(neighborId);
    }
    delta /= neighbors.size();

    cg3::Pointd targetPoint = differentialCoordinates.at(vId) + delta;

    return targetPoint;
}


/**
 * @brief Validate move of a vertex. Height-field check
 * @param[in] targetMesh Target mesh
 * @param[in] data Four axis fabrication data
 * @param[in] vId Vertex id
 * @param[in] newPos New position
 * @param[in] vertexFaceAdjacencies Vertex-face adjacencies of the mesh
 * @param[in] heightfieldAngle Limit angle with triangles normal in order to be a heightfield
 * @return True if the move is valid, false otherwise
 */
bool isMoveValid(
        const cg3::EigenMesh& targetMesh,
        const Data& data,
        const unsigned int vId,
        const cg3::Pointd& newPoint,
        const std::vector<std::vector<int>>& vertexFaceAdjacencies,
        const double heightfieldAngle,
        const double normalAngle)
{    
    const double heightfieldLimit = cos(heightfieldAngle);
    const double normalAngleLimit = cos(normalAngle);

    const std::vector<int>& faces = vertexFaceAdjacencies.at(vId);
    for (const int& fId : faces) {
        const cg3::Pointi face = targetMesh.face(fId);
        const cg3::Vec3 faceNormal = targetMesh.faceNormal(fId);

        cg3::Pointd p1, p2, p3;

        //Get the triangle with new point
        if (face.x() == vId) {
            p1 = newPoint;
            p2 = targetMesh.vertex(face.y());
            p3 = targetMesh.vertex(face.z());
        }
        else if (face.y() == vId) {
            p1 = targetMesh.vertex(face.x());
            p2 = newPoint;
            p3 = targetMesh.vertex(face.z());
        }
        else {
            assert(face.z() == vId);
            p1 = targetMesh.vertex(face.x());
            p2 = targetMesh.vertex(face.y());
            p3 = newPoint;
        }
        cg3::Triangle3Dd triangle(p1, p2, p3);

        cg3::Vec3 newNormal = triangle.normal();

        //Check if normal has not changed too much
        if (newNormal.dot(faceNormal) < normalAngleLimit)
            return false;

        //Check if the triangle normal has an angle less than 90° with the given direction
        const cg3::Vec3& associatedDirection = data.directions[data.association[fId]];
        if (newNormal.dot(associatedDirection) <= heightfieldLimit)
            return false;
    }
    return true;
}

} //namespace internal

} //namespace FourAxisFabrication

