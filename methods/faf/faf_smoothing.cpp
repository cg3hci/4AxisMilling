/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_smoothing.h"

#include <set>

#ifdef CG3_LIBIGL_DEFINED
#include <cg3/libigl/mesh_adjacencies.h>
#endif

#include <cg3/geometry/triangle.h>
#include <cg3/geometry/2d/point2d.h>

#include <cg3/geometry/transformations.h>

#define BINARY_SEARCH_ITERATIONS 20

namespace FourAxisFabrication {


/* ----- INTERNAL FUNCTION DECLARATION ----- */

namespace internal {

struct ChartData;

bool validateMove(
        const cg3::EigenMesh& mesh,
        const Data& data,
        const int vId,
        const cg3::Pointd& newPoint,
        const std::vector<std::vector<int>>& vertexFaceAdjacencies,
        const std::vector<ChartData>& chartDataVector);

std::vector<cg3::Vec3> computeDifferentialCoordinates(
        const cg3::EigenMesh& mesh,
        const std::vector<std::vector<int>>& vertexAdjacencies);

void initializeChartDataVector(
        const cg3::EigenMesh& mesh,
        const Data& data,
        const std::vector<std::vector<int>>& faceAdjacencies,
        std::vector<ChartData>& chartDataVector);

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

    //Get vertex-vertex adjacencies
    const std::vector<std::vector<int>> vertexAdjacencies =
            cg3::libigl::getVertexVertexAdjacencies(mesh);
    assert(vertexAdjacencies.size() == mesh.getNumberVertices());

    //Get vertex-face adjacencies
    const std::vector<std::vector<int>> vertexFaceAdjacencies =
            cg3::libigl::getVertexFaceAdjacencies(mesh);
    assert(vertexFaceAdjacencies.size() == mesh.getNumberVertices());

    //Get face-face adjacencies
    const std::vector<std::vector<int>> faceAdjacencies =
            cg3::libigl::getFaceFaceAdjacencies(mesh);
    assert(faceAdjacencies.size() == mesh.getNumberFaces());

    const std::vector<cg3::Vec3> differentialCoordinates =
            internal::computeDifferentialCoordinates(mesh, vertexAdjacencies);

    //Get chart attributes
    std::vector<internal::ChartData> chartDataVector;
    internal::initializeChartDataVector(mesh, data, faceAdjacencies, chartDataVector);

    for(int i = 0; i < iterations; ++i) {
        for(unsigned int vId = 0; vId < targetMesh.getNumberVertices(); ++vId) {
            cg3::Pointd delta(0,0,0);

            const std::vector<int>& neighbors = vertexAdjacencies.at(vId);

            cg3::Pointd currentPoint = targetMesh.getVertex(vId);

            //Calculate delta
            for(const int& neighborId : neighbors) {
                delta += targetMesh.getVertex(neighborId);
            }
            delta /= neighbors.size();

            cg3::Pointd newPoint = differentialCoordinates.at(vId) + delta;

            //Do binary search until the face normals do not violate the conditions
            int count = 0;
            while (!internal::validateMove(targetMesh, data, vId, newPoint, vertexFaceAdjacencies, chartDataVector) &&
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
 * @brief Struct to represent attributes of the chart
 */
struct ChartData {
    int label;
    double rotationMatrix[3][3];

    std::vector<int> faces;
    std::set<int> vertices;

    double minZ;
    double maxZ;

    std::vector<int> contourIndex;
    std::vector<cg3::Point2Dd> contour;
};


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
 * @param[in] chartDataVector Data of the charts
 * @return True if the move is valid, false otherwise
 */
bool validateMove(
        const cg3::EigenMesh& mesh,
        const Data& data,
        const int vId,
        const cg3::Pointd& newPoint,
        const std::vector<std::vector<int>>& vertexFaceAdjacencies,
        const std::vector<ChartData>& chartDataVector)
{
    //Check if the triangle has an angle less than 90° with the given direction
    const std::vector<int>& faces = vertexFaceAdjacencies.at(vId);
    for (const int& fId : faces) {
        const cg3::Pointi face = mesh.getFace(fId);

        cg3::Pointd p1, p2, p3;

        //Get the triangle with new point
        if (face.x() == vId) {
            p1 = newPoint;
            p2 = mesh.getVertex(face.y());
            p3 = mesh.getVertex(face.z());
        }
        else if (face.y() == vId) {
            p1 = mesh.getVertex(face.x());
            p2 = newPoint;
            p3 = mesh.getVertex(face.z());
        }
        else {
            assert(face.z() == vId);
            p1 = mesh.getVertex(face.x());
            p2 = mesh.getVertex(face.y());
            p3 = newPoint;
        }
        cg3::Triangle3Dd triangle(p1, p2, p3);

        cg3::Vec3 faceNormal = triangle.normal();

        const cg3::Vec3& associatedDirection = data.directions[data.association[fId]];
        if (faceNormal.dot(associatedDirection) < 0)
            return false;
    }
    return true;
}

/**
 * @brief Initialize data associated to the charts
 * @param[in] mesh Input mesh
 * @param[in] data Four axis fabrication data
 * @param[in] faceAdjacencies Face-face adjacencies of the mesh
 * @param[out] chartDataVector Data of the charts
 */
void initializeChartDataVector(
        const cg3::EigenMesh& mesh,
        const Data& data,
        const std::vector<std::vector<int>>& faceAdjacencies,
        std::vector<ChartData>& chartDataVector)
{
    //Visited flag vector
    std::vector<bool> visited(mesh.getNumberFaces(), false);

    for (unsigned int fId = 0; fId < mesh.getNumberFaces(); fId++) {
        if (!visited[fId]) {
            int label = data.association[fId];

            //If a label has been assigned
            if (label > 0) {
                //Initialize chart data
                ChartData chartData;
                chartData.minZ = std::numeric_limits<double>::max();
                chartData.maxZ = -std::numeric_limits<double>::max();
                cg3::getRotationMatrix(
                            cg3::Vec3(1,0,0),
                            data.angles[label],
                            chartData.rotationMatrix);

                //Stack for iterating on adjacent faces
                std::stack<int> stack;
                stack.push(fId);

                while (!stack.empty()) {
                    int faceId = stack.top();
                    stack.pop();

                    //Add face index to the chart
                    chartData.faces.push_back(faceId);

                    //Get face data
                    cg3::Pointi face = mesh.getFace(faceId);
                    int p1Index = face.x();
                    int p2Index = face.y();
                    int p3Index = face.z();

                    //Add index of vertices to the set of vertices
                    chartData.vertices.insert(p1Index);
                    chartData.vertices.insert(p2Index);
                    chartData.vertices.insert(p3Index);

                    //Rotate point to get the projection (x,y coordinate)
                    cg3::Pointd p1Proj = mesh.getVertex(p1Index);
                    cg3::Pointd p2Proj = mesh.getVertex(p2Index);
                    cg3::Pointd p3Proj = mesh.getVertex(p3Index);
                    p1Proj.rotate(chartData.rotationMatrix);
                    p2Proj.rotate(chartData.rotationMatrix);
                    p3Proj.rotate(chartData.rotationMatrix);

                    //Set min and max Z
                    chartData.minZ = std::min(chartData.minZ, p1Proj.z());
                    chartData.minZ = std::min(chartData.minZ, p2Proj.z());
                    chartData.minZ = std::min(chartData.minZ, p3Proj.z());
                    chartData.maxZ = std::max(chartData.maxZ, p1Proj.z());
                    chartData.maxZ = std::max(chartData.maxZ, p2Proj.z());
                    chartData.maxZ = std::max(chartData.maxZ, p3Proj.z());

                    //Add adjacent faces
                    const std::vector<int>& adjFaces = faceAdjacencies[faceId];
                    for (int adjFace : adjFaces) {
                        //The adjacent face has the same label
                        if (data.association[adjFace] == label) {
                            if (!visited[adjFace]) {
                                stack.push(adjFace);
                            }
                        }
                        //The adjacent face has a different label
                        //i.e. it is a face of the contour
                        else {

                        }
                    }

                    visited[faceId] = true; //Set the visited flag
                }

                //Add chart data
                chartDataVector.push_back(chartData);
            }
        }
    }
}

}

}

