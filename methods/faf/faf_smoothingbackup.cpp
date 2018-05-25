///**
// * @author Stefano Nuvoli
// * @author Alessandro Muntoni
// */
//#include "faf_smoothing.h"

//#include <set>
//#include <unordered_set>
//#include <utility>

//#ifdef CG3_LIBIGL_DEFINED
//#include <cg3/libigl/mesh_adjacencies.h>
//#endif

//#include <cg3/geometry/triangle.h>
//#include <cg3/geometry/2d/triangle2d.h>
//#include <cg3/geometry/plane.h>

//#include <cg3/geometry/2d/triangle2d_utils.h>
//#include <cg3/geometry/utils3d.h>

//#include <cg3/geometry/transformations.h>


//#include <cg3/data_structures/trees/aabbtree.h>

//#include "faf_utilities.h"

//#define BINARY_SEARCH_ITERATIONS 20

//namespace FourAxisFabrication {


///* ----- INTERNAL FUNCTION DECLARATION ----- */

//namespace internal {

///**
// * @brief Struct to represent attributes and data structures of a chart
// */
//struct Chart {
//    typedef cg3::AABBTree<2, cg3::Triangle2Dd, std::pair<int, cg3::Triangle2Dd>> ChartAABBTree;

//    Chart() :
//        aabbTree(ChartAABBTree(&triangle2DAABBExtractor, &triangle2DComparator))
//    {

//    }

//    size_t id;

//    int label;
//    double rotationMatrix[3][3];

//    std::vector<int> faces;
//    std::set<int> vertices;

//    ChartAABBTree aabbTree;
//};

///**
// * @brief Struct all chart data
// */
//struct ChartData {
//    std::vector<size_t> faceChartMap;
//    std::vector<Chart> charts;
//};

//bool restoreFrequenciesValidHeightfields(
//        cg3::EigenMesh& mesh,
//        const std::vector<cg3::Vec3>& differentialCoordinates,
//        const std::vector<std::vector<int>>& vertexVertexAdjacencies,
//        const std::vector<std::vector<int>>& vertexFaceAdjacencies,
//        const Data& data);

//bool restoreFrequenciesWithOcclusions(
//        cg3::EigenMesh& mesh,
//        const std::vector<cg3::Vec3>& differentialCoordinates,
//        const std::vector<std::vector<int>>& vertexVertexAdjacencies,
//        const std::vector<std::vector<int>>& vertexFaceAdjacencies,
//        const Data& data,
//        ChartData& chartData);

//std::vector<cg3::Vec3> computeDifferentialCoordinates(
//        const cg3::EigenMesh& mesh,
//        const std::vector<std::vector<int>>& vertexVertexAdjacencies);

//cg3::Pointd getTargetPoint(
//        const cg3::EigenMesh& mesh,
//        const std::vector<cg3::Vec3>& differentialCoordinates,
//        const int vId,
//        const std::vector<std::vector<int>>& vertexVertexAdjacencies);

//void initializeChartData(
//        const cg3::EigenMesh& mesh,
//        const Data& data,
//        const std::vector<std::vector<int>>& faceFaceAdjacencies,
//        ChartData& chartData);

//void updateChart(
//        const cg3::EigenMesh& mesh,
//        ChartData& chartData,
//        const size_t currentChartId);

//bool isHeightFieldValid(
//        const cg3::EigenMesh& mesh,
//        const Data& data,
//        const int vId,
//        const cg3::Pointd& newPoint,
//        const std::vector<std::vector<int>>& vertexFaceAdjacencies);

//bool areOcclusionsValid(
//        const cg3::EigenMesh& mesh,
//        const Data& data,
//        const int vId,
//        const cg3::Pointd& newPoint,
//        const std::vector<std::vector<int>>& vertexFaceAdjacencies,
//        ChartData& chartData,
//        const size_t currentChartId);

//inline double coplanarDeterminant(
//        const cg3::Pointd& p0,
//        const cg3::Pointd& p1,
//        const cg3::Pointd& p2,
//        const cg3::Pointd& p3);
//}

///* ----- RESTORE FREQUENCIES ----- */

///**
// * @brief Restore frequencies of a smoothed mesh.
// * Note that the meshes must have the same number of vertices and faces.
// * @param[in] iterations Number of iterations of the algorithm
// * @param[in] occlusionsCheck Occlusion check in restoring frequncies
// * @param[in] originalMesh Original detailed mesh
// * @param[out] targetMesh Target mesh
// * @param[out] data Four axis fabrication data
// */
//void restoreFrequencies(
//        const unsigned int iterations,
//        const bool occlusionsCheck,
//        const cg3::EigenMesh& originalMesh,
//        cg3::EigenMesh& smoothedMesh,
//        Data& data)
//{
//    assert(originalMesh.getNumberVertices() == smoothedMesh.getNumberVertices());
//    assert(originalMesh.getNumberFaces() == smoothedMesh.getNumberFaces());

//    //Get vertex-vertex adjacencies
//    const std::vector<std::vector<int>> vertexVertexAdjacencies =
//            cg3::libigl::getVertexVertexAdjacencies(originalMesh);
//    assert(vertexVertexAdjacencies.size() == originalMesh.getNumberVertices());

//    //Get vertex-face adjacencies
//    const std::vector<std::vector<int>> vertexFaceAdjacencies =
//            cg3::libigl::getVertexFaceAdjacencies(originalMesh);
//    assert(vertexFaceAdjacencies.size() == originalMesh.getNumberVertices());

//    //Get face-face adjacencies
//    const std::vector<std::vector<int>> faceFaceAdjacencies =
//            cg3::libigl::getFaceFaceAdjacencies(originalMesh);
//    assert(faceFaceAdjacencies.size() == originalMesh.getNumberFaces());

//    const std::vector<cg3::Vec3> differentialCoordinates =
//            internal::computeDifferentialCoordinates(originalMesh, vertexVertexAdjacencies);

//    //Copy the target mesh
//    cg3::EigenMesh& targetMesh = data.restoredMesh;
//    targetMesh = smoothedMesh;


//    if (occlusionsCheck) {
//        //Initialize chart data
//        internal::ChartData chartData;
//        internal::initializeChartData(originalMesh, data, faceFaceAdjacencies, chartData);

//        for (unsigned int i = 0; i < iterations; ++i) {
//            //Process the restoring for each chart
//            internal::restoreFrequenciesWithOcclusions(
//                        targetMesh,
//                        differentialCoordinates,
//                        vertexVertexAdjacencies,
//                        vertexFaceAdjacencies,
//                        data,
//                        chartData);

//        }
//    }
//    else {
//        for (unsigned int i = 0; i < iterations; ++i) {
//            internal::restoreFrequenciesValidHeightfields(
//                        targetMesh,
//                        differentialCoordinates,
//                        vertexVertexAdjacencies,
//                        vertexFaceAdjacencies,
//                        data);
//        }
//    }

//    //Needed because they changed
//    targetMesh.updateFacesAndVerticesNormals();
//    targetMesh.updateBoundingBox();
//}


///* ----- INTERNAL FUNCTION DEFINITION ----- */

//namespace internal {


///**
// * @brief Restore frequencies with no occlusion (just heightfield validation)
// * @param[out] targetMesh Target mesh
// * @param[in] differentialCoordinates Differential coordinate of the mesh
// * @param[in] vertexVertexAdjacencies Vertex-vertex adjacencies of the mesh
// * @param[in] vertexFaceAdjacencies Vertex-Face adjacencies of the mesh
// * @param[in] data Four axis fabrication data
// * @return True if at least a vertex has been moved
// */
//bool restoreFrequenciesValidHeightfields(
//        cg3::EigenMesh& targetMesh,
//        const std::vector<cg3::Vec3>& differentialCoordinates,
//        const std::vector<std::vector<int>>& vertexVertexAdjacencies,
//        const std::vector<std::vector<int>>& vertexFaceAdjacencies,
//        const Data& data)
//{
//    bool aVertexHasMoved = false;
//    for(unsigned int vId = 0; vId < targetMesh.getNumberVertices(); ++vId) {
//        //Get current and target point
//        cg3::Pointd currentPoint = targetMesh.getVertex(vId);
//        cg3::Pointd targetPoint = internal::getTargetPoint(targetMesh, differentialCoordinates, vId, vertexVertexAdjacencies);

//        //Do binary search until the face normals do not violate the heightfield conditions
//        int count = 0;
//        while (!internal::isHeightFieldValid(targetMesh, data, vId, targetPoint, vertexFaceAdjacencies) &&
//               count < BINARY_SEARCH_ITERATIONS)
//        {
//            targetPoint = 0.5 * (targetPoint + currentPoint);

//            count++;
//        }

//        if (count < BINARY_SEARCH_ITERATIONS) {
//            targetMesh.setVertex(vId, targetPoint);
//            aVertexHasMoved = true;
//        }
//    }
//    return aVertexHasMoved;
//}

///**
// * @brief Restore frequencies with occlusion
// * @param[out] mesh Target mesh
// * @param[in] differentialCoordinates Differential coordinate of the mesh
// * @param[in] vertexVertexAdjacencies Vertex-vertex adjacencies of the mesh
// * @param[in] vertexFaceAdjacencies Vertex-Face adjacencies of the mesh
// * @param[in] data Four axis fabrication data
// * @param[out] chartData Data of the charts
// * @return True if at least a vertex has been moved
// */
//bool restoreFrequenciesWithOcclusions(
//        cg3::EigenMesh& mesh,
//        const std::vector<cg3::Vec3>& differentialCoordinates,
//        const std::vector<std::vector<int>>& vertexVertexAdjacencies,
//        const std::vector<std::vector<int>>& vertexFaceAdjacencies,
//        const Data& data,
//        ChartData& chartData)
//{
//    bool aVertexHasMoved = false;

//    //Visited flag vector
//    std::vector<bool> moved(mesh.getNumberVertices(), false);

//    //For each chart
//    for (size_t c = 0; c < chartData.charts.size(); c++) {
//        internal::Chart& chart = chartData.charts[c];
//        std::set<int>& chartVertices = chart.vertices;

//        std::set<size_t> affectedCharts; //Charts to be updated at the end

//        //For each vertex of the chart
//        for(int vId : chartVertices) {
//            if (!moved[vId]) {
//                //Get current and target point
//                cg3::Pointd currentPoint = mesh.getVertex(vId);
//                cg3::Pointd targetPoint = internal::getTargetPoint(mesh, differentialCoordinates, vId, vertexVertexAdjacencies);

//                //Do binary search until the components do not violate the conditions
//                int count = 0;
//                while (
//                       !(
//                           internal::isHeightFieldValid(mesh, data, vId, targetPoint, vertexFaceAdjacencies) &&
//                           internal::areOcclusionsValid(mesh, data, vId, targetPoint, vertexFaceAdjacencies, chartData, c)
//                       ) &&
//                       count < BINARY_SEARCH_ITERATIONS)
//                {
//                    targetPoint = 0.5 * (targetPoint + currentPoint);

//                    count++;
//                }

//                if (count < BINARY_SEARCH_ITERATIONS) {
//                    mesh.setVertex(vId, targetPoint);
//                    aVertexHasMoved = true;
//                    moved[vId] = true;

//                    //Add to the set the affected charts that must be updated
//                    const std::vector<int>& adjFaces = vertexFaceAdjacencies[vId];
//                    for (int adjFace : adjFaces) {
//                        affectedCharts.insert(chartData.faceChartMap[adjFace]);
//                    }
//                }
//            }
//        }

//        if (aVertexHasMoved) {
//            //Update charts
//            //TODO MORE EFFICIENTLY: WE CAN REMOVE/ADD FROM THE AABBTREE THE AFFECTED TRIANGLES!
//            for (size_t updateChartId : affectedCharts) {
//                updateChart(mesh, chartData, updateChartId);
//            }
//        }
//    }

//    return aVertexHasMoved;
//}


///**
// * @brief Compute differential coordinates for the vertices of a mesh
// * @param[in] mesh Input mesh
// * @param[in] vertexVertexAdjacencies Vertex-vertex adjacencies
// * @return differentialCoordinates Vector of differential coordinates for each vertex
// */
//std::vector<cg3::Vec3> computeDifferentialCoordinates(
//        const cg3::EigenMesh& mesh,
//        const std::vector<std::vector<int>>& vertexVertexAdjacencies)
//{
//    //Resulting vector
//    std::vector<cg3::Vec3> differentialCoordinates;
//    differentialCoordinates.resize(mesh.getNumberVertices());

//    #pragma omp parallel for
//    for(unsigned int vId = 0; vId < mesh.getNumberVertices(); ++vId) {
//        //Calculate differential coordinates for each point
//        cg3::Pointd currentPoint = mesh.getVertex(vId);
//        cg3::Vec3 delta(0,0,0);

//        const std::vector<int>& neighbors = vertexVertexAdjacencies.at(vId);
//        for(const int& neighborId : neighbors) {
//            delta += currentPoint - mesh.getVertex(neighborId);
//        }

//        delta /= neighbors.size();

//        differentialCoordinates[vId] = delta;
//    }

//    return differentialCoordinates;
//}

///**
// * @brief Get the target point using the differential coordinates with
// * respect to the neighborhood
// * @param targetMesh Target mesh
// * @param differentialCoordinates Differential coordinate of the mesh
// * @param vId Target vertex id
// * @param vertexVertexAdjacencies Vertex-vertex adjacencies of the mesh
// * @return Target point
// */
//cg3::Pointd getTargetPoint(
//        const cg3::EigenMesh& targetMesh,
//        const std::vector<cg3::Vec3>& differentialCoordinates,
//        const int vId,
//        const std::vector<std::vector<int>>& vertexVertexAdjacencies)
//{
//    const std::vector<int>& neighbors = vertexVertexAdjacencies.at(vId);

//    cg3::Pointd delta(0,0,0);

//    //Calculate delta
//    for(const int& neighborId : neighbors) {
//        delta += targetMesh.getVertex(neighborId);
//    }
//    delta /= neighbors.size();

//    cg3::Pointd targetPoint = differentialCoordinates.at(vId) + delta;

//    return targetPoint;
//}

///**
// * @brief Initialize data associated to the charts
// * @param[in] targetMesh Target mesh
// * @param[in] data Four axis fabrication data
// * @param[in] faceFaceAdjacencies Face-face adjacencies of the mesh
// * @param[out] chartData Data of the charts
// */
//void initializeChartData(
//        const cg3::EigenMesh& targetMesh,
//        const Data& data,
//        const std::vector<std::vector<int>>& faceFaceAdjacencies,
//        ChartData& chartData)
//{
//    unsigned int nFaces = targetMesh.getNumberFaces();

//    //Visited flag vector
//    std::vector<bool> visited(nFaces, false);

//    //Clear data
//    chartData.faceChartMap.resize(nFaces);

//    for (unsigned int fId = 0; fId < targetMesh.getNumberFaces(); fId++) {
//        if (!visited[fId]) {
//            int label = data.association[fId];

//            //If a label has been assigned
//            if (label > 0) {
//                //Initialize chart data
//                Chart chart;

//                chart.id = chartData.charts.size();
//                chartData.faceChartMap[fId] = chart.id;

//                chart.label = label;

//                cg3::getRotationMatrix(
//                            cg3::Vec3(1,0,0),
//                            data.angles[label],
//                            chart.rotationMatrix);

//                //Stack for iterating on adjacent faces
//                std::stack<int> stack;
//                stack.push(fId);

//                //Vector for keeping track of the projected triangles
//                std::vector<std::pair<cg3::Triangle2Dd, std::pair<int, cg3::Triangle2Dd>>> projectedFacesVector;

//                //Region growing algorithm to get all chart faces
//                while (!stack.empty()) {
//                    int faceId = stack.top();
//                    stack.pop();

//                    //Add face index to the chart
//                    chart.faces.push_back(faceId);

//                    //Get face data
//                    cg3::Pointi face = targetMesh.getFace(faceId);
//                    int p1Index = face.x();
//                    int p2Index = face.y();
//                    int p3Index = face.z();

//                    //Add index of vertices to the set of vertices
//                    chart.vertices.insert(p1Index);
//                    chart.vertices.insert(p2Index);
//                    chart.vertices.insert(p3Index);

//                    //Get current vertex points
//                    cg3::Pointd p1Proj = targetMesh.getVertex(p1Index);
//                    cg3::Pointd p2Proj = targetMesh.getVertex(p2Index);
//                    cg3::Pointd p3Proj = targetMesh.getVertex(p3Index);

//                    //Rotate point to get the projection (x,y coordinate)
//                    p1Proj.rotate(chart.rotationMatrix);
//                    p2Proj.rotate(chart.rotationMatrix);
//                    p3Proj.rotate(chart.rotationMatrix);

//                    //Get 2D point projected on the z plane
//                    cg3::Point2Dd p1Proj2D(p1Proj.x(), p1Proj.y());
//                    cg3::Point2Dd p2Proj2D(p2Proj.x(), p2Proj.y());
//                    cg3::Point2Dd p3Proj2D(p3Proj.x(), p3Proj.y());

//                    //Create 2D triangle
//                    cg3::Triangle2Dd triangleProjected(p1Proj2D, p2Proj2D, p3Proj2D);
//                    cg3::sortTriangle2DPointsAndReorderCounterClockwise(triangleProjected);

//                    //Add to the projected faces
//                    projectedFacesVector.push_back(std::make_pair(triangleProjected, std::make_pair(faceId, triangleProjected)));

//                    //Add adjacent faces
//                    const std::vector<int>& adjFaces = faceFaceAdjacencies[faceId];
//                    for (int adjFace : adjFaces) {
//                        //The adjacent face has the same label
//                        if (data.association[adjFace] == label) {
//                            if (!visited[adjFace]) {
//                                stack.push(adjFace);
//                            }
//                        }
//                        //The adjacent face has a different label
//                        //i.e. it is a face of the contour
//                        //else {
//                        //}
//                    }

//                    visited[faceId] = true;
//                }

//                //Construct the AABBTree
//                chart.aabbTree.construction(projectedFacesVector);

//                //Add chart data
//                chartData.charts.push_back(chart);
//            }
//        }
//    }
//}

////TODO
//void updateChart(
//        const cg3::EigenMesh& targetMesh,
//        ChartData& chartData,
//        const size_t currentChartId)
//{
//    Chart& currentChart = chartData.charts[currentChartId];

//    //Vector for keeping track of the projected triangles
//    std::vector<std::pair<cg3::Triangle2Dd, std::pair<int, cg3::Triangle2Dd>>> projectedFacesVector;

//    //Region growing algorithm to get all chart faces
//    for (int faceId : currentChart.faces) {
//        //Get face data
//        cg3::Pointi face = targetMesh.getFace(faceId);
//        int p1Index = face.x();
//        int p2Index = face.y();
//        int p3Index = face.z();

//        //Get current vertex points
//        cg3::Pointd p1Proj = targetMesh.getVertex(p1Index);
//        cg3::Pointd p2Proj = targetMesh.getVertex(p2Index);
//        cg3::Pointd p3Proj = targetMesh.getVertex(p3Index);

//        //Rotate point to get the projection (x,y coordinate)
//        p1Proj.rotate(currentChart.rotationMatrix);
//        p2Proj.rotate(currentChart.rotationMatrix);
//        p3Proj.rotate(currentChart.rotationMatrix);

//        //Get 2D point projected on the z plane
//        cg3::Point2Dd p1Proj2D(p1Proj.x(), p1Proj.y());
//        cg3::Point2Dd p2Proj2D(p2Proj.x(), p2Proj.y());
//        cg3::Point2Dd p3Proj2D(p3Proj.x(), p3Proj.y());

//        //Create 2D triangle
//        cg3::Triangle2Dd triangleProjected(p1Proj2D, p2Proj2D, p3Proj2D);
//        cg3::sortTriangle2DPointsAndReorderCounterClockwise(triangleProjected);

//        //Add to the projected faces
//        projectedFacesVector.push_back(std::make_pair(triangleProjected, std::make_pair(faceId, triangleProjected)));
//    }

//    //Construct the AABBTree
//    currentChart.aabbTree.construction(projectedFacesVector);
//}

///**
// * @brief Validate move of a vertex. Height-field check
// * @param[in] targetMesh Target mesh
// * @param[in] data Four axis fabrication data
// * @param[in] vId Vertex id
// * @param[in] newPos New position
// * @param[in] vertexFaceAdjacencies Vertex-face adjacencies of the mesh
// * @return True if the move is valid, false otherwise
// */
//bool isHeightFieldValid(
//        const cg3::EigenMesh& targetMesh,
//        const Data& data,
//        const int vId,
//        const cg3::Pointd& newPoint,
//        const std::vector<std::vector<int>>& vertexFaceAdjacencies)
//{
//    const std::vector<int>& faces = vertexFaceAdjacencies.at(vId);
//    for (const int& fId : faces) {
//        const cg3::Pointi face = targetMesh.getFace(fId);

//        cg3::Pointd p1, p2, p3;

//        //Get the triangle with new point
//        if (face.x() == vId) {
//            p1 = newPoint;
//            p2 = targetMesh.getVertex(face.y());
//            p3 = targetMesh.getVertex(face.z());
//        }
//        else if (face.y() == vId) {
//            p1 = targetMesh.getVertex(face.x());
//            p2 = newPoint;
//            p3 = targetMesh.getVertex(face.z());
//        }
//        else {
//            assert(face.z() == vId);
//            p1 = targetMesh.getVertex(face.x());
//            p2 = targetMesh.getVertex(face.y());
//            p3 = newPoint;
//        }
//        cg3::Triangle3Dd triangle(p1, p2, p3);

//        cg3::Vec3 faceNormal = triangle.normal();

//        //Check if the triangle normal has an angle less than 90° with the given direction
//        const cg3::Vec3& associatedDirection = data.directions[data.association[fId]];
//        if (faceNormal.dot(associatedDirection) < 0)
//            return false;
//    }
//    return true;
//}

///**
// * @brief Validate move of a vertex. Check for occlusions in other charts.
// * @param[in] targetMesh Target mesh
// * @param[in] data Four axis fabrication data
// * @param[in] vId Vertex id
// * @param[in] newPos New position
// * //TODO
// * @return True if the move is valid, false otherwise
// */
//bool areOcclusionsValid(
//        const cg3::EigenMesh& targetMesh,
//        const Data& data,
//        const int vId,
//        const cg3::Pointd& newPoint,
//        const std::vector<std::vector<int>>& vertexFaceAdjacencies,
//        ChartData& chartData,
//        const size_t currentChartId)
//{
//    typedef Chart::ChartAABBTree::iterator AABBIterator;

//    //Get a set of face adjacent to the vertex
//    const std::vector<int>& incidentFaces = vertexFaceAdjacencies.at(vId);

//    //Get current chart data
//    const Chart& currentChart = chartData.charts[currentChartId];

//    //For each chart different from the current one
//    for (size_t c = 0; c < chartData.charts.size(); c++) {
//        if (chartData.charts[c].label == data.directions.size() - 1)
//            continue;
//        if (chartData.charts[c].label == data.directions.size() - 2)
//            continue;
//        if (currentChart.label == data.directions.size() - 1)
//            continue;
//        if (currentChart.label == data.directions.size() - 2)
//            continue;

//        internal::Chart& chart = chartData.charts[c];

//        //Rotate point to get the projection (x,y coordinate)
//        cg3::Pointd vRotated = newPoint;
//        vRotated.rotate(chart.rotationMatrix);

//        //Get 2D projected point on z plane
//        cg3::Point2Dd vProj2D = cg3::Point2Dd(vRotated.x(), vRotated.y());

//        //Create the triangle (point)
//        cg3::Triangle2Dd queryTriangle(vProj2D, vProj2D, vProj2D);

//        //Get the triangles with an overlapping AABB
//        std::vector<AABBIterator> out;
//        chart.aabbTree.aabbOverlapQuery(queryTriangle, std::back_inserter(out));

//        for (AABBIterator it : out) {
//            //Get data from AABBTree
//            const int& outTriangleId = (*it).first;
//            const cg3::Triangle2Dd& outTriangle = (*it).second;

//            //If face is incident to the vertex, then the move is valid for that face
//            bool isIncidentFace =
//                    (std::find(incidentFaces.begin(), incidentFaces.end(), outTriangleId) != incidentFaces.end());

//            if (!isIncidentFace) {
//                //If the projected 2D point is inside the projected 2D triangle
//                if (cg3::isPointLyingInTriangle(outTriangle, vProj2D, false)) {
//                    cg3::Pointi face = targetMesh.getFace(outTriangleId);
//                    cg3::Pointd p1 = targetMesh.getVertex(face.x());
//                    cg3::Pointd p2 = targetMesh.getVertex(face.y());
//                    cg3::Pointd p3 = targetMesh.getVertex(face.z());

//                    if (coplanarDeterminant(p1, p2, p3, newPoint) <= 0) {
//                        return false;
//                    }
//                }
//            }
//        }

//    }

//    return true;
//}

//double coplanarDeterminant(
//        const cg3::Pointd& p0,
//        const cg3::Pointd& p1,
//        const cg3::Pointd& p2,
//        const cg3::Pointd& p3)
//{
//    Eigen::Matrix4d m;
//    m << p0.x(), p0.y(), p0.z(), 1,
//         p1.x(), p1.y(), p1.z(), 1,
//         p2.x(), p2.y(), p2.z(), 1,
//         p3.x(), p3.y(), p3.z(), 1;

//    return m.determinant();
//}

//} //namespace internal

//} //namespace FourAxisFabrication

