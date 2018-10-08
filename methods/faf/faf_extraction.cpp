/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_extraction.h"
#include "faf_charts.h"

#include <cg3/geometry/transformations.h>
#include <cg3/geometry/2d/point2d.h>

#include <cg3/libigl/mesh_adjacencies.h>

#include <cg3/meshes/eigenmesh/algorithms/eigenmesh_algorithms.h>

#include <cg3/cgal/2d/triangulation2d.h>
#include <cg3/cgal/holefilling.h>

#include <cg3/libigl/booleans.h>
#include <cg3/libigl/connected_components.h>

#include <lib/clipper/clipper.hpp>

#define CYLINDER_SUBD 100

namespace FourAxisFabrication {

/* Useful function declaration */

namespace internal {

std::vector<unsigned int> computeExternalBorder(const cg3::SimpleEigenMesh& m);
std::vector<cg3::Point2Dd> offsetPolygon(std::vector<cg3::Point2Dd>& polygon, const double offset);
void getFabricationOrder(
        const std::vector<unsigned int>& association,
        const cg3::Array2D<int>& costMatrix,
        const size_t minLabel,
        const size_t maxLabel,
        const bool extremeResults,
        std::vector<int>& cost,
        std::vector<int>& gain,
        size_t& currentPosition,
        double& totalCost,
        std::vector<size_t>& resultPosition);

} //namespace internal


/**
 * @brief Extract results meshes
 * @param[out] data Four axis fabrication data
 * @param[in] stockLength Length of the stock
 * @param[in] stockDiameter Diameter of the stock
 * @param[in] supportHeight Height of the support
 * @param[in] firstLayerAngle Angle needed to allow the fabrication of the surface near the mesh
 * @param[in] secondLayerAngle Angle needed to allow the fabrication of the surface
 * @param[in] firstLayerHeight Height of the first layer
 * @param[in] xDirectionsAfter xDirections are fabricated at the end. If false they are fabricated before.
 * @param[in] rotateMeshes Rotate resulting meshes on the given direction
 */
void extractResults(
        Data& data,
        const double modelLength,
        const double stockLength,
        const double stockDiameter,
        const double supportHeight, //TODO
        const double firstLayerAngle,
        const double secondLayerAngle,
        const double firstLayerHeight,
        const bool xDirectionsAfter,
        const bool rotateResults)
{    
    typedef cg3::libigl::CSGTree CSGTree;

    //Referencing input data
    const std::vector<unsigned int>& minExtremes = data.minExtremes;
    const std::vector<unsigned int>& maxExtremes = data.maxExtremes;

    const std::vector<double>& angles = data.angles;
    const std::vector<unsigned int>& targetDirections = data.targetDirections;

    const cg3::Array2D<int>& fourAxisVisibility = data.fourAxisVisibility;

    const cg3::EigenMesh& minComponent = data.minComponent;
    const cg3::EigenMesh& maxComponent = data.maxComponent;
    const cg3::EigenMesh& fourAxisComponent = data.fourAxisComponent;

    const std::vector<int>& fourAxisAssociation = data.fourAxisAssociation;


    //Referencing output data    
    std::vector<cg3::EigenMesh>& boxes = data.boxes;
    std::vector<cg3::EigenMesh>& stocks = data.stocks;
    cg3::EigenMesh& minResult = data.minResult;
    cg3::EigenMesh& maxResult = data.maxResult;
    std::vector<cg3::EigenMesh>& results = data.results;
    std::vector<unsigned int>& resultsAssociation = data.resultsAssociation;

    cg3::EigenMesh& minSupport = data.minSupport;
    cg3::EigenMesh& maxSupport = data.maxSupport;

    //Common values
    unsigned int minLabel = data.targetDirections[targetDirections.size()-2];
    unsigned int maxLabel = data.targetDirections[targetDirections.size()-1];

    const cg3::Vec3 xAxis(1,0,0);
    const cg3::Vec3 yAxis(0,1,0);


    /* ----- SCALE AND TRANSLATE MESHES ----- */

    //Copy four axis mesh
    cg3::EigenMesh fourAxisScaled = fourAxisComponent;
    cg3::EigenMesh minScaled = minComponent;
    cg3::EigenMesh maxScaled = maxComponent;


    //Get the scale factor
    cg3::BoundingBox bb = fourAxisComponent.boundingBox();
    double minX = bb.minX();
    double maxX = bb.maxX();
    const double scaleFactor = modelLength / (maxX - minX);

    //Scale meshes
    const cg3::Vec3 scaleVec(scaleFactor, scaleFactor, scaleFactor);
    fourAxisScaled.scale(scaleVec);
    minScaled.scale(scaleVec);
    maxScaled.scale(scaleVec);

    fourAxisScaled.updateBoundingBox();

    //Center meshes
    cg3::Vec3 translateVec = -fourAxisScaled.boundingBox().center();
    fourAxisScaled.translate(translateVec);
    minScaled.translate(translateVec);
    maxScaled.translate(translateVec);


    /* ----- COLORS ----- */

    std::vector<cg3::Color> colorMap(data.directions.size(), cg3::Color(128,128,128));
    int subd = 255 / (targetDirections.size()-1);
    for (size_t i = 0; i < targetDirections.size(); i++) {
        unsigned int label = targetDirections[i];

        //Color mesh
        int positionInTargetDirections = std::distance(
            targetDirections.begin(),
            std::find(targetDirections.begin(), targetDirections.end(), label));

        cg3::Color color;
        color.setHsv(subd * positionInTargetDirections, 255, 255);

        colorMap[label] = color;
    }

    /* ----- GET CHARTS DATA ----- */

    //Get chart data
    ChartData fourAxisChartData = getChartData(fourAxisComponent, fourAxisAssociation);
    std::vector<int> chartToResult;
    std::vector<size_t> resultToChart;

    /* ----- ADD SURFACE OF THE CHARTS TO THE RESULTS ----- */

    std::vector<cg3::EigenMesh> tmpResults;
    std::vector<unsigned int> tmpResultsAssociation;

    //Initialize results with the chart data
    int rId = 0;
    std::vector<std::unordered_map<size_t, size_t>> resultFacesToMeshFaces; //Map to the mesh face
    for (const Chart& chart : fourAxisChartData.charts) {
        if (chart.label >= 0) {
            cg3::EigenMesh chartResult;

            std::unordered_map<size_t, size_t> resultFaceToMeshFacesMap;

            std::unordered_map<int, unsigned int> vertexMap;

            for (unsigned int vId : chart.vertices) {
                unsigned int newID = chartResult.addVertex(fourAxisScaled.vertex(vId));
                vertexMap.insert(std::make_pair(vId, newID));
            }

            for (unsigned int fId : chart.faces) {
                cg3::Pointi f = fourAxisScaled.face(fId);
                unsigned int newFaceId = chartResult.addFace(vertexMap.at(f.x()), vertexMap.at(f.y()), vertexMap.at(f.z()));
                resultFaceToMeshFacesMap.insert(std::make_pair(newFaceId, fId));
            }

            chartResult.updateFacesAndVerticesNormals();
            chartResult.updateBoundingBox();

            tmpResults.push_back(chartResult);
            tmpResultsAssociation.push_back(static_cast<unsigned int>(chart.label));


            chartResult.setFaceColor(colorMap[chart.label]);

            chartToResult.push_back(rId);
            resultToChart.push_back(chart.id);

            resultFacesToMeshFaces.push_back(resultFaceToMeshFacesMap);

            rId++;
        }
        else {
            chartToResult.push_back(-1);
        }
    }
    size_t nResults = tmpResults.size();



    /* ----- RESULTS WITH BOX ----- */



    std::cout << std::endl <<  "----- RESULT WITH BOX ----- " << std::endl << std::endl;

    //Size of the box
    const double boxWidth = stockLength*2;
    const double boxHeight = stockDiameter*2;

    //Offset to get the first layer polygon with the desired angle
    const double firstLayerOffset = tan(firstLayerAngle) * firstLayerHeight;

    //Offset to get second layer polygon with the desired angle
    const double secondLayerOffset = tan(secondLayerAngle) * (stockDiameter - firstLayerHeight);

    //Triangulation data-structures
    std::vector<std::array<cg3::Point2Dd, 3>> triangulation;
    std::vector<std::vector<cg3::Point2Dd>> holes;

    for (size_t rId = 0; rId < nResults; rId++) {
        //Copying the surface and getting its label
        cg3::EigenMesh& result = tmpResults[rId];
        unsigned int targetLabel = tmpResultsAssociation[rId];


        //Get projection matrix and its inverse for 2D projection
        Eigen::Matrix3d projectionMatrix;
        Eigen::Matrix3d inverseProjectionMatrix;
        if (targetLabel == minLabel) {
            cg3::rotationMatrix(yAxis, M_PI/2, projectionMatrix);
            cg3::rotationMatrix(yAxis, -M_PI/2, inverseProjectionMatrix);
        }
        else if (targetLabel == maxLabel) {
            cg3::rotationMatrix(yAxis, -M_PI/2, projectionMatrix);
            cg3::rotationMatrix(yAxis, M_PI/2, inverseProjectionMatrix);
        }
        else {
            double directionAngle = angles[targetLabel];
            cg3::rotationMatrix(xAxis, -directionAngle, projectionMatrix);
            cg3::rotationMatrix(xAxis, directionAngle, inverseProjectionMatrix);
        }


        //Compute external borders
        std::vector<unsigned int> borderVertices = internal::computeExternalBorder(result);
        size_t nBorderVertices = borderVertices.size();

        //Projected vertices data
        std::vector<cg3::Pointd> projectedPoints3D(nBorderVertices);
        std::vector<cg3::Point2Dd> projectedPoints2D(nBorderVertices);
        std::map<cg3::Point2Dd, unsigned int> projectedPoints2DMap;



        std::unordered_map<size_t, size_t>& resultFaceToMeshFacesMap = resultFacesToMeshFaces.at(rId);

        //Compute minZ and save projected points
        double minZ = std::numeric_limits<double>::max();
        for (size_t i = 0; i < nBorderVertices; i++) {
            //Get vertices and rotate them
            unsigned int vId = borderVertices[i];
            cg3::Pointd p = result.vertex(vId);
            p.rotate(projectionMatrix);

            //Projected points 3D
            projectedPoints3D[i] = p;

            //Projected points 2D
            cg3::Point2Dd p2D(p.x(), p.y());
            projectedPoints2D[i] = p2D;

            //Fill map
            projectedPoints2DMap.insert(std::make_pair(p2D, vId));

            minZ = std::min(minZ, p.z());
        }

        // VF adjacencies
        std::vector<std::vector<int>> vfAdj = cg3::libigl::vertexToFaceIncidences(result);
        std::unordered_set<unsigned int> borderVerticesWithNonVisibleFaces; //Risky vertices


        //Select vertices with non visible faces
        for (size_t i = 0; i < nBorderVertices; i++) {
            //Get vertex
            const cg3::Point2Dd& p2D = projectedPoints2D[i];
            unsigned int vId = projectedPoints2DMap.at(p2D);

            bool hasNonVisibleIncidentFaces = false;
            for (int fId : vfAdj.at(vId)) {
                if (fourAxisVisibility(targetLabel, resultFaceToMeshFacesMap.at(fId)) == 0) {
                    hasNonVisibleIncidentFaces = true;
                }
            }
            if (hasNonVisibleIncidentFaces) {
                borderVerticesWithNonVisibleFaces.insert(vId);
            }
        }



        /* ------- FIRST LAYER OFFSET POLYGONS ------- */

        std::vector<cg3::Point2Dd> firstLayerPoints2D = projectedPoints2D;
        std::map<cg3::Point2Dd, unsigned int> firstLayerPoints2DMap = projectedPoints2DMap;
        size_t discardedFaces = 0;

        //Get min length of the edges
        size_t nFirstLayerVertices = firstLayerPoints2D.size();
        double minLength = std::numeric_limits<double>::max();
        for (size_t i = 0; i < nFirstLayerVertices; i++) {
            cg3::Vec2 edgeVec = firstLayerPoints2D[(i+1) % nFirstLayerVertices] - firstLayerPoints2D[i];
            minLength = std::min(edgeVec.length(), minLength);
        }

        //Get step
        double currentStepOffset = minLength;

        double totalOffset = 0;
        while (totalOffset < firstLayerOffset) {
            //Calculate offset and height
            totalOffset += currentStepOffset;
            if (totalOffset > firstLayerOffset) {
                currentStepOffset -= totalOffset - firstLayerOffset;
                totalOffset = firstLayerOffset;
            }
            const double currentStepHeight = currentStepOffset / tan(firstLayerAngle);

            currentStepOffset *= 2; //Double it for next iteration

            //Get new layer projected points
            std::vector<cg3::Point2Dd> newLayerPoints2D =
                    internal::offsetPolygon(projectedPoints2D, totalOffset);
            size_t nNewLayerVertices = newLayerPoints2D.size();

            //New layer vertices data
            std::set<cg3::Point2Dd> newLayerPoints2DSet;
            std::map<cg3::Point2Dd, unsigned int> newLayerPoints2DMap;

            //Add to the set
            for (size_t i = 0; i < nNewLayerVertices; i++) {
                newLayerPoints2DSet.insert(newLayerPoints2D[i]);
            }


            //Delaunay triangulation of the offset with the surface
            holes.resize(1);
            holes[0] = firstLayerPoints2D;
            triangulation = cg3::cgal::triangulate(newLayerPoints2D, holes);

            //Add triangulation to result
            for (std::array<cg3::Point2Dd, 3>& triangle : triangulation) {
                bool isThereNewVertex = false; //Flag to check if a new point has been created
                bool isThereHoleVertex = false; //Flag to check if at least a surface vertex has been found
                bool isThereNonVisibleFaces = false; //Flag to check if the vertex is incident in non visible face

                double minHoleHeight = std::numeric_limits<double>::max(); //Height for the current triangle

                unsigned int v[3];

                //Set existing hole points, calculate min height in the hole and check if there is new vertex
                for (unsigned int i = 0; i < 3 && !isThereNewVertex && !isThereNonVisibleFaces; i++) {
                    //Hole vertices
                    if (firstLayerPoints2DMap.find(triangle[i]) != firstLayerPoints2DMap.end()) {
                        v[i] = firstLayerPoints2DMap.at(triangle[i]);

                        //If the vertex has a incident non visible face
                        if (borderVerticesWithNonVisibleFaces.find(v[i]) != borderVerticesWithNonVisibleFaces.end()) {
                            isThereNonVisibleFaces = true;
                            discardedFaces++;
                        }

                        //Project on 3D
                        cg3::Pointd p3D = result.vertex(v[i]);
                        p3D.rotate(projectionMatrix);

                        //Get first layer height
                        minHoleHeight = std::min(p3D.z() + currentStepHeight, minHoleHeight);

                        isThereHoleVertex = true;
                    }
                    //If the vertex is not among the existing vertices
                    else if (newLayerPoints2DSet.find(triangle[i]) == newLayerPoints2DSet.end()) {
                        isThereNewVertex = true;
                    }
                }

                //If there is a hole vertex and no new vertices
                if (isThereHoleVertex && !isThereNewVertex && !isThereNonVisibleFaces) {
                    for (unsigned int i = 0; i < 3; i++) {
                        //First layer vertices in the triangle
                        if (newLayerPoints2DSet.find(triangle[i]) != newLayerPoints2DSet.end()) {
                            //Already inserted in the mesh
                            if (newLayerPoints2DMap.find(triangle[i]) != newLayerPoints2DMap.end()) {
                                v[i] = newLayerPoints2DMap.at(triangle[i]);
                            }
                            //Not yet inserted in the mesh
                            else {
                                const cg3::Point2Dd& p2D = triangle[i];

                                cg3::Pointd newPoint(p2D.x(), p2D.y(), minHoleHeight);

                                //Inverse projection
                                newPoint.rotate(inverseProjectionMatrix);

                                //Create new result point
                                const unsigned int newPointId = result.addVertex(newPoint);
                                newLayerPoints2DMap.insert(std::make_pair(p2D, newPointId));

                                v[i] = newPointId;
                            }
                        }
                    }

                    result.addFace(v[0], v[1], v[2]);
                }
            }

            //Get used first layer points
            std::vector<cg3::Point2Dd> usedNewLayerPoints;
            for (size_t i = 0; i < nNewLayerVertices; i++) {
                const cg3::Point2Dd& p2D = newLayerPoints2D[i];
                if (newLayerPoints2DMap.find(p2D) != newLayerPoints2DMap.end()) {
                    usedNewLayerPoints.push_back(p2D);
                }
            }

            //Setting data for next iteration
            firstLayerPoints2D = usedNewLayerPoints;
            firstLayerPoints2DMap = newLayerPoints2DMap;
        }


        /* ------- SECOND LAYER POLYGON ------- */

        //Get second-layer projected points
        std::vector<cg3::Point2Dd> secondLayerPoints2D =
                internal::offsetPolygon(firstLayerPoints2D, secondLayerOffset);
        size_t nSecondLayerVertices = secondLayerPoints2D.size();

        //Second layer vertices data
        std::set<cg3::Point2Dd> secondLayerPoints2DSet;
        std::map<cg3::Point2Dd, unsigned int> secondLayerPoints2DMap;

        //Add to the set
        for (size_t i = 0; i < nSecondLayerVertices; i++) {
            secondLayerPoints2DSet.insert(secondLayerPoints2D[i]);
        }

        //Delaunay triangulation of the offset with the surface
        holes.resize(1);
        holes[0] = firstLayerPoints2D;
        triangulation = cg3::cgal::triangulate(secondLayerPoints2D, holes);

        //Add triangulation to result
        for (std::array<cg3::Point2Dd, 3>& triangle : triangulation) {
            bool isThereNewVertex = false; //Flag to check if a new point has been created
            bool isThereHoleVertex = false; //Flag to check if at least a surface vertex has been found

            unsigned int v[3];

            //Set existing hole points and check if there is new vertex
            for (unsigned int i = 0; i < 3 && !isThereNewVertex; i++) {
                //Hole vertices
                if (firstLayerPoints2DMap.find(triangle[i]) != firstLayerPoints2DMap.end()) {
                    v[i] = firstLayerPoints2DMap.at(triangle[i]);

                    isThereHoleVertex = true;
                }
                //If the vertex is not among the existing vertices
                else if (secondLayerPoints2DSet.find(triangle[i]) == secondLayerPoints2DSet.end()) {
                    isThereNewVertex = true;
                }
            }

            //If there is a hole vertex and no new vertices
            if (isThereHoleVertex && !isThereNewVertex) {
                for (unsigned int i = 0; i < 3; i++) {
                    //Second layer vertices in the triangle
                    if (secondLayerPoints2DSet.find(triangle[i]) != secondLayerPoints2DSet.end()) {
                        //Already inserted in the mesh
                        if (secondLayerPoints2DMap.find(triangle[i]) != secondLayerPoints2DMap.end()) {
                            v[i] = secondLayerPoints2DMap.at(triangle[i]);
                        }
                        //Not yet inserted in the mesh
                        else {
                            const cg3::Point2Dd& p2D = triangle[i];

                            cg3::Pointd newPoint(p2D.x(), p2D.y(), stockDiameter);

                            //Inverse projection
                            newPoint.rotate(inverseProjectionMatrix);

                            //Create new result point
                            unsigned int newPointId = result.addVertex(newPoint);

                            secondLayerPoints2DMap.insert(std::make_pair(p2D, newPointId));

                            v[i] = newPointId;
                        }
                    }
                }

                result.addFace(v[0], v[1], v[2]);
            }
        }

        //Get used second layer points
        std::vector<cg3::Point2Dd> usedSecondLayerPoints;
        for (size_t i = 0; i < nSecondLayerVertices; i++) {
            const cg3::Point2Dd& p2D = secondLayerPoints2D[i];
            if (secondLayerPoints2DMap.find(p2D) != secondLayerPoints2DMap.end()) {
                usedSecondLayerPoints.push_back(p2D);
            }
        }
        secondLayerPoints2D = usedSecondLayerPoints;



        /* ----- BOX ----- */

        //Create new 2D square
        std::map<cg3::Point2Dd, unsigned int> squarePoints2DMap;
        std::vector<cg3::Point2Dd> squarePoints2D(4);
        squarePoints2D[0] = cg3::Point2Dd(-boxWidth, -boxHeight);
        squarePoints2D[1] = cg3::Point2Dd(+boxWidth, -boxHeight);
        squarePoints2D[2] = cg3::Point2Dd(+boxWidth, +boxHeight);
        squarePoints2D[3] = cg3::Point2Dd(-boxWidth, +boxHeight);

        //Adding box vertices
        std::vector<unsigned int> squareVertices(4);

        for (size_t i = 0; i < squarePoints2D.size(); i++) {
            const cg3::Point2Dd& p2D = squarePoints2D[i];

            cg3::Pointd newPoint(p2D.x(), p2D.y(), boxHeight);
            newPoint.rotate(inverseProjectionMatrix);

            unsigned int vid = result.addVertex(newPoint);
            squareVertices[i] = vid;

            squarePoints2DMap[p2D] = vid;
        }

        //Delaunay triangulation between second layer polygon and square polygon
        holes.resize(1);
        holes[0] = secondLayerPoints2D;
        triangulation = cg3::cgal::triangulate(squarePoints2D, holes);

        //Add triangulation to result
        for (std::array<cg3::Point2Dd, 3>& triangle : triangulation) {
            bool isThereNewVertex = false; //Flag to check if a new point has been created (flipped triangles on projection)

            unsigned int v[3];

            for (unsigned int i = 0; i < 3 && !isThereNewVertex; i++) {
                if (secondLayerPoints2DMap.find(triangle[i]) != secondLayerPoints2DMap.end()) {
                    v[i] = secondLayerPoints2DMap.at(triangle[i]);
                }
                else if (squarePoints2DMap.find(triangle[i]) != squarePoints2DMap.end()) {
                    v[i] = squarePoints2DMap.at(triangle[i]);
                }
                //If the vertex is not among the existing vertices
                else {
                    isThereNewVertex = true;
                }
            }

            if (!isThereNewVertex) {
                result.addFace(v[0], v[1], v[2]);
            }
        }

        //Box for closing the mesh
        std::vector<unsigned int> boxVertices;
        boxVertices.resize(8);

        boxVertices[0] = squareVertices[0];
        boxVertices[1] = squareVertices[1];
        boxVertices[2] = squareVertices[2];
        boxVertices[3] = squareVertices[3];

        cg3::Pointd p;

        p = cg3::Pointd(-boxWidth, -boxHeight, -boxHeight);
        p.rotate(inverseProjectionMatrix);
        boxVertices[4] = result.addVertex(p);

        p = cg3::Pointd(boxWidth, -boxHeight, -boxHeight);
        p.rotate(inverseProjectionMatrix);
        boxVertices[5] = result.addVertex(p);

        p = cg3::Pointd(boxWidth, boxHeight, -boxHeight);
        p.rotate(inverseProjectionMatrix);
        boxVertices[6] = result.addVertex(p);

        p = cg3::Pointd(-boxWidth, boxHeight, -boxHeight);
        p.rotate(inverseProjectionMatrix);
        boxVertices[7] = result.addVertex(p);


        //Creating faces to close the mesh
        result.addFace(boxVertices[2], boxVertices[1], boxVertices[5]);
        result.addFace(boxVertices[2], boxVertices[5], boxVertices[6]);

        result.addFace(boxVertices[5], boxVertices[1], boxVertices[0]);
        result.addFace(boxVertices[5], boxVertices[0], boxVertices[4]);

        result.addFace(boxVertices[6], boxVertices[5], boxVertices[4]);
        result.addFace(boxVertices[6], boxVertices[4], boxVertices[7]);

        result.addFace(boxVertices[7], boxVertices[4], boxVertices[0]);
        result.addFace(boxVertices[7], boxVertices[0], boxVertices[3]);

        result.addFace(boxVertices[7], boxVertices[3], boxVertices[2]);
        result.addFace(boxVertices[7], boxVertices[2], boxVertices[6]);

        std::cout << "Result " << rId << ": " << discardedFaces << " discarded faces." << std::endl;
    }

    /* ----- HOLE FILLING ----- */


    std::cout << std::endl <<  "----- HOLE FILLING ----- " << std::endl << std::endl;

    for (size_t rId = 0; rId < nResults; rId++) {
        //Copying the surface and getting its label
        cg3::EigenMesh& result = tmpResults[rId];

        std::cout << "Result " << rId << ":" << std::endl;

        //Hole filling
        result = cg3::EigenMesh(cg3::cgal::holeFillingTriangulation(result));
    }



    /* ----- UNION WITH THE ORIGINAL MESH ----- */

    std::cout << std::endl <<  "----- UNION WITH ORIGINAL MESH ----- " << std::endl << std::endl;

    //Csg tree
    CSGTree csgFourAxisScaled = cg3::libigl::eigenMeshToCSGTree(fourAxisScaled);

    cg3::Array2D<int> costMatrix(nResults, nResults, 0);

    boxes.resize(nResults);
    for (size_t rId = 0; rId < nResults; rId++) {
        //Copying the surface and getting its label
        cg3::EigenMesh& result = tmpResults[rId];
        cg3::EigenMesh& box = boxes[rId];

        unsigned int resultNFaces = result.numberFaces();

        //CSG tree union
        CSGTree csgTmpResult = cg3::libigl::eigenMeshToCSGTree(result);
        CSGTree csgResult = cg3::libigl::union_(csgFourAxisScaled, csgTmpResult);

        //Counting birth faces
        CSGTree::VectorJ birthFaces = csgResult.J();

        //Saving the mesh from the result
        result = cg3::EigenMesh(cg3::libigl::CSGTreeToEigenMesh(csgResult));

        box = result;

        unsigned int nFirstFaces = csgFourAxisScaled.F().rows();
        unsigned int nSecondFaces = csgTmpResult.F().rows();

        int totalCost = 0;

        for (unsigned int i = 0; i < birthFaces.rows(); i++) {
            unsigned int birthFace = birthFaces[i];

            //If the birth face is in the first mesh
            if (birthFace < nFirstFaces) {
                unsigned int birthChartId = fourAxisChartData.faceChartMap.at(birthFace);

                costMatrix(rId, chartToResult[birthChartId])++;

                totalCost++;

                //Color other chart face of the mesh
                box.setFaceColor(colorMap[fourAxisAssociation[birthFace]], i);
            }
        }

        if (rotateResults) {
            unsigned int& targetLabel = tmpResultsAssociation[rId];

            Eigen::Matrix3d resultRotationMatrix;
            if (targetLabel == minLabel) {
                cg3::rotationMatrix(yAxis, M_PI/2, resultRotationMatrix);
            }
            else if (targetLabel == maxLabel) {
                cg3::rotationMatrix(yAxis, -M_PI/2, resultRotationMatrix);
            }
            else {
                double directionAngle = angles[targetLabel];
                cg3::rotationMatrix(xAxis, -directionAngle, resultRotationMatrix);
            }

            box.rotate(resultRotationMatrix);
        }

        //Difference with the number of faces
        int diffFaces = static_cast<int>(result.numberFaces()) - static_cast<int>(resultNFaces);

        std::cout << "Result " << rId << " > " << "CSG: " << totalCost << " - Diff: " << diffFaces << std::endl;
    }

    std::cout << std::endl << "Cost matrix: " << std::endl << costMatrix << std::endl;



    /* ----- RESULTS FABRICATION ORDER AND RESULTS VECTOR FILLING ----- */

    //Total cost of the fabrication in term of faces
    double totalCost = 0;

    //Position of the results
    std::vector<size_t> resultPosition(nResults, std::numeric_limits<int>::max());
    size_t currentPosition = 0;

    //Generate cost and gain data
    std::vector<int> cost(nResults, 0);
    std::vector<int> gain(nResults, 0);
    for (size_t i = 0; i < nResults; i++) {
        for (size_t j = 0; j < nResults; j++) {
            cost[i] += costMatrix(i,j);
            gain[j] += costMatrix(i,j);
        }
    }
    std::cout << "Result\tCost\tGain" << std::endl;
    for (size_t i = 0; i < nResults; i++) {
        std::cout << i << "\t" << cost[i] << "\t" << gain[i] << std::endl;
    }
    std::cout << std::endl;


    std::cout << std::endl << "Result\tCost" << std::endl;

    //X directions before
    if (!xDirectionsAfter) {
        internal::getFabricationOrder(tmpResultsAssociation, costMatrix, minLabel, maxLabel, true, cost, gain, currentPosition, totalCost, resultPosition);
    }

    //Four axis result
    internal::getFabricationOrder(tmpResultsAssociation, costMatrix, minLabel, maxLabel, false, cost, gain, currentPosition, totalCost, resultPosition);

    //X directions after
    if (xDirectionsAfter) {
        internal::getFabricationOrder(tmpResultsAssociation, costMatrix, minLabel, maxLabel, true, cost, gain, currentPosition, totalCost, resultPosition);
    }

    //Filling results
    results.resize(nResults);
    resultsAssociation.resize(nResults);

    for (size_t i = 0; i < nResults; i++) {
        size_t newPosition = resultPosition[i];

        results[newPosition] = std::move(tmpResults[i]);
        resultsAssociation[newPosition] = std::move(tmpResultsAssociation[i]);
    }

    tmpResults.clear();
    tmpResultsAssociation.clear();

    std::cout << std::endl << "Faces fabricated from other directions: " << totalCost << std::endl;


    /* ----- INITIAL STOCK GENERATION ----- */


    std::cout << std::endl <<  "----- STOCK GENERATION ----- " << std::endl << std::endl;

    //Creating stock mesh
    double stockRadius = stockDiameter/2;
    double stockHalfLength = stockLength/2;
    cg3::EigenMesh currentStock = cg3::EigenMesh(cg3::EigenMeshAlgorithms::makeCylinder(cg3::Pointd(-stockHalfLength,0,0), cg3::Pointd(+stockHalfLength,0,0), stockRadius, CYLINDER_SUBD));



    /* ----- IMMERSION IN THE STOCK AND STOCK GENERATIONS ----- */

    std::cout << std::endl <<  "----- INTERSECTION WITH THE STOCK ----- " << std::endl << std::endl;

    stocks.resize(nResults);
    for (size_t rId = 0; rId < nResults; rId++) {
        //Copying the surface and getting its label
        cg3::EigenMesh& result = results[rId];

        //Intersection with the stock
        result = cg3::libigl::intersection(currentStock, result);

        //Add stock
        stocks[rId] = currentStock;

        //New stock for next iteration
        currentStock = result;

        std::cout << "Result " << rId << " done." << std::endl;
    }

    /* ----- MIN AND MAX RESULTS ----- */

    minResult = minScaled;
    maxResult = maxScaled;


    /* ----- SUPPORTS ----- */

    cg3::BoundingBox bbScaled = fourAxisScaled.boundingBox();

    //Scale of a 1.1 factor
    bbScaled.setMin(bbScaled.min()*1.1);
    bbScaled.setMax(bbScaled.max()*1.1);

    if (minComponent.numberFaces() == 0 && minExtremes.size() > 0) {
        //Get maximum x in the faces of the min extremes
        double minLevelSetX = fourAxisScaled.vertex(fourAxisScaled.face(minExtremes[0]).x()).x();
        for (int minFace : minExtremes) {
            cg3::Pointi face = fourAxisScaled.face(minFace);

            minLevelSetX = std::max(minLevelSetX, fourAxisScaled.vertex(face.x()).x());
            minLevelSetX = std::max(minLevelSetX, fourAxisScaled.vertex(face.y()).x());
            minLevelSetX = std::max(minLevelSetX, fourAxisScaled.vertex(face.z()).x());
        }
        //Set min extremes bounding box
        cg3::BoundingBox minBB = bbScaled;
        minBB.setMaxX(minLevelSetX);

        //Min support
        cg3::EigenMesh minBBMesh = cg3::EigenMeshAlgorithms::makeBox(minBB);
        minSupport = cg3::libigl::difference(minBBMesh, fourAxisScaled);
    }


    if (maxComponent.numberFaces() == 0 && maxExtremes.size() > 0) {
        //Get minimum x in the faces of the max extremes
        double maxLevelSetX = fourAxisScaled.vertex(fourAxisScaled.face(maxExtremes[0]).x()).x();
        for (int maxFace : maxExtremes) {
            cg3::Pointi face = fourAxisScaled.face(maxFace);

            maxLevelSetX = std::min(maxLevelSetX, fourAxisScaled.vertex(face.x()).x());
            maxLevelSetX = std::min(maxLevelSetX, fourAxisScaled.vertex(face.y()).x());
            maxLevelSetX = std::min(maxLevelSetX, fourAxisScaled.vertex(face.z()).x());
        }

        //Set max extremes bounding box
        cg3::BoundingBox maxBB = bbScaled;
        maxBB.setMinX(maxLevelSetX);

        //Max support
        cg3::EigenMesh maxBBMesh = cg3::EigenMeshAlgorithms::makeBox(maxBB);
        maxSupport = cg3::libigl::difference(maxBBMesh, fourAxisScaled);
    }


    /* ----- UPDATING MESHES DATA ----- */

    for (size_t i = 0; i < boxes.size(); i++) {
        cg3::EigenMesh& box = boxes[i];

        box.updateBoundingBox();
        box.updateFacesAndVerticesNormals();
    }

    for (size_t i = 0; i < stocks.size(); i++) {
        cg3::EigenMesh& stock = stocks[i];

        if (rotateResults) {
            unsigned int& targetLabel = resultsAssociation[i];

            Eigen::Matrix3d resultRotationMatrix;
            if (targetLabel == minLabel) {
                cg3::rotationMatrix(yAxis, M_PI/2, resultRotationMatrix);
            }
            else if (targetLabel == maxLabel) {
                cg3::rotationMatrix(yAxis, -M_PI/2, resultRotationMatrix);
            }
            else {
                double directionAngle = angles[targetLabel];
                cg3::rotationMatrix(xAxis, -directionAngle, resultRotationMatrix);
            }

            stock.rotate(resultRotationMatrix);
        }

        stock.updateBoundingBox();
        stock.updateFacesAndVerticesNormals();
    }

    for (size_t i = 0; i < nResults; i++) {
        cg3::EigenMesh& result = results[i];

        if (rotateResults) {
            unsigned int& targetLabel = resultsAssociation[i];

            Eigen::Matrix3d resultRotationMatrix;
            if (targetLabel == minLabel) {
                cg3::rotationMatrix(yAxis, M_PI/2, resultRotationMatrix);
            }
            else if (targetLabel == maxLabel) {
                cg3::rotationMatrix(yAxis, -M_PI/2, resultRotationMatrix);
            }
            else {
                double directionAngle = angles[targetLabel];
                cg3::rotationMatrix(xAxis, -directionAngle, resultRotationMatrix);
            }

            result.rotate(resultRotationMatrix);
        }

        result.updateBoundingBox();
        result.updateFacesAndVerticesNormals();
    }

    if (rotateResults) {
        Eigen::Matrix3d rotationMatrix;

        cg3::rotationMatrix(yAxis, -M_PI/2, rotationMatrix);
        minSupport.rotate(rotationMatrix);

        cg3::rotationMatrix(yAxis, M_PI/2, rotationMatrix);
        maxSupport.rotate(rotationMatrix);
    }

    minSupport.updateBoundingBox();
    minSupport.updateFacesAndVerticesNormals();
    maxSupport.updateBoundingBox();
    maxSupport.updateFacesAndVerticesNormals();

    if (rotateResults) {
        Eigen::Matrix3d rotationMatrix;

        cg3::rotationMatrix(yAxis, M_PI/2, rotationMatrix);
        minResult.rotate(rotationMatrix);
        minResult.translate(-minResult.boundingBox().center());

        cg3::rotationMatrix(yAxis, -M_PI/2, rotationMatrix);
        maxResult.rotate(rotationMatrix);
        maxResult.translate(-maxResult.boundingBox().center());
    }

    minResult.updateBoundingBox();
    minResult.updateFacesAndVerticesNormals();
    maxResult.updateBoundingBox();
    maxResult.updateFacesAndVerticesNormals();
}


namespace internal {

/**
 * @brief Compute borders of an eigen mesh
 * @param m Eigen mesh
 * @return Vector of id vertex of the borders
 */
std::vector<unsigned int> computeExternalBorder(const cg3::SimpleEigenMesh& m)
{
    typedef cg3::Dcel Dcel;
    typedef cg3::Dcel::HalfEdge HalfEdge;
    typedef cg3::Dcel::Vertex Vertex;

    //Translate to DCEL
    Dcel dcel(m);

    //Result
    std::vector<unsigned int> externalBorders;

    if (m.numberFaces() == 0)
        return externalBorders;

    unsigned int nVertices = m.numberVertices();

    //Center of the chart
    cg3::Pointd chartCenter(0,0,0);
    for (const Vertex* vertex : dcel.vertexIterator()) {
        chartCenter += vertex->coordinate();
    }
    chartCenter /= nVertices;

    //Border vertices and next map
    std::map<unsigned int, unsigned int> vNext;


    //Furthest vertex
    int furthestVertex = -1;
    double maxDistance = 0;

    for (const HalfEdge* he : dcel.halfEdgeIterator()) {
        if (he->twin() == nullptr) {
            const Vertex* fromV = he->fromVertex();
            const Vertex* toV = he->toVertex();

            const unsigned int fromId = fromV->id();
            const unsigned int toId = toV->id();

            //Fill next map
            vNext[fromId] = toId;

            //Get furthest point from center: it is certainly part of the external borders
            const cg3::Vec3 vec = fromV->coordinate() - chartCenter;
            double distance = vec.length();
            if (distance >= maxDistance) {
                maxDistance = distance;
                furthestVertex = fromId;
            }
        }
    }
    assert(furthestVertex >= 0);

    unsigned int vStart;
    unsigned int vCurrent;

    //Get external borders
    vStart = (unsigned int) furthestVertex;
    vCurrent = vStart;
    do {
        externalBorders.push_back(vCurrent);

        vCurrent = vNext.at(vCurrent);
    }
    while (vCurrent != vStart);

    return externalBorders;
}

/**
 * @brief Compute polygon offset
 * @param polygon Polygon
 * @param offset Offset
 * @return
 */
std::vector<cg3::Point2Dd> offsetPolygon(std::vector<cg3::Point2Dd>& polygon, const double offset) {
    using namespace ClipperLib;

    const double INT_DOUBLE_TRANSLATION = 1e+6;

    Path subj;
    Paths solution;

    for (cg3::Point2Dd& point : polygon) {
        subj <<
                IntPoint(
                    static_cast<long long int>(point.x() * INT_DOUBLE_TRANSLATION),
                    static_cast<long long int>(point.y() * INT_DOUBLE_TRANSLATION));
    }

    ClipperOffset co;
    co.AddPath(subj, jtSquare, etClosedPolygon);
    co.Execute(solution, offset * INT_DOUBLE_TRANSLATION);

    size_t bestIndex = 0;
    long long int bestDifference = std::numeric_limits<long long int>::max();
    for (size_t i = 0; i < solution.size(); i++) {
        Path& path = solution[i];
        long long int difference = static_cast<long long int>(path.size() - polygon.size());
        if (difference < 0)
            difference = -difference;

        if (difference < bestDifference) {
            bestDifference = difference;
            bestIndex = i;
        }
    }

    std::vector<cg3::Point2Dd> result;
    for (IntPoint& ip : solution[bestIndex]) {
        result.push_back(cg3::Point2Dd(ip.X / INT_DOUBLE_TRANSLATION, ip.Y / INT_DOUBLE_TRANSLATION));
    }
    return result;
}


/**
 * @brief Get best order of fabrication
 * @param association
 * @param costMatrix
 * @param minLabel
 * @param maxLabel
 * @param extremes
 * @param cost
 * @param gain
 * @param currentPosition
 * @param totalCost
 * @param resultPosition
 */
void getFabricationOrder(
        const std::vector<unsigned int>& association,
        const cg3::Array2D<int>& costMatrix,
        const size_t minLabel,
        const size_t maxLabel,
        const bool extremeResults,
        std::vector<int>& cost,
        std::vector<int>& gain,
        size_t& currentPosition,
        double& totalCost,
        std::vector<size_t>& resultPosition)
{
    size_t nResults = association.size();

    //Useful variables
    size_t minCostIndex;
    int minCost;

    do {
        minCostIndex = 0;
        minCost = std::numeric_limits<int>::max();
        for (size_t i = 0; i < nResults; i++) {
            if (resultPosition[i] == std::numeric_limits<int>::max()) {
                bool isExtremes = association[i] == minLabel || association[i] == maxLabel;

                if (extremeResults == isExtremes) {
                    if (cost[i] < minCost || (cost[i] == minCost && gain[i] > gain[minCostIndex])) {
                        minCostIndex = i;
                        minCost = cost[i];
                    }
                }
            }
        }

        if (minCost < std::numeric_limits<int>::max()) {
            std::cout << minCostIndex << "\t" << cost[minCostIndex] << std::endl;

            resultPosition[minCostIndex] = currentPosition;
            currentPosition++;

            totalCost += cost[minCostIndex];

            for (size_t i = 0; i < nResults; i++) {
                cost[i] -= costMatrix(i, minCostIndex);
                gain[i] -= costMatrix(minCostIndex, i);
            }
        }
    }
    while (minCost < std::numeric_limits<int>::max());


}


} //namespace internal

}
