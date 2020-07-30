/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_extraction.h"
#include "faf_charts.h"
#include "faf_various.h"

#include <cg3/utilities/utils.h>

#include <cg3/geometry/transformations3.h>
#include <cg3/geometry/point2.h>

#include <cg3/libigl/mesh_adjacencies.h>

#include <cg3/meshes/eigenmesh/algorithms/eigenmesh_algorithms.h>

#include <cg3/cgal/triangulation2.h>
#include <cg3/cgal/hole_filling.h>

#include <cg3/libigl/booleans.h>
#include <cg3/libigl/connected_components.h>

#include <lib/clipper/clipper.hpp>

namespace FourAxisFabrication {

/* Useful function declaration */

namespace internal {

std::vector<cg3::Point2d> offsetPolygon(std::vector<cg3::Point2d>& polygon, const double offset);
void getFabricationOrder(
        const std::vector<unsigned int>& association,
        const cg3::Array2D<double>& costMatrix,
        const size_t minLabel,
        const size_t maxLabel,
        std::vector<double>& cost,
        std::vector<double>& gain,
        size_t& currentPosition,
        double& totalCost,
        std::vector<size_t>& resultPosition);

std::vector<cg3::Point2d> createSquare(
        unsigned int nVertex,
        const cg3::Point3d& minCoord,
        const cg3::Point3d& maxCoord);

} //namespace internal


/**
 * @brief Extract results meshes
 * @param[out] data Four axis fabrication data
 * @param[in] stockLength Length of the stock
 * @param[in] stockDiameter Diameter of the stock
 * @param[in] supportHeight Height of the support
 * @param[in] firstLayerAngle Angle needed to allow the fabrication of the surface near the mesh
 * @param[in] firstLayerOffset Offset of the first layer
 * @param[in] secondLayerStepWidth Height of each step of the second layers
 * @param[in] secondLayerStepHeight Width of each step of the second layers
 * @param[in] heightfieldAngle Limit angle with triangles normal in order to be a heightfield
 * @param[in] xDirectionsAfter xDirections are fabricated at the end. If false they are fabricated before.
 * @param[in] rotateMeshes Rotate resulting meshes on the given direction
 */
void extractResults(
        Data& data,
        const double stockLength,
        const double stockDiameter,
        const double firstLayerAngle,
        const double firstLayerOffset,
        const unsigned int firstLayerSmoothingIterations,
        const double firstLayerSmoothingWeight,
        const double secondLayerStepWidth,
        const double secondLayerStepHeight,
        const bool secondLayerSideSubdivision,
        const double heightfieldAngle,
        const bool xDirectionsAfter,
        const bool minFirst,
        const bool rotateResults)
{
    typedef cg3::libigl::CSGTree CSGTree;

    //Referencing input data
    const std::vector<unsigned int>& minExtremes = data.minExtremes;
    const std::vector<unsigned int>& maxExtremes = data.maxExtremes;

    const std::vector<double>& angles = data.angles;
    const std::vector<unsigned int>& targetDirections = data.targetDirections;

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

    const cg3::Vec3d xAxis(1,0,0);
    const cg3::Vec3d yAxis(0,1,0);




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
    ChartData fourAxisChartData = getChartData(fourAxisComponent, fourAxisAssociation, minExtremes, maxExtremes);
    std::vector<int> chartToResult;
    std::vector<size_t> resultToChart;



    /* ----- ADD SURFACE OF THE CHARTS TO THE RESULTS ----- */

    std::vector<cg3::EigenMesh> tmpResults;
    std::vector<unsigned int> tmpResultsAssociation;
    std::vector<std::vector<unsigned int>> chartExternalBorders;

    //Initialize results with the chart data
    int rId = 0;
    std::vector<std::unordered_map<size_t, size_t>> resultFacesToMeshFaces; //Map to the mesh face
    for (const Chart& chart : fourAxisChartData.charts) {
        if (chart.label >= 0) {
            cg3::EigenMesh chartResult;

            std::unordered_map<size_t, size_t> resultFaceToMeshFaceMap;

            std::unordered_map<unsigned int, unsigned int> meshVertexToResultVertexMap;

            for (unsigned int vId : chart.vertices) {
                unsigned int newID = chartResult.addVertex(fourAxisComponent.vertex(vId));
                meshVertexToResultVertexMap.insert(std::make_pair(vId, newID));
            }

            for (unsigned int fId : chart.faces) {
                cg3::Point3i f = fourAxisComponent.face(fId);
                unsigned int newFaceId = chartResult.addFace(
                            meshVertexToResultVertexMap.at(static_cast<unsigned int>(f.x())),
                            meshVertexToResultVertexMap.at(static_cast<unsigned int>(f.y())),
                            meshVertexToResultVertexMap.at(static_cast<unsigned int>(f.z())));
                resultFaceToMeshFaceMap.insert(std::make_pair(newFaceId, fId));
            }

            chartResult.updateFacesAndVerticesNormals();
            chartResult.updateBoundingBox();

            tmpResults.push_back(chartResult);
            tmpResultsAssociation.push_back(static_cast<unsigned int>(chart.label));


            chartResult.setFaceColor(colorMap[static_cast<size_t>(chart.label)]);

            chartToResult.push_back(rId);
            resultToChart.push_back(chart.id);

            resultFacesToMeshFaces.push_back(resultFaceToMeshFaceMap);

            std::vector<unsigned int> externalBorders(chart.borderVertices.size());
            for (size_t i = 0; i < chart.borderVertices.size(); i++) {
                unsigned int borderVertexId = chart.borderVertices[i];
                externalBorders[i] = meshVertexToResultVertexMap.at(borderVertexId);
            }
            chartExternalBorders.push_back(externalBorders);

            rId++;
        }
        else {
            chartToResult.push_back(-1);
        }
    }
    size_t nResults = tmpResults.size();


    /* ----- SUPPORTS ----- */

    //Create a big box
    cg3::BoundingBox3 bbSupport;
    bbSupport.setMin(cg3::Point3d(-stockLength/2, -stockDiameter/2, -stockDiameter/2));
    bbSupport.setMax(cg3::Point3d(stockLength/2, stockDiameter/2, stockDiameter/2));

    double minLevelSetX, maxLevelSetX;
    getMinAndMaxHeightfieldLevelSet(fourAxisComponent, heightfieldAngle, cg3::libigl::faceToFaceAdjacencies(fourAxisComponent), minLevelSetX, maxLevelSetX);

    //Set min extremes bounding box
    cg3::BoundingBox3 minBB = bbSupport;
    minBB.setMaxX(minLevelSetX);

    //Min support
    cg3::EigenMesh minBBMesh = cg3::EigenMeshAlgorithms::makeBox(minBB);
    minSupport = cg3::libigl::difference(minBBMesh, fourAxisComponent);

    //Set max extremes bounding box
    cg3::BoundingBox3 maxBB = bbSupport;
    maxBB.setMinX(maxLevelSetX);

    //Max support
    cg3::EigenMesh maxBBMesh = cg3::EigenMeshAlgorithms::makeBox(maxBB);
    maxSupport = cg3::libigl::difference(maxBBMesh, fourAxisComponent);



    /* ----- RESULTS WITH BOX ----- */

    std::cout << std::endl <<  "----- RESULT WITH BOX ----- " << std::endl << std::endl;

    //Size of the box
    const double boxWidth = stockLength*2;
    const double boxHeight = stockDiameter*2;

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
        std::vector<unsigned int>& borderVertices = chartExternalBorders.at(rId);
        size_t nBorderVertices = borderVertices.size();

        //Projected vertices data
        std::vector<cg3::Point3d> borderProjectedPoints3D(nBorderVertices);
        std::vector<cg3::Point2d> borderProjectedPoints2D(nBorderVertices);

        //Save projected points
        for (size_t i = 0; i < nBorderVertices; i++) {
            //Get vertex and coordinate
            unsigned int vId = borderVertices[i];
            cg3::Point3d p3d = result.vertex(vId);

            //Project the point (rotate on Z the current direction)
            p3d.rotate(projectionMatrix);

            //Projected points 3D
            borderProjectedPoints3D[i] = p3d;

            //Projected points 2D
            cg3::Point2d p2D(p3d.x(), p3d.y());
            borderProjectedPoints2D[i] = p2D;
        }


        /* ------- FIRST LAYER: OFFSET POLYGON ------- */

        const double firstLayerHeight = firstLayerOffset / tan(firstLayerAngle);

        //Get the offset polygon
        std::vector<cg3::Point2d> offsetPolygon2D =
                internal::offsetPolygon(borderProjectedPoints2D, firstLayerOffset);
        size_t nOffsetVertices = offsetPolygon2D.size();

        //Get the avg length
        double avgLength = 0;
        for (size_t i = 0; i < nBorderVertices; i++) {
            avgLength += (borderProjectedPoints2D[(i + 1) % nBorderVertices] - borderProjectedPoints2D[i]).length();
        }
        avgLength /= nBorderVertices;

        //Sample offset polygon
        std::vector<cg3::Point2d> sampledOffsetPolygon2D;
        sampledOffsetPolygon2D.reserve(nOffsetVertices);
        for (size_t i = 0; i < nOffsetVertices; i++) {
            sampledOffsetPolygon2D.push_back(offsetPolygon2D[i]);

            cg3::Vec2d vec = offsetPolygon2D[(i + 1) % nOffsetVertices] - offsetPolygon2D[i];
            double edgeLength = vec.length();

            unsigned int subdivision = std::ceil(edgeLength / (avgLength * 2));
            double unitstep = 1.0 / subdivision;

            for (unsigned int j = 1; j < subdivision - 1; j++) {
                sampledOffsetPolygon2D.push_back(offsetPolygon2D[i] + (unitstep * j) * vec);
            }
        }
        offsetPolygon2D = sampledOffsetPolygon2D;
        nOffsetVertices = offsetPolygon2D.size();

        //Get offset polygon vertex normals
        std::vector<cg3::Vec2d> offsetPolygonNormals(nOffsetVertices);
        for (size_t i = 1; i <= nOffsetVertices; i++) {
            size_t prevId = i - 1;
            size_t currentId = i % nOffsetVertices;
            size_t nextId = (i + 1) % nOffsetVertices;

            cg3::Vec2d vec1 = sampledOffsetPolygon2D[currentId] - sampledOffsetPolygon2D[prevId];
            cg3::Vec2d vec2 = sampledOffsetPolygon2D[nextId] - sampledOffsetPolygon2D[currentId];

            cg3::Vec2d n1(vec1.y(), -vec1.x());
            cg3::Vec2d n2(vec2.y(), -vec2.x());

            offsetPolygonNormals[currentId] = n1 + n2;
            offsetPolygonNormals[currentId].normalize();
        }

        //Smooth offset polygon vertex normals
        for (size_t it = 0; it < 3; it++) {
            std::vector<cg3::Vec2d> auxOffsetPolygonNormals = offsetPolygonNormals;
            for (size_t i = 1; i <= nOffsetVertices; i++) {
                size_t prevId = i - 1;
                size_t currentId = i % nOffsetVertices;
                size_t nextId = (i + 1) % nOffsetVertices;

                offsetPolygonNormals[currentId] = auxOffsetPolygonNormals[currentId] + auxOffsetPolygonNormals[prevId] + auxOffsetPolygonNormals[nextId];
                offsetPolygonNormals[currentId].normalize();
            }
        }

        //Get bprder polygon vertex normals
        std::vector<cg3::Vec2d> borderPolygonNormals(nBorderVertices);
        for (size_t i = 1; i <= nBorderVertices; i++) {
            size_t prevId = i - 1;
            size_t currentId = i % nBorderVertices;
            size_t nextId = (i + 1) % nBorderVertices;

            cg3::Vec2d vec1 = borderProjectedPoints2D[currentId] - borderProjectedPoints2D[prevId];
            cg3::Vec2d vec2 = borderProjectedPoints2D[nextId] - borderProjectedPoints2D[currentId];

            cg3::Vec2d n1(vec1.y(), -vec1.x());
            cg3::Vec2d n2(vec2.y(), -vec2.x());

            borderPolygonNormals[currentId] = n1 + n2;
            borderPolygonNormals[currentId].normalize();
        }

        //Smooth offset polygon vertex normals
        for (size_t it = 0; it < 3; it++) {
            std::vector<cg3::Vec2d> auxBorderPolygonNormals = borderPolygonNormals;
            for (size_t i = 1; i <= nBorderVertices; i++) {
                size_t prevId = i - 1;
                size_t currentId = i % nBorderVertices;
                size_t nextId = (i + 1) % nBorderVertices;

                borderPolygonNormals[currentId] = auxBorderPolygonNormals[currentId] + auxBorderPolygonNormals[prevId] + auxBorderPolygonNormals[nextId];
                borderPolygonNormals[currentId].normalize();
            }
        }

        //Border-offset triangulation
        size_t startBorderId = 0; //Starting vertex border
        size_t startOffsetId = 0; //Starting offset border
        unsigned int numberConsecutiveBorder = 0;
        unsigned int numberConsecutiveOffset = 0;
        double bestScore = std::numeric_limits<double>::max();

        //Get index of the starting vertices
        for (size_t i = 0; i < nBorderVertices; i++) {
            for (size_t j = 0; j < nOffsetVertices; j++) {
                cg3::Vec2d vec = (offsetPolygon2D[j] - borderProjectedPoints2D[i]);
                double distance = vec.length();

                double dot = borderPolygonNormals[i].dot(offsetPolygonNormals[j]);

                double multiplier = 1 - dot;
                if (dot <= 0) {
                    multiplier = 2;
                }

                double score = std::fabs(distance - firstLayerOffset)  * (0.9 + multiplier * 0.1);

                if (score < bestScore) {
                    bestScore = score;
                    startBorderId = i;
                    startOffsetId = j;
                }
            }
        }

        //Create offset vertices in the mesh
        std::vector<unsigned int> offsetVertices(nOffsetVertices);
        for (size_t i = 0; i < nOffsetVertices; i++) {
            cg3::Point3d p(offsetPolygon2D[i].x(), offsetPolygon2D[i].y(), boxHeight);

            //Project the point (rotate on Z the current direction)
            p.rotate(inverseProjectionMatrix);

            unsigned int id = result.addVertex(p);
            offsetVertices[i] = id;
        }

        //Border-offset actual triangulation
        size_t currentBorderId = startBorderId;
        size_t currentOffsetId = startOffsetId;
        bool firstIterationBorder = true;
        bool firstIterationOffset = true;
        do {
            //Get border vertex move score
            size_t nextBorderId = (currentBorderId + 1) % nBorderVertices;
            double scoreBorder = std::numeric_limits<double>::max();
            if ((firstIterationBorder || currentBorderId != startBorderId) && numberConsecutiveBorder <= 5) {
                cg3::Vec2d vec = offsetPolygon2D[currentOffsetId] - borderProjectedPoints2D[nextBorderId];
                double distanceBorder = vec.length();

                double dot = borderPolygonNormals[nextBorderId].dot(offsetPolygonNormals[currentOffsetId]);

                double multiplier = 1 - dot;
                if (dot <= 0) {
                    multiplier = 2;
                }

                scoreBorder = distanceBorder * (0.9 + multiplier * 0.1);
            }

            //Get offset vertex move score
            size_t nextOffsetId = (currentOffsetId + 1) % nOffsetVertices;
            double scoreOffset = std::numeric_limits<double>::max();
            if ((firstIterationOffset || currentOffsetId != startOffsetId) && numberConsecutiveOffset <= 5) {
                cg3::Vec2d vec = offsetPolygon2D[nextOffsetId] - borderProjectedPoints2D[currentBorderId];
                double distanceOffset = vec.length();

                double dot = borderPolygonNormals[currentBorderId].dot(offsetPolygonNormals[nextOffsetId]);

                double multiplier = 1 - dot;
                if (dot <= 0) {
                    multiplier = 2;
                }

                scoreOffset = distanceOffset * (0.9 + multiplier * 0.1);
            }

            //Get projected point (current border)
            cg3::Point3d currentBorderProjectedPoint = borderProjectedPoints3D[currentBorderId];

            //Triangulate with a border vertex
            if (scoreBorder < scoreOffset) {
                //Add triangle
                result.addFace(borderVertices[currentBorderId], offsetVertices[currentOffsetId], borderVertices[nextBorderId]);

                //Calculate height for the current offset vertex and set the new z
                cg3::Point3d nextBorderProjectedPoint = borderProjectedPoints3D[nextBorderId];
                cg3::Point3d currentOffsetProjectedPoint = result.vertex(offsetVertices[currentOffsetId]);
                currentOffsetProjectedPoint.rotate(projectionMatrix);

                currentOffsetProjectedPoint.setZ(
                    std::min(
                        currentOffsetProjectedPoint.z(),
                        std::min(
                            currentBorderProjectedPoint.z() + firstLayerHeight,
                            nextBorderProjectedPoint.z() + firstLayerHeight
                        )
                    )
                );

                currentOffsetProjectedPoint.rotate(inverseProjectionMatrix);
                result.setVertex(offsetVertices[currentOffsetId], currentOffsetProjectedPoint);

                //Next iteration
                currentBorderId = nextBorderId;
                firstIterationBorder = false;
                numberConsecutiveBorder++;
                numberConsecutiveOffset = 0;
            }
            else { //Triangulate with a offset vertex
                //Add triangle
                result.addFace(borderVertices[currentBorderId], offsetVertices[currentOffsetId], offsetVertices[nextOffsetId]);

                //Calculate height for the current offset vertex and set the new z
                cg3::Point3d currentOffsetProjectedPoint = result.vertex(offsetVertices[currentOffsetId]);
                currentOffsetProjectedPoint.rotate(projectionMatrix);

                currentOffsetProjectedPoint.setZ(
                    std::min(
                        currentOffsetProjectedPoint.z(),
                        currentBorderProjectedPoint.z() + firstLayerHeight
                    )
                );

                currentOffsetProjectedPoint.rotate(inverseProjectionMatrix);
                result.setVertex(offsetVertices[currentOffsetId], currentOffsetProjectedPoint);

                //Calculate height for the next offset vertex and set the new z
                cg3::Point3d nextOffsetProjectedPoint = result.vertex(offsetVertices[nextOffsetId]);
                nextOffsetProjectedPoint.rotate(projectionMatrix);

                double nextOffsetMinZ = std::min(
                    nextOffsetProjectedPoint.z(),
                    currentBorderProjectedPoint.z() + firstLayerHeight);

                nextOffsetProjectedPoint.setZ(nextOffsetMinZ);

                nextOffsetProjectedPoint.rotate(inverseProjectionMatrix);
                result.setVertex(offsetVertices[nextOffsetId], nextOffsetProjectedPoint);

                //Next iteration
                currentOffsetId = nextOffsetId;
                firstIterationOffset = false;
                numberConsecutiveOffset++;
                numberConsecutiveBorder = 0;
            }
        }
        while (currentBorderId != startBorderId || currentOffsetId != startOffsetId);

        /* ------- FIRST LAYER SMOOTHING ------- */

        if (firstLayerSmoothingIterations > 0) {
            //Get the projected point
            std::vector<cg3::Point3d> projectedOffsetVertices(nOffsetVertices);
            for (size_t i = 0; i < nOffsetVertices; i++) {
                cg3::Point3d point = result.vertex(offsetVertices[i]);
                point.rotate(projectionMatrix);
                projectedOffsetVertices[i] = point;
            }

            //Get parameters
            const double smoothingWeight = firstLayerSmoothingWeight;
            const double neighborWeight = (1 - firstLayerSmoothingWeight) * 0.5;

            //Execute smoothing
            for (unsigned int it = 0; it < firstLayerSmoothingIterations; it++) {
                const std::vector<cg3::Point3d> auxOffsetVertices = projectedOffsetVertices;

                for (size_t i = 1; i <= nOffsetVertices; i++) {
                    size_t prevId = i - 1;
                    size_t currentId = i % nOffsetVertices;
                    size_t nextId = (i + 1) % nOffsetVertices;

                    const cg3::Point3d& prevPoint = auxOffsetVertices[prevId];
                    const cg3::Point3d& currentPoint = auxOffsetVertices[currentId];
                    const cg3::Point3d& nextPoint = auxOffsetVertices[nextId];

                    projectedOffsetVertices[currentId] = (currentPoint * smoothingWeight) + ((prevPoint + nextPoint) * neighborWeight);
                }
            }

            //Unproject the points
            for (size_t i = 0; i < nOffsetVertices; i++) {
                cg3::Point3d point = projectedOffsetVertices[i];
                point.rotate(inverseProjectionMatrix);
                result.setVertex(offsetVertices[i], point);
            }
        }


        /* ------- SECOND LAYER POLYGON ------- */

        //New layer vertices and map (used by CGAL)
        std::vector<cg3::Point2d> currentSecondLayerPoints2D(nOffsetVertices);
        std::map<cg3::Point2d, unsigned int> currentSecondLayerPoints2DMap;

        //Min coord of the new points inserted
        cg3::Point3d minCoord(std::numeric_limits<double>::max(),std::numeric_limits<double>::max(),std::numeric_limits<double>::max());
        //Max coord of the new points inserted
        cg3::Point3d maxCoord(std::numeric_limits<double>::min(),std::numeric_limits<double>::min(),std::numeric_limits<double>::min());

        //Fill the above data
        for (size_t i = 0; i < nOffsetVertices; i++) {
            unsigned int vId = offsetVertices[i];

            cg3::Point3d point = result.vertex(vId);
            point.rotate(projectionMatrix);

            currentSecondLayerPoints2D[i] = cg3::Point2d(point.x(), point.y());

            currentSecondLayerPoints2DMap.insert(std::make_pair(currentSecondLayerPoints2D[i], vId));

            //Set min and max coordinates
            minCoord.setX(std::min(minCoord.x(), point.x()));
            minCoord.setY(std::min(minCoord.y(), point.y()));
            minCoord.setZ(std::min(minCoord.z(), point.z()));
            maxCoord.setX(std::max(maxCoord.x(), point.x()));
            maxCoord.setY(std::max(maxCoord.y(), point.y()));
            maxCoord.setZ(std::max(maxCoord.z(), point.z()));
        }

        //Execute the second layer, until the target height has been reached
        double totalHeight = maxCoord.z();
        while (totalHeight < boxHeight) {
            minCoord.x() -= secondLayerStepWidth;
            minCoord.y() -= secondLayerStepWidth;

            maxCoord.x() += secondLayerStepWidth;
            maxCoord.y() += secondLayerStepWidth;            

            //Calculate vertices per side (if enabled)
            unsigned int nSideSubdivision = 1;
            while (secondLayerSideSubdivision && nSideSubdivision * 4 < currentSecondLayerPoints2D.size()) {
                nSideSubdivision *= 2;
            }

            //Create new 2D square (down)
            std::map<cg3::Point2d, unsigned int> downPoints2DMap;
            std::vector<cg3::Point2d> squarePoints2D = FourAxisFabrication::internal::createSquare(nSideSubdivision, minCoord, maxCoord);

            size_t newVerticesNumber = squarePoints2D.size();

            std::vector<unsigned int> downVertices(newVerticesNumber);

            for (size_t i = 0; i < squarePoints2D.size(); i++) {
                const cg3::Point2d& p2D = squarePoints2D[i];

                cg3::Point3d newPoint(p2D.x(), p2D.y(), totalHeight);
                newPoint.rotate(inverseProjectionMatrix);

                unsigned int vId = result.addVertex(newPoint);

                downVertices[i] = vId;
                downPoints2DMap[p2D] = vId;
            }

            //Delaunay triangulation between second layer polygon and new square
            std::vector<std::array<cg3::Point2d, 3>> triangulation;
            std::vector<std::vector<cg3::Point2d>> holes;

            holes.resize(1);
            holes[0] = currentSecondLayerPoints2D;
            triangulation = cg3::cgal::triangulate2(squarePoints2D, holes);

            //Add triangulation to result
            for (std::array<cg3::Point2d, 3>& triangle : triangulation) {
                bool isThereNewVertex = false; //Flag to check if a new point has been created (flipped triangles on projection)

                unsigned int v[3];

                for (unsigned int i = 0; i < 3 && !isThereNewVertex; i++) {
                    if (currentSecondLayerPoints2DMap.find(triangle[i]) != currentSecondLayerPoints2DMap.end()) {
                        v[i] = currentSecondLayerPoints2DMap.at(triangle[i]);
                    }
                    else if (downPoints2DMap.find(triangle[i]) != downPoints2DMap.end()) {
                        v[i] = downPoints2DMap.at(triangle[i]);
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


            totalHeight += secondLayerStepHeight;
            totalHeight = std::min(totalHeight, boxHeight);

            //Create new 2D square (up)
            std::map<cg3::Point2d, unsigned int> upPoints2DMap;

            //Adding up square vertices
            std::vector<unsigned int> upVertices(newVerticesNumber);

            for (size_t i = 0; i < squarePoints2D.size(); i++) {
                const cg3::Point2d& p2D = squarePoints2D[i];

                cg3::Point3d newPoint(p2D.x(), p2D.y(), totalHeight);
                newPoint.rotate(inverseProjectionMatrix);

                unsigned int vid = result.addVertex(newPoint);
                upVertices[i] = vid;

                upPoints2DMap[p2D] = vid;
            }

            for(unsigned int i = 0; i < newVerticesNumber; i++){
                result.addFace(upVertices[i], downVertices[(i+1)%newVerticesNumber], downVertices[i]);
                result.addFace(upVertices[i], upVertices[(i+1)%newVerticesNumber], downVertices[(i+1)%newVerticesNumber]);
            }

            currentSecondLayerPoints2D = squarePoints2D;
            currentSecondLayerPoints2DMap = upPoints2DMap;
        }


        /* ----- BOX ----- */

        //Create new 2D square
        std::map<cg3::Point2d, unsigned int> boxUpperPoints2DMap;
        std::vector<cg3::Point2d> boxUpperPoints2D(4);
        boxUpperPoints2D[0] = cg3::Point2d(-boxWidth, -boxHeight);
        boxUpperPoints2D[1] = cg3::Point2d(+boxWidth, -boxHeight);
        boxUpperPoints2D[2] = cg3::Point2d(+boxWidth, +boxHeight);
        boxUpperPoints2D[3] = cg3::Point2d(-boxWidth, +boxHeight);

        //Adding box vertices
        std::vector<unsigned int> squareVertices(4);

        for (size_t i = 0; i < boxUpperPoints2D.size(); i++) {
            const cg3::Point2d& p2D = boxUpperPoints2D[i];

            cg3::Point3d newPoint(p2D.x(), p2D.y(), boxHeight);
            newPoint.rotate(inverseProjectionMatrix);

            unsigned int vid = result.addVertex(newPoint);
            squareVertices[i] = vid;

            boxUpperPoints2DMap[p2D] = vid;
        }

        //Delaunay triangulation between second layer polygon and square polygon
        std::vector<std::array<cg3::Point2d, 3>> triangulation;
        std::vector<std::vector<cg3::Point2d>> holes;

        holes.resize(1);
        holes[0] = currentSecondLayerPoints2D;
        triangulation = cg3::cgal::triangulate2(boxUpperPoints2D, holes);

        //Add triangulation to result
        for (std::array<cg3::Point2d, 3>& triangle : triangulation) {
            bool isThereNewVertex = false; //Flag to check if a new point has been created (flipped triangles on projection)

            unsigned int v[3];

            for (unsigned int i = 0; i < 3 && !isThereNewVertex; i++) {
                if (currentSecondLayerPoints2DMap.find(triangle[i]) != currentSecondLayerPoints2DMap.end()) {
                    v[i] = currentSecondLayerPoints2DMap.at(triangle[i]);
                }
                else if (boxUpperPoints2DMap.find(triangle[i]) != boxUpperPoints2DMap.end()) {
                    v[i] = boxUpperPoints2DMap.at(triangle[i]);
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

        cg3::Point3d p;

        p = cg3::Point3d(-boxWidth, -boxHeight, -boxHeight);
        p.rotate(inverseProjectionMatrix);
        boxVertices[4] = result.addVertex(p);

        p = cg3::Point3d(boxWidth, -boxHeight, -boxHeight);
        p.rotate(inverseProjectionMatrix);
        boxVertices[5] = result.addVertex(p);

        p = cg3::Point3d(boxWidth, boxHeight, -boxHeight);
        p.rotate(inverseProjectionMatrix);
        boxVertices[6] = result.addVertex(p);

        p = cg3::Point3d(-boxWidth, boxHeight, -boxHeight);
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
    }

    /* ----- HOLE FILLING ----- */

    std::cout << std::endl <<  "----- HOLE FILLING ----- " << std::endl << std::endl;

    for (size_t rId = 0; rId < nResults; rId++) {
        //Copying the surface and getting its label
        cg3::EigenMesh& result = tmpResults[rId];

        std::cout << "Hole filling for result " << rId << ":" << std::endl;

        //Hole filling
        result = cg3::EigenMesh(cg3::cgal::holeFillingTriangulation(result));
    }


    std::cout << std::endl <<  "----- CLEANING ----- " << std::endl << std::endl;

    for (size_t rId = 0; rId < nResults; rId++) {
        if (fourAxisChartData.isExtreme.at(resultToChart.at(rId))) {
            //Copying the surface and getting its label
            cg3::EigenMesh& result = tmpResults[rId];
            unsigned int targetLabel = tmpResultsAssociation[rId];

            std::cout << "Extreme " << rId << ": cleaning." << std::endl;

            //If it is an extreme, clear all the stock around it (until the extreme level set)
            result = cg3::libigl::difference(result, targetLabel == minLabel ? minSupport : maxSupport);
        }
    }


    /* ----- UNION WITH THE ORIGINAL MESH ----- */

    std::cout << std::endl <<  "----- UNION WITH ORIGINAL MESH ----- " << std::endl << std::endl;

    //Csg tree
    CSGTree csgFourAxisComponent = cg3::libigl::eigenMeshToCSGTree(fourAxisComponent);

    cg3::Array2D<double> costMatrix(nResults, nResults, 0);

    boxes.resize(nResults);
    for (size_t rId = 0; rId < nResults; rId++) {
        //Copying the surface and getting its label
        cg3::EigenMesh& result = tmpResults[rId];
        cg3::EigenMesh& box = boxes[rId];
        unsigned int resultLabel = tmpResultsAssociation[rId];

        unsigned int resultNFaces = result.numberFaces();

        //CSG tree union
        CSGTree csgTmpResult = cg3::libigl::eigenMeshToCSGTree(result);
        CSGTree csgResult = cg3::libigl::union_(csgFourAxisComponent, csgTmpResult);

        //Counting birth faces
        CSGTree::VectorJ birthFaces = csgResult.J();

        //Saving the mesh from the result
        result = cg3::EigenMesh(cg3::libigl::CSGTreeToEigenMesh(csgResult));

        box = result;

        unsigned int nFirstFaces = csgFourAxisComponent.F().rows();
        unsigned int nSecondFaces = csgTmpResult.F().rows();

        double totalCost = 0;

        for (unsigned int i = 0; i < birthFaces.rows(); i++) {
            unsigned int birthFace = birthFaces[i];

            //If the birth face is in the first mesh
            if (birthFace < nFirstFaces) {
                unsigned int birthChartId = fourAxisChartData.faceChartMap.at(birthFace);

                unsigned int conflictLabel = tmpResultsAssociation[chartToResult[birthChartId]];

                if (conflictLabel != resultLabel) {
                    costMatrix(rId, chartToResult[birthChartId]) += result.faceArea(i);

                    totalCost += result.faceArea(i);

                    //Color other chart face of the mesh
                    box.setFaceColor(colorMap[fourAxisAssociation[birthFace]], i);
                }
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
    std::vector<double> cost(nResults, 0);
    std::vector<double> gain(nResults, 0);
    for (size_t i = 0; i < nResults; i++) {
        for (size_t j = 0; j < nResults; j++) {
            if (i != j) {
                cost[i] += costMatrix(i,j);
                gain[j] += costMatrix(i,j);
            }
        }
    }
    std::cout << "Result\tCost\tGain" << std::endl;
    for (size_t i = 0; i < nResults; i++) {
        std::cout << i << "\t" << cost[i] << "\t" << gain[i] << std::endl;
    }
    std::cout << std::endl;


    std::cout << std::endl << "Result\tCost" << std::endl;

    size_t minLabelResult = std::numeric_limits<size_t>::max(), maxLabelResult = std::numeric_limits<size_t>::max();
    for (size_t i = 0; i < nResults; i++) {
        if (tmpResultsAssociation[i] == minLabel) {
            minLabelResult = i;
        }
        if (tmpResultsAssociation[i] == maxLabel) {
            maxLabelResult = i;
        }
    }

    //X directions before
    if (!xDirectionsAfter) {
        resultPosition[minFirst ? minLabelResult : maxLabelResult] = currentPosition;
        currentPosition++;

        resultPosition[minFirst ? maxLabelResult : minLabelResult] = currentPosition;
        currentPosition++;
    }

    //Four axis result
    internal::getFabricationOrder(tmpResultsAssociation, costMatrix, minLabel, maxLabel, cost, gain, currentPosition, totalCost, resultPosition);

    //X directions after
    if (xDirectionsAfter) {
        resultPosition[minFirst ? minLabelResult : maxLabelResult] = currentPosition;
        currentPosition++;

        resultPosition[minFirst ? maxLabelResult : minLabelResult] = currentPosition;
        currentPosition++;
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

    double totalArea = 0;
    for (unsigned int i = 0; i < fourAxisComponent.numberFaces(); i++)
        totalArea += fourAxisComponent.faceArea(i);
    std::cout << std::endl << "Area fabricated from other directions: " << totalCost << " (w.r.t. total area (" << totalArea << "): " << totalCost/totalArea << ")" << std::endl;



    /* ----- IMMERSION IN THE STOCK AND STOCK GENERATIONS ----- */

    //Retrieving stock mesh
    cg3::EigenMesh currentStock = data.stock;

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

    minResult = minComponent;
    maxResult = maxComponent;


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
 * @brief Compute polygon offset
 * @param polygon Polygon
 * @param offset Offset
 * @return
 */
std::vector<cg3::Point2d> offsetPolygon(std::vector<cg3::Point2d>& polygon, const double offset) {
    using namespace ClipperLib;

    const double INT_DOUBLE_TRANSLATION = 1e+6;

    Path subj;
    Paths solution;

    for (cg3::Point2d& point : polygon) {
        subj <<
                IntPoint(
                    static_cast<long long int>(point.x() * INT_DOUBLE_TRANSLATION),
                    static_cast<long long int>(point.y() * INT_DOUBLE_TRANSLATION));
    }

    ClipperOffset co;
    co.AddPath(subj, jtSquare, etClosedPolygon);
    co.Execute(solution, offset * INT_DOUBLE_TRANSLATION);

    size_t bestIndex = 0;
    double bestArea = -std::numeric_limits<double>::max();
    for (size_t i = 0; i < solution.size(); i++) {
        Path& path = solution[i];
        double area = Area(path);

        if (area >= bestArea) {
            bestArea = area;
            bestIndex = i;
        }
    }

    std::vector<cg3::Point2d> result;
    for (IntPoint& ip : solution[bestIndex]) {
        result.push_back(cg3::Point2d(ip.X / INT_DOUBLE_TRANSLATION, ip.Y / INT_DOUBLE_TRANSLATION));
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
        const std::vector<unsigned int>& resultAssociation,
        const cg3::Array2D<double>& costMatrix,
        const size_t minLabel,
        const size_t maxLabel,
        std::vector<double>& cost,
        std::vector<double>& gain,
        size_t& currentPosition,
        double& totalCost,
        std::vector<size_t>& resultPosition)
{
    size_t nResults = resultAssociation.size();

    size_t minCostIndex;
    double minCost;

    do {
        minCostIndex = 0;
        minCost = std::numeric_limits<int>::max();
        for (size_t i = 0; i < nResults; i++) {
            if (resultPosition[i] == std::numeric_limits<int>::max()) {
                bool isExtremes = resultAssociation[i] == minLabel || resultAssociation[i] == maxLabel;

                if (!isExtremes) {
                    if (cost[i] < minCost || (cg3::epsilonEqual(cost[i], minCost) && gain[i] > gain[minCostIndex])) {
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


std::vector<cg3::Point2d> createSquare(
        unsigned int nSideVertex,
        const cg3::Point3d& minCoord,
        const cg3::Point3d& maxCoord)
{
    std::vector<cg3::Point2d> squarePoints2D(nSideVertex * 4);

    double xStepLenght = (maxCoord.x() - minCoord.x()) / nSideVertex;
    double yStepLenght = (maxCoord.y() - minCoord.y()) / nSideVertex;

    unsigned int offset = 0;
    for(unsigned int i = 0; i < nSideVertex; i++) {
        squarePoints2D[offset + i] = cg3::Point2d(minCoord.x() + (xStepLenght * (i % nSideVertex)), minCoord.y());
    }

    offset += nSideVertex;
    for(unsigned int i = 0; i < nSideVertex; i++) {
        squarePoints2D[offset + i] = cg3::Point2d(maxCoord.x(), minCoord.y() + (yStepLenght * (i % nSideVertex)));
    }

    offset += nSideVertex;
    for(unsigned int i = 0; i < nSideVertex; i++) {
        squarePoints2D[offset + i] = cg3::Point2d(maxCoord.x() - (xStepLenght * (i % nSideVertex)), maxCoord.y());
    }

    offset += nSideVertex;
    for(unsigned int i = 0; i < nSideVertex; i++) {
        squarePoints2D[offset + i] = cg3::Point2d(minCoord.x(), maxCoord.y() - (yStepLenght * (i % nSideVertex)));
    }

    return squarePoints2D;
}


} //namespace internal

}
