/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_extraction.h"

#include <cg3/geometry/transformations.h>
#include <cg3/geometry/2d/point2d.h>

#include <cg3/libigl/mesh_adjacencies.h>

#include <cg3/meshes/eigenmesh/algorithms/eigenmesh_algorithms.h>
#include <cg3/meshes/dcel/dcel.h>

#include <cg3/cgal/2d/triangulation2d.h>
#include <cg3/geometry/2d/utils2d.h>

#include <cg3/libigl/booleans.h>
#include <cg3/libigl/connected_components.h>

#define CYLINDER_SUBD 100

namespace FourAxisFabrication {

/* Useful function declaration */

namespace internal {

cg3::EigenMesh extractSurfaceWithLabel(
        const cg3::EigenMesh& mesh,
        const std::vector<int>& association,
        const std::vector<std::vector<int>>& vfAdj,
        const unsigned int targetLabel);

std::vector<unsigned int> computeExternalBorder(const cg3::SimpleEigenMesh& m);

} //namespace internal


/**
 * @brief Extract results meshes
 * @param[out] data Four axis fabrication data
 * @param[in] stockLength Length of the stock
 * @param[in] stockDiameter Diameter of the stock
 * @param[in] stepHeight Max millable height
 * @param[in] stepWidth Length of each step
 * @param[in] rotateMeshes Rotate resulting meshes on the given direction
 */
void extractResults(
        Data& data,
        const double stockLength,
        const double stockDiameter,
        const double stepHeight,
        const double stepWidth,
        const bool rotateMeshes)
{
    //Referencing input data
    const std::vector<unsigned int>& targetDirections = data.targetDirections;

    const cg3::EigenMesh& minComponent = data.minComponent;
    const cg3::EigenMesh& maxComponent = data.maxComponent;
    const cg3::EigenMesh& fourAxisComponent = data.fourAxisComponent;

    const std::vector<int>& fourAxisComponentAssociation = data.fourAxisComponentAssociation;
    const std::vector<int>& minComponentAssociation = data.minComponentAssociation;
    const std::vector<int>& maxComponentAssociation = data.maxComponentAssociation;

    //Referencing output data
    cg3::EigenMesh& stock = data.stock;

    std::vector<cg3::EigenMesh>& results = data.results;
    std::vector<unsigned int>& resultsAssociation = data.resultsAssociation;

    std::vector<cg3::EigenMesh>& surfaces = data.surfaces;
    std::vector<unsigned int>& surfacesAssociation = data.surfacesAssociation;


    //Common values
    unsigned int minLabel = data.targetDirections[data.targetDirections.size()-2];
    unsigned int maxLabel = data.targetDirections[data.targetDirections.size()-1];

    Eigen::Matrix3d rotationMatrix;
    const cg3::Vec3 xAxis(1,0,0);
    const cg3::Vec3 yAxis(0,1,0);


    //Get vertex-face adjacencies
    std::vector<std::vector<int>> fourAxisVFAdj = cg3::libigl::vertexToFaceIncidences(fourAxisComponent);
    std::vector<std::vector<int>> minVFAdj = cg3::libigl::vertexToFaceIncidences(minComponent);
    std::vector<std::vector<int>> maxVFAdj = cg3::libigl::vertexToFaceIncidences(maxComponent);



    /* ----- SCALE AND TRANSLATE MESHES ----- */

    //Copy four axis mesh
    cg3::EigenMesh fourAxisScaled = fourAxisComponent;
    cg3::EigenMesh minScaled = minComponent;
    cg3::EigenMesh maxScaled = maxComponent;

    //Center mesh
    cg3::Vec3 translateVec = -fourAxisScaled.boundingBox().center();
    fourAxisScaled.translate(translateVec);
    minScaled.translate(translateVec);
    maxScaled.translate(translateVec);

    //Get the scale factor
    cg3::BoundingBox bb = fourAxisComponent.boundingBox();
    double minX = bb.minX();
    double maxX = bb.maxX();
    const double scaleFactor = stockLength / (maxX - minX);

    //Scale meshes
    const cg3::Vec3 scaleVec(scaleFactor, scaleFactor, scaleFactor);
    fourAxisScaled.scale(scaleVec);
    minScaled.scale(scaleVec);
    maxScaled.scale(scaleVec);



    /* ----- SURFACES ----- */

    //Split the component in surface components
    for (size_t i = 0; i < targetDirections.size()-2; i++) {
        //Current direction label
        int targetLabel = targetDirections[i];

        //New surface mesh
        cg3::EigenMesh surface = internal::extractSurfaceWithLabel(
                    fourAxisScaled,
                    fourAxisComponentAssociation,
                    fourAxisVFAdj,
                    targetLabel);

        //Get connected components
        std::vector<cg3::SimpleEigenMesh> connectedComponents = cg3::libigl::connectedComponents(surface);

        //Add each components to the results
        for (cg3::SimpleEigenMesh& simpleComponent : connectedComponents) {
            cg3::EigenMesh component(simpleComponent);

            surfaces.push_back(component);
            surfacesAssociation.push_back(targetLabel);
        }
    }

    //Get surface of min component
    cg3::EigenMesh minSurface =
            internal::extractSurfaceWithLabel(
                minScaled,
                minComponentAssociation,
                minVFAdj,
                minLabel);

    surfaces.push_back(minSurface);
    surfacesAssociation.push_back(minLabel);

    //Get surface of max component
    cg3::EigenMesh maxSurface =
            internal::extractSurfaceWithLabel(
                maxScaled,
                maxComponentAssociation,
                maxVFAdj,
                maxLabel);

    if (rotateMeshes) {
        cg3::rotationMatrix(yAxis, -M_PI/2, rotationMatrix);
        maxSurface.rotate(rotationMatrix);
        maxSurface.translate(-maxSurface.boundingBox().center());
    }

    surfaces.push_back(maxSurface);
    surfacesAssociation.push_back(maxLabel);



    /* ----- STOCK ----- */

    //Creating stock mesh
    double stockRadius = stockDiameter/2;
    double stockHalfLength = stockLength/2;
    stock = cg3::EigenMesh(cg3::EigenMeshAlgorithms::makeCylinder(cg3::Pointd(-stockHalfLength,0,0), cg3::Pointd(+stockHalfLength,0,0), stockRadius, CYLINDER_SUBD));




    /* ----- RESULTS ----- */

    //Split the component in result components
    for (size_t sId = 0; sId < surfaces.size()-2; sId++) {
        //Copying the surface and getting its label
        cg3::EigenMesh result = surfaces[sId];

        int targetLabel = surfacesAssociation[sId];

        //Rotate the mesh in the z-axis
        double angle = data.angles[targetLabel];
        cg3::rotationMatrix(xAxis, angle, rotationMatrix);
        result.rotate(rotationMatrix);

        //Compute external borders
        std::vector<unsigned int> externalBorders = internal::computeExternalBorder(result);


        //Compute barycenter
        cg3::Pointd barycenter = 0;
        double minZ = std::numeric_limits<double>::max();
        for (unsigned int currentBorderId : externalBorders) {
            cg3::Pointd currentPoint = result.vertex(currentBorderId);
            minZ = std::min(minZ, currentPoint.z());

            barycenter += currentPoint;
        }
        barycenter /= externalBorders.size();


        //Width of the box
        const double width = stockHalfLength*1.2;
        const double height = stockRadius*1.2;


        //Compute steps layer
        double layerHeight = minZ + stepHeight;
        while (layerHeight < height) {
            double angle = M_PI/6;

            // ----- Creating vertices above the external borders -----

            //Compute new layer vertices
            std::vector<unsigned int> newLayer(externalBorders.size());
            for (size_t i = 0; i < externalBorders.size(); i++) {
                unsigned int currentBorderId = externalBorders[i];
                cg3::Pointd currentPoint = result.vertex(currentBorderId);

                cg3::Pointd newPoint = currentPoint;
                newPoint.setZ(layerHeight);

                if (currentPoint.z() > layerHeight) {
                    //TODO
                }

                unsigned int newPointId = result.addVertex(newPoint);

                newLayer[i] = newPointId;
            }

            //Add faces to the layer
            for (size_t i = 0; i < externalBorders.size(); i++) {
                unsigned int currentBorderId = externalBorders[i];
                unsigned int nextBorderId = externalBorders[(i+1) % externalBorders.size()];

                unsigned int currentNewId = newLayer[i];
                unsigned int nextNewId = newLayer[(i+1) % newLayer.size()];

                result.addFace(currentBorderId, currentNewId, nextBorderId);
                result.addFace(nextBorderId, currentNewId, nextNewId);
            }



            // ----- Triangulation among the layer vertices and the borders -----

            //Get bounding box
            result.updateBoundingBox();
            cg3::BoundingBox bb = result.boundingBox();

            std::map<cg3::Point2Dd, unsigned int> projectionMap;

            //Internal borders
            std::vector<cg3::Point2Dd> projectionHole(newLayer.size());
            for (unsigned int i = 0; i < newLayer.size(); i++) {
                cg3::Pointd point3D = result.vertex(newLayer[i]);
                cg3::Point2Dd projection = cg3::Point2Dd(point3D.x(), point3D.y());

                projectionHole[i] = projection;

                projectionMap[projection] = newLayer[i];
            }

            //External borders
            std::vector<unsigned int> newExternalBorders;

            //Create new 2D square
            std::vector<cg3::Point2Dd> newExternalBordersProjection(4);
            newExternalBordersProjection[0] = cg3::Point2Dd(bb.minX() - stepWidth, bb.minY() - stepWidth);
            newExternalBordersProjection[1] = cg3::Point2Dd(bb.maxX() + stepWidth, bb.minY() - stepWidth);
            newExternalBordersProjection[2] = cg3::Point2Dd(bb.maxX() + stepWidth, bb.maxY() + stepWidth);
            newExternalBordersProjection[3] = cg3::Point2Dd(bb.minX() - stepWidth, bb.maxY() + stepWidth);


            for (size_t i = 0; i < newExternalBordersProjection.size(); i++) {
                cg3::Point2Dd& squarePoint = newExternalBordersProjection[i];

                unsigned int vid = result.addVertex(squarePoint.x(), squarePoint.y(), layerHeight);
                projectionMap[squarePoint] = vid;

                newExternalBorders.push_back(vid);
            }

            //Holes
            std::vector<std::vector<cg3::Point2Dd>> holes;
            holes.push_back(projectionHole);

            //Triangulation
            std::vector<std::array<cg3::Point2Dd, 3>> triang =
                    cg3::cgal::triangulate(newExternalBordersProjection, holes);

            //Add triangulation to result
            for (std::array<cg3::Point2Dd, 3>& triangle : triang) {
                //Flag to check if a new point has been created (flipped triangles on projection)
                bool isValid = true;

                unsigned int v[3];

                for (unsigned int i = 0; i < 3 && isValid; i++) {
                    //If the vertex is not new (new triangle could be created
                    if (projectionMap.find(triangle[i]) != projectionMap.end()) {
                        v[i] = projectionMap.at(triangle[i]);
                    }
                    else {
                        isValid = false;
                    }
                }

                if (isValid) {
                    result.addFace(v[0], v[1], v[2]);
                }
            }


            // ----- Setting data for next iteration -----

            layerHeight += stepHeight;
            externalBorders = newExternalBorders;
        }

        //Creating new points of the mesh block
        result.setVertex(externalBorders[0], -width, -height, height);
        result.setVertex(externalBorders[1], width, -height, height);
        result.setVertex(externalBorders[2], width, height, height);
        result.setVertex(externalBorders[3], -width, height, height);
        externalBorders.push_back(result.addVertex(cg3::Pointd(-width, -height, -height)));
        externalBorders.push_back(result.addVertex(cg3::Pointd(width, -height, -height)));
        externalBorders.push_back(result.addVertex(cg3::Pointd(width, height, -height)));
        externalBorders.push_back(result.addVertex(cg3::Pointd(-width, height, -height)));

        //Creating faces to close the mesh
        result.addFace(externalBorders[2], externalBorders[1], externalBorders[5]);
        result.addFace(externalBorders[2], externalBorders[5], externalBorders[6]);

        result.addFace(externalBorders[5], externalBorders[1], externalBorders[0]);
        result.addFace(externalBorders[5], externalBorders[0], externalBorders[4]);

        result.addFace(externalBorders[6], externalBorders[5], externalBorders[4]);
        result.addFace(externalBorders[6], externalBorders[4], externalBorders[7]);

        result.addFace(externalBorders[7], externalBorders[4], externalBorders[0]);
        result.addFace(externalBorders[7], externalBorders[0], externalBorders[3]);

        result.addFace(externalBorders[7], externalBorders[3], externalBorders[2]);
        result.addFace(externalBorders[7], externalBorders[2], externalBorders[6]);


        //Intersection with the stock
//        result = cg3::libigl::intersection(stock, result);


        if (!rotateMeshes) {
            //Rotate back the mesh
            cg3::rotationMatrix(xAxis, -angle, rotationMatrix);
            result.rotate(rotationMatrix);
        }

        results.push_back(result);
        resultsAssociation.push_back(targetLabel);



//        //Point-id map
//        std::map<cg3::Point2Dd, unsigned int> idMap;

//        //Compute mesh borders and project in 2D
//        std::vector<unsigned int> internalBorderIds = internal::computeExternalBorder(surface);
//        std::vector<cg3::Point2Dd> internalBorder(internalBorderIds.size());

//        for (unsigned int i = 0; i < internalBorderIds.size(); i++) {
//            cg3::Pointd point3D = surface.vertex(internalBorderIds[i]);
//            cg3::Point2Dd projection = cg3::Point2Dd(point3D.x(), point3D.y());

//            internalBorder[i] = projection;

//            idMap[projection] = internalBorderIds[i];
//        }


//        const double length = stockHalfLength*1.2;
//        const double height = stockRadius*1.2;
//        const double zHeightUp = height * sin(surroundingAngle);
//        const double zHeightDown = stockRadius*1.2;

//        //Creating new points of the mesh block
//        std::vector<cg3::Pointd> newPoints(8);
//        std::vector<unsigned int> newPointsId(8);

//        newPoints[0] = cg3::Pointd(-length, -height, zHeightUp);
//        newPoints[1] = cg3::Pointd(length, -height, zHeightUp);
//        newPoints[2] = cg3::Pointd(length, height, zHeightUp);
//        newPoints[3] = cg3::Pointd(-length, height, zHeightUp);
//        newPoints[4] = cg3::Pointd(-length, -height, -zHeightDown);
//        newPoints[5] = cg3::Pointd(length, -height, -zHeightDown);
//        newPoints[6] = cg3::Pointd(length, height, -zHeightDown);
//        newPoints[7] = cg3::Pointd(-length, height, -zHeightDown);

//        for (unsigned int i = 0; i < newPoints.size(); i++) {
//            unsigned int vid = surface.addVertex(newPoints[i]);
//            newPointsId[i] = vid;
//        }

//        surface.addFace(newPointsId[2], newPointsId[1], newPointsId[5]);
//        surface.addFace(newPointsId[2], newPointsId[5], newPointsId[6]);

//        surface.addFace(newPointsId[5], newPointsId[1], newPointsId[0]);
//        surface.addFace(newPointsId[5], newPointsId[0], newPointsId[4]);

//        surface.addFace(newPointsId[6], newPointsId[5], newPointsId[4]);
//        surface.addFace(newPointsId[6], newPointsId[4], newPointsId[7]);

//        surface.addFace(newPointsId[7], newPointsId[4], newPointsId[0]);
//        surface.addFace(newPointsId[7], newPointsId[0], newPointsId[3]);

//        surface.addFace(newPointsId[7], newPointsId[3], newPointsId[2]);
//        surface.addFace(newPointsId[7], newPointsId[2], newPointsId[6]);

//        //External border
//        std::vector<cg3::Point2Dd> externalBorder(4);
//        for (int i = 0; i < 4; i++) {
//            externalBorder[i] = cg3::Point2Dd(newPoints[i].x(), newPoints[i].y());
//            idMap[externalBorder[i]] = newPointsId[i];
//        }

//        //Holes
//        std::vector<std::vector<cg3::Point2Dd>> holes;
//        holes.push_back(internalBorder);

//        //Triangulation
//        std::vector<std::array<cg3::Point2Dd, 3>> triang =
//                cg3::cgal::triangulate(externalBorder, holes);

//        for (std::array<cg3::Point2Dd, 3>& triangle : triang) {
//            //Flag to check if a new point has been created (flipped triangles on projection)
//            bool isValid = true;

//            unsigned int v[3];

//            for (unsigned int i = 0; i < 3 && isValid; i++) {
//                if (idMap.find(triangle[i]) != idMap.end()) {
//                    v[i] = idMap.at(triangle[i]);
//                }
//                else {
//                    isValid = false;
//                }
//            }

//            if (isValid) {
//                surface.addFace(v[0], v[1], v[2]);
//            }
//        }



//        result = cg3::libigl::intersection(stock, result);


//        results.push_back(result);
//        resultsAssociation.push_back(targetLabel);
    }


    //Min result
    cg3::EigenMesh minResult = minScaled;
    cg3::rotationMatrix(yAxis, M_PI/2, rotationMatrix);

    if (rotateMeshes) {
        minResult.rotate(rotationMatrix);
        minResult.translate(-minResult.boundingBox().center());
    }

    results.push_back(minResult);
    resultsAssociation.push_back(minLabel);


    //Max result
    cg3::EigenMesh maxResult = maxScaled;
    cg3::rotationMatrix(yAxis, -M_PI/2, rotationMatrix);

    if (rotateMeshes) {
        maxResult.rotate(rotationMatrix);
        maxResult.translate(-maxResult.boundingBox().center());
    }

    results.push_back(maxResult);
    resultsAssociation.push_back(maxLabel);




    //Update meshes data
    for (cg3::EigenMesh& surface : data.surfaces) {
        surface.updateBoundingBox();
        surface.updateFacesAndVerticesNormals();
    }
    for (cg3::EigenMesh& result : data.results) {
        result.updateBoundingBox();
        result.updateFacesAndVerticesNormals();
    }

}


namespace internal {

/**
 * @brief Extract all faces of a mesh with a given label.
 * The result could be non-watertight.
 * @param[in] mesh Input mesh
 * @param[in] association Association of the mesh with labels
 * @param[in] vfAdj vertex-face adjacencies of the mesh
 * @param[in] targetLabel Target label
 * @return Output mesh, it could be non-watertight
 */
cg3::EigenMesh extractSurfaceWithLabel(
        const cg3::EigenMesh& mesh,
        const std::vector<int>& association,
        const std::vector<std::vector<int>>& vfAdj,
        const unsigned int targetLabel)
{
    cg3::EigenMesh result;

    //Data structures to keep track of the new vertex
    std::vector<int> newVertexMap(mesh.numberVertices(), -1);
    unsigned int newVertexId = 0;

    //For each vertex
    for (unsigned int vId = 0; vId < mesh.numberVertices(); vId++) {
        const std::vector<int>& adjFaces = vfAdj[vId];

        //Check if the vertex belongs to at least one face with the given target label
        bool valid = false;
        for (unsigned int i = 0; i < adjFaces.size() && !valid; i++) {
            if (association[adjFaces[i]] == (int) targetLabel) {
                valid = true;
            }
        }

        //Add vertex and fill map
        if (valid) {
            result.addVertex(mesh.vertex(vId));
            newVertexMap[vId] = newVertexId;
            newVertexId++;
        }
    }

    //For each face
    for (unsigned int fId = 0; fId < mesh.numberFaces(); fId++) {
        const cg3::Pointi face = mesh.face(fId);

        if (association[fId] == (int) targetLabel) {
            result.addFace(
                    newVertexMap[face.x()],
                    newVertexMap[face.y()],
                    newVertexMap[face.z()]);
        }
    }

    return result;

}

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
            assert(vNext.find(fromId) == vNext.end());
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

} //namespace internal

}
