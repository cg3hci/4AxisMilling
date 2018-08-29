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
#include <cg3/geometry/2d/intersections2d.h>

#include <cg3/libigl/booleans.h>
#include <cg3/libigl/connected_components.h>

#include <lib/clipper/clipper.hpp>

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

std::vector<cg3::Point2Dd> offsetPolygon(std::vector<cg3::Point2Dd>& polygon, const double offset);

} //namespace internal


/**
 * @brief Extract results meshes
 * @param[out] data Four axis fabrication data
 * @param[in] stockLength Length of the stock
 * @param[in] stockDiameter Diameter of the stock
 * @param[in] stepHeight Max millable height
 * @param[in] stepWidth Length of each step
 * @param[in] millableAngle Angle needed to allow the fabrication of the surface
 * @param[in] rotateMeshes Rotate resulting meshes on the given direction
 */
void extractResults(
        Data& data,
        const double stockLength,
        const double stockDiameter,
        const double millableAngle,
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
    std::vector<cg3::EigenMesh>& stocks = data.stocks;

    std::vector<cg3::EigenMesh>& results = data.results;
    std::vector<unsigned int>& resultsAssociation = data.resultsAssociation;

    std::vector<cg3::EigenMesh>& surfaces = data.surfaces;
    std::vector<unsigned int>& surfacesAssociation = data.surfacesAssociation;


    //Common values
    unsigned int minLabel = data.targetDirections[data.targetDirections.size()-2];
    unsigned int maxLabel = data.targetDirections[data.targetDirections.size()-1];

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
        unsigned int targetLabel = targetDirections[i];

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

    if (rotateMeshes) {
        Eigen::Matrix3d rotationMatrix;
        cg3::rotationMatrix(yAxis, M_PI/2, rotationMatrix);
        minSurface.rotate(rotationMatrix);
        minSurface.translate(-minSurface.boundingBox().center());
    }

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
        Eigen::Matrix3d rotationMatrix;
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
    cg3::EigenMesh stock = cg3::EigenMesh(cg3::EigenMeshAlgorithms::makeCylinder(cg3::Pointd(-stockHalfLength,0,0), cg3::Pointd(+stockHalfLength,0,0), stockRadius, CYLINDER_SUBD));

    //Size of the box
    const double boxWidth = stockLength*2;
    const double boxHeight = stockDiameter*2;

    //Offset to get the millable angle
    double offset = tan(millableAngle) * stockDiameter; //TODO QUESTO NON E' GIUSTO!!!

    /* ----- RESULTS ----- */

    //Triangulation structures
    std::vector<std::array<cg3::Point2Dd, 3>> triangulation;
    std::vector<std::vector<cg3::Point2Dd>> holes;

    for (size_t sId = 0; sId < surfaces.size()-2; sId++) {
        //Copying the surface and getting its label
        cg3::EigenMesh result = surfaces[sId];
        cg3::EigenMesh rotateResult = result;

        unsigned int targetLabel = surfacesAssociation[sId];

        //Get projection matrix and its inverse for 2D projection
        Eigen::Matrix3d projectionMatrix;
        Eigen::Matrix3d inverseProjectionMatrix;
        double directionAngle = data.angles[targetLabel];
        cg3::rotationMatrix(xAxis, directionAngle, projectionMatrix);
        cg3::rotationMatrix(xAxis, -directionAngle, inverseProjectionMatrix);

        //Project the copy of the mesh
        rotateResult.rotate(projectionMatrix);

        //Compute external borders
        std::vector<unsigned int> extVertices = internal::computeExternalBorder(result);

        //Compute minZ
        double minZ = std::numeric_limits<double>::max();
        for (unsigned int vId : extVertices) {
            cg3::Pointd p = rotateResult.vertex(vId);
            minZ = std::min(minZ, p.z());
        }



        //Get the 2D projection of the borders
        std::map<cg3::Point2Dd, unsigned int> extProjectionMap;
        std::vector<cg3::Point2Dd> polygon2D;
        for (unsigned int vId : extVertices) {
            cg3::Pointd p = rotateResult.vertex(vId);
            cg3::Point2Dd p2D(p.x(), p.y());
            polygon2D.push_back(p2D);

            extProjectionMap[p2D] = vId;
        }

        //Get offset polygon
        std::vector<cg3::Point2Dd> offsetPolygon2D =
                internal::offsetPolygon(polygon2D, offset);

        //Set for finding points belonging to offset polygon
        std::set<cg3::Point2Dd> offsetPolygonPointSet;
        for (const cg3::Point2Dd& p : offsetPolygon2D) {
            offsetPolygonPointSet.insert(p);
        }

        //Delaunay triangulation of the offset with the surface
        holes.resize(1);
        holes[0] = offsetPolygon2D;
        triangulation = cg3::cgal::triangulate(polygon2D, holes);

        //Add triangulation to result
        std::map<cg3::Point2Dd, unsigned int> newProjectionMap;
        for (std::array<cg3::Point2Dd, 3>& triangle : triangulation) {
            //Flag to check if the triangle is not composed of only vertices of the offset polygon
            bool isThereExtVertex = false;

            //Flag to check if a new point has been created (flipped triangles on projection)
            bool isThereNewVertex = false;

            for (unsigned int i = 0; i < 3 && !isThereNewVertex; i++) {
                //If the vertex is among the external vertices
                if (extProjectionMap.find(triangle[i]) != extProjectionMap.end()) {
                    isThereExtVertex = true;
                }
                //If the vertex is not among the existing vertices
                else if (offsetPolygonPointSet.find(triangle[i]) == offsetPolygonPointSet.end()) {
                    isThereNewVertex = true;
                }
            }

            if (isThereExtVertex && !isThereNewVertex) {
                unsigned int v[3];

                //Find new vertices which have to be higher in z-coordinate
                std::vector<unsigned int> triangleNewVertices;
                std::vector<unsigned int> triangleExtVertices;

                for (unsigned int i = 0; i < 3; i++) {
                    unsigned int newPointId;

                    if (offsetPolygonPointSet.find(triangle[i]) != offsetPolygonPointSet.end()) {
                        std::map<cg3::Point2Dd, unsigned int>::const_iterator findIt = newProjectionMap.find(triangle[i]);

                        if (findIt != newProjectionMap.end()) {
                            newPointId = findIt->second;
                        }
                        else {
                            const cg3::Point2Dd& p = triangle[i];
                            cg3::Pointd newPoint(p.x(), p.y(), stockDiameter);

                            //Inverse projection
                            newPoint.rotate(inverseProjectionMatrix);

                            newPointId = result.addVertex(newPoint);
                            triangleNewVertices.push_back(newPointId);

                            newProjectionMap[p] = newPointId;
                        }

                        v[i] = newPointId;
                    }
                    else {
                        unsigned int extPointId = extProjectionMap.at(triangle[i]);
                        triangleExtVertices.push_back(extPointId);

                        v[i] = extPointId;
                    }
                }

                result.addFace(v[0], v[1], v[2]);
            }
        }

        //Get the vertices of the offset polygon that have been included
        std::vector<unsigned int> newVertices;
        std::vector<cg3::Point2Dd> remainingOffsetPolygon2D;
        for (const cg3::Point2Dd& p : offsetPolygon2D) {
            std::map<cg3::Point2Dd, unsigned int>::const_iterator findIt = newProjectionMap.find(p);
            if (findIt != newProjectionMap.end()) {
                remainingOffsetPolygon2D.push_back(p);
                newVertices.push_back(findIt->second);
            }
        }

        //Get bounding box
        rotateResult = result;
        rotateResult.rotate(projectionMatrix);
        rotateResult.updateBoundingBox();
        cg3::BoundingBox bb = rotateResult.boundingBox();

        //Create new 2D square
        std::map<cg3::Point2Dd, unsigned int> squareProjectionMap;
        std::vector<cg3::Point2Dd> squarePolygon(4);
        squarePolygon[0] = cg3::Point2Dd(-boxWidth, -boxHeight);
        squarePolygon[1] = cg3::Point2Dd(+boxWidth, -boxHeight);
        squarePolygon[2] = cg3::Point2Dd(+boxWidth, +boxHeight);
        squarePolygon[3] = cg3::Point2Dd(-boxWidth, +boxHeight);

        //Adding box vertices
        std::vector<unsigned int> squareVertices(4);

        for (size_t i = 0; i < squarePolygon.size(); i++) {
            const cg3::Point2Dd& countourPoint = squarePolygon[i];
            cg3::Pointd newPoint(countourPoint.x(), countourPoint.y(), boxHeight);
            newPoint.rotate(inverseProjectionMatrix);

            unsigned int vid = result.addVertex(newPoint);
            squareVertices[i] = vid;

            squareProjectionMap[countourPoint] = vid;
        }

        //Delaunay triangulation between offset polygon and square polygon
        holes[0] = remainingOffsetPolygon2D;
        std::vector<std::array<cg3::Point2Dd, 3>> triang =
                cg3::cgal::triangulate(squarePolygon, holes);

        //Add triangulation to result
        for (std::array<cg3::Point2Dd, 3>& triangle : triang) {
            //Flag to check if a new point has been created (flipped triangles on projection)
            bool isValid = true;

            unsigned int v[3];

            for (unsigned int i = 0; i < 3 && isValid; i++) {
                if (newProjectionMap.find(triangle[i]) != newProjectionMap.end()) {
                    v[i] = newProjectionMap.at(triangle[i]);
                }
                else if (squareProjectionMap.find(triangle[i]) != squareProjectionMap.end()) {
                    v[i] = squareProjectionMap.at(triangle[i]);
                }
                //If the vertex is not among the existing vertices
                else {
                    isValid = false;
                }
            }

            if (isValid) {
                result.addFace(v[0], v[1], v[2]);
            }
        }

        //Box for closing the mesh
        std::vector<unsigned int> boxVertices;
        boxVertices.reserve(8);

        boxVertices.push_back(squareVertices[0]);
        boxVertices.push_back(squareVertices[1]);
        boxVertices.push_back(squareVertices[2]);
        boxVertices.push_back(squareVertices[3]);

        cg3::Pointd p;

        p = cg3::Pointd(-boxWidth, -boxHeight, -boxHeight);
        p.rotate(inverseProjectionMatrix);
        boxVertices.push_back(result.addVertex(p));

        p = cg3::Pointd(boxWidth, -boxHeight, -boxHeight);
        p.rotate(inverseProjectionMatrix);
        boxVertices.push_back(result.addVertex(p));

        p = cg3::Pointd(boxWidth, boxHeight, -boxHeight);
        p.rotate(inverseProjectionMatrix);
        boxVertices.push_back(result.addVertex(p));

        p = cg3::Pointd(-boxWidth, boxHeight, -boxHeight);
        p.rotate(inverseProjectionMatrix);
        boxVertices.push_back(result.addVertex(p));

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

        unsigned int resultNFaces = result.numberFaces();                


        result = cg3::libigl::union_(fourAxisScaled, result);

        //Check if there is some difference with the number of faces
        int diffFaces = static_cast<int>(result.numberFaces()) - static_cast<int>(resultNFaces);
        if (diffFaces != 0)
            std::cout << "Result " << sId << ": difference of " << diffFaces << " faces." << std::endl;

        //Intersection with the stock
        result = cg3::libigl::intersection(stock, result);


        //Add results
        results.push_back(result);
        resultsAssociation.push_back(targetLabel);

        //Add stock
        stocks.push_back(stock);

        //New stock for next iteration
        stock = result;
    }

    //Min result
    cg3::EigenMesh minResult = minScaled;

    if (rotateMeshes) {
        Eigen::Matrix3d rotationMatrix;
        cg3::rotationMatrix(yAxis, M_PI/2, rotationMatrix);
        minResult.rotate(rotationMatrix);
        minResult.translate(-minResult.boundingBox().center());
    }

    minResult.updateBoundingBox();
    minResult.updateFacesAndVerticesNormals();

    results.push_back(minResult);
    resultsAssociation.push_back(minLabel);


    //Max result
    cg3::EigenMesh maxResult = maxScaled;

    if (rotateMeshes) {
        Eigen::Matrix3d rotationMatrix;
        cg3::rotationMatrix(yAxis, -M_PI/2, rotationMatrix);
        maxResult.rotate(rotationMatrix);
        maxResult.translate(-maxResult.boundingBox().center());
    }

    maxResult.updateBoundingBox();
    maxResult.updateFacesAndVerticesNormals();

    results.push_back(maxResult);
    resultsAssociation.push_back(maxLabel);




    //Update meshes data
    for (size_t i = 0; i < data.surfaces.size()-2; i++) {
        cg3::EigenMesh& surface = data.surfaces[i];

        if (rotateMeshes) {
            unsigned int& targetLabel = data.resultsAssociation[i];

            Eigen::Matrix3d resultRotationMatrix;
            double directionAngle = data.angles[targetLabel];
            cg3::rotationMatrix(xAxis, directionAngle, resultRotationMatrix);

            surface.rotate(resultRotationMatrix);
        }

        surface.updateBoundingBox();
        surface.updateFacesAndVerticesNormals();
    }

    for (size_t i = 0; i < data.stocks.size(); i++) {
        cg3::EigenMesh& stock = data.stocks[i];

        if (rotateMeshes) {
            unsigned int& targetLabel = data.resultsAssociation[i];

            Eigen::Matrix3d resultRotationMatrix;
            double directionAngle = data.angles[targetLabel];
            cg3::rotationMatrix(xAxis, directionAngle, resultRotationMatrix);

            stock.rotate(resultRotationMatrix);
        }

        stock.updateBoundingBox();
        stock.updateFacesAndVerticesNormals();
    }

    for (size_t i = 0; i < data.results.size()-2; i++) {
        cg3::EigenMesh& result = data.results[i];

        if (rotateMeshes) {
            unsigned int& targetLabel = data.resultsAssociation[i];

            Eigen::Matrix3d resultRotationMatrix;
            double directionAngle = data.angles[targetLabel];
            cg3::rotationMatrix(xAxis, directionAngle, resultRotationMatrix);

            result.rotate(resultRotationMatrix);
        }

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


} //namespace internal

}
