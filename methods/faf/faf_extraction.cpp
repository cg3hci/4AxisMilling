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
 * @param[in] surroundingAngle Angle for the surrounding triangle
 * @param[in] rotateMeshes Rotate resulting meshes on the given direction
 */
void extractResults(
        Data& data,
        const double stockLength,
        const double stockDiameter,
        const double surroundingAngle,
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
    cg3::Vec3 translateVec = -fourAxisScaled.getBoundingBox().center();
    fourAxisScaled.translate(translateVec);
    minScaled.translate(translateVec);
    maxScaled.translate(translateVec);

    //Get the scale factor
    cg3::BoundingBox bb = fourAxisComponent.getBoundingBox();
    double minX = bb.getMinX();
    double maxX = bb.getMaxX();
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


        if (rotateMeshes) {
            double angle = data.angles[targetLabel];
            cg3::getRotationMatrix(xAxis, angle, rotationMatrix);

            surface.rotate(rotationMatrix);
        }

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
        cg3::getRotationMatrix(yAxis, M_PI/2, rotationMatrix);
        minSurface.rotate(rotationMatrix);
        minSurface.translate(-minSurface.getBoundingBox().center());
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
        cg3::getRotationMatrix(yAxis, -M_PI/2, rotationMatrix);
        maxSurface.rotate(rotationMatrix);
        maxSurface.translate(-maxSurface.getBoundingBox().center());
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
    for (size_t i = 0; i < targetDirections.size()-2; i++) {
        //Current direction label
        int targetLabel = targetDirections[i];

        //New component mesh
        cg3::EigenMesh surface = internal::extractSurfaceWithLabel(
                    fourAxisScaled,
                    fourAxisComponentAssociation,
                    fourAxisVFAdj,
                    targetLabel);


        //Rotate the mesh in the z-axis
        double angle = data.angles[targetLabel];
        cg3::getRotationMatrix(xAxis, angle, rotationMatrix);
        surface.rotate(rotationMatrix);


        //Get connected components
        std::vector<cg3::SimpleEigenMesh> connectedComponents = cg3::libigl::connectedComponents(surface);

        //Add each components to the results
        for (cg3::SimpleEigenMesh& simpleComponent : connectedComponents) {
            cg3::EigenMesh component(simpleComponent);

            //Point-id map
            std::map<cg3::Point2Dd, unsigned int> idMap;

            //Compute mesh borders and project in 2D
            std::vector<unsigned int> internalBorderIds = internal::computeExternalBorder(component);
            std::vector<cg3::Point2Dd> internalBorder(internalBorderIds.size());

            for (unsigned int i = 0; i < internalBorderIds.size(); i++) {
                cg3::Pointd point3D = component.getVertex(internalBorderIds[i]);
                cg3::Point2Dd projection = cg3::Point2Dd(point3D.x(), point3D.y());

                internalBorder[i] = projection;

                idMap[projection] = internalBorderIds[i];
            }


            const double length = stockHalfLength*1.2;
            const double height = stockRadius*1.2;
            const double zHeightUp = height * sin(surroundingAngle);
            const double zHeightDown = stockRadius*1.05;

            //Creating new points of the mesh block
            std::vector<cg3::Pointd> newPoints(8);
            std::vector<unsigned int> newPointsId(8);

            newPoints[0] = cg3::Pointd(-length, -height, zHeightUp);
            newPoints[1] = cg3::Pointd(length, -height, zHeightUp);
            newPoints[2] = cg3::Pointd(length, height, zHeightUp);
            newPoints[3] = cg3::Pointd(-length, height, zHeightUp);
            newPoints[4] = cg3::Pointd(-length, -height, -zHeightDown);
            newPoints[5] = cg3::Pointd(length, -height, -zHeightDown);
            newPoints[6] = cg3::Pointd(length, height, -zHeightDown);
            newPoints[7] = cg3::Pointd(-length, height, -zHeightDown);

            for (unsigned int i = 0; i < newPoints.size(); i++) {
                unsigned int vid = component.addVertex(newPoints[i]);
                newPointsId[i] = vid;
            }

            component.addFace(newPointsId[2], newPointsId[1], newPointsId[5]);
            component.addFace(newPointsId[2], newPointsId[5], newPointsId[6]);

            component.addFace(newPointsId[5], newPointsId[1], newPointsId[0]);
            component.addFace(newPointsId[5], newPointsId[0], newPointsId[4]);

            component.addFace(newPointsId[6], newPointsId[5], newPointsId[4]);
            component.addFace(newPointsId[6], newPointsId[4], newPointsId[7]);

            component.addFace(newPointsId[7], newPointsId[4], newPointsId[0]);
            component.addFace(newPointsId[7], newPointsId[0], newPointsId[3]);

            component.addFace(newPointsId[7], newPointsId[3], newPointsId[2]);
            component.addFace(newPointsId[7], newPointsId[2], newPointsId[6]);

            //External border
            std::vector<cg3::Point2Dd> externalBorder(4);
            for (int i = 0; i < 4; i++) {
                externalBorder[i] = cg3::Point2Dd(newPoints[i].x(), newPoints[i].y());
                idMap[externalBorder[i]] = newPointsId[i];
            }

            //Holes
            std::vector<std::vector<cg3::Point2Dd>> holes;
            holes.push_back(internalBorder);

            //Triangulation
            std::vector<std::array<cg3::Point2Dd, 3>> triang =
                    cg3::cgal::triangulate(externalBorder, holes);

            for (std::array<cg3::Point2Dd, 3>& triangle : triang) {
                //Flag to check if a new point has been created (flipped triangles on projection)
                bool isValid = true;

                unsigned int v[3];

                for (unsigned int i = 0; i < 3 && isValid; i++) {
                    if (idMap.find(triangle[i]) != idMap.end()) {
                        v[i] = idMap.at(triangle[i]);
                    }
                    else {
                        isValid = false;
                    }
                }

                if (isValid) {
                    component.addFace(v[0], v[1], v[2]);
                }
            }


            component = cg3::libigl::intersection(stock, component);


            if (!rotateMeshes) {
                //Rotate back the mesh
                cg3::getRotationMatrix(xAxis, -angle, rotationMatrix);
                component.rotate(rotationMatrix);
            }

            results.push_back(component);
            resultsAssociation.push_back(targetLabel);
        }
    }


    //Min result
    cg3::EigenMesh minResult = minScaled;
    cg3::getRotationMatrix(yAxis, M_PI/2, rotationMatrix);

    if (rotateMeshes) {
        minResult.rotate(rotationMatrix);
        minResult.translate(-minResult.getBoundingBox().center());
    }

    results.push_back(minResult);
    resultsAssociation.push_back(minLabel);


    //Max result
    cg3::EigenMesh maxResult = maxScaled;
    cg3::getRotationMatrix(yAxis, -M_PI/2, rotationMatrix);

    if (rotateMeshes) {
        maxResult.rotate(rotationMatrix);
        maxResult.translate(-maxResult.getBoundingBox().center());
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
    std::vector<int> newVertexMap(mesh.getNumberVertices(), -1);
    unsigned int newVertexId = 0;

    //For each vertex
    for (unsigned int vId = 0; vId < mesh.getNumberVertices(); vId++) {
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
            result.addVertex(mesh.getVertex(vId));
            newVertexMap[vId] = newVertexId;
            newVertexId++;
        }
    }

    //For each face
    for (unsigned int fId = 0; fId < mesh.getNumberFaces(); fId++) {
        const cg3::Pointi face = mesh.getFace(fId);

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
 * @brief Compute borders of a eigen mesh
 * @param m Eigen mesh
 * @return Vector of id vertex of the borders
 */
std::vector<unsigned int> computeExternalBorder(const cg3::SimpleEigenMesh& m)
{
    //TODO without dcel

    cg3::Dcel d(m);

    //Border calculation
    std::map<unsigned int, unsigned int> nextMap;

    for (const cg3::Dcel::HalfEdge* he : d.halfEdgeIterator()){
        if (he->getTwin() == nullptr){
            assert(nextMap.find(he->getFromVertex()->getId()) == nextMap.end());

            nextMap[he->getFromVertex()->getId()] = he->getToVertex()->getId();
        }
    }


    std::vector<unsigned int> polygon;

    bool externalBorder;

    std::map<unsigned int, unsigned int>::iterator it = nextMap.begin();

    do {
        unsigned int firstId = it->first;

        polygon.clear();

        unsigned int actual = firstId;
        do {
            polygon.push_back(actual);
            actual = nextMap.at(actual);
        } while (actual != firstId);

        std::vector<cg3::Point2Dd> polygonPoints(polygon.size());
        for (unsigned int vId : polygon) {
            cg3::Pointd p = m.getVertex(vId);
            polygonPoints.push_back(cg3::Point2Dd(p.x(), p.y()));
        }
        externalBorder = cg3::isPolygonCounterClockwise(polygonPoints);

        it++;
    }
    while (!externalBorder);

    return polygon;
}

} //namespace internal

}
