/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_various.h"

#include <cg3/libigl/booleans.h>

#include <cg3/meshes/eigenmesh/algorithms/eigenmesh_algorithms.h>

#define CYLINDER_SUBD 100

namespace FourAxisFabrication {


/* Useful function declaration */

namespace internal {

void resetDataAfterCutting(
        const cg3::libigl::CSGTree& csgMesh,
        const cg3::libigl::CSGTree& csgResult,
        const std::vector<int>& meshAssociation,
        std::vector<int>& resultAssociation,
        const cg3::Array2D<int>& meshVisibility,
        cg3::Array2D<int>& resultVisibility,
        const int secondMeshLabel);
}


/**
 * @brief Get min and max heightfield level sets
 * The algorithm stops when the current triangle is not visible
 * by related the direction (-x for min and +x for max).
 * @param[in] mesh Input mesh
 * @param[in] heightFieldAngle Height field angle
 * @param[in] ffAdj Face-face adjacencies of the mesh
 */
void getMinAndMaxHeightfieldLevelSet(
        const cg3::EigenMesh& mesh,
        const double heightFieldAngle,
        const std::vector<std::vector<int>> ffAdj,
        double& levelSetMinX,
        double& levelSetMaxX)
{
    //Cos of height-field angle
    const double heightFieldAngleLimit = cos(heightFieldAngle);

    //Remember that the milling direction is the opposite if we want to
    //check the dot product. So (1,0,0) is the min, (-1,0,0) is the max.
    const cg3::Vec3d minDirection(-1,0,0);
    const cg3::Vec3d maxDirection(1,0,0);

    //Height-field on the extremes
    std::set<unsigned int> minHeightFieldSet;
    std::set<unsigned int> maxHeightFieldSet;

    //Initializing vector of face indices
    std::vector<unsigned int> fIndices(mesh.numberFaces());
    for (unsigned int i = 0; i < fIndices.size(); i++)
        fIndices[i] = i;

    //Order the vector by x-coordinate
    std::sort(fIndices.begin(), fIndices.end(), internal::XMinComparator(mesh));


    /* ----- MIN EXTREMES ----- */

    //Get min height-field faces
    size_t iMin = 0;
    while(mesh.faceNormal(fIndices[iMin]).dot(minDirection) >= heightFieldAngleLimit){
        minHeightFieldSet.insert(fIndices[iMin]);
        iMin++;
    }

    //Get the min x coordinate of the non-heightfield faces (it will be the level set)
    levelSetMinX = mesh.vertex(mesh.face(fIndices[iMin]).x()).x();
    while(iMin < fIndices.size()){
        cg3::Point3i face = mesh.face(fIndices[iMin]);

        levelSetMinX = std::min(levelSetMinX, mesh.vertex(face.x()).x());
        levelSetMinX = std::min(levelSetMinX, mesh.vertex(face.y()).x());
        levelSetMinX = std::min(levelSetMinX, mesh.vertex(face.z()).x());

        iMin++;
    }

    /* ----- MAX EXTREMES ----- */

    //Get max height-field faces
    int iMax = fIndices.size()-1;
    while(mesh.faceNormal(fIndices[iMax]).dot(maxDirection) >= heightFieldAngleLimit){
        maxHeightFieldSet.insert(fIndices[iMax]);
        iMax--;
    }

    //Get the max x coordinate of the non-selected faces (level set)
    levelSetMaxX = mesh.vertex(mesh.face(fIndices[iMax]).x()).x();
    while(iMax >= 0){
        cg3::Point3i face = mesh.face(fIndices[iMax]);

        levelSetMaxX = std::max(levelSetMaxX, mesh.vertex(face.x()).x());
        levelSetMaxX = std::max(levelSetMaxX, mesh.vertex(face.y()).x());
        levelSetMaxX = std::max(levelSetMaxX, mesh.vertex(face.z()).x());

        iMax--;
    }
}



/**
 * @brief Scale mesh and stock generation
 * @param mesh Model
 * @param modelLength Lenght of the model
 */
void centerAndScale(
        cg3::EigenMesh& mesh,
        const bool scaleModel,
        const double modelLength)
{
    if (scaleModel) {
        //Get the scale factor
        cg3::BoundingBox3 bb = mesh.boundingBox();
        double minX = bb.minX();
        double maxX = bb.maxX();
        double minY = bb.minY();
        double maxY = bb.maxY();
        double minZ = bb.minZ();
        double maxZ = bb.maxZ();
        const double scaleFactor = modelLength / std::max(std::max(maxX - minX, maxY - minY), maxZ - minZ);

        //Scale meshes
        const cg3::Vec3d scaleVec(scaleFactor, scaleFactor, scaleFactor);
        mesh.scale(scaleVec);

        mesh.updateBoundingBox();
    }

    //Center mesh
    cg3::Vec3d translateVec = -mesh.boundingBox().center();
    mesh.translate(translateVec);
    mesh.updateBoundingBox();
}

/**
 * @brief Scale mesh and stock generation
 * @param data Four axis fabrication data
 * @param modelLength Lenght of the model
 */
void centerAndScale(
        Data &data,
        const bool scaleModel,
        const double modelLength)
{
    cg3::EigenMesh& mesh = data.mesh;

    centerAndScale(mesh, scaleModel, modelLength);
}

void generateStock(
        Data& data,
        const double stockLength,
        const double stockDiameter)
{
    cg3::EigenMesh& stock = data.stock;

    double stockRadius = stockDiameter/2;
    double stockHalfLength = stockLength/2;

    stock = cg3::EigenMesh(cg3::EigenMeshAlgorithms::makeCylinder(cg3::Point3d(-stockHalfLength,0,0), cg3::Point3d(+stockHalfLength,0,0), static_cast<float>(stockRadius), CYLINDER_SUBD));
    stock.updateBoundingBox();
    stock.updateFacesAndVerticesNormals();
}

/**
 * @brief Cut components (min and max extremes)
 * @param[out] data Four axis fabrication data
 */
void cutComponents(
        Data& data,
        const bool cutComponents)
{    
    typedef cg3::libigl::CSGTree CSGTree;

    const std::vector<unsigned int>& maxExtremes = data.maxExtremes;
    const std::vector<unsigned int>& minExtremes = data.minExtremes;

    const cg3::EigenMesh& restoredMesh = data.restoredMesh;
    const std::vector<int>& restoredMeshAssociation = data.restoredMeshAssociation;
    const cg3::Array2D<int>& restoredMeshVisibility = data.restoredMeshVisibility;
    const std::vector<unsigned int>& restoredMeshNonVisibleFaces = data.restoredMeshNonVisibleFaces;

    const std::vector<unsigned int>& targetDirections = data.targetDirections;

    cg3::EigenMesh& minComponent = data.minComponent;
    cg3::EigenMesh& maxComponent = data.maxComponent;
    cg3::EigenMesh& fourAxisComponent = data.fourAxisComponent;

    std::vector<int>& fourAxisAssociation = data.fourAxisAssociation;
    cg3::Array2D<int>& fourAxisVisibility = data.fourAxisVisibility;
    std::vector<unsigned int>& fourAxisNonVisibleFaces = data.fourAxisNonVisibleFaces;


    int minLabel = targetDirections[targetDirections.size()-2];
    int maxLabel = targetDirections[targetDirections.size()-1];

    //Referencing bounding box of the mesh
    cg3::BoundingBox3 bb = restoredMesh.boundingBox();



    if (cutComponents && !maxExtremes.empty() && !minExtremes.empty()) {
        //Get maximum x in the faces of the min extremes
        double minLevelSetX = restoredMesh.vertex(restoredMesh.face(minExtremes[0]).x()).x();
        for (int minFace : minExtremes) {
            if (restoredMeshAssociation[minFace] == minLabel) {
                cg3::Point3i face = restoredMesh.face(minFace);

                minLevelSetX = std::max(minLevelSetX, restoredMesh.vertex(face.x()).x());
                minLevelSetX = std::max(minLevelSetX, restoredMesh.vertex(face.y()).x());
                minLevelSetX = std::max(minLevelSetX, restoredMesh.vertex(face.z()).x());
            }
        }
        //Set min extremes bounding box
        cg3::BoundingBox3 minBB = bb;
        minBB.setMaxX(minLevelSetX);


        //Get minimum x in the faces of the max extremes
        double maxLevelSetX = restoredMesh.vertex(restoredMesh.face(maxExtremes[0]).x()).x();
        for (int maxFace : maxExtremes) {
            if (restoredMeshAssociation[maxFace] == maxLabel) {
                cg3::Point3i face = restoredMesh.face(maxFace);

                maxLevelSetX = std::min(maxLevelSetX, restoredMesh.vertex(face.x()).x());
                maxLevelSetX = std::min(maxLevelSetX, restoredMesh.vertex(face.y()).x());
                maxLevelSetX = std::min(maxLevelSetX, restoredMesh.vertex(face.z()).x());
            }
        }
        //Set max extremes bounding box
        cg3::BoundingBox3 maxBB = bb;
        maxBB.setMinX(maxLevelSetX);




        //Get CSGTrees
        CSGTree csgMesh = cg3::libigl::eigenMeshToCSGTree(restoredMesh);
        CSGTree csgMaxBB = cg3::libigl::eigenMeshToCSGTree(cg3::EigenMeshAlgorithms::makeBox(maxBB));
        CSGTree csgMinBB = cg3::libigl::eigenMeshToCSGTree(cg3::EigenMeshAlgorithms::makeBox(minBB));


        //Cut min extreme
        CSGTree csgMinResult = cg3::libigl::intersection(csgMesh, csgMinBB);

        //Cut max extreme
        CSGTree csgMaxResult = cg3::libigl::intersection(csgMesh, csgMaxBB);


        //Cut four axis resulting mesh
        CSGTree csgFourAxisResult;
        std::vector<int> currentAssociation = restoredMeshAssociation;
        cg3::Array2D<int> currentVisibility = restoredMeshVisibility;

        //Min difference
        csgFourAxisResult = cg3::libigl::difference(csgMesh, csgMinBB);
        internal::resetDataAfterCutting(csgMesh, csgFourAxisResult, currentAssociation, fourAxisAssociation, currentVisibility, fourAxisVisibility, minLabel);

        //Max difference
        csgMesh = cg3::libigl::eigenMeshToCSGTree(cg3::libigl::CSGTreeToEigenMesh(csgFourAxisResult));
        currentAssociation = fourAxisAssociation;
        currentVisibility = fourAxisVisibility;

        csgFourAxisResult = cg3::libigl::difference(csgMesh, csgMaxBB);
        internal::resetDataAfterCutting(csgMesh, csgFourAxisResult, currentAssociation, fourAxisAssociation, currentVisibility, fourAxisVisibility, maxLabel);



        //Get meshes
        minComponent = cg3::libigl::CSGTreeToEigenMesh(csgMinResult);
        maxComponent = cg3::libigl::CSGTreeToEigenMesh(csgMaxResult);
        fourAxisComponent = cg3::libigl::CSGTreeToEigenMesh(csgFourAxisResult);


        //Update mesh data
        minComponent.updateFacesAndVerticesNormals();
        minComponent.updateBoundingBox();
        maxComponent.updateFacesAndVerticesNormals();
        maxComponent.updateBoundingBox();
        fourAxisComponent.updateFacesAndVerticesNormals();
        fourAxisComponent.updateBoundingBox();

        //Detect non-visible faces
        fourAxisNonVisibleFaces.clear();
        for (unsigned int faceId = 0; faceId < fourAxisVisibility.sizeY(); faceId++){
            //Check if at least a direction has been found for each face
            bool found = false;

            for (unsigned int l = 0; l < fourAxisVisibility.sizeX() && !found; l++){
                if (fourAxisVisibility(l, faceId) == 1) {
                    found = true;
                }
            }

            //Add to the non-visible face
            if (!found)
                fourAxisNonVisibleFaces.push_back(faceId);
        }
    }
    else {
        fourAxisComponent = restoredMesh;
        fourAxisAssociation = restoredMeshAssociation;
        fourAxisVisibility = restoredMeshVisibility;
        fourAxisNonVisibleFaces = restoredMeshNonVisibleFaces;
    }
}

namespace internal {

/**
 * @brief Reassociate association data of the first mesh after a boolean operation on cutting the extremes
 * @param[in] csgMesh Initial mesh
 * @param[in] csgResult Resulting mesh
 * @param[in] meshAssociation Association of the initial mesh
 * @param[out] resultAssociation Resulting association
 * @param[in] meshVisibility Visibility of the initial mesh
 * @param[out] resultVisibility Resulting visibility
 * @param[in] secondMeshLabel Label to be assigned if the birth face is in the second mesh
 */
void resetDataAfterCutting(
        const cg3::libigl::CSGTree& csgMesh,
        const cg3::libigl::CSGTree& csgResult,
        const std::vector<int>& meshAssociation,
        std::vector<int>& resultAssociation,
        const cg3::Array2D<int>& meshVisibility,
        cg3::Array2D<int>& resultVisibility,
        const int secondMeshLabel)
{
    typedef cg3::libigl::CSGTree CSGTree;

    CSGTree::VectorJ birthFaces = csgResult.J();

    unsigned int nResultFaces = csgResult.F().rows();
    unsigned int nFirstMeshFaces = csgMesh.F().rows();

    resultAssociation.clear();
    resultAssociation.resize(nResultFaces);

    resultVisibility.clear();
    resultVisibility.resize(meshVisibility.sizeX(), nResultFaces);
    resultVisibility.fill(0);

    for (unsigned int i = 0; i < nResultFaces; i++) {
        unsigned int birthFace = birthFaces[i];

        //If the birth face is in the first mesh
        if (birthFace < nFirstMeshFaces) {
            resultAssociation[i] = meshAssociation[birthFace];
            for (unsigned int l = 0; l < meshVisibility.sizeX(); l++) {
                resultVisibility(l, i) = meshVisibility(l, birthFace);
            }
        }
        //If the birth face is in the second mesh
        else {
            resultAssociation[i] = secondMeshLabel;
            resultVisibility(secondMeshLabel, i) = 1;
        }
    }
}

}

}
