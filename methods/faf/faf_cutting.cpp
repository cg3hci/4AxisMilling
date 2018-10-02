/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_cutting.h"

#include <cg3/libigl/booleans.h>

#include <cg3/meshes/eigenmesh/algorithms/eigenmesh_algorithms.h>

namespace FourAxisFabrication {


/* Useful function declaration */

namespace internal {

void resetAssociationData(
        const cg3::libigl::CSGTree& csgMesh,
        const cg3::libigl::CSGTree& csgResult,
        const std::vector<int>& meshAssociation,
        const int secondMeshLabel,
        std::vector<int>& resultAssociation);
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

    const std::vector<unsigned int>& targetDirections = data.targetDirections;

    std::vector<int>& minComponentAssociation = data.minComponentAssociation;
    std::vector<int>& maxComponentAssociation = data.maxComponentAssociation;
    std::vector<int>& fourAxisComponentAssociation = data.fourAxisComponentAssociation;

    cg3::EigenMesh& minComponent = data.minComponent;
    cg3::EigenMesh& maxComponent = data.maxComponent;
    cg3::EigenMesh& fourAxisComponent= data.fourAxisComponent;

    int minLabel = targetDirections[targetDirections.size()-2];
    int maxLabel = targetDirections[targetDirections.size()-1];

    //Referencing bounding box of the mesh
    cg3::BoundingBox bb = restoredMesh.boundingBox();



    if (cutComponents && !maxExtremes.empty() && !minExtremes.empty()) {
        //Get maximum x in the faces of the min extremes
        double minLevelSetX = restoredMesh.vertex(restoredMesh.face(minExtremes[0]).x()).x();
        for (int minFace : minExtremes) {
            if (restoredMeshAssociation[minFace] == minLabel) {
                cg3::Pointi face = restoredMesh.face(minFace);

                minLevelSetX = std::max(minLevelSetX, restoredMesh.vertex(face.x()).x());
                minLevelSetX = std::max(minLevelSetX, restoredMesh.vertex(face.y()).x());
                minLevelSetX = std::max(minLevelSetX, restoredMesh.vertex(face.z()).x());
            }
        }
        //Set min extremes bounding box
        cg3::BoundingBox minBB = bb;
        minBB.setMaxX(minLevelSetX);


        //Get minimum x in the faces of the max extremes
        double maxLevelSetX = restoredMesh.vertex(restoredMesh.face(maxExtremes[0]).x()).x();
        for (int maxFace : maxExtremes) {
            if (restoredMeshAssociation[maxFace] == maxLabel) {
                cg3::Pointi face = restoredMesh.face(maxFace);

                maxLevelSetX = std::min(maxLevelSetX, restoredMesh.vertex(face.x()).x());
                maxLevelSetX = std::min(maxLevelSetX, restoredMesh.vertex(face.y()).x());
                maxLevelSetX = std::min(maxLevelSetX, restoredMesh.vertex(face.z()).x());
            }
        }
        //Set max extremes bounding box
        cg3::BoundingBox maxBB = bb;
        maxBB.setMinX(maxLevelSetX);




        //Get CSGTrees
        CSGTree csgMesh = cg3::libigl::eigenMeshToCSGTree(restoredMesh);
        CSGTree csgMaxBB = cg3::libigl::eigenMeshToCSGTree(cg3::EigenMeshAlgorithms::makeBox(maxBB));
        CSGTree csgMinBB = cg3::libigl::eigenMeshToCSGTree(cg3::EigenMeshAlgorithms::makeBox(minBB));


        //Cut min extreme
        CSGTree csgMinResult = cg3::libigl::intersection(csgMesh, csgMinBB);        
        internal::resetAssociationData(csgMesh, csgMinResult, restoredMeshAssociation, -1, minComponentAssociation); //Restore association of cut component
        for (size_t i = 0; i < minComponentAssociation.size(); i++) {
            if (minComponentAssociation[i] >= 0)
                minComponentAssociation[i] = minLabel;
        }

        //Cut max extreme
        CSGTree csgMaxResult = cg3::libigl::intersection(csgMesh, csgMaxBB);
        internal::resetAssociationData(csgMesh, csgMaxResult, restoredMeshAssociation, -1, maxComponentAssociation); //Restore association of cut component
        for (size_t i = 0; i < maxComponentAssociation.size(); i++) {
            if (maxComponentAssociation[i] >= 0)
                maxComponentAssociation[i] = maxLabel;
        }


        //Cut four axis resulting mesh
        CSGTree csgFourAxisResult;
        std::vector<int> currentAssociation = restoredMeshAssociation;

        csgFourAxisResult = cg3::libigl::difference(csgMesh, csgMinBB);
        internal::resetAssociationData(csgMesh, csgFourAxisResult, currentAssociation, minLabel, fourAxisComponentAssociation);

        csgMesh = cg3::libigl::eigenMeshToCSGTree(cg3::libigl::CSGTreeToEigenMesh(csgFourAxisResult));
        currentAssociation = fourAxisComponentAssociation;

        csgFourAxisResult = cg3::libigl::difference(csgMesh, csgMaxBB);
        internal::resetAssociationData(csgMesh, csgFourAxisResult, currentAssociation, maxLabel, fourAxisComponentAssociation);



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
    }
    else {
        fourAxisComponent = restoredMesh;
        fourAxisComponentAssociation = restoredMeshAssociation;
    }
}

namespace internal {

/**
 * @brief Reassociate association data of the first mesh after a boolean operation on cutting the extremes
 * @param[in] csgMesh Initial mesh
 * @param[in] csgResult Resulting mesh
 * @param[in] meshAssociation Association of the initial mesh
 * @param[in] secondMeshLabel Label to be assigned if the birth face is in the second mesh
 * @param[out] resultAssociation Resulting association
 */
void resetAssociationData(
        const cg3::libigl::CSGTree& csgMesh,
        const cg3::libigl::CSGTree& csgResult,
        const std::vector<int>& meshAssociation,
        const int secondMeshLabel,
        std::vector<int>& resultAssociation)
{
    typedef cg3::libigl::CSGTree CSGTree;

    CSGTree::VectorJ birthFaces = csgResult.J();

    unsigned int nResultFaces = csgResult.F().rows();
    unsigned int nFirstMeshFaces = csgMesh.F().rows();

    resultAssociation.clear();
    resultAssociation.resize(nResultFaces);

    for (unsigned int i = 0; i < nResultFaces; i++) {
        unsigned int birthFace = birthFaces[i];

        //If the birth face is in the first mesh
        if (birthFace < nFirstMeshFaces) {
            resultAssociation[i] = meshAssociation[birthFace];
        }
        //If the birth face is in the second mesh
        else {
            resultAssociation[i] = secondMeshLabel;
        }
    }
}

}

}
