/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_cutting.h"

#include <cg3/libigl/booleans.h>

#include <cg3/meshes/eigenmesh/algorithms/eigenmesh_algorithms.h>

namespace FourAxisFabrication {

/* Useful function declaration */

void resetAssociationData(
        const cg3::libigl::CSGTree& csgMesh,
        const cg3::libigl::CSGTree& csgResult,
        const std::vector<int>& association,
        std::vector<int>& resultAssociation,
        const int fixedAssociation = -1);

/**
 * @brief Cut components (min and max extremes)
 * @param[out] data Four axis fabrication data
 */
void cutComponents(
        Data& data,
        const bool cutComponents)
{    
    typedef cg3::libigl::CSGTree CSGTree;

    const cg3::EigenMesh& restoredMesh = data.restoredMesh;


    int minLabel = data.targetDirections[data.targetDirections.size()-2];
    int maxLabel = data.targetDirections[data.targetDirections.size()-1];

    //Referencing bounding box of the mesh
    cg3::BoundingBox bb = restoredMesh.boundingBox();



    if (cutComponents && !data.maxExtremes.empty() && !data.minExtremes.empty()) {
        //Get maximum x in the faces of the min extremes
        double minLevelSetX = restoredMesh.vertex(restoredMesh.face(data.minExtremes[0]).x()).x();
        for (int minFace : data.minExtremes) {
            if (data.restoredMeshAssociation[minFace] == minLabel) {
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
        double maxLevelSetX = restoredMesh.vertex(restoredMesh.face(data.maxExtremes[0]).x()).x();
        for (int maxFace : data.maxExtremes) {
            if (data.restoredMeshAssociation[maxFace] == maxLabel) {
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
        CSGTree csgMaxBB = cg3::libigl::eigenMeshToCSGTree(
                    cg3::EigenMeshAlgorithms::makeBox(maxBB));
        CSGTree csgMinBB = cg3::libigl::eigenMeshToCSGTree(
                    cg3::EigenMeshAlgorithms::makeBox(minBB));


        //Cut min extreme
        CSGTree csgMinResult = cg3::libigl::intersection(csgMesh, csgMinBB);
        cg3::EigenMesh minComponent =
                cg3::libigl::CSGTreeToEigenMesh(csgMinResult);

        //Cut max extreme
        CSGTree csgMaxResult = cg3::libigl::intersection(csgMesh, csgMaxBB);
        cg3::EigenMesh maxComponent =
                cg3::libigl::CSGTreeToEigenMesh(csgMaxResult);


        //Cut four axis resulting mesh
        CSGTree csgUnion = cg3::libigl::union_(csgMaxBB, csgMinBB);
        CSGTree csgFourAxisResult = cg3::libigl::difference(csgMesh, csgUnion);
        cg3::EigenMesh fourAxisComponent = cg3::libigl::CSGTreeToEigenMesh(csgFourAxisResult);


        //Restore association of cut components
        resetAssociationData(csgMesh, csgMinResult, data.restoredMeshAssociation, data.minComponentAssociation,
                             minLabel); //Set everything to min index
        resetAssociationData(csgMesh, csgMaxResult, data.restoredMeshAssociation, data.maxComponentAssociation,
                             maxLabel); //Set eveyrthing to max index
        resetAssociationData(csgMesh, csgFourAxisResult, data.restoredMeshAssociation, data.fourAxisComponentAssociation);


        //Update mesh data
        minComponent.updateFacesAndVerticesNormals();
        minComponent.updateBoundingBox();
        maxComponent.updateFacesAndVerticesNormals();
        maxComponent.updateBoundingBox();
        fourAxisComponent.updateFacesAndVerticesNormals();
        fourAxisComponent.updateBoundingBox();

        //Add to components
        data.minComponent = minComponent;
        data.maxComponent = maxComponent;
        data.fourAxisComponent = fourAxisComponent;
    }
    else {
        data.fourAxisComponent = restoredMesh;
        data.fourAxisComponentAssociation = data.association;
    }
}

/**
 * @brief Reassociate association data after a boolean operation
 * @param csgMesh First mesh (the main mesh)
 * @param csgResult Resulting mesh
 * @param association Association of the full mesh
 * @param resultAssociation Resulting association
 */
void resetAssociationData(
        const cg3::libigl::CSGTree& csgMesh,
        const cg3::libigl::CSGTree& csgResult,
        const std::vector<int>& association,
        std::vector<int>& resultAssociation,
        const int fixedAssociation)
{
    typedef cg3::libigl::CSGTree CSGTree;

    CSGTree::VectorJ birthFaces = csgResult.J();

    unsigned int nFaces = csgResult.F().rows();
    unsigned int nA = csgMesh.F().rows();

    resultAssociation.resize(nFaces);

    for (unsigned int i = 0; i < nFaces; i++) {
        unsigned int birthFace = birthFaces[i];

        //If the birth face is in the first mesh
        if (birthFace < nA) {            
            resultAssociation[i] =
                    (fixedAssociation >= 0 ? fixedAssociation : association[birthFace]);
        }
        //If the birth face is in the second mesh
        else {
            resultAssociation[i] = -1;
        }
    }
}

}
