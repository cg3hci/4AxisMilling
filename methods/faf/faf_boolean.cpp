/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_boolean.h"

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
 * @param mesh Input mesh
 * @param data Four axis fabrication data
 */
void cutComponents(
        const cg3::EigenMesh& mesh,
        Data& data)
{    
    typedef cg3::libigl::CSGTree CSGTree;


    //Referencing ounding box of the mesh
    cg3::BoundingBox bb = mesh.getBoundingBox();



    //Get minimum x in the faces of the max extremes
    double maxLevelSetX = mesh.getVertex(mesh.getFace(data.maxExtremes[0]).x()).x();
    for (int maxIndex : data.maxExtremes) {
        cg3::Pointi face = mesh.getFace(maxIndex);

        maxLevelSetX = std::min(maxLevelSetX, mesh.getVertex(face.x()).x());
        maxLevelSetX = std::min(maxLevelSetX, mesh.getVertex(face.y()).x());
        maxLevelSetX = std::min(maxLevelSetX, mesh.getVertex(face.z()).x());
    }    
    //Set max extremes bounding box
    cg3::BoundingBox maxBB = bb;
    maxBB.setMinX(maxLevelSetX);


    //Get maximum x in the faces of the min extremes
    double minLevelSetX = mesh.getVertex(mesh.getFace(data.minExtremes[0]).x()).x();
    for (int minIndex : data.minExtremes) {
        cg3::Pointi face = mesh.getFace(minIndex);

        minLevelSetX = std::max(minLevelSetX, mesh.getVertex(face.x()).x());
        minLevelSetX = std::max(minLevelSetX, mesh.getVertex(face.y()).x());
        minLevelSetX = std::max(minLevelSetX, mesh.getVertex(face.z()).x());
    } 
    //Set min extremes bounding box
    cg3::BoundingBox minBB = bb;
    minBB.setMaxX(minLevelSetX);


    //Get CSGTrees
    CSGTree csgMesh = cg3::libigl::eigenMeshToCSGTree(mesh);
    CSGTree csgMaxBB = cg3::libigl::eigenMeshToCSGTree(
                cg3::EigenMeshAlgorithms::makeBox(maxBB));
    CSGTree csgMinBB = cg3::libigl::eigenMeshToCSGTree(
                cg3::EigenMeshAlgorithms::makeBox(minBB));


    //Cut min extreme
    CSGTree csgMinResult = cg3::libigl::intersection(csgMesh, csgMinBB);
    cg3::EigenMesh minResult =
            cg3::libigl::CSGTreeToEigenMesh(csgMinResult);

    //Cut max extreme
    CSGTree csgMaxResult = cg3::libigl::intersection(csgMesh, csgMaxBB);
    cg3::EigenMesh maxResult =
            cg3::libigl::CSGTreeToEigenMesh(csgMaxResult);


    //Cut four axis resulting mesh
    CSGTree csgUnion = cg3::libigl::union_(csgMaxBB, csgMinBB);
    CSGTree csgFourAxisResult = cg3::libigl::difference(csgMesh, csgUnion);
    cg3::EigenMesh fourAxisResult = cg3::libigl::CSGTreeToEigenMesh(csgFourAxisResult);


    //Restore association of cut components
    std::vector<int>& minAssociation = data.minAssociation;
    std::vector<int>& maxAssociation = data.maxAssociation;
    std::vector<int>& fourAxisAssociation = data.fourAxisAssociation;

    int minIndex = data.targetDirections[data.targetDirections.size()-2];
    int maxIndex = data.targetDirections[data.targetDirections.size()-1];

    resetAssociationData(csgMesh, csgMinResult, data.association, minAssociation, minIndex);
    resetAssociationData(csgMesh, csgMaxResult, data.association, maxAssociation, maxIndex);
    resetAssociationData(csgMesh, csgFourAxisResult, data.association, fourAxisAssociation);


    //Update normals and everything
    minResult.updateFacesAndVerticesNormals();    
    minResult.updateBoundingBox();
    maxResult.updateFacesAndVerticesNormals();
    maxResult.updateBoundingBox();
    fourAxisResult.updateFacesAndVerticesNormals();
    fourAxisResult.updateBoundingBox();

    //Updating resulting meshes
    data.minResult = minResult;
    data.maxResult = maxResult;
    data.fourAxisResult = fourAxisResult;
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
