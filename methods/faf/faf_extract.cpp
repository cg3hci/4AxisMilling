/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_extract.h"

#include <cg3/libigl/vertex_adjacencies.h>

namespace FourAxisFabrication {

/* Useful function declaration */

cg3::EigenMesh extractFacesWithALabel(
        cg3::EigenMesh& mesh,
        std::vector<int>& association,
        unsigned int targetLabel);

/**
 * @brief Cut components (min and max extremes)
 * @param mesh Input mesh
 * @param data Four axis fabrication data
 */
void extractResults(
        Data& data)
{    
    //Referencing data
    std::vector<unsigned int>& targetDirections = data.targetDirections;

    std::vector<int>& minComponentAssociation = data.minComponentAssociation;
    std::vector<int>& maxComponentAssociation = data.maxComponentAssociation;
    std::vector<int>& fourAxisComponentAssociation = data.fourAxisComponentAssociation;

    cg3::EigenMesh& minComponent = data.minComponent;
    cg3::EigenMesh& maxComponent = data.maxComponent;
    cg3::EigenMesh& fourAxisComponent = data.fourAxisComponent;

    std::vector<cg3::EigenMesh>& results = data.results;
    std::vector<unsigned int>& resultsAssociation = data.resultsAssociation;


    unsigned int minLabel = data.targetDirections[data.targetDirections.size()-2];
    unsigned int maxLabel = data.targetDirections[data.targetDirections.size()-1];


    //Split the component in surface components
    for (size_t i = 0; i < targetDirections.size()-2; i++) {
        //Current direction label
        int targetLabel = targetDirections[i];

        //New component mesh
        cg3::EigenMesh result = extractFacesWithALabel(
                    fourAxisComponent,
                    fourAxisComponentAssociation,
                    targetLabel);

        //Add to the results
        results.push_back(result);
        resultsAssociation.push_back(targetLabel);
    }

    //Get surface of min component
    cg3::EigenMesh minResult =
            extractFacesWithALabel(
                minComponent,
                minComponentAssociation,
                minLabel);
    results.push_back(minResult);
    resultsAssociation.push_back(minLabel);

    //Get surface of min component
    cg3::EigenMesh maxResult =
            extractFacesWithALabel(
                maxComponent,
                maxComponentAssociation,
                maxLabel);
    results.push_back(maxResult);
    resultsAssociation.push_back(maxLabel);

    //Update mesh data
    for (cg3::EigenMesh& result : data.results) {
        result.updateBoundingBox();
        result.updateFacesAndVerticesNormals();
    }

}

/**
 * @brief Extract all faces of a mesh with a given label.
 * The result could be non-watertight.
 * @param mesh Input mesh
 * @param association Association of the mesh with labels
 * @param targetLabel Target labels
 * @return Output mesh, it could be non-watertight
 */
cg3::EigenMesh extractFacesWithALabel(
        cg3::EigenMesh& mesh,
        std::vector<int>& association,
        unsigned int targetLabel)
{
    std::vector<std::vector<int>> vfAdj = cg3::libigl::getVertexFaceAdjacencies(mesh);

    cg3::EigenMesh result;

    //Data structures to keep track of the new vertex
    std::vector<int> newVertexMap(mesh.getNumberVertices(), -1);
    unsigned int newVertexId = 0;

    //For each vertex
    for (unsigned int vId = 0; vId < mesh.getNumberVertices(); vId++) {
        std::vector<int>& adjFaces = vfAdj[vId];

        //Check if the belong to at least one face with the given target label
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

        if (association[fId] > -1 && association[fId] == (int) targetLabel) {
            result.addFace(
                    newVertexMap[face.x()],
                    newVertexMap[face.y()],
                    newVertexMap[face.z()]);
        }
    }

    return result;
}

}
