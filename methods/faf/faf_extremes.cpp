#include "faf_extremes.h"

#include <vector>

#include "faf_utilities.h"


namespace FourAxisFabrication {


/* ----- EXTREMES ----- */

/**
 * @brief Get min and max extremes of the mesh along the x direction.
 * The algorithm stops when the current triangle is not visible
 * by related the direction (-x for min and +x for max).
 * @param[in] mesh Input mesh
 * @param[out] data Four axis fabrication data
 */
void selectExtremesOnXAxis(
        const cg3::EigenMesh &mesh,
        Data& data)
{
    //Referencing output data
    std::vector<unsigned int>& minExtremes = data.minExtremes;
    std::vector<unsigned int>& maxExtremes = data.maxExtremes;

    //Height-field on the extremes
    std::vector<unsigned int> heightFieldMin;
    std::vector<unsigned int> heightFieldMax;

    //Clearing current data (if any)
    heightFieldMin.clear();
    heightFieldMax.clear();

    //Initializing vector of face indices
    std::vector<unsigned int> fIndices(mesh.getNumberFaces());
    for (unsigned int i = 0; i < fIndices.size(); i++)
        fIndices[i] = i;

    //Order the vector by x-coordinate
    std::sort(fIndices.begin(), fIndices.end(), internal::BarycenterXComparator(mesh));

    //Remind that the milling direction is the opposite if we want to
    //check the dot product. So (1,0,0) is the min, (-1,0,0) is the max.
    const cg3::Vec3 minDirection(-1,0,0);
    const cg3::Vec3 maxDirection(1,0,0);





    //Get min height-field faces
    size_t iMin = 0;
    while(mesh.getFaceNormal(fIndices[iMin]).dot(minDirection) >= -std::numeric_limits<double>::epsilon()){
        heightFieldMin.push_back(fIndices[iMin]);
        iMin++;
    }

    //Get the min x coordinate of the non-selected faces (level set)
    double levelSetMinX = mesh.getVertex(mesh.getFace(fIndices[iMin]).x()).x();
    while(iMin < fIndices.size()){
        cg3::Pointi face = mesh.getFace(fIndices[iMin]);

        levelSetMinX = std::min(levelSetMinX, mesh.getVertex(face.x()).x());
        levelSetMinX = std::min(levelSetMinX, mesh.getVertex(face.y()).x());
        levelSetMinX = std::min(levelSetMinX, mesh.getVertex(face.z()).x());

        iMin++;
    }

    //Get the min extremes
    for (unsigned int faceId : heightFieldMin) {
        cg3::Pointi face = mesh.getFace(faceId);

        //If every face coordinate is under the level set
        if (mesh.getVertex(face.x()).x() < levelSetMinX &&
            mesh.getVertex(face.y()).x() < levelSetMinX &&
            mesh.getVertex(face.z()).x() < levelSetMinX)
        {
            minExtremes.push_back(faceId);
        }
    }



    //Get max height-field faces
    int iMax = fIndices.size()-1;
    while(mesh.getFaceNormal(fIndices[iMax]).dot(maxDirection) >= -std::numeric_limits<double>::epsilon()){
        heightFieldMax.push_back(fIndices[iMax]);
        iMax--;
    }

    //Get the max x coordinate of the non-selected faces (level set)
    double levelSetMaxX = mesh.getVertex(mesh.getFace(fIndices[iMax]).x()).x();
    while(iMax >= 0){
        cg3::Pointi face = mesh.getFace(fIndices[iMax]);

        levelSetMaxX = std::max(levelSetMaxX, mesh.getVertex(face.x()).x());
        levelSetMaxX = std::max(levelSetMaxX, mesh.getVertex(face.y()).x());
        levelSetMaxX = std::max(levelSetMaxX, mesh.getVertex(face.z()).x());

        iMax--;
    }

    //Get the max extremes
    for (unsigned int faceId : heightFieldMax) {
        cg3::Pointi face = mesh.getFace(faceId);

        //If every face coordinate is above the level set
        if (mesh.getVertex(face.x()).x() > levelSetMaxX &&
            mesh.getVertex(face.y()).x() > levelSetMaxX &&
            mesh.getVertex(face.z()).x() > levelSetMaxX)
        {
            maxExtremes.push_back(faceId);
        }
    }
}




}
