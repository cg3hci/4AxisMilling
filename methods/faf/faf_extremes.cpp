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
        const bool selectExtremes,
        Data& data)
{
    if (selectExtremes) {
        //Referencing output data
        std::vector<unsigned int>& minExtremes = data.minExtremes;
        std::vector<unsigned int>& maxExtremes = data.maxExtremes;


        //Remember that the milling direction is the opposite if we want to
        //check the dot product. So (1,0,0) is the min, (-1,0,0) is the max.
        const cg3::Vec3 minDirection(-1,0,0);
        const cg3::Vec3 maxDirection(1,0,0);


        //Height-field on the extremes
        std::vector<unsigned int> heightFieldMin;
        std::vector<unsigned int> heightFieldMax;

        //Clearing current data (if any)
        heightFieldMin.clear();
        heightFieldMax.clear();

        //Initializing vector of face indices
        std::vector<unsigned int> fIndices(mesh.numberFaces());
        for (unsigned int i = 0; i < fIndices.size(); i++)
            fIndices[i] = i;

        //Order the vector by x-coordinate
        std::sort(fIndices.begin(), fIndices.end(), internal::BarycenterXComparator(mesh));

        //Get min height-field faces
        size_t iMin = 0;
        while(mesh.faceNormal(fIndices[iMin]).dot(minDirection) >= -std::numeric_limits<double>::epsilon()){
            heightFieldMin.push_back(fIndices[iMin]);
            iMin++;
        }

        //Get the min x coordinate of the non-selected faces (level set)
        double levelSetMinX = mesh.vertex(mesh.face(fIndices[iMin]).x()).x();
        while(iMin < fIndices.size()){
            cg3::Pointi face = mesh.face(fIndices[iMin]);

            levelSetMinX = std::min(levelSetMinX, mesh.vertex(face.x()).x());
            levelSetMinX = std::min(levelSetMinX, mesh.vertex(face.y()).x());
            levelSetMinX = std::min(levelSetMinX, mesh.vertex(face.z()).x());

            iMin++;
        }

        //Get the min extremes
        for (unsigned int faceId : heightFieldMin) {
            cg3::Pointi face = mesh.face(faceId);

            //If every face coordinate is under the level set
            if (mesh.vertex(face.x()).x() < levelSetMinX &&
                mesh.vertex(face.y()).x() < levelSetMinX &&
                mesh.vertex(face.z()).x() < levelSetMinX)
            {
                minExtremes.push_back(faceId);
            }
        }



        //Get max height-field faces
        int iMax = fIndices.size()-1;
        while(mesh.faceNormal(fIndices[iMax]).dot(maxDirection) >= -std::numeric_limits<double>::epsilon()){
            heightFieldMax.push_back(fIndices[iMax]);
            iMax--;
        }

        //Get the max x coordinate of the non-selected faces (level set)
        double levelSetMaxX = mesh.vertex(mesh.face(fIndices[iMax]).x()).x();
        while(iMax >= 0){
            cg3::Pointi face = mesh.face(fIndices[iMax]);

            levelSetMaxX = std::max(levelSetMaxX, mesh.vertex(face.x()).x());
            levelSetMaxX = std::max(levelSetMaxX, mesh.vertex(face.y()).x());
            levelSetMaxX = std::max(levelSetMaxX, mesh.vertex(face.z()).x());

            iMax--;
        }

        //Get the max extremes
        for (unsigned int faceId : heightFieldMax) {
            cg3::Pointi face = mesh.face(faceId);

            //If every face coordinate is above the level set
            if (mesh.vertex(face.x()).x() > levelSetMaxX &&
                mesh.vertex(face.y()).x() > levelSetMaxX &&
                mesh.vertex(face.z()).x() > levelSetMaxX)
            {
                maxExtremes.push_back(faceId);
            }
        }
    }
}




}
