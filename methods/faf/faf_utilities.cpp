/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_utilities.h"

#include <cg3/geometry/transformations.h>
#include <cg3/geometry/point.h>

#include <cg3/algorithms/global_optimal_rotation_matrix.h>



namespace FourAxisFabrication {

namespace internal {

/* Comparators */

struct BarycenterXComparator {    const cg3::EigenMesh& m;
    BarycenterXComparator(const cg3::EigenMesh& m) : m(m) {}
    bool operator()(unsigned int f1, unsigned int f2){
        const cg3::Pointi& ff1 = m.getFace(f1);
        const cg3::Pointi& ff2 = m.getFace(f2);
        cg3::Pointd c1 = (m.getVertex(ff1.x()) + m.getVertex(ff1.y()) + m.getVertex(ff1.z()))/3;
        cg3::Pointd c2 = (m.getVertex(ff2.x()) + m.getVertex(ff2.y()) + m.getVertex(ff2.z()))/3;
        return c1 < c2;
    }
};

}


/* ----- OPTIMAL ROTATION ----- */

/**
 * @brief Get optimal orientation on x-axis for four axis
 * fabrication. Both meshes are rotated in the same way.
 *
 * @param[out] mesh Original mesh
 * @param[out] smoothedMesh Smoothed mesh
 * @param[in] nDirections Number of directions to check
 * @param[in] deterministic Deterministic approach (if false it is randomized)
 */
void rotateToOptimalOrientation(
        cg3::EigenMesh& mesh,
        cg3::EigenMesh& smoothedMesh,
        const unsigned int nOrientations,
        const bool deterministic)
{
    smoothedMesh.updateFaceNormals();

    //Get the optimal rotation matrix
    Eigen::Matrix3d rot = cg3::globalOptimalRotationMatrix(smoothedMesh, nOrientations, deterministic);

    //Rotate meshes
    mesh.rotate(rot);
    smoothedMesh.rotate(rot);

    cg3::BoundingBox b = mesh.getBoundingBox();

    //If Y length is the greatest one then rotate by 90° around zAxis
    if (b.getLengthY() > b.getLengthX() && b.getLengthY() > b.getLengthZ()){
        cg3::Vec3 zAxis(0,0,1);
        double angle = M_PI/2;

        mesh.rotate(cg3::getRotationMatrix(zAxis, angle));
        smoothedMesh.rotate(cg3::getRotationMatrix(zAxis, angle));
    }
    //If Z length is the greatest one then rotate by 90° around yAxis
    else if (b.getLengthZ() > b.getLengthX() && b.getLengthZ() > b.getLengthY()){
        cg3::Vec3 yAxis(0,1,0);
        double angle = M_PI/2;

        mesh.rotate(cg3::getRotationMatrix(yAxis, angle));
        smoothedMesh.rotate(cg3::getRotationMatrix(yAxis, angle));
    }

    //Translate meshes to the center
    mesh.translate(-mesh.getBoundingBox().center());
    smoothedMesh.translate(-smoothedMesh.getBoundingBox().center());
}





/* ----- EXTREMES ----- */

/**
 * @brief Get min and max extremes of the mesh along the x direction.
 * The algorithm stops when the current triangle is not visible
 * by related the direction (-x for min and +x for max).
 * @param mesh Input mesh
 * @param[out] minExtremes Min extremes on the x-axis of the mesh
 * @param[out] maxExtremes Max extremes on the x-axis of the mesh
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