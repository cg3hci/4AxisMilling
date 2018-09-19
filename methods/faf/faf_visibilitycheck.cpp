/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_visibilitycheck.h"

#include <cg3/geometry/transformations.h>

#include "includes/view_renderer.h"


namespace FourAxisFabrication {


/* ----- INTERNAL FUNCTION DECLARATION ----- */

namespace internal {


/* Methods for computing visibility */

void computeVisibility(
        const cg3::EigenMesh& mesh,
        const unsigned int nDirections,
        std::vector<cg3::Vec3>& directions,
        std::vector<double>& directionsAngle,
        cg3::Array2D<int>& visibility,
        const std::vector<unsigned int>& minExtremes,
        const std::vector<unsigned int>& maxExtremes,
        const bool includeXDirections,
        const double heightfieldAngle);

void computeVisibility(
        ViewRenderer& vr,
        const unsigned int dirIndex,
        const std::vector<cg3::Vec3>& directions,
        const std::vector<cg3::Vec3>& faceNormals,
        const double heightfieldAngle,
        cg3::Array2D<int>& visibility);

void detectNonVisibleFaces(
        const cg3::Array2D<int>& visibility,
        std::vector<unsigned int>& nonVisibleFaces);


} //namespace internal



/* ----- VISIBILITY METHODS ----- */


/**
 * @brief Get visibility of each face of the mesh from a given number of
 * different directions.
 * It is implemented by a ray casting algorithm or checking the intersections
 * in a 2D projection from a given direction.
 * @param[in] mesh Input mesh
 * @param[in] numberDirections Number of directions to be checked
 * @param[out] data Four axis fabrication data
 * @param[in] heightfieldAngle Limit angle with triangles normal in order to be a heightfield
 * @param[in] checkMode Visibility check mode. Default is projection mode.
 */
void getVisibility(
        const cg3::EigenMesh& mesh,
        const unsigned int nDirections,
        Data& data,
        const double heightfieldAngle,
        const bool includeXDirections)
{
    internal::computeVisibility(mesh, nDirections, data.directions, data.angles, data.visibility, data.minExtremes, data.maxExtremes, includeXDirections, heightfieldAngle);
    internal::detectNonVisibleFaces(data.visibility, data.nonVisibleFaces);
}




/* ----- INTERNAL FUNCTION DEFINITION ----- */

namespace internal {


/* ----- METHODS FOR COMPUTING VISIBILITY ----- */

/**
 * @brief Get visibility of each face of the mesh from a given number of
 * different directions.
 * It is implemented by a ray casting algorithm or checking the intersections
 * in a 2D projection from a given direction.
 * @param[in] mesh Input mesh
 * @param[in] numberDirections Number of directions to be checked
 * @param[out] directions Vector of directions
 * @param[out] angles Vector of angle (respect to z-axis)
 * @param[out] visibility Output visibility
 * @param[in] heightfieldAngle Limit angle with triangles normal in order to be a heightfield
 * @param[in] checkMode Visibility check mode. Default is projection mode.
 */
void computeVisibility(
        const cg3::EigenMesh& mesh,
        const unsigned int nDirections,
        std::vector<cg3::Vec3>& directions,
        std::vector<double>& angles,
        cg3::Array2D<int>& visibility,
        const std::vector<unsigned int>& minExtremes,
        const std::vector<unsigned int>& maxExtremes,
        const bool includeXDirections,
        const double heightfieldAngle)
{
    //Initialize visibility (number of directions, two for extremes)
    visibility.clear();
    visibility.resize(nDirections + 2, mesh.numberFaces());
    visibility.fill(0);

    //Initialize direction vector
    directions.clear();
    directions.resize(nDirections + 2);

    //Initialize angles
    angles.clear();
    angles.resize(nDirections);

    //Cos of the height field angle
    const double heightFieldLimit = cos(heightfieldAngle);

    //View renderer
    ViewRenderer vr(mesh, mesh.boundingBox(), 8096);

    //Set target faces to be checked
    std::vector<unsigned int> targetFaces;
    for (unsigned int fId = 0; fId < mesh.numberFaces(); fId++) {
        targetFaces.push_back(fId);
    }

    //Compute normals
    std::vector<cg3::Vec3> faceNormals(mesh.numberFaces());
    for (unsigned int fId = 0; fId < mesh.numberFaces(); fId++) {
        faceNormals[fId] = mesh.faceNormal(fId);
    }

    //Step angle for getting all the directions (on 180 degrees)
    double stepAngle = 2*M_PI / nDirections;


    //Index for min, max extremes
    const unsigned int minIndex = nDirections;
    const unsigned int maxIndex = nDirections + 1;

    //Get rotation matrix around the x-axis
    const cg3::Vec3 xAxis(1,0,0);
    Eigen::Matrix3d rot;
    cg3::rotationMatrix(xAxis, stepAngle, rot);

    //Set angles and directions
    cg3::Vec3 dir(0,0,1);
    double sum = 0;
    for(unsigned int i = 0; i < nDirections; i++) {
        angles[i] = sum;
        directions[i] = dir;

        sum += stepAngle;
        dir.rotate(rot);
    }

    //For each direction
    for(unsigned int dirIndex = 0; dirIndex < nDirections; dirIndex++) {
        computeVisibility(vr, dirIndex, directions, faceNormals, heightFieldLimit, visibility);
    }

    //Add min and max extremes directions
    directions[minIndex] = cg3::Vec3(-1,0,0);
    directions[maxIndex] = cg3::Vec3(1,0,0);

    if (includeXDirections) {
        //Compute -x and +x visibility        
        computeVisibility(vr, minIndex, directions, faceNormals, heightFieldLimit, visibility);
        computeVisibility(vr, maxIndex, directions, faceNormals, heightFieldLimit, visibility);
    }
    else {
        //Set visibility of the min extremes
        #pragma omp parallel for
        for (size_t i = 0; i < minExtremes.size(); i++){
            unsigned int faceId = minExtremes[i];
            visibility(minIndex, faceId) = 1;
        }

        //Set visibility of the max extremes
        #pragma omp parallel for
        for (size_t i = 0; i < maxExtremes.size(); i++){
            unsigned int faceId = maxExtremes[i];
            visibility(maxIndex, faceId) = 1;
        }
    }
}

/**
 * @brief Compute visibility from a given direction
 * @param[in] vr View renderer
 * @param[in] dirIndex Index of the direction
 * @param[in] directions Directions
 * @param[in] faceNormals Face normals
 * @param[in] heightFieldLimit Cos of the angle
 * @param[in] visibility Visibility output array
 */
void computeVisibility(
        ViewRenderer& vr,
        const unsigned int dirIndex,
        const std::vector<cg3::Vec3>& directions,
        const std::vector<cg3::Vec3>& faceNormals,
        const double heightFieldLimit,
        cg3::Array2D<int>& visibility)
{
    const cg3::Vec3& dir = directions[dirIndex];

    //Compute visibility from the direction
    std::vector<bool> dirVisibility = vr.renderVisibility(dir, true, false);

    #pragma omp parallel for
    for (size_t fId = 0; fId < dirVisibility.size(); fId++) {
        if (dirVisibility[fId] && faceNormals[fId].dot(dir) >= heightFieldLimit) {
            visibility(dirIndex, fId) = true;
        }
        else {
            visibility(dirIndex, fId) = false;
        }
    }
}

/**
 * @brief Detect faces that are not visible
 * @param[in] visibility Visibility
 * @param[out] nonVisibleFaces Collection of non-visible faces
 */
void detectNonVisibleFaces(        
        const cg3::Array2D<int>& visibility,
        std::vector<unsigned int>& nonVisibleFaces)
{

    //Detect non-visible faces
    nonVisibleFaces.clear();
    for (unsigned int faceId = 0; faceId < visibility.sizeY(); faceId++){
        //Check if at least a direction has been found for each face
        bool found = false;

        for (unsigned int i = 0; i < visibility.sizeX() && !found; i++){
            if (visibility(i, faceId) == 1) {
                found = true;
            }
        }

        //Add to the non-visible face
        if (!found)
            nonVisibleFaces.push_back(faceId);
    }

}




}

}
