/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_visibilitycheck.h"

#include <cg3/utilities/const.h>

#include <cg3/geometry/transformations.h>
#include <cg3/geometry/2d/point2d.h>
#include <cg3/geometry/point.h>
#include <cg3/geometry/2d/triangle2d.h>
#include <cg3/geometry/2d/triangle2d_utils.h>

#include <cg3/data_structures/trees/aabbtree.h>

#include <cg3/cgal/aabbtree.h>

#include "faf_utilities.h"

//Uncomment if you want to check conditions
//(more efficient when uncommented)
//#define RELEASECHECK


namespace FourAxisFabrication {


/* ----- INTERNAL FUNCTION DECLARATION ----- */

namespace internal {


/* Methods for computing visibility */

void initializeDataForVisibilityCheck(
        const cg3::EigenMesh& mesh,
        const unsigned int nDirections,
        Data& data);

void computeVisibility(
        const cg3::EigenMesh& mesh,
        const unsigned int nDirections,
        std::vector<cg3::Vec3>& directions,
        std::vector<double>& directionsAngle,
        cg3::Array2D<int>& visibility,
        const double heightfieldAngle,
        CheckMode checkMode = PROJECTION);

void detectNonVisibleFaces(
        Data& data);



/* Check visibility (ray shooting) */

void getVisibilityRayShootingOnZ(
        const cg3::EigenMesh& mesh,
        const std::vector<unsigned int>& faces,
        const unsigned int nDirections,
        const unsigned int directionIndex,
        cg3::Array2D<int>& visibility,
        const double heightfieldAngle);

void getVisibilityRayShootingOnZ(
        const cg3::EigenMesh& mesh,
        const unsigned int faceId,
        const unsigned int directionIndex,
        const cg3::Vec3& direction,
        cg3::cgal::AABBTree& aabbTree,
        cg3::Array2D<int>& visibility,
        const double heightfieldAngle);



/* Check visibility (projection) */

void getVisibilityProjectionOnZ(
        const cg3::EigenMesh& mesh,
        const std::vector<unsigned int>& faces,
        const unsigned int nDirections,
        const unsigned int directionIndex,
        cg3::Array2D<int>& visibility,
        const double heightfieldAngle);

void getVisibilityProjectionOnZ(
        const cg3::EigenMesh& mesh,
        const unsigned int faceId,
        const unsigned int directionIndex,
        const cg3::Vec3& direction,
        cg3::AABBTree<2, cg3::Triangle2Dd>& aabbTree,
        cg3::Array2D<int>& visibility,
        const double heightfieldAngle);

}



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
        CheckMode checkMode)
{
    internal::initializeDataForVisibilityCheck(mesh, nDirections, data);
    internal::computeVisibility(mesh, nDirections, data.directions, data.angles, data.visibility, heightfieldAngle, checkMode);
    internal::detectNonVisibleFaces(data);
}




/* ----- INTERNAL FUNCTION DEFINITION ----- */

namespace internal {


/* ----- METHODS FOR COMPUTING VISIBILITY ----- */

/**
 * @brief Initialize data for visibility check
 * @param[in] mesh Input mesh
 * @param[in] nDirections Number of directions
 * @param[out] data Four axis fabrication data
 */
void initializeDataForVisibilityCheck(
        const cg3::EigenMesh& mesh,
        const unsigned int nDirections,
        Data& data)
{
    const std::vector<unsigned int>& minExtremes = data.minExtremes;
    const std::vector<unsigned int>& maxExtremes = data.maxExtremes;
    cg3::Array2D<int>& visibility = data.visibility;
    std::vector<int>& association = data.association;
    std::vector<cg3::Vec3>& directions = data.directions;


    //Initialize visibility (number of directions, two for extremes)
    visibility.clear();
    visibility.resize(2*nDirections + 2, mesh.getNumberFaces());
    visibility.fill(0);

    //Initialize to -1 direction association for each face
    association.clear();
    association.resize(mesh.getNumberFaces());
    std::fill(association.begin(), association.end(), -1);

    //Initialize direction vector
    directions.clear();
    directions.resize(2*nDirections + 2);



    //Index for min, max extremes
    unsigned int minIndex = 2*nDirections;
    unsigned int maxIndex = 2*nDirections + 1;


    //Add min and max extremes directions
    directions[minIndex] = cg3::Vec3(-1,0,0);
    directions[maxIndex] = cg3::Vec3(1,0,0);


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
        const double heightfieldAngle,
        CheckMode checkMode)
{
    //Set target faces to be checked
    std::vector<unsigned int> targetFaces;

    for (unsigned int i = 0; i < mesh.getNumberFaces(); i++) {
        targetFaces.push_back(i);
    }

    //Copy the mesh
    cg3::EigenMesh rotatingMesh = mesh;

    //Step angle for getting all the directions (on 180 degrees)
    double stepAngle = M_PI / nDirections;

    //Get rotation matrix
    Eigen::Matrix3d rotationMatrix;
    Eigen::Matrix3d rotationMatrixDirection;
    cg3::Vec3 rotationAxis(1,0,0);
    cg3::getRotationMatrix(rotationAxis, stepAngle, rotationMatrix);
    cg3::getRotationMatrix(rotationAxis, -stepAngle, rotationMatrixDirection);

    //Vector that is opposite to the milling direction
    cg3::Vec3 dir(0,0,1);

    //Set angles
    angles.resize(nDirections*2);
    double sum = 0;
    for(unsigned int i = 0; i < nDirections*2; i++) {
        angles[i] = sum;
        sum += stepAngle;
    }

    //For each direction
    for(unsigned int dirIndex = 0; dirIndex < nDirections; dirIndex++){
        if (checkMode == RAYSHOOTING) {
            //Check visibility ray shooting
            internal::getVisibilityRayShootingOnZ(rotatingMesh, targetFaces, nDirections, dirIndex, visibility, heightfieldAngle);
        }
        else {
            //Check visibility with projection
            internal::getVisibilityProjectionOnZ(rotatingMesh, targetFaces, nDirections, dirIndex, visibility, heightfieldAngle);
        }

        //Add the current directions
        directions[dirIndex] = dir;
        directions[nDirections + dirIndex] = -dir;

        //Rotate the direction and the mesh
        rotatingMesh.rotate(rotationMatrix);

        //Rotate the current
        dir.rotate(rotationMatrixDirection);
    }
}

/**
 * @brief Detect faces that are not visible
 * @param data Four axis fabrication data
 */
void detectNonVisibleFaces(
        Data& data)
{
    const cg3::Array2D<int>& visibility = data.visibility;
    std::vector<unsigned int>& nonVisibleFaces = data.nonVisibleFaces;

    //Detect non-visible faces
    nonVisibleFaces.clear();
    for (unsigned int faceId = 0; faceId < visibility.getSizeY(); faceId++){
        //Check if at least a direction has been found for each face
        bool found = false;

        for (unsigned int i = 0; i < visibility.getSizeX() && !found; i++){
            if (visibility(i, faceId) == 1) {
                found = true;
            }
        }

        //Set visibility to the unknown index
        if (!found)
            nonVisibleFaces.push_back(faceId);
    }

}


/* ----- CHECK VISIBILITY (RAY SHOOTING) ----- */

/**
 * @brief Check visibility of each face of the mesh from a given direction.
 * It is implemented by a ray casting algorithm.
 * @param[in] mesh Input mesh. Normals must be updated before.
 * @param[in] nDirection Total number of directions
 * @param[in] directionIndex Index of the direction
 * @param[out] direction Current direction
 * @param[out] visibility Map the visibility from the given directions
 * to each face.
 * @param[in] heightfieldAngle Limit angle with triangles normal in order to be a heightfield
 */
void getVisibilityRayShootingOnZ(
        const cg3::EigenMesh& mesh,
        const std::vector<unsigned int>& faces,
        const unsigned int nDirections,
        const unsigned int directionIndex,
        cg3::Array2D<int>& visibility,
        const double heightfieldAngle)
{
    const double heightFieldLimit = cos(heightfieldAngle);

    //Create cgal AABB on the current mesh
    cg3::cgal::AABBTree tree(mesh);

    //Get bounding box min and max z-coordinate
    double minZ = mesh.getBoundingBox().minZ()-50;
    double maxZ = mesh.getBoundingBox().maxZ()+50;

    for(unsigned int faceIndex : faces){
        //Get the face data
        cg3::Pointi f = mesh.getFace(faceIndex);
        cg3::Vec3 v1 = mesh.getVertex(f.x());
        cg3::Vec3 v2 = mesh.getVertex(f.y());
        cg3::Vec3 v3 = mesh.getVertex(f.z());

        //Barycenter of the face
        cg3::Pointd bar((v1 + v2 + v3) / 3);

        //Calculate the intersection in the mesh
        std::list<int> barIntersection;

        tree.getIntersectedEigenFaces(
                    cg3::Pointd(bar.x(), bar.y(), maxZ),
                    cg3::Pointd(bar.x(), bar.y(), minZ),
                    barIntersection);


        assert(barIntersection.size() >= 2);
#ifdef RELEASECHECK
        if (barIntersection.size() < 2) {
            std::cout << "ERROR: intersections are less than 2 from the direction " << std::endl;
            exit(1);
        }
#endif

        //Directions to be checked
        cg3::Vec3 zDirMax(0,0,1);
        cg3::Vec3 zDirMin(0,0,-1);

        //Set the visibility of the face which has
        //the highest z-coordinate barycenter
        double maxZCoordinate = minZ;
        int maxZFace = -1;

        //Set the visibility of the face which has
        //the lowest z-coordinate barycenter
        double minZCoordinate = maxZ;
        int minZFace = -1;

        for (int intersectedFace : barIntersection) {
            //Get the face data
            cg3::Pointi faceData = mesh.getFace(intersectedFace);
            cg3::Pointd currentBarycenter = (
                        mesh.getVertex(faceData.x()) +
                        mesh.getVertex(faceData.y()) +
                        mesh.getVertex(faceData.z())
            ) / 3;



            //Save face with the maximum Z barycenter and that is visible
            //from (0,0,1)
            if (currentBarycenter.z() > maxZCoordinate &&
                    zDirMax.dot(mesh.getFaceNormal(intersectedFace)) >= heightFieldLimit)
            {
                maxZFace = intersectedFace;
                maxZCoordinate = currentBarycenter.z();
            }

            //Save face with the minimum Z barycenter and that is visible
            //from (0,0,-1) direction
            if (currentBarycenter.z() < minZCoordinate &&
                    zDirMin.dot(mesh.getFaceNormal(intersectedFace)) >= heightFieldLimit)
            {
                minZFace = intersectedFace;
                minZCoordinate = currentBarycenter.z();
            }
        }

        //Set the visibility
        visibility(directionIndex, maxZFace) = 1;
        visibility(nDirections + directionIndex, minZFace) = 1;



        assert(zDirMax.dot(mesh.getFaceNormal(maxZFace)) >= heightFieldLimit);
#ifdef RELEASECHECK
        //TO BE DELETED ON FINAL RELEASE
        if (zDirMax.dot(mesh.getFaceNormal(maxZFace)) < heightFieldLimit) {
            std::cout << "ERROR: not visible triangle, dot product with z-axis is less than 0 for the direction " << mesh.getFaceNormal(maxZFace) << std::endl;
            exit(1);
        }
#endif

        assert(maxZFace >= 0);
#ifdef RELEASECHECK
        //TO BE DELETED ON FINAL RELEASE
        if (maxZFace < 0) {
            std::cout << "ERROR: no candidate max face has been found" << std::endl;
            exit(1);
        }
#endif

        assert(zDirMin.dot(mesh.getFaceNormal(minZFace)) >= heightFieldLimit);
#ifdef RELEASECHECK
        //TO BE DELETED ON FINAL RELEASE
        if (zDirMin.dot(mesh.getFaceNormal(minZFace)) < heightFieldLimit) {
            std::cout << "ERROR: not visible triangle, dot product with opposite of z-axis is less than 0 for the direction " << mesh.getFaceNormal(minZFace) << std::endl;
            exit(1);
        }
#endif

        assert(minZFace >= 0);
#ifdef RELEASECHECK
        //TO BE DELETED ON FINAL RELEASE
        if (minZFace < 0) {
            std::cout << "ERROR: no candidate min face has been found" << std::endl;
            exit(1);
        }
#endif
    }
}




/* ----- CHECK VISIBILITY (PROJECTION) ----- */

/**
 * @brief Check visibility of each face of the mesh from a given direction.
 * It is implemented by checking the intersection of the projection of the
 * segments from a given direction.
 * @param[in] mesh Input mesh. Normals must be updated before.
 * @param[in] nDirection Total number of directions
 * @param[in] directionIndex Index of the direction
 * @param[out] direction Current direction
 * @param[out] visibility Map the visibility from the given directions
 * to each face.
 * @param[in] heightfieldAngle Limit angle with triangles normal in order to be a heightfield
 */
void getVisibilityProjectionOnZ(
        const cg3::EigenMesh& mesh,
        const std::vector<unsigned int>& faces,
        const unsigned int nDirections,
        const unsigned int directionIndex,
        cg3::Array2D<int>& visibility,
        const double heightfieldAngle)
{
    cg3::AABBTree<2, cg3::Triangle2Dd> aabbTreeMax(
                &internal::triangle2DAABBExtractor, &internal::triangle2DComparator);
    cg3::AABBTree<2, cg3::Triangle2Dd> aabbTreeMin(
                &internal::triangle2DAABBExtractor, &internal::triangle2DComparator);

    //Order the face by z-coordinate of the barycenter
    std::vector<unsigned int> orderedZFaces(faces);
    std::sort(orderedZFaces.begin(), orderedZFaces.end(), internal::BarycenterZComparator(mesh));

    //Directions to be checked
    cg3::Vec3 zDirMax(0,0,1);
    cg3::Vec3 zDirMin(0,0,-1);

    //Start from the max z-coordinate face
    for (int i = orderedZFaces.size()-1; i >= 0; i--) {
        unsigned int faceId = orderedZFaces[i];        

        internal::getVisibilityProjectionOnZ(
                    mesh, faceId, directionIndex, zDirMax, aabbTreeMax, visibility, heightfieldAngle);
    }

    //Start from the min z-coordinate face
    for (unsigned int i = 0; i < orderedZFaces.size(); i++) {
        unsigned int faceId = orderedZFaces[i];

        internal::getVisibilityProjectionOnZ(
                    mesh, faceId, nDirections + directionIndex, zDirMin, aabbTreeMin, visibility, heightfieldAngle);
    }
}

/**
 * @brief Check visibility of a face from a given direction.
 * It is implemented by checking the intersection of the projection of the
 * segments from a given direction.
 * @param[in] mesh Input mesh
 * @param[in] faceId Id of the face of the mesh
 * @param[in] directionIndex Current direction index
 * @param[in] direction Direction
 * @param[out] aabbTree AABB tree with projections
 * @param[out] visibility Map the visibility from the given directions
 * to each face.
 * @param[in] heightfieldAngle Limit angle with triangles normal in order to be a heightfield
 */
void getVisibilityProjectionOnZ(
        const cg3::EigenMesh& mesh,
        const unsigned int faceId,
        const unsigned int directionIndex,
        const cg3::Vec3& direction,
        cg3::AABBTree<2, cg3::Triangle2Dd>& aabbTree,
        cg3::Array2D<int>& visibility,
        const double heightfieldAngle)
{
    const double heightFieldLimit = cos(heightfieldAngle);

    //If it is visible (checking the angle between normal and the target direction)
    if (direction.dot(mesh.getFaceNormal(faceId)) >= heightFieldLimit) {
        const cg3::Pointi& faceData = mesh.getFace(faceId);
        const cg3::Pointd& v1 = mesh.getVertex(faceData.x());
        const cg3::Pointd& v2 = mesh.getVertex(faceData.y());
        const cg3::Pointd& v3 = mesh.getVertex(faceData.z());

        //Project on the z plane
        cg3::Point2Dd v1Projected(v1.x(), v1.y());
        cg3::Point2Dd v2Projected(v2.x(), v2.y());
        cg3::Point2Dd v3Projected(v3.x(), v3.y());

        //Create triangle
        cg3::Triangle2Dd triangle(v1Projected, v2Projected, v3Projected);        
        cg3::sortTriangle2DPointsAndReorderCounterClockwise(triangle);

        //Check for intersections
        bool intersectionFound =
                aabbTree.aabbOverlapCheck(triangle, &cg3::triangleOverlap);

        //If no intersections have been found
        if (!intersectionFound) {
            //Set visibility
            visibility(directionIndex, faceId) = 1;

            aabbTree.insert(triangle);
        }
    }

}


}

}
