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
        const std::vector<unsigned int>& minExtremes,
        const std::vector<unsigned int>& maxExtremes,
        const bool includeXDirections,
        const double heightfieldAngle,
        CheckMode checkMode = PROJECTION);

void detectNonVisibleFaces(
        Data& data);



/* Check visibility (ray shooting) */

void getVisibilityRayShootingOnZ(
        const cg3::EigenMesh& mesh,
        const std::vector<unsigned int>& faces,
        const unsigned int directionIndex,
        const int oppositeDirectionIndex,
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
        const unsigned int directionIndex,
        const int oppositeDirectionIndex,
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


/* Comparators */

struct TriangleZComparator {
    const cg3::SimpleEigenMesh& m;
    TriangleZComparator(const cg3::SimpleEigenMesh& m);
    bool operator()(unsigned int f1, unsigned int f2);
};
bool triangle2DComparator(const cg3::Triangle2Dd& t1, const cg3::Triangle2Dd& t2);


/* AABB extractor functions */

double triangle2DAABBExtractor(
        const cg3::Triangle2Dd& triangle,
        const cg3::AABBValueType& valueType,
        const int& dim);


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
        const bool includeXDirections,
        CheckMode checkMode)
{
    internal::initializeDataForVisibilityCheck(mesh, nDirections, data);
    internal::computeVisibility(mesh, nDirections, data.directions, data.angles, data.visibility, data.minExtremes, data.maxExtremes, includeXDirections, heightfieldAngle, checkMode);
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
    cg3::Array2D<int>& visibility = data.visibility;
    std::vector<int>& association = data.association;
    std::vector<cg3::Vec3>& directions = data.directions;

    //Initialize visibility (number of directions, two for extremes)
    visibility.clear();
    visibility.resize(2*nDirections + 2, mesh.numberFaces());
    visibility.fill(0);

    //Initialize to -1 direction association for each face
    association.clear();
    association.resize(mesh.numberFaces());
    std::fill(association.begin(), association.end(), -1);

    //Initialize direction vector
    directions.clear();
    directions.resize(2*nDirections + 2);
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
        const std::vector<unsigned int>& minExtremes,
        const std::vector<unsigned int>& maxExtremes,
        const bool includeXDirections,
        const double heightfieldAngle,
        CheckMode checkMode)
{
    //Set target faces to be checked
    std::vector<unsigned int> targetFaces;

    for (unsigned int i = 0; i < mesh.numberFaces(); i++) {
        targetFaces.push_back(i);
    }

    //Copy the mesh
    cg3::EigenMesh rotatingMesh = mesh;

    //Step angle for getting all the directions (on 180 degrees)
    double stepAngle = M_PI / nDirections;

    const cg3::Vec3 xAxis(1,0,0);
    const cg3::Vec3 yAxis(0,1,0);

    //Get rotation matrix
    Eigen::Matrix3d rotationMatrix;
    Eigen::Matrix3d rotationMatrixDirection;
    cg3::rotationMatrix(xAxis, stepAngle, rotationMatrix);
    cg3::rotationMatrix(xAxis, -stepAngle, rotationMatrixDirection);

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
            internal::getVisibilityRayShootingOnZ(rotatingMesh, targetFaces, dirIndex, nDirections + dirIndex, visibility, heightfieldAngle);
        }
        else {
            //Check visibility with projection
            internal::getVisibilityProjectionOnZ(rotatingMesh, targetFaces, dirIndex, nDirections + dirIndex, visibility, heightfieldAngle);
        }

        //Add the current directions
        directions[dirIndex] = dir;
        directions[nDirections + dirIndex] = -dir;

        //Rotate the direction and the mesh
        rotatingMesh.rotate(rotationMatrix);

        //Rotate the current
        dir.rotate(rotationMatrixDirection);
    }

    //Index for min, max extremes
    unsigned int minIndex = nDirections*2;
    unsigned int maxIndex = nDirections*2 + 1;

    //Add min and max extremes directions
    directions[minIndex] = cg3::Vec3(-1,0,0);
    directions[maxIndex] = cg3::Vec3(1,0,0);

    if (includeXDirections) {
        //Compute -x and +x visibility
        rotatingMesh = mesh;
        cg3::rotationMatrix(yAxis, M_PI/2, rotationMatrix);
        rotatingMesh.rotate(rotationMatrix);
        if (checkMode == RAYSHOOTING) {
            //Check visibility ray shooting
            internal::getVisibilityRayShootingOnZ(rotatingMesh, targetFaces, minIndex, maxIndex, visibility, heightfieldAngle);
        }
        else {
            //Check visibility with projection
            internal::getVisibilityProjectionOnZ(rotatingMesh, targetFaces, minIndex, maxIndex, visibility, heightfieldAngle);
        }
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
        const unsigned int directionIndex,
        const int oppositeDirectionIndex,
        cg3::Array2D<int>& visibility,
        const double heightfieldAngle)
{
    const double heightFieldLimit = cos(heightfieldAngle);

    //Create cgal AABB on the current mesh
    cg3::cgal::AABBTree tree(mesh);

    //Get bounding box min and max z-coordinate
    double minZ = mesh.boundingBox().minZ()-1;
    double maxZ = mesh.boundingBox().maxZ()+1;

    for(unsigned int faceIndex : faces){
        //Get the face data
        cg3::Pointi f = mesh.face(faceIndex);
        cg3::Vec3 v1 = mesh.vertex(f.x());
        cg3::Vec3 v2 = mesh.vertex(f.y());
        cg3::Vec3 v3 = mesh.vertex(f.z());

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
            cg3::Pointi faceData = mesh.face(intersectedFace);
            cg3::Pointd currentBarycenter = (
                        mesh.vertex(faceData.x()) +
                        mesh.vertex(faceData.y()) +
                        mesh.vertex(faceData.z())
            ) / 3;



            //Save face with the maximum Z barycenter and that is visible
            //from (0,0,1)
            if (currentBarycenter.z() > maxZCoordinate &&
                    zDirMax.dot(mesh.faceNormal(intersectedFace)) >= heightFieldLimit)
            {
                maxZFace = intersectedFace;
                maxZCoordinate = currentBarycenter.z();
            }


            if (oppositeDirectionIndex >= 0) {
                //Save face with the minimum Z barycenter and that is visible
                //from (0,0,-1) direction
                if (currentBarycenter.z() < minZCoordinate &&
                        zDirMin.dot(mesh.faceNormal(intersectedFace)) >= heightFieldLimit)
                {
                    minZFace = intersectedFace;
                    minZCoordinate = currentBarycenter.z();
                }
            }
        }

        //Set the visibility
        visibility(directionIndex, maxZFace) = 1;

        if (oppositeDirectionIndex >= 0) {
            visibility(oppositeDirectionIndex, minZFace) = 1;
        }


        assert(zDirMax.dot(mesh.faceNormal(maxZFace)) >= heightFieldLimit);
#ifdef RELEASECHECK
        //TO BE DELETED ON FINAL RELEASE
        if (zDirMax.dot(mesh.faceNormal(maxZFace)) < heightFieldLimit) {
            std::cout << "ERROR: not visible triangle, dot product with z-axis is less than 0 for the direction " << mesh.faceNormal(maxZFace) << std::endl;
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


        if (oppositeDirectionIndex >= 0) {
            assert(zDirMin.dot(mesh.faceNormal(minZFace)) >= heightFieldLimit);
#ifdef RELEASECHECK
            //TO BE DELETED ON FINAL RELEASE
            if (zDirMin.dot(mesh.faceNormal(minZFace)) < heightFieldLimit) {
                std::cout << "ERROR: not visible triangle, dot product with opposite of z-axis is less than 0 for the direction " << mesh.faceNormal(minZFace) << std::endl;
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
        const unsigned int directionIndex,
        const int oppositeDirectionIndex,
        cg3::Array2D<int>& visibility,
        const double heightfieldAngle)
{
    cg3::AABBTree<2, cg3::Triangle2Dd> aabbTreeMax(
                &internal::triangle2DAABBExtractor, &internal::triangle2DComparator);
    cg3::AABBTree<2, cg3::Triangle2Dd> aabbTreeMin(
                &internal::triangle2DAABBExtractor, &internal::triangle2DComparator);

    //Order the face by z-coordinate of the barycenter
    std::vector<unsigned int> orderedZFaces(faces);
    std::sort(orderedZFaces.begin(), orderedZFaces.end(), internal::TriangleZComparator(mesh));

    //Directions to be checked
    cg3::Vec3 zDirMax(0,0,1);
    cg3::Vec3 zDirMin(0,0,-1);

    //Start from the max z-coordinate face
    for (int i = orderedZFaces.size()-1; i >= 0; i--) {
        unsigned int faceId = orderedZFaces[i];        

        internal::getVisibilityProjectionOnZ(
                    mesh, faceId, directionIndex, zDirMax, aabbTreeMax, visibility, heightfieldAngle);
    }

    if (oppositeDirectionIndex >= 0) {
        //Start from the min z-coordinate face
        for (unsigned int i = 0; i < orderedZFaces.size(); i++) {
            unsigned int faceId = orderedZFaces[i];

            internal::getVisibilityProjectionOnZ(
                        mesh, faceId, oppositeDirectionIndex, zDirMin, aabbTreeMin, visibility, heightfieldAngle);
        }
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
    if (direction.dot(mesh.faceNormal(faceId)) >= heightFieldLimit) {
        const cg3::Pointi& faceData = mesh.face(faceId);
        const cg3::Pointd& v1 = mesh.vertex(faceData.x());
        const cg3::Pointd& v2 = mesh.vertex(faceData.y());
        const cg3::Pointd& v3 = mesh.vertex(faceData.z());

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


/* ----- COMPARATORS ----- */

/**
 * @brief Comparator for barycenter on z values
 * @param m Input mesh
 */
TriangleZComparator::TriangleZComparator(const cg3::SimpleEigenMesh& m) : m(m) {}
bool TriangleZComparator::operator()(unsigned int f1, unsigned int f2){
    const cg3::Pointi& ff1 = m.face(f1);
    const cg3::Pointi& ff2 = m.face(f2);
    double minZ1 = std::min(std::min(m.vertex(ff1.x()).z(), m.vertex(ff1.y()).z()),m.vertex(ff1.z()).z());
    double minZ2 = std::min(std::min(m.vertex(ff2.x()).z(), m.vertex(ff2.y()).z()),m.vertex(ff2.z()).z());
    return minZ1 < minZ2;
}

/**
 * @brief Comparator for triangles
 * @param t1 Triangle 1
 * @param t2 Triangle 2
 * @return True if triangle 1 is less than triangle 2
 */
bool triangle2DComparator(const cg3::Triangle2Dd& t1, const cg3::Triangle2Dd& t2) {
    if (t1.v1() < t2.v1())
        return true;
    if (t2.v1() < t1.v1())
        return false;

    if (t1.v2() < t2.v2())
        return true;
    if (t2.v2() < t1.v2())
        return false;

    return t1.v3() < t2.v3();
}

/* ----- TRIANGLE OVERLAP AND AABB FUNCTIONS ----- */

/**
 * @brief Extract a 2D triangle AABB
 * @param[in] triangle Input triangle
 * @param[in] valueType Type of the value requested (MIN or MAX)
 * @param[in] dim Dimension requested of the value (0 for x, 1 for y)
 * @return Requested coordinate of the AABB
 */
double triangle2DAABBExtractor(
        const cg3::Triangle2Dd& triangle,
        const cg3::AABBValueType& valueType,
        const int& dim)
{
    if (valueType == cg3::AABBValueType::MIN) {
        switch (dim) {
        case 1:
            return (double) std::min(std::min(triangle.v1().x(), triangle.v2().x()), triangle.v3().x());
        case 2:
            return (double) std::min(std::min(triangle.v1().y(), triangle.v2().y()), triangle.v3().y());
        }
    }
    else if (valueType == cg3::AABBValueType::MAX) {
        switch (dim) {
        case 1:
            return (double) std::max(std::max(triangle.v1().x(), triangle.v2().x()), triangle.v3().x());
        case 2:
            return (double) std::max(std::max(triangle.v1().y(), triangle.v2().y()), triangle.v3().y());
        }
    }
    throw new std::runtime_error("Impossible to extract an AABB value.");
}




}

}
