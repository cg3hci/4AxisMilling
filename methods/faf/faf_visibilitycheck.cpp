/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_visibilitycheck.h"

#ifdef CG3_CGAL_DEFINED
/* Uncomment to use CGAL intersection system */
//#define USE_CGAL_INTERSECTIONS
#endif

#include <cg3/geometry/transformations.h>
#include <cg3/geometry/2d/point2d.h>
#include <cg3/geometry/point.h>
#include <cg3/geometry/2d/segment2d.h>

#include <cg3/data_structures/trees/aabbtree.h>

#include <cg3/cgal/cgal_aabbtree.h>

#ifdef USE_CGAL_INTERSECTIONS
#include <cg3/cgal/2d/cgal_intersections2d.h>
#else
#include <cg3/geometry/2d/intersections2d.h>
#endif

//Uncomment if you want to check conditions
//(more efficient when uncommented)
#define RELEASECHECK


namespace FourAxisFabrication {


/* ----- INTERNAL FUNCTION DECLARATION ----- */

namespace internal {

/* Check visibility (ray shooting) */

void checkVisibilityRayShootingOnZ(
        const cg3::EigenMesh& mesh,
        const std::vector<unsigned int>& faces,
        const unsigned int nDirections,
        const unsigned int directionIndex,
        cg3::Array2D<int>& visibility);

void checkVisibilityRayShootingOnZ(
        const cg3::EigenMesh& mesh,
        const unsigned int faceId,
        const unsigned int directionIndex,
        const cg3::Vec3& direction,
        cg3::cgal::AABBTree& aabbTree,
        cg3::Array2D<int>& visibility);



/* Check visibility (projection) */

void checkVisibilityProjectionOnZ(
        const cg3::EigenMesh& mesh,
        const std::vector<unsigned int>& faces,
        const unsigned int nDirections,
        const unsigned int directionIndex,
        cg3::Array2D<int>& visibility);

void checkVisibilityProjectionOnZ(
        const cg3::EigenMesh& mesh,
        const unsigned int faceId,
        const unsigned int directionIndex,
        const cg3::Vec3& direction,
        cg3::AABBTree<2, cg3::Segment2Dd>& aabbTree,
        cg3::Array2D<int>& visibility);


/* Segment intersection and AABB functions */

double segmentAABBExtractor(
        const cg3::Segment2Dd& triangle,
        const cg3::AABBValueType& valueType,
        const int& dim);

bool segmentIntersectionChecker(
        const cg3::Segment2Dd& s1,
        const cg3::Segment2Dd& s2);


/* Comparators */

struct BarycenterZComparator {
    const cg3::EigenMesh& m;
    BarycenterZComparator(const cg3::EigenMesh& m) : m(m) {}
    bool operator()(unsigned int f1, unsigned int f2){
        const cg3::Pointi& ff1 = m.getFace(f1);
        const cg3::Pointi& ff2 = m.getFace(f2);
        cg3::Pointd c1 = (m.getVertex(ff1.x()) + m.getVertex(ff1.y()) + m.getVertex(ff1.z()))/3;
        cg3::Pointd c2 = (m.getVertex(ff2.x()) + m.getVertex(ff2.y()) + m.getVertex(ff2.z()))/3;
        if (c1.z() < c2.z())
            return true;
        if (c2.z() < c1.z())
            return false;
        return c1 < c2;
    }
};

}



/* ----- VISIBILITY CHECK ----- */


/**
 * @brief Initialize data for visibility check
 * @param mesh Input mesh
 * @param nDirections Number of directions
 * @param fixExtremeAssociation Set if faces in the extremes must be already and unconditionally
 * assigned to the x-axis directions
 * @param data Four axis fabrication data
 */
void initializeDataForVisibilityCheck(
        const cg3::EigenMesh& mesh,
        const unsigned int nDirections,
        const bool fixExtremeAssociation,
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


    //Set visibility and association of the min extremes
    #pragma omp parallel for
    for (unsigned int i = 0; i < minExtremes.size(); i++){
        unsigned int faceId = minExtremes[i];

        visibility(minIndex, faceId) = 1;

        if (fixExtremeAssociation) {
            association[faceId] = (int) minIndex;
        }
    }

    //Set visibility and association of the max extremes
    #pragma omp parallel for
    for (unsigned int i = 0; i < maxExtremes.size(); i++){
        unsigned int faceId = maxExtremes[i];

        visibility(maxIndex, faceId) = 1;

        if (fixExtremeAssociation) {
            association[faceId] = (int) maxIndex;
        }
    }

}


/**
 * @brief Check visibility of each face of the mesh from a given number of
 * different directions.
 * It is implemented by a ray casting algorithm.
 * @param[in] mesh Input mesh
 * @param[in] numberDirections Number of directions to be checked
 * @param[out] data Four axis fabrication data
 * @param[in] checkMode Visibility check mode. Default is projection mode.
 */
void checkVisibility(
        const cg3::EigenMesh& mesh,
        const unsigned int nDirections,
        Data& data,
        CheckMode checkMode = PROJECTION)
{
    const std::vector<int>& association = data.association;
    std::vector<cg3::Vec3>& directions = data.directions;
    cg3::Array2D<int>& visibility = data.visibility;

    //Set target faces to be checked
    std::vector<unsigned int> targetFaces;

    for (unsigned int i = 0; i < mesh.getNumberFaces(); i++) {
        //We do not include faces that have already an association
        if (association[i] < 0) {
            targetFaces.push_back(i);
        }
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


    //For each direction
    for(unsigned int dirIndex = 0; dirIndex < nDirections; dirIndex++){
        rotatingMesh.updateFaceNormals();

        if (checkMode == RAYSHOOTING) {
            //Check visibility ray shooting
            internal::checkVisibilityRayShootingOnZ(rotatingMesh, targetFaces, nDirections, dirIndex, visibility);
        }
        else {
            //Check visibility with projection
            internal::checkVisibilityProjectionOnZ(rotatingMesh, targetFaces, nDirections, dirIndex, visibility);
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





namespace internal {


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
 */
void checkVisibilityRayShootingOnZ(
        const cg3::EigenMesh& mesh,
        const std::vector<unsigned int>& faces,
        const unsigned int nDirections,
        const unsigned int directionIndex,
        cg3::Array2D<int>& visibility)
{
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
        std::vector<int> barIntersection;

        tree.getIntersectEigenFaces(
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
                    zDirMax.dot(mesh.getFaceNormal(intersectedFace)) >= 0)
            {
                maxZFace = intersectedFace;
                maxZCoordinate = currentBarycenter.z();
            }

            //Save face with the minimum Z barycenter and that is visible
            //from (0,0,-1) direction
            if (currentBarycenter.z() < minZCoordinate &&
                    zDirMin.dot(mesh.getFaceNormal(intersectedFace)) >= 0)
            {
                minZFace = intersectedFace;
                minZCoordinate = currentBarycenter.z();
            }
        }

        //Set the visibility
        visibility(directionIndex, maxZFace) = 1;
        visibility(nDirections + directionIndex, minZFace) = 1;



        assert(zDirMax.dot(mesh.getFaceNormal(maxZFace)) >= 0);
#ifdef RELEASECHECK
        //TO BE DELETED ON FINAL RELEASE
        if (zDirMax.dot(mesh.getFaceNormal(maxZFace)) < 0) {
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

        assert(zDirMin.dot(mesh.getFaceNormal(minZFace)) >= 0);
#ifdef RELEASECHECK
        //TO BE DELETED ON FINAL RELEASE
        if (zDirMin.dot(mesh.getFaceNormal(minZFace)) < 0) {
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
 */
void checkVisibilityProjectionOnZ(
        const cg3::EigenMesh& mesh,
        const std::vector<unsigned int>& faces,
        const unsigned int nDirections,
        const unsigned int directionIndex,
        cg3::Array2D<int>& visibility)
{
    cg3::AABBTree<2, cg3::Segment2Dd> aabbTreeMax(
                &internal::segmentAABBExtractor);
    cg3::AABBTree<2, cg3::Segment2Dd> aabbTreeMin(
                &internal::segmentAABBExtractor);

    //Order the face by z-coordinate of the barycenter
    std::vector<unsigned int> orderedZFaces(faces);
    std::sort(orderedZFaces.begin(), orderedZFaces.end(), internal::BarycenterZComparator(mesh));

    //Directions to be checked
    cg3::Vec3 zDirMax(0,0,1);
    cg3::Vec3 zDirMin(0,0,-1);

    //Start from the max z-coordinate face
    for (int i = orderedZFaces.size()-1; i >= 0; i--) {
        unsigned int faceId = orderedZFaces[i];        

        internal::checkVisibilityProjectionOnZ(
                    mesh, faceId, directionIndex, zDirMax, aabbTreeMax, visibility);
    }

    //Start from the min z-coordinate face
    for (unsigned int i = 0; i < orderedZFaces.size(); i++) {
        unsigned int faceId = orderedZFaces[i];

        internal::checkVisibilityProjectionOnZ(
                    mesh, faceId, nDirections + directionIndex, zDirMin, aabbTreeMin, visibility);
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
 */
void checkVisibilityProjectionOnZ(
        const cg3::EigenMesh& mesh,
        const unsigned int faceId,
        const unsigned int directionIndex,
        const cg3::Vec3& direction,
        cg3::AABBTree<2, cg3::Segment2Dd>& aabbTree,
        cg3::Array2D<int>& visibility)
{
    //If it is visible checking the angle
    if (direction.dot(mesh.getFaceNormal(faceId)) >= 0) {
        const cg3::Pointi& faceData = mesh.getFace(faceId);
        const cg3::Pointd& v1 = mesh.getVertex(faceData.x());
        const cg3::Pointd& v2 = mesh.getVertex(faceData.y());
        const cg3::Pointd& v3 = mesh.getVertex(faceData.z());

        //Project on the z plane
        cg3::Point2Dd v1Projected(v1.x(), v1.y());
        cg3::Point2Dd v2Projected(v2.x(), v2.y());
        cg3::Point2Dd v3Projected(v3.x(), v3.y());

        //Create segments
        cg3::Segment2Dd seg1(std::min(v1Projected, v2Projected), std::max(v1Projected, v2Projected));
        cg3::Segment2Dd seg2(std::min(v2Projected, v3Projected), std::max(v2Projected, v3Projected));
        cg3::Segment2Dd seg3(std::min(v3Projected, v1Projected), std::max(v3Projected, v1Projected));

        //Check for intersections
        bool intersectionFound =
                aabbTree.aabbOverlapCheck(seg1, &internal::segmentIntersectionChecker) |
                aabbTree.aabbOverlapCheck(seg2, &internal::segmentIntersectionChecker) |
                aabbTree.aabbOverlapCheck(seg3, &internal::segmentIntersectionChecker);

        //If no intersections have been found
        if (!intersectionFound) {
            //Set visibility
            visibility(directionIndex, faceId) = 1;

            aabbTree.insert(seg1);
            aabbTree.insert(seg2);
            aabbTree.insert(seg3);
        }
    }

}



/* ----- SEGMENT INTERSECTION AND AABB FUNCTIONS ----- */

/**
 * @brief Extract a 2D segment AABB
 * @param[in] segment Input segment
 * @param[in] valueType Type of the value requested (MIN or MAX)
 * @param[in] dim Dimension requested of the value (0 for x, 1 for y)
 * @return Requested coordinate of the AABB
 */
double segmentAABBExtractor(
        const cg3::Segment2Dd& segment,
        const cg3::AABBValueType& valueType,
        const int& dim)
{
    if (valueType == cg3::AABBValueType::MIN) {
        switch (dim) {
        case 1:
            return (double) std::min(segment.getP1().x(), segment.getP2().x());
        case 2:
            return (double) std::min(segment.getP1().y(), segment.getP2().y());
        }
    }
    else if (valueType == cg3::AABBValueType::MAX) {
        switch (dim) {
        case 1:
            return (double) std::max(segment.getP1().x(), segment.getP2().x());
        case 2:
            return (double) std::max(segment.getP1().y(), segment.getP2().y());
        }
    }
    throw new std::runtime_error("Impossible to extract an AABB value.");
}

/**
 * @brief Check if two segment have an (internal) intersection.
 * Intersections of segments with same endpoints are not included.
 *
 * Note also that CGAL intersection checker could report as intersection
 * the segments having a vertex which lie in the other segment line.
 *
 * @param[in] s1 Segment 1
 * @param[in] s2 Segment 2
 * @return True if the segment have an intersection.
 */
bool segmentIntersectionChecker(
        const cg3::Segment2Dd& s1, const cg3::Segment2Dd& s2)
{
#ifdef USE_CGAL_INTERSECTIONS
    return cg3::cgal::checkSegmentIntersection2D(s1, s2, true);
#else
    return cg3::checkSegmentIntersection2D(s1, s2, true);
#endif
}

}

}
