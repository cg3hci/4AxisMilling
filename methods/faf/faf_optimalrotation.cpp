#include "faf_optimalrotation.h"

#include <cg3/geometry/transformations.h>

//OLD METHOD!
//#include <cg3/algorithms/global_optimal_rotation_matrix.h>
#include <cg3/algorithms/sphere_coverage.h>

namespace FourAxisFabrication {
namespace internal {

Eigen::Matrix3d optimalOrientationRotationMatrix(
        const cg3::SimpleEigenMesh& inputMesh,        
        const unsigned int nDirs,
        const double weightPower,
        const bool deterministic);

void defineRotation(
        const cg3::Vec3& zAxis,
        cg3::Vec3& rotationAxis,
        double& angle);


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
        const unsigned int nDirs,
        const double weightPower,
        const bool deterministic)
{
    cg3::Pointd bbCenter = smoothedMesh.boundingBox().center();

    mesh.translate(-bbCenter);
    smoothedMesh.translate(-bbCenter);

    smoothedMesh.updateFaceNormals();

    //Get the optimal rotation matrix
    Eigen::Matrix3d rot = internal::optimalOrientationRotationMatrix(smoothedMesh, nDirs, weightPower, deterministic);

//OLD METHOD!
//    Eigen::Matrix3d rot = cg3::globalOptimalRotationMatrix(smoothedMesh, nOrientations, deterministic);

    //Rotate meshes
    mesh.rotate(rot);
    smoothedMesh.rotate(rot);

    smoothedMesh.updateBoundingBox();
    cg3::BoundingBox b = smoothedMesh.boundingBox();

    //If Y length is the greatest one then rotate by 90° around zAxis
    if (b.lengthY() > b.lengthX() && b.lengthY() > b.lengthZ()){
        cg3::Vec3 zAxis(0,0,1);
        double angle = M_PI/2;

        mesh.rotate(cg3::rotationMatrix(zAxis, angle));
        smoothedMesh.rotate(cg3::rotationMatrix(zAxis, angle));
    }
    //If Z length is the greatest one then rotate by 90° around yAxis
    else if (b.lengthZ() > b.lengthX() && b.lengthZ() > b.lengthY()){
        cg3::Vec3 yAxis(0,1,0);
        double angle = M_PI/2;

        mesh.rotate(cg3::rotationMatrix(yAxis, angle));
        smoothedMesh.rotate(cg3::rotationMatrix(yAxis, angle));
    }

    mesh.updateBoundingBox();
    smoothedMesh.updateBoundingBox();
    mesh.updateFacesAndVerticesNormals();
    smoothedMesh.updateFacesAndVerticesNormals();
}

namespace internal {

/**
 * @brief Get optimal orientation on x-axis for four axis
 * fabrication.
 * @param[out] input Input mesh. The bounding box center of the mesh must be on the origin.
 * @param[in] nDirections Number of directions to check
 * @param[in] deterministic Deterministic approach (if false it is randomized)
 * @return Rotation matrix
 */
Eigen::Matrix3d optimalOrientationRotationMatrix(
        const cg3::SimpleEigenMesh& inputMesh,
        const unsigned int nDirs,
        const double weightPower,
        const bool deterministic)
{
    //Get the direction pool (sphere coverage, fibonacci sampling)l
    std::vector<cg3::Vec3> dirPool = cg3::sphereCoverage(nDirs, deterministic);

    //Bounding box center of the mesh
    cg3::Pointd bbCenter = inputMesh.boundingBox().center();

    //Save vertices and get max distance from bounding box center
    std::vector<cg3::Pointd> vertices(inputMesh.numberVertices());
    double maxDistance = 0;
    for(unsigned int vId = 0; vId < inputMesh.numberVertices(); vId++) {
        const cg3::Pointd v = inputMesh.vertex(vId);

        vertices[vId] = v;

        //Compute max distance
        double distance = (v - bbCenter).length();
        if (distance >= maxDistance) {
            maxDistance = distance;
        }
    }

    //Compute face barycenter and normals
    std::vector<cg3::Pointd> faceBarycenters(inputMesh.numberFaces());
    std::vector<cg3::Pointd> faceNormals(inputMesh.numberFaces());
    std::vector<double> faceAreas(inputMesh.numberFaces());
    for(unsigned int fId = 0; fId < inputMesh.numberFaces(); fId++) {
        cg3::Pointi f = inputMesh.face(fId);
        const cg3::Pointd& v1 = vertices.at(f.x());
        const cg3::Pointd& v2 = vertices.at(f.y());
        const cg3::Pointd& v3 = vertices.at(f.z());

        //Face barycenter
        const cg3::Pointd faceBarycenter = (v1+v2+v3)/3;
        faceBarycenters[fId] = faceBarycenter;

        //Face normal
        const cg3::Vec3 n = inputMesh.faceNormal(fId);
        faceNormals[fId] = n;

        //Face normal
        const double a = inputMesh.faceArea(fId);
        faceAreas[fId] = a;
    }

    //Compute the best orientation
    double bestScore = std::numeric_limits<double>::max();
    cg3::Vec3 bestOrientation;
    for (cg3::Vec3& dir : dirPool) {
        cg3::Vec3 axis;
        double angle;
        dir.normalize();

        //Get rotation matrix of current direction
        defineRotation(dir, axis, angle);
        Eigen::Matrix3d mr = rotationMatrix(axis, angle);

        //Compute current score
        double score = 0.0;
        for(unsigned int fId = 0; fId < inputMesh.numberFaces(); fId++) {
            //Get normal and rotate it
            cg3::Vec3 n = faceNormals.at(fId);
            n.rotate(mr);

            //Get barycenter and rotate it
            cg3::Pointd b = faceBarycenters.at(fId);
            b.rotate(mr);

            //Get face area
            double a = faceAreas.at(fId);

            //Compute weight
            double weight = pow((b - bbCenter).length() / maxDistance, weightPower);

            //Add to score
            score += a * weight * (std::fabs(n.x()) + std::fabs(n.y()) + std::fabs(n.z()));
        }

        //Select the best orientation
        if (score <= bestScore) {
            bestOrientation = dir;
            bestScore = score;
        }
    }

    cg3::Vec3 axis;
    double angle;
    bestOrientation.normalize();

    defineRotation(bestOrientation, axis, angle);

    return cg3::rotationMatrix(cg3::Vec3(axis), angle);
}

void defineRotation(const cg3::Vec3& zAxis,
                    cg3::Vec3& rotationAxis,
                    double& angle)
{
    const cg3::Vec3 Z(0,0,1);
    rotationAxis = zAxis.cross(Z);
    rotationAxis.normalize();
    angle = acos(zAxis.dot(Z));
    assert(!std::isnan(angle));
}


}

}
