#include "faf_optimalrotation.h"

#include <cg3/geometry/transformations.h>

//OLD METHOD!
//#include <cg3/algorithms/global_optimal_rotation_matrix.h>
#include <cg3/algorithms/sphere_coverage.h>

namespace FourAxisFabrication {
namespace internal {

Eigen::Matrix3d optimalOrientationRotationMatrix(
        const cg3::SimpleEigenMesh& inputMesh,
        unsigned int nDirs,
        bool deterministic);

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
        const unsigned int nOrientations,
        const bool deterministic)
{
    cg3::Pointd bbCenter = smoothedMesh.boundingBox().center();

    mesh.translate(-bbCenter);
    smoothedMesh.translate(-bbCenter);

    smoothedMesh.updateFaceNormals();

    //Get the optimal rotation matrix
    Eigen::Matrix3d rot = internal::optimalOrientationRotationMatrix(smoothedMesh, nOrientations, deterministic);

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

Eigen::Matrix3d optimalOrientationRotationMatrix(
        const cg3::SimpleEigenMesh& inputMesh,
        unsigned int nDirs,
        bool deterministic)
{
    //Get the direction pool (sphere coverage, fibonacci sampling)l
    std::vector<cg3::Vec3> dirPool = cg3::sphereCoverage(nDirs, deterministic);

    double bestScore = std::numeric_limits<double>::max();
    cg3::Vec3 bestOrientation;
    for (cg3::Vec3& dir : dirPool) {
        cg3::Vec3 axis;
        double angle;
        dir.normalize();

        defineRotation(dir, axis, angle);
        Eigen::Matrix3d mr = rotationMatrix(axis, angle);

        //Get rotated vertices and bounding box
        cg3::BoundingBox bb(
            cg3::Pointd(
                -std::numeric_limits<double>().max(),
                -std::numeric_limits<double>().max(),
                -std::numeric_limits<double>().max()
                ),
            cg3::Pointd(
                std::numeric_limits<double>().max(),
                std::numeric_limits<double>().max(),
                std::numeric_limits<double>().max())
            );

        //Rotate vertices and set bounding box
        std::vector<cg3::Pointd> rotatedVertices(inputMesh.numberVertices());
        for(unsigned int vId = 0; vId < inputMesh.numberVertices(); vId++) {
            cg3::Pointd v = inputMesh.vertex(vId);
            v.rotate(mr);
            rotatedVertices[vId] = v;

            bb.setMaxX(std::max(v.x(), bb.maxX()));
            bb.setMaxY(std::max(v.y(), bb.maxY()));
            bb.setMaxZ(std::max(v.z(), bb.maxZ()));

            bb.setMinX(std::min(v.x(), bb.minX()));
            bb.setMinY(std::min(v.y(), bb.minY()));
            bb.setMinZ(std::min(v.z(), bb.minZ()));
        }

        cg3::Pointd bbCenter = bb.center();

        //Max distance from the bounding box center
        cg3::Pointd maxDistance(0,0,0);
        for(unsigned int vId = 0; vId < inputMesh.numberVertices(); vId++) {
            cg3::Pointd& v = rotatedVertices.at(vId);

            cg3::Vec3 distance = v - bbCenter;

            maxDistance.setX(std::max(std::fabs(distance.x()), maxDistance.x()));
            maxDistance.setY(std::max(std::fabs(distance.y()), maxDistance.y()));
            maxDistance.setZ(std::max(std::fabs(distance.z()), maxDistance.z()));
        }

        double score = 0.0;
        for(unsigned int fId = 0; fId < inputMesh.numberFaces(); fId++) {
            cg3::Pointi f = inputMesh.face(fId);

            cg3::Pointd& v1 = rotatedVertices.at(f.x());
            cg3::Pointd& v2 = rotatedVertices.at(f.y());
            cg3::Pointd& v3 = rotatedVertices.at(f.z());

            cg3::Pointd faceBarycenter = (v1+v2+v3)/3;

            cg3::Vec3 n = inputMesh.faceNormal(fId);
            n.rotate(mr);

            double a = inputMesh.faceArea(fId);
            cg3::Pointd w(
                        1-(std::fabs(faceBarycenter.x() - bbCenter.x()) / maxDistance.x()),
                        1-(std::fabs(faceBarycenter.y() - bbCenter.y()) / maxDistance.y()),
                        1-(std::fabs(faceBarycenter.z() - bbCenter.z()) / maxDistance.z())
            );
            score += a * (w.x() * std::fabs(n.x()) + w.y() * std::fabs(n.y()) + w.z() * std::fabs(n.z()));
        }

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
