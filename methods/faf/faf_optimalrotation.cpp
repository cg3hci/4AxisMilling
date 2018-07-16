#include "faf_optimalrotation.h"

#include <cg3/geometry/transformations.h>

#include <cg3/algorithms/global_optimal_rotation_matrix.h>

namespace FourAxisFabrication {


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

    cg3::BoundingBox b = mesh.boundingBox();

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

    //Translate meshes to the center
    mesh.translate(-mesh.boundingBox().center());
    smoothedMesh.translate(-smoothedMesh.boundingBox().center());
}



}
