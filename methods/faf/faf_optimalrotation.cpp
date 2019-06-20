#include "faf_optimalrotation.h"

#include "faf_extremes.h"

#include <cg3/geometry/transformations3.h>
#include <cg3/algorithms/sphere_coverage.h>
#include <cg3/libigl/mesh_adjacencies.h>

#include <Eigen/Dense>

//OLD METHOD!
//#include <cg3/algorithms/global_optimal_rotation_matrix.h>

namespace FourAxisFabrication {
namespace internal {

void principalComponentAnalysis(
        const cg3::EigenMesh& mesh,
        Eigen::Vector3d& eigenValues,
        Eigen::Matrix3d& eigenVectors);

void principalComponentAnalysis(
        const Eigen::MatrixX3d& mat,
        Eigen::Vector3d& eigenValues,
        Eigen::Matrix3d& eigenVectors);

void rotateToPrincipalComponents(
        cg3::EigenMesh& mesh,
        cg3::EigenMesh& smoothedMesh);

//Eigen::Matrix3d optimalOrientationRotationMatrix(
//        const cg3::SimpleEigenMesh& inputMesh,
//        const unsigned int nDirs,
//        const double weightPower,
//        const bool deterministic);

//void defineRotation(
//        const cg3::Vec3& zAxis,
//        cg3::Vec3& rotationAxis,
//        double& angle);


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
        const double extremeWeight,
        const double BBweight,
        const bool deterministic)
{
    cg3::Vec3 xAxis(1,0,0);

    const double normalWeight = 1 - extremeWeight - BBweight;

    const double heightFieldAngle = M_PI/2;

    //Translate mesh on the centre
    cg3::Point3d bbCenter = smoothedMesh.boundingBox().center();
    mesh.translate(-bbCenter);
    smoothedMesh.translate(-bbCenter);



//    //Rotate to the longest side of the bounding box
//    smoothedMesh.updateBoundingBox();
//    cg3::BoundingBox b = smoothedMesh.boundingBox();

//    //If Y length is the greatest one then rotate by 90° around zAxis
//    if (b.lengthY() > b.lengthX() && b.lengthY() > b.lengthZ()){
//        cg3::Vec3 zAxis(0,0,1);
//        double angle = M_PI/2;

//        mesh.rotate(cg3::rotationMatrix(zAxis, angle));
//        smoothedMesh.rotate(cg3::rotationMatrix(zAxis, angle));
//    }
//    //If Z length is the greatest one then rotate by 90° around yAxis
//    else if (b.lengthZ() > b.lengthX() && b.lengthZ() > b.lengthY()){
//        cg3::Vec3 yAxis(0,1,0);
//        double angle = M_PI/2;

//        mesh.rotate(cg3::rotationMatrix(yAxis, angle));
//        smoothedMesh.rotate(cg3::rotationMatrix(yAxis, angle));
//    }


    //Get the direction pool (sphere coverage, fibonacci sampling)l
    std::vector<cg3::Vec3> dirPool = cg3::sphereCoverage(nDirs, deterministic);

//    internal::rotateToPrincipalComponents(mesh, smoothedMesh);

//    //Get candidate rotation dirs
//    const double offsetAngle = M_PI/2;
//    const double offsetAngleLimit = cos(offsetAngle);
//    const cg3::Vec3 xAxis(1,0,0);
//    std::vector<cg3::Vec3> candidateDirs;
//    for (cg3::Vec3& dir : dirPool) {
//        dir.normalize();
//        if (dir.dot(xAxis) >= offsetAngleLimit) {
//            candidateDirs.push_back(dir);
//        }
//    }


    //Get candidate rotation dirs
    std::vector<cg3::Vec3> candidateDirs;
    for (cg3::Vec3& dir : dirPool) {
        dir.normalize();
        candidateDirs.push_back(dir);
    }

    //Faces adjacencies
    const std::vector<std::vector<int>> ffAdj = cg3::libigl::faceToFaceAdjacencies(mesh);

    std::vector<double> normalScores(candidateDirs.size(), 0.0);
    std::vector<double> extremeScores(candidateDirs.size(), 0.0);
    std::vector<double> BBScores(candidateDirs.size(), 0.0);

    double maxNormalScore = -std::numeric_limits<double>::max();
    double maxBBScore = -std::numeric_limits<double>::max();
    double maxExtremeScore = -std::numeric_limits<double>::max();

    for (size_t i = 0; i < candidateDirs.size(); i++) {
        const cg3::Vec3& dir = candidateDirs[i];
        cg3::Vec3 rotationAxis;
        double angle;

        rotationAxis = dir.cross(xAxis);
        rotationAxis.normalize();
        double dirDot = dir.dot(xAxis);
        angle = acos(dirDot);
        assert(!std::isnan(angle));
        Eigen::Matrix3d rot = cg3::rotationMatrix(rotationAxis, angle);

        cg3::EigenMesh copyMesh = smoothedMesh;
        copyMesh.rotate(rot);
        copyMesh.updateBoundingBox();
        copyMesh.updateFaceNormals();
        cg3::BoundingBox3 copyBB = copyMesh.boundingBox();

        BBScores[i] = copyBB.lengthX();

        std::vector<unsigned int> minExtremes;
        std::vector<unsigned int> maxExtremes;
        double levelSetMinX;
        double levelSetMaxX;
        selectExtremesOnXAxis(copyMesh, heightFieldAngle, ffAdj, minExtremes, maxExtremes, levelSetMinX, levelSetMaxX);

        std::set<unsigned int> extremesSet;
        extremesSet.insert(minExtremes.begin(), minExtremes.end());
        extremesSet.insert(maxExtremes.begin(), maxExtremes.end());

        double totalArea = 0;
        double minArea = 0;
        double maxArea = 0;

        //Get normal scores
        for (size_t fId = 0; fId < minExtremes.size(); fId++) {
            cg3::Vec3 n = copyMesh.faceNormal(minExtremes[fId]);
            double a = copyMesh.faceArea(fId);
            normalScores[i] += a * n.dot(-xAxis);
            minArea += a;
            totalArea += a;
        }
        for (size_t fId = 0; fId < maxExtremes.size(); fId++) {
            cg3::Vec3 n = copyMesh.faceNormal(maxExtremes[fId]);
            double a = copyMesh.faceArea(fId);
            normalScores[i] += a * n.dot(xAxis);
            maxArea += a;
            totalArea += a;
        }
        for(unsigned int fId = 0; fId < copyMesh.numberFaces(); fId++) {
            if (extremesSet.find(fId) == extremesSet.end()) {
                cg3::Vec3 n = copyMesh.faceNormal(fId);
                double a = copyMesh.faceArea(fId);
                normalScores[i] += a * (1 - std::fabs(n.dot(xAxis)));                
                totalArea += a;
            }
        }

        //Flatness
        extremeScores[i] = (minArea + maxArea) / totalArea;


        //Get the maximum normal score
        maxNormalScore = std::max(normalScores[i], maxNormalScore);
        maxBBScore = std::max(BBScores[i], maxBBScore);
        maxExtremeScore = std::max(extremeScores[i], maxExtremeScore);
    }

    //Normalize scores
    for (size_t i = 0; i < candidateDirs.size(); i++) {
        normalScores[i] = normalScores[i] / maxNormalScore;
        BBScores[i] = BBScores[i]/ maxBBScore;
        extremeScores[i] = extremeScores[i]/ maxExtremeScore;
    }

    //Compute the best orientation
    double bestScore = -std::numeric_limits<double>::max();
    cg3::Vec3 bestOrientation;

    for (size_t i = 0; i < candidateDirs.size(); i++) {
        const cg3::Vec3& dir = candidateDirs[i];

        //Get the score
        double score =
                normalWeight * normalScores[i] +
                BBweight * BBScores[i] +
                extremeWeight * extremeScores[i];

        //Select the best orientation
        if (score >= bestScore) {
            bestOrientation = dir;
            bestScore = score;
        }
    }

    cg3::Vec3 rotationAxis;
    double angle;

    rotationAxis = bestOrientation.cross(xAxis);
    rotationAxis.normalize();
    double dirDot = bestOrientation.dot(xAxis);
    angle = acos(dirDot);
    assert(!std::isnan(angle));

    Eigen::Matrix3d rot = cg3::rotationMatrix(rotationAxis, angle);

    mesh.rotate(rot);
    smoothedMesh.rotate(rot);

    mesh.updateBoundingBox();
    smoothedMesh.updateBoundingBox();
    mesh.updateFacesAndVerticesNormals();
    smoothedMesh.updateFacesAndVerticesNormals();

}

///**
// * @brief Get optimal orientation on x-axis for four axis
// * fabrication. Both meshes are rotated in the same way.
// *
// * @param[out] mesh Original mesh
// * @param[out] smoothedMesh Smoothed mesh
// * @param[in] nDirections Number of directions to check
// * @param[in] deterministic Deterministic approach (if false it is randomized)
// */
//void rotateToOptimalOrientation(
//        cg3::EigenMesh& mesh,
//        cg3::EigenMesh& smoothedMesh,
//        const unsigned int nDirs,
//        const double weightPower,
//        const bool deterministic)
//{
//    cg3::Pointd bbCenter = smoothedMesh.boundingBox().center();

//    mesh.translate(-bbCenter);
//    smoothedMesh.translate(-bbCenter);

//    smoothedMesh.updateFaceNormals();

//    //Get the optimal rotation matrix
//    Eigen::Matrix3d rot = internal::optimalOrientationRotationMatrix(smoothedMesh, nDirs, weightPower, deterministic);

////OLD METHOD!
////    Eigen::Matrix3d rot = cg3::globalOptimalRotationMatrix(smoothedMesh, nOrientations, deterministic);

//    //Rotate meshes
//    mesh.rotate(rot);
//    smoothedMesh.rotate(rot);

//    smoothedMesh.updateBoundingBox();
//    cg3::BoundingBox b = smoothedMesh.boundingBox();

//    //If Y length is the greatest one then rotate by 90° around zAxis
//    if (b.lengthY() > b.lengthX() && b.lengthY() > b.lengthZ()){
//        cg3::Vec3 zAxis(0,0,1);
//        double angle = M_PI/2;

//        mesh.rotate(cg3::rotationMatrix(zAxis, angle));
//        smoothedMesh.rotate(cg3::rotationMatrix(zAxis, angle));
//    }
//    //If Z length is the greatest one then rotate by 90° around yAxis
//    else if (b.lengthZ() > b.lengthX() && b.lengthZ() > b.lengthY()){
//        cg3::Vec3 yAxis(0,1,0);
//        double angle = M_PI/2;

//        mesh.rotate(cg3::rotationMatrix(yAxis, angle));
//        smoothedMesh.rotate(cg3::rotationMatrix(yAxis, angle));
//    }

//    mesh.updateBoundingBox();
//    smoothedMesh.updateBoundingBox();
//    mesh.updateFacesAndVerticesNormals();
//    smoothedMesh.updateFacesAndVerticesNormals();
//}

namespace internal {

void principalComponentAnalysis(
        const cg3::EigenMesh& mesh,
        Eigen::Vector3d& eigenValues,
        Eigen::Matrix3d& eigenVectors)
{
    using namespace Eigen;

    const unsigned int nVertices = mesh.numberVertices();

    MatrixX3d mat(nVertices, 3);
    for(unsigned int vId = 0; vId < nVertices; vId++) {
        cg3::Point3d p = mesh.vertex(vId);

        mat(vId, 0) = p.x();
        mat(vId, 1) = p.y();
        mat(vId, 2) = p.z();
    }

    principalComponentAnalysis(mat, eigenValues, eigenVectors);
}

void principalComponentAnalysis(
        const Eigen::MatrixX3d& mat,
        Eigen::Vector3d& eigenValues,
        Eigen::Matrix3d& eigenVectors)
{
    using namespace Eigen;

    MatrixX3d centered = mat.rowwise() - mat.colwise().mean();
    Matrix3d cov = (centered.adjoint() * centered) / mat.rows();

    SelfAdjointEigenSolver<Matrix3d> eig(cov);

    eigenValues = eig.eigenvalues();
    eigenVectors = eig.eigenvectors();
}

void rotateToPrincipalComponents(
        cg3::EigenMesh& mesh,
        cg3::EigenMesh& smoothedMesh)
{
    using namespace Eigen;

    //Get eigenvalues and eigenvectors of the PCA
    Eigen::Vector3d eigenValues;
    Eigen::Matrix3d eigenVectors;
    internal::principalComponentAnalysis(smoothedMesh, eigenValues, eigenVectors);

    std::cout << "Principal components: " << std::endl;
    std::cout << "Eigenvalues: " << eigenValues.transpose() << std::endl;
    std::cout << "Eigenvectors: " << std::endl << eigenVectors << std::endl;
    std::cout << std::endl;

    //Swap vectors if not orthogonal on the proper side
    const Vector3d& eigenVec1 = eigenVectors.row(0);
    const Vector3d& eigenVec2 = eigenVectors.row(1);
    const Vector3d& eigenVec3 = eigenVectors.row(2);
    if (eigenVec1.cross(eigenVec2).dot(eigenVec3) <= 0)
        eigenVectors.row(2) = -eigenVectors.row(2);

    //Get transformation matrix
    Matrix3d rot;
    Matrix3d axisMatrix;
    axisMatrix <<
           1,0,0,
           0,1,0,
           0,0,1;
    rot = eigenVectors * axisMatrix;

    //Rotate meshes
    for(unsigned int vId = 0; vId < mesh.numberVertices(); vId++) {
        cg3::Point3d p = mesh.vertex(vId);

        Vector3d pEig;
        pEig << p.x(), p.y(), p.z();

        Vector3d newPEig = rot * pEig;
        cg3::Point3d newP(newPEig(0), newPEig(1), newPEig(2));

        mesh.setVertex(vId, newP);
    }
    for(unsigned int vId = 0; vId < smoothedMesh.numberVertices(); vId++) {
        cg3::Point3d p = smoothedMesh.vertex(vId);

        Vector3d pEig;
        pEig << p.x(), p.y(), p.z();

        Vector3d newPEig = rot * pEig;
        cg3::Point3d newP(newPEig(0), newPEig(1), newPEig(2));

        smoothedMesh.setVertex(vId, newP);
    }
}

///**
// * @brief Get optimal orientation on x-axis for four axis
// * fabrication.
// * @param[out] input Input mesh. The bounding box center of the mesh must be on the origin.
// * @param[in] nDirections Number of directions to check
// * @param[in] deterministic Deterministic approach (if false it is randomized)
// * @return Rotation matrix
// */
//Eigen::Matrix3d optimalOrientationRotationMatrix(
//        const cg3::SimpleEigenMesh& inputMesh,
//        const unsigned int nDirs,
//        const double weightPower,
//        const bool deterministic)
//{
//    //Get the direction pool (sphere coverage, fibonacci sampling)l
//    std::vector<cg3::Vec3> dirPool = cg3::sphereCoverage(nDirs, deterministic);

//    //Bounding box center of the mesh
//    cg3::Pointd bbCenter = inputMesh.boundingBox().center();

//    //Save vertices and get max distance from bounding box center
//    std::vector<cg3::Pointd> vertices(inputMesh.numberVertices());
//    double maxDistance = 0;
//    for(unsigned int vId = 0; vId < inputMesh.numberVertices(); vId++) {
//        const cg3::Pointd v = inputMesh.vertex(vId);

//        vertices[vId] = v;

//        //Compute max distance
//        double distance = (v - bbCenter).length();
//        if (distance >= maxDistance) {
//            maxDistance = distance;
//        }
//    }

//    //Compute face barycenter and normals
//    std::vector<cg3::Pointd> faceBarycenters(inputMesh.numberFaces());
//    std::vector<cg3::Pointd> faceNormals(inputMesh.numberFaces());
//    std::vector<double> faceAreas(inputMesh.numberFaces());
//    for(unsigned int fId = 0; fId < inputMesh.numberFaces(); fId++) {
//        cg3::Pointi f = inputMesh.face(fId);
//        const cg3::Pointd& v1 = vertices.at(f.x());
//        const cg3::Pointd& v2 = vertices.at(f.y());
//        const cg3::Pointd& v3 = vertices.at(f.z());

//        //Face barycenter
//        const cg3::Pointd faceBarycenter = (v1+v2+v3)/3;
//        faceBarycenters[fId] = faceBarycenter;

//        //Face normal
//        const cg3::Vec3 n = inputMesh.faceNormal(fId);
//        faceNormals[fId] = n;

//        //Face normal
//        const double a = inputMesh.faceArea(fId);
//        faceAreas[fId] = a;
//    }

//    //Compute the best orientation
//    double bestScore = std::numeric_limits<double>::max();
//    cg3::Vec3 bestOrientation;
//    for (cg3::Vec3& dir : dirPool) {
//        cg3::Vec3 axis;
//        double angle;
//        dir.normalize();

//        //Get rotation matrix of current direction
//        defineRotation(dir, axis, angle);
//        Eigen::Matrix3d mr = rotationMatrix(axis, angle);

//        //Compute current score
//        double score = 0.0;
//        for(unsigned int fId = 0; fId < inputMesh.numberFaces(); fId++) {
//            //Get normal and rotate it
//            cg3::Vec3 n = faceNormals.at(fId);
//            n.rotate(mr);

//            //Get barycenter and rotate it
//            cg3::Pointd b = faceBarycenters.at(fId);
//            b.rotate(mr);

//            //Get face area
//            double a = faceAreas.at(fId);

//            //Compute weight
//            double weight = pow((b - bbCenter).length() / maxDistance, weightPower);

//            //Add to score
//            score += a * weight * (std::fabs(n.x()) + std::fabs(n.y()) + std::fabs(n.z()));
//        }

//        //Select the best orientation
//        if (score <= bestScore) {
//            bestOrientation = dir;
//            bestScore = score;
//        }
//    }

//    cg3::Vec3 axis;
//    double angle;
//    bestOrientation.normalize();

//    defineRotation(bestOrientation, axis, angle);

//    return cg3::rotationMatrix(cg3::Vec3(axis), angle);
//}

//void defineRotation(const cg3::Vec3& zAxis,
//                    cg3::Vec3& rotationAxis,
//                    double& angle)
//{
//    const cg3::Vec3 Z(0,0,1);
//    rotationAxis = zAxis.cross(Z);
//    rotationAxis.normalize();
//    angle = acos(zAxis.dot(Z));
//    assert(!std::isnan(angle));
//}


}

}
