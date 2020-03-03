/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_association.h"

#if defined(CG3_LIBIGL_DEFINED) && defined(MULTI_LABEL_OPTIMIZATION_INCLUDED)

#include "lib/MultiLabelOptimization/GCoptimization.h"

#include "faf_charts.h"

#include <cg3/libigl/mesh_adjacencies.h>

#include <unordered_set>

#define MAXCOST GCO_MAX_ENERGYTERM

namespace FourAxisFabrication {

namespace internal {

struct SmoothData {
    const cg3::EigenMesh& mesh;
    double smoothSigma;
    double compactness;
};

void setupDataCost(
        const cg3::EigenMesh& mesh,
        const std::vector<unsigned int> targetLabels,
        const double dataSigma,
        const bool fixExtremes,
        const Data& data,
        std::vector<float>& dataCost);

//void setupSmoothCost(
//        const std::vector<unsigned int> targetLabels,
//        const double compactness,
//        std::vector<float>& smoothCost);


float getSmoothTerm(
        int f1, int f2,
        int l1, int l2,
        void *extra_data);

}

/* Get optimal association for each face */

/**
 * @brief Associate each face of the mesh to a direction using a graph-cut algorithm
 * @param[in] Input mesh
 * @param[in] dataSigma Sigma of the gaussian function for calculating data term
 * @param[in] compactness Compactness
 * @param[in] fixExtremes Fix extremes on the given directions
 * @param[out] data Four axis fabrication data
 */
void getAssociation(
        const cg3::EigenMesh& mesh,
        const double dataSigma,
        const double smoothSigma,
        const double compactness,
        const bool fixExtremes,
        Data& data)
{
    //Get fabrication data
    const std::vector<cg3::Vec3d>& directions = data.directions;
    const std::vector<unsigned int>& nonVisibleFaces = data.nonVisibleFaces;
    std::vector<unsigned int>& targetDirections = data.targetDirections;
    std::vector<int>& association = data.association;
    std::vector<unsigned int>& associationNonVisibleFaces = data.associationNonVisibleFaces;
    std::vector<unsigned int>& minExtremes = data.minExtremes;
    std::vector<unsigned int>& maxExtremes = data.maxExtremes;

    //Setting target directions
    std::vector<unsigned int> targetLabels(directions.size());
    for (size_t i = 0; i < targetLabels.size(); i++)
        targetLabels[i] = i;

    //Initialize to -1 direction association for each face
    association.clear();
    association.resize(mesh.numberFaces());
    std::fill(association.begin(), association.end(), -1);

    const unsigned int nFaces = mesh.numberFaces();
    const unsigned int nLabels = targetLabels.size();

    //Get mesh adjacencies
    std::vector<std::vector<int>> ffAdj = cg3::libigl::faceToFaceAdjacencies(mesh);

    //Creating cost data arrays
    std::vector<float> dataCost(nFaces * nLabels);
    //Get the costs
    internal::setupDataCost(mesh, targetLabels, dataSigma, fixExtremes, data, dataCost);

    //Fixed compactness cost
//    std::vector<float> smoothCost(nLabels * nLabels);
//    internal::setupSmoothCost(targetLabels, compactness, smoothCost);

    try {
        GCoptimizationGeneralGraph* gc = new GCoptimizationGeneralGraph(nFaces, nLabels);

        gc->setDataCost(dataCost.data());
        //Set smooth cost
        internal::SmoothData smoothData = {mesh, smoothSigma, compactness};
        gc->setSmoothCost(internal::getSmoothTerm, (void*) &smoothData);

        //Set adjacencies
        std::vector<bool> visited(nFaces, false);
        for (unsigned int f = 0; f < nFaces; f++) {
            visited[f] = true;
            for (int i = 0; i < 3; ++i) {
                int nid = ffAdj[f][i];
                if (!visited[nid])
                    gc->setNeighbors(f, nid);
            }
        }

        //Compute graph cut
        gc->swap(-1); // -1 => run until convergence [convergence is guaranteed]

        //Set associations
        for (unsigned int fId = 0; fId < nFaces; fId++){
            int associatedDirectionIndex = gc->whatLabel(fId);

            association[fId] = targetLabels[associatedDirectionIndex];
        }

        //Delete data
        delete gc;

        //Set non-visible faces for association
        associationNonVisibleFaces = nonVisibleFaces;

        //The remaining directions are the new target directions
        std::vector<bool> usedDirections(directions.size(), false);
        for (unsigned int fId = 0; fId < nFaces; fId++){
            usedDirections[association[fId]] = true;
        }
        //Set target directions
        for (unsigned int lId = 0; lId < directions.size(); ++lId) {
            if (usedDirections[lId]) {
                targetDirections.push_back(lId);
            }
        }

        int minLabel = targetDirections[targetDirections.size()-2];
        int maxLabel = targetDirections[targetDirections.size()-1];
        minExtremes.clear();
        maxExtremes.clear();
        for (size_t fId = 0; fId < nFaces; fId++) {
            if (association[fId] == minLabel)
                minExtremes.push_back(fId);
            else if (association[fId] == maxLabel)
                maxExtremes.push_back(fId);
        }
    }
    catch (GCException e) {
        std::cerr << "\n\n!!!GRAPH-CUT EXCEPTION!!!\nCheck logfile\n\n" << std::endl;
        e.Report();
    }
}

namespace internal {

/**
 * @brief Setup data cost
 * @param[in] Input mesh
 * @param[in] targetLabel Target labels
 * @param[in] dataSigma Sigma of the gaussian function for calculating data term
 * @param[in] fixExtremes Fix extremes on the given directions
 * @param[in] data Four axis fabrication data
 * @param[out] dataCost Data cost output
 */
void setupDataCost(
        const cg3::EigenMesh& mesh,
        const std::vector<unsigned int> targetLabels,
        const double dataSigma,
        const bool fixExtremes,
        const Data& data,
        std::vector<float>& dataCost)
{
    const std::vector<cg3::Vec3d>& directions = data.directions;
    const cg3::Array2D<int>& visibility = data.visibility;
    const std::vector<unsigned int>& minExtremes = data.minExtremes;
    const std::vector<unsigned int>& maxExtremes = data.maxExtremes;

    const unsigned int nFaces = mesh.numberFaces();
    const unsigned int nLabels = targetLabels.size();

//    #pragma omp parallel for
//    for (unsigned int faceId = 0; faceId < nFaces; faceId++){
//        for (unsigned int label = 0; label < nLabels; ++label) {
//            const unsigned int& directionIndex = targetLabels[label];
//            const cg3::Vec3& labelNormal = directions[directionIndex];

//            double cost;

//            //Visible
//            if (visibility(directionIndex, faceId) == 1) {
//                const cg3::Vec3 faceNormal = mesh.faceNormal(faceId);

//                double dot = faceNormal.dot(labelNormal);

//                //If the angle is greater than the limit angle
//                if (dot < freeCostDot) {
//                    double normalizedDot = (freeCostDot - dot)/freeCostDot;
//                    cost = pow(normalizedDot, dataSigma);
//                }
//                else {
//                    cost = 0;
//                }
//            }
//            //Not visibile
//            else {
//                cost = MAXCOST;
//            }

//            dataCost[faceId * nLabels + label] = cost;
//        }
//    }

//    #pragma omp parallel for
//    for (unsigned int faceId = 0; faceId < nFaces; faceId++){
//        for (unsigned int label = 0; label < nLabels; ++label) {
//            const unsigned int& directionIndex = targetLabels[label];
//            const cg3::Vec3& labelNormal = directions[directionIndex];

//            double cost;

//            //Visible
//            if (visibility(directionIndex, faceId) == 1) {
//                const cg3::Vec3 faceNormal = mesh.faceNormal(faceId);

//                double dot = faceNormal.dot(labelNormal);

//                cost = 1.f - exp(-0.5 * pow((dot - 1.f)/dataSigma, 2.f));
//            }
//            //Not visibile
//            else {
//                cost = MAXCOST;
//            }

//            dataCost[faceId * nLabels + label] = cost;
//        }
//    }

    #pragma omp parallel for
    for (unsigned int faceId = 0; faceId < nFaces; faceId++){
        for (unsigned int label = 0; label < nLabels; ++label) {
            const unsigned int& directionIndex = targetLabels[label];
            const cg3::Vec3d& labelNormal = directions[directionIndex];

            double cost;

            const cg3::Vec3d faceNormal = mesh.faceNormal(faceId);
            double dot = faceNormal.dot(labelNormal);

            //Visible
            if (visibility(directionIndex, faceId) == 1) {
                cost = pow(1.f - dot, dataSigma);
            }
            //Not visibile
            else {
                cost = MAXCOST;
            }

            dataCost[faceId * nLabels + label] = cost;
        }
    }

    //Fix extremes on the +x and -x
    if (fixExtremes) {
        #pragma omp parallel for
        for (unsigned int label = 0; label < nLabels-2; ++label) {
            #pragma omp parallel for
            for (size_t i = 0; i < minExtremes.size(); i++) {
                unsigned int faceId = minExtremes[i];
                dataCost[faceId * nLabels + label] = MAXCOST;
            }

            #pragma omp parallel for
            for (size_t i = 0; i < maxExtremes.size(); i++){
                unsigned int faceId = maxExtremes[i];
                dataCost[faceId * nLabels + label] = MAXCOST;
            }
        }
    }
}

///**
// * @brief Setup smooth cost
// * @param[in] targetLabel Target labels
// * @param[in] compactness Compactness
// * @param[out] smoothCost Smooth cost output
// */
//void setupSmoothCost(
//        const std::vector<unsigned int> targetLabels,
//        const double compactness,
//        std::vector<float>& smoothCost)
//{
//    const unsigned int nLabels = targetLabels.size();

//    #pragma omp parallel for
//    for (unsigned int l1 = 0; l1 < nLabels; ++l1) {

//        #pragma omp parallel for
//        for (unsigned int l2 = 0; l2 < nLabels; ++l2) {
//            double cost;

//            if (l1 == l2) {
//                cost = 0.f;
//            }
//            else {
//                cost = compactness;
//            }

//            smoothCost[l1 * nLabels + l2] = cost;
//        }
//    }
//}

float getSmoothTerm(
        int f1, int f2,
        int l1, int l2,
        void *extra_data)
{
    SmoothData* smoothData = (SmoothData*) extra_data;
    const cg3::EigenMesh& mesh = smoothData->mesh;
    float smoothSigma = smoothData->smoothSigma;
    float compactness = smoothData->compactness;

    if (l1 == l2)
        return 0.f;

    cg3::Vec3d faceNormal1 = mesh.faceNormal(f1);
    cg3::Vec3d faceNormal2 = mesh.faceNormal(f2);

    float dot = faceNormal1.dot(faceNormal2);

    if (dot < 0) {
        return 0.0f;
    }
    return compactness;


//    if (l1 == l2)
//        return 0.0;

//    cg3::Pointi face1 = mesh.face(f1);
//    cg3::Pointi face2 = mesh.face(f2);

//    //TODO FARLO PIU' EFFICIENTE
//    std::set<int> vertices1;
//    vertices1.insert(face1.x());
//    vertices1.insert(face1.y());
//    vertices1.insert(face1.z());
//    std::set<int> vertices2;
//    vertices2.insert(face2.x());
//    vertices2.insert(face2.y());
//    vertices2.insert(face2.z());
//    std::set<int> intersect;
//    std::set_intersection(vertices1.begin(), vertices1.end(), vertices2.begin(), vertices2.end(), std::inserter(intersect,intersect.begin()));

//    if (intersect.size() != 2) {
//        std::cout << "Error intersection" << std::endl;
//        exit(-5);
//    }

//    std::set<int>::iterator it = intersect.begin();
//    cg3::Pointd v1 = mesh.vertex(*it);
//    it++;
//    cg3::Pointd v2 = mesh.vertex(*it);

//    cg3::Vec3 dir = v1 - v2;
//    dir.normalize();

//    float directionCost = static_cast<float>(1 - (dir.x() * dir.x()));

//    float smoothTerm = compactness + (directionCost * smoothSigma);
}

}

} //namespace cg3

#endif
