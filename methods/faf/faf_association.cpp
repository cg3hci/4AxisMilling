/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_association.h"

#if defined(CG3_LIBIGL_DEFINED) && defined(MULTI_LABEL_OPTIMIZATION_INCLUDED)

#include <cg3/libigl/mesh_adjacencies.h>

#include "lib/MultiLabelOptimization/GCoptimization.h"

#include "faf_charts.h"

#endif

//Uncomment if you want information on console
//#define OPTIMIZATION_VERBOSE

#define MAXCOST GCO_MAX_ENERGYTERM

namespace FourAxisFabrication {

namespace internal {

void setupDataCost(
        const cg3::EigenMesh& mesh,
        const double freeCostAngle,
        const double dataSigma,
        const Data& data,
        float* dataCost);

void setupSmoothCost(
        const double maxLabelAngle,
        const double compactness,
        const Data& data,
        float* smoothCost);

}


#if defined(CG3_LIBIGL_DEFINED) && defined(MULTI_LABEL_OPTIMIZATION_INCLUDED)


/* Get optimal association for each face */

/**
 * @brief Associate each face of the mesh to a direction
 * @param[in] Input mesh
 * @param[in] freeCostAngle Angles lower than the values, are not considered in data term
 * @param[in] dataSigma Sigma of the gaussian function for calculating data term
 * @param[in] maxLabelAngle Maximum angles between adjacent labels
 * @param[in] compactness Compactness
 * @param[out] data Four axis fabrication data
 */
void getAssociation(
        const cg3::EigenMesh& mesh,
        const double freeCostAngle,
        const double dataSigma,
        const double maxLabelAngle,
        const double compactness,
        const unsigned int optimizationIterations,
        const unsigned int minFaces,
        Data& data)
{
    //Get fabrication data
    const std::vector<unsigned int>& targetDirections = data.targetDirections;
    std::vector<int>& association = data.association;
    const unsigned int nFaces = mesh.getNumberFaces();
    const unsigned int nLabels = targetDirections.size();

    //Get mesh adjacencies
    std::vector<std::vector<int>> faceAdj = cg3::libigl::getFaceFaceAdjacencies(mesh);

    try {
        GCoptimizationGeneralGraph* gc = new GCoptimizationGeneralGraph(nFaces, nLabels);

#ifdef OPTIMIZATION_VERBOSE
        std::cout << "Survived directions: " << nLabels << "\n";
#endif

        //Creating cost data arrays
        float* dataCost = new float[nFaces * nLabels];
        float* smoothCost = new float[nLabels * nLabels];

        //Get the costs
        internal::setupDataCost(mesh, freeCostAngle, dataSigma, data, dataCost);
        internal::setupSmoothCost(maxLabelAngle, compactness, data, smoothCost);

        gc->setDataCost(dataCost);
        gc->setSmoothCost(smoothCost);

        //Set adjacencies
        std::vector<bool> visited(nFaces, false);
        for (unsigned int f = 0; f < nFaces; f++) {
            visited[f] = true;
            for (int i=0; i<3; ++i) {
                int nid = faceAdj[f][i];
                if (!visited[nid]) gc->setNeighbors(f, nid);
            }
        }

        //Compute graph cut
        gc->swap(-1); // -1 => run until convergence [convergence is guaranteed]

        //Set associations
        for (unsigned int faceId = 0; faceId < nFaces; faceId++){
            int associatedDirectionIndex = gc->whatLabel(faceId);

            association[faceId] = data.targetDirections[associatedDirectionIndex];
        }


        //Get chart data
        data.chartData = getChartData(mesh, data.association, faceAdj);

        //Optimization

        int iterations = optimizationIterations;
        bool done = false;

        while (!done && iterations > 0) {
            done = true;

            for (const Chart& chart : data.chartData.charts) {
                if (chart.adjacentLabels.size() == 1) {
                    int targetLabel = *(chart.adjacentLabels.begin());
                    if (targetLabel > 0) {
                        std::vector<unsigned int>::const_iterator targetLabelIt = std::find(targetDirections.begin(), targetDirections.end(), (unsigned int) targetLabel);
                        size_t targetLabelIndex = std::distance(targetDirections.begin(), targetLabelIt);

                        for (int faceId : chart.faces) {
                            if (dataCost[faceId * nLabels + targetLabelIndex] < MAXCOST) {
                                dataCost[faceId * nLabels + targetLabelIndex] /= 2;
                            }
                        }

                        done = false;
                    }
                }
            }

            if (!done) {
                //Compute graph cut
                gc->setDataCost(dataCost);

                gc->swap(-1); // -1 => run until convergence [convergence is guaranteed]

                //Set associations
                for (unsigned int f = 0; f < nFaces; f++){
                    int associatedDirectionIndex = gc->whatLabel(f);

                    association[f] = data.targetDirections[associatedDirectionIndex];
                }
            }

            iterations--;

            //Get chart data
            data.chartData = getChartData(mesh, data.association, faceAdj);
        }



        delete gc;
        delete dataCost;
        delete smoothCost;

        //The remaining directions are the new target directions
        std::vector<bool> usedLabels(nLabels, false);
        std::vector<unsigned int> newTargetDirections;
        for (unsigned int f = 0; f < nFaces; f++){
            usedLabels[association[f]] = true;
        }
        for (unsigned int i = 0; i < nLabels; ++i) {
            if (usedLabels[i]) {
                newTargetDirections.push_back(targetDirections[i]);
            }
        }
        data.targetDirections = newTargetDirections;

//        //We associate non visible faces to the -1 index
//        for (unsigned int face : nonVisibleFaces) {
//            association[face] = -1;
//        }
    }
    catch (GCException e) {
        std::cerr << "\n\n!!!GRAPH-CUT EXCEPTION!!!\nCheck logfile\n\n" << std::endl;
        e.Report();
    }
}

#else
/* Get optimal association for each face */

/**
 * @brief Associate each face of the mesh to a direction
 * @param[in] Input mesh.
 * @param[out] data Four axis fabrication data
 */
void getOptimizedAssociation(
        cg3::EigenMesh& mesh,
        Data& data)
{
    //TODO TO BE IMPLEMENTED IN BRUTE FORCE
    exit(1);
}
#endif



namespace internal {

/**
 * @brief Setup data cost
 * @param[in] Input mesh
 * @param[in] freeCostAngle Angles lower than the values, are not considered in data term
 * @param[in] dataSigma Sigma of the gaussian function for calculating data term
 * @param[in] data Four axis fabrication data
 * @param[out] dataCost Data cost output
 */
void setupDataCost(
        const cg3::EigenMesh& mesh,
        const double freeCostAngle,
        const double dataSigma,
        const Data& data,
        float* dataCost)
{
    const std::vector<cg3::Vec3>& directions = data.directions;
    const std::vector<unsigned int>& targetDirections = data.targetDirections;
    const cg3::Array2D<int>& visibility = data.visibility;
    const std::vector<int>& association = data.association;
    const unsigned int nFaces = mesh.getNumberFaces();
    const unsigned int nLabels = targetDirections.size();


    for (unsigned int faceId = 0; faceId < nFaces; faceId++){
        for (unsigned int label = 0; label < nLabels; ++label) {
            const int& directionIndex = targetDirections[label];
            const cg3::Vec3& labelNormal = directions[directionIndex];

            double cost;

            if (association[faceId] < 0) {
                //Visible
                if (visibility(directionIndex, faceId) == 1) {
                    const cg3::Vec3 faceNormal = mesh.getFaceNormal(faceId);

                    double dot = faceNormal.dot(labelNormal);
                    double angle = acos(dot);

                    //If the angle is greater than the limit angle
                    if (angle > freeCostAngle) {
                        double normalizedAngle = (angle - freeCostAngle)/(M_PI/2 - freeCostAngle);
                        cost = pow(normalizedAngle, dataSigma);
                    }
                    else {
                        cost = 0;
                    }
                }
                //Not visibile
                else {
                    cost = MAXCOST;
                }
            }
            else {
                //Already associated with that label
                if (association[faceId] == directionIndex) {
                    cost = 0;
                }
                //Already associated with another label
                else {
                    cost = MAXCOST;
                }

            }

            dataCost[faceId * nLabels + label] = cost;
        }
    }
}

/**
 * @brief Setup data cost
 * @param[in] Input mesh
 * @param[in] maxLabelAngle Maximum angles between adjacent labels
 * @param[in] compactness Compactness
 * @param[in] data Four axis fabrication data
 * @param[out] smoothCost Smooth cost output
 */
void setupSmoothCost(
        const double maxLabelAngle,
        const double compactness,
        const Data& data,
        float* smoothCost)
{
    const std::vector<cg3::Vec3>& directions = data.directions;
    const std::vector<unsigned int>& targetDirections = data.targetDirections;
    const unsigned int nLabels = targetDirections.size();

    for (unsigned int l1 = 0; l1 < nLabels; ++l1) {
        const int& l1DirIndex = targetDirections[l1];
        const cg3::Vec3& l1Normal = directions[l1DirIndex];

        for (unsigned int l2 = 0; l2 < nLabels; ++l2) {
            double cost;

            if (l1 == l2) {
                cost = 0.f;
            }
            else {
                const int& l2DirIndex = targetDirections[l2];
                const cg3::Vec3& l2Normal = directions[l2DirIndex];

                double dot = l1Normal.dot(l2Normal);

                if (acos(dot) > maxLabelAngle) {
                    cost = MAXCOST;
                }
                else {
                    cost = compactness;
                }
            }

            smoothCost[l1 * nLabels + l2] = cost;
        }
    }
}

}


} //namespace cg3

