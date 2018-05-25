/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_optimization.h"

#if defined(CG3_LIBIGL_DEFINED) && defined(MULTI_LABEL_OPTIMIZATION_INCLUDED)

#include <cg3/libigl/mesh_adjacencies.h>

#include "lib/MultiLabelOptimization/GCoptimization.h"

#endif

//Uncomment if you want information on console
//#define OPTIMIZATION_VERBOSE


#define MAXCOST GCO_MAX_ENERGYTERM

namespace FourAxisFabrication {


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
void getOptimizedAssociation(
        const cg3::EigenMesh& mesh,
        const double freeCostAngle,
        const double dataSigma,
        const double maxLabelAngle,
        const double compactness,
        Data& data)
{
    //Get fabrication data
    const std::vector<unsigned int>& targetDirections = data.targetDirections;
    const cg3::Array2D<int> &visibility = data.visibility;
    std::vector<int>& association = data.association;
    std::vector<unsigned int>& nonVisibleFaces = data.nonVisibleFaces;
    const unsigned int nFaces = mesh.getNumberFaces();

    //Get mesh adjacencies
    std::vector<std::vector<int>> faceAdj = cg3::libigl::getFaceFaceAdjacencies(mesh);

    std::vector<bool> usedLabels(targetDirections.size(), false);

    try {
        GCoptimizationGeneralGraph *gc = new GCoptimizationGeneralGraph(nFaces, targetDirections.size());

#ifdef OPTIMIZATION_VERBOSE
        std::cout << "Survived directions: " << targetDirections.size() << "\n";
#endif

        for (unsigned int label = 0; label < targetDirections.size(); ++label) {            
            int directionIndex = targetDirections[label];

            const cg3::Vec3& labelNormal = data.directions[directionIndex];

            GCoptimizationGeneralGraph::SparseDataCost *gcData = new GCoptimizationGeneralGraph::SparseDataCost[nFaces];

            int dataCount = 0;
            for (unsigned int faceId = 0; faceId < nFaces; faceId++){
                //Record data
                GCoptimizationGeneralGraph::SparseDataCost record;
                record.site = faceId;

                if (association[faceId] < 0) {
                    //Visible
                    if (visibility(directionIndex, faceId) == 1) {
                        const cg3::Vec3 faceNormal = mesh.getFaceNormal(faceId);

                        double dot = faceNormal.dot(labelNormal);
                        double angle = acos(dot);

                        //If the angle is greater than the limit angle
                        if (angle > freeCostAngle) {
                            double normalizedAngle = (angle - freeCostAngle)/(M_PI/2 - freeCostAngle);
                            record.cost = pow(normalizedAngle, dataSigma);
                        }
                        else {
                            record.cost = 0;
                        }
                    }
                    //Not visibile
                    else {
                        record.cost = MAXCOST;
                    }
                }
                else {
                    //Already associated with that label
                    if (association[faceId] == directionIndex) {                        
                        record.cost = 0;
                    }
                    //Already associated with another label
                    else {
                        record.cost = MAXCOST;
                    }

                }

                gcData[dataCount] = record;
                dataCount++;
            }

            //Set data cost for the label
            gc->setDataCost(label, gcData, dataCount);
            delete [] gcData;


            for (unsigned int l = 0; l < targetDirections.size(); ++l) {
                double cost;

                if (l == label) {
                    cost = 0.f;
                }
                else {
                    cost = compactness;
                    const cg3::Vec3& lNormal = data.directions[targetDirections[l]];

                    double dot = labelNormal.dot(lNormal);

                    if (acos(dot) > maxLabelAngle) {
                        cost = MAXCOST;
                    }
                    else {
                        cost = compactness;
                    }
                }

                gc->setSmoothCost(label, l, cost);
            }
        }

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
        for (unsigned int f = 0; f < nFaces; f++){
            int associatedDirectionIndex = gc->whatLabel(f);

            association[f] = data.targetDirections[associatedDirectionIndex];

            usedLabels[associatedDirectionIndex] = true;
        }

        delete gc;
    }
    catch (GCException e) {
        std::cerr << "\n\n!!!GRAPH-CUT EXCEPTION!!!\nCheck logfile\n\n" << std::endl;
        e.Report();
    }

    //The remaining directions are the new target directions
    std::vector<unsigned int> newTargetDirections;
    for (unsigned int i = 0; i < targetDirections.size(); ++i) {
        if (usedLabels[i]) {
            newTargetDirections.push_back(targetDirections[i]);
        }
    }
    data.targetDirections = newTargetDirections;

    //We associate non visible faces to the -1 index
    for (unsigned int face : nonVisibleFaces) {
        association[face] = -1;
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

} //namespace cg3

