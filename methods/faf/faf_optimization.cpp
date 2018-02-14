/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_optimization.h"

#if defined(CG3_LIBIGL_DEFINED) && defined(MULTI_LABEL_OPTIMIZATION_INCLUDED)

#include <cg3/libigl/libigl.h>

#include "lib/MultiLabelOptimization/GCoptimization.h"

#else

#endif

//Uncomment if you want information on console
//#define OPTIMIZATION_VERBOSE


#define MAXCOST 10000000

namespace FourAxisFabrication {


#if defined(CG3_LIBIGL_DEFINED) && defined(MULTI_LABEL_OPTIMIZATION_INCLUDED)

namespace internal {
float smoothTerm(int site_1, int site_2, int label_1, int label_2, void *extra_data);
}

/* Get optimal association for each face */

/**
 * @brief Associate each face of the mesh to a direction
 * @param[in] Input mesh.
 * @param[out] data Four axis fabrication data
 */
void getOptimizedAssociation(
        const cg3::EigenMesh& mesh,
        Data& data)
{

    //Get fabrication data
    const std::vector<unsigned int>& targetDirections = data.targetDirections;
    const cg3::Array2D<int> &visibility = data.visibility;
    std::vector<int>& association = data.association;
    std::vector<unsigned int>& nonVisibleFaces = data.nonVisibleFaces;
    const unsigned int nFaces = mesh.getNumberFaces();

    //Get mesh adjacencies
    Eigen::MatrixXi adj = cg3::libigl::getFaceAdjacences(mesh);

    std::vector<bool> usedLabels(targetDirections.size(), false);

    try {
        GCoptimizationGeneralGraph *gc = new GCoptimizationGeneralGraph(nFaces, targetDirections.size());

#ifdef OPTIMIZATION_VERBOSE
        std::cout << "Survived directions: " << targetDirections.size() << "\n";
#endif

        for (unsigned int label = 0; label < targetDirections.size(); ++label) {
            int directionIndex = targetDirections[label];

            GCoptimizationGeneralGraph::SparseDataCost *gcData = new GCoptimizationGeneralGraph::SparseDataCost[nFaces];

            int dataCount = 0;
            for (unsigned int faceId = 0; faceId < nFaces; faceId++){
                //Record data
                GCoptimizationGeneralGraph::SparseDataCost record;
                record.site = faceId;

                if (association[faceId] < 0) {
                    //Visible
                    if (visibility(directionIndex, faceId) == 1){
                        record.cost = 0;
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
        }

        gc->setSmoothCost(internal::smoothTerm, nullptr);

        //Set adjacencies
        std::vector<bool> visited(nFaces, false);
        for (unsigned int f = 0; f < nFaces; f++) {
            visited[f] = true;
            for (int i=0; i<3; ++i) {
                int nid = adj(f, i);
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

namespace internal {

float smoothTerm(int site_1, int site_2, int label_1, int label_2, void *extra_data) {
    CG3_SUPPRESS_WARNING(site_1);
    CG3_SUPPRESS_WARNING(site_2);
    CG3_SUPPRESS_WARNING(extra_data);

    if (label_1 == label_2)
        return 0.f;
    else
        return 2.f;
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

}



}

