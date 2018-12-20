/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_association.h"

#if defined(CG3_LIBIGL_DEFINED) && defined(MULTI_LABEL_OPTIMIZATION_INCLUDED)

#include "lib/MultiLabelOptimization/GCoptimization.h"

#include "faf_charts.h"

#include <cg3/meshes/dcel/dcel.h>

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
        const double freeCostAngle,
        const double dataSigma,
        const bool fixExtremes,
        const Data& data,
        std::vector<float>& dataCost);

void setupSmoothCost(
        const std::vector<unsigned int> targetLabels,
        const double compactness,
        std::vector<float>& smoothCost);


float getSmoothTerm(
        int f1, int f2,
        int l1, int l2,
        void *extra_data);

}

/* Get optimal association for each face */

/**
 * @brief Associate each face of the mesh to a direction using a graph-cut algorithm
 * @param[in] Input mesh
 * @param[in] freeCostAngle Angles lower than the values, are not considered in data term
 * @param[in] dataSigma Sigma of the gaussian function for calculating data term
 * @param[in] compactness Compactness
 * @param[in] fixExtremes Fix extremes on the given directions
 * @param[out] data Four axis fabrication data
 */
void getAssociation(
        const cg3::EigenMesh& mesh,
        const double freeCostAngle,
        const double dataSigma,
        const double smoothSigma,
        const double compactness,
        const bool fixExtremes,
        Data& data)
{
    //Get fabrication data
    const std::vector<cg3::Vec3>& directions = data.directions;
    const std::vector<unsigned int>& nonVisibleFaces = data.nonVisibleFaces;
    std::vector<unsigned int>& targetDirections = data.targetDirections;
    std::vector<int>& association = data.association;
    std::vector<unsigned int>& associationNonVisibleFaces = data.associationNonVisibleFaces;

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
    std::vector<float> smoothCost(nLabels * nLabels);

    //Get the costs
    internal::setupDataCost(mesh, targetLabels, freeCostAngle, dataSigma, fixExtremes, data, dataCost);
    //Fixed compactness cost
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
    }
    catch (GCException e) {
        std::cerr << "\n\n!!!GRAPH-CUT EXCEPTION!!!\nCheck logfile\n\n" << std::endl;
        e.Report();
    }
}

/**
 * @brief Try to optimize the association, deleting holes and little charts.
 * @param[in] Input mesh
 * @param[in] relaxHoles Relax cost in hole-charts, assigning surrounding chart label
 * if a face is visible
 * @param[in] relaxHoles Lose details on holes
 * @param[in] minChartArea Min area for which a chart can be deleted
 * @param[out] data Four axis fabrication data
 */
void optimization(
        const cg3::EigenMesh& mesh,
        const bool relaxHoles,
        const bool loseHoles,
        const double minChartArea,
        Data& data)
{
    //Get fabrication data    
    const std::vector<unsigned int>& minExtremes = data.minExtremes;
    const std::vector<unsigned int>& maxExtremes = data.maxExtremes;
    const std::vector<cg3::Vec3>& directions = data.directions;
    const cg3::Array2D<int>& visibility = data.visibility;
    std::vector<unsigned int>& targetDirections = data.targetDirections;
    std::vector<int>& association = data.association;
    std::vector<unsigned int>& associationNonVisibleFaces = data.associationNonVisibleFaces;

    const unsigned int nFaces = mesh.numberFaces();

    int minLabel = targetDirections[targetDirections.size()-1];
    int maxLabel = targetDirections[targetDirections.size()-2];

    //Get mesh adjacencies
    std::vector<std::vector<int>> ffAdj = cg3::libigl::faceToFaceAdjacencies(mesh);

    //Get chart data
    ChartData chartData = getChartData(mesh, association, minExtremes, maxExtremes);

    //Relaxing data cost for holes
    if (relaxHoles) {
        unsigned int facesAffected = 0;
        unsigned int chartAffected = 0;
        for (const Chart& surroundingChart : chartData.charts) {
            const int surroundingChartLabel = surroundingChart.label;

            for (unsigned int holeChartId : surroundingChart.holeCharts) {
                const Chart& holeChart = chartData.charts.at(holeChartId);

                if (!chartData.isExtreme.at(holeChart.id)) {
                    std::set<unsigned int> remainingHoleChartFaces(holeChart.faces.begin(), holeChart.faces.end());

                    std::vector<unsigned int> facesToBeRelaxed;
                    do {
                        facesToBeRelaxed.clear();

                        for (const unsigned int fId : remainingHoleChartFaces) {
                            bool isOnBorder = false;
                            for (const int adjF : ffAdj[fId]) {
                                if (association[adjF] == surroundingChartLabel) {
                                    isOnBorder = true;
                                }
                            }

                            if (isOnBorder && visibility(surroundingChartLabel, fId) > 0)
                                facesToBeRelaxed.push_back(fId);
                        }

                        for (const unsigned int fId : facesToBeRelaxed) {
                            association[fId] = surroundingChartLabel;
                            facesAffected++;

                            remainingHoleChartFaces.erase(fId);
                        }
                    }
                    while (!facesToBeRelaxed.empty());

                    chartAffected++;
                }
            }
        }

        if (chartAffected > 0) {
            //Get new chart data
            chartData = getChartData(mesh, association, minExtremes, maxExtremes);
        }

        std::cout << "Relaxed constraints for " << chartAffected << " hole charts. Faces affected: " << facesAffected << std::endl;
    }


    //Deleting small charts
    if (minChartArea > std::numeric_limits<double>::epsilon()) {
        //Mesh area
        double meshArea = 0;
        for (unsigned int fId = 0; fId < nFaces; fId++)
            meshArea += mesh.faceArea(fId);

        double limitArea = meshArea * minChartArea;


        unsigned int facesAffected = 0;
        unsigned int facesNoLongerVisible = 0;
        unsigned int chartAffected = 0;

        //Smallest chart
        int smallestChartId;

        do {
            smallestChartId = -1;
            double smallestArea = meshArea; //Smallest chart area

            //For each chart
            for (size_t cId = 0; cId < chartData.charts.size(); cId++) {
                const Chart& chart = chartData.charts.at(cId);

                if (!chartData.isExtreme.at(chart.id)) {
                    //Chart area
                    double chartArea = 0;
                    for (unsigned int fId : chart.faces)
                        chartArea += mesh.faceArea(fId);

                    //Get the smallest chart which has area less than the limit area
                    if (chartArea <= limitArea && chartArea <= smallestArea) {
                        smallestChartId = chart.id;
                        smallestArea = chartArea;
                    }
                }
            }

            //If a target chart exists
            if (smallestChartId >= 0) {
                const Chart& chart = chartData.charts.at(smallestChartId);

                int chartLabel = chart.label;

                //Border faces to be computed
                std::vector<unsigned int> borderFaces = chart.borderFaces;

                std::unordered_set<unsigned int> visitedFaces;
                std::queue<unsigned int> queue;

                //Add each face in the border in the queue
                for (unsigned int fId : borderFaces) {
                    queue.push(fId);
                }

                while (!queue.empty()) {
                    unsigned int fId = queue.front();
                    queue.pop();

                    visitedFaces.insert(fId);

                    cg3::Vec3 normal = mesh.faceNormal(fId);
                    const std::vector<int>& adjacentFaces = ffAdj.at(fId);

                    //The best label for the face is one among the adjacent
                    //which has the less dot product with the normal
                    double maxDot = -1;
                    int bestLabel = -1;

                    for (const unsigned int adjId : adjacentFaces) {
                        int adjLabel = association[adjId];

                        if (chartLabel == adjLabel && chartData.faceChartMap.at(adjId) == chart.id) {
                            //Add the not visited faces in the border to the queue
                            if (visitedFaces.find(adjId) == visitedFaces.end())
                                queue.push(adjId);
                        }
                        else {
                            double dot = normal.dot(directions[adjLabel]);

                            if (dot >= maxDot) {
                                maxDot = dot;
                                bestLabel = adjLabel;
                            }
                        }
                    }

                    association[fId] = bestLabel;                    

                    if (visibility(bestLabel, fId) == 0) {
                        facesNoLongerVisible++;
                    }

                    facesAffected++;
                }

                chartAffected++;


                //Get new chart data
                chartData = getChartData(mesh, association, minExtremes, maxExtremes);
            }
        } while (smallestChartId >= 0);



        std::cout << "Small chart details lost for " << chartAffected << " charts. Faces affected: " << facesAffected << ". Faces no longer visible: " << facesNoLongerVisible << std::endl;
    }


    //Lose details on holes
    if (loseHoles) {
        unsigned int facesAffected = 0;
        unsigned int facesNoLongerVisible = 0;
        unsigned int chartAffected = 0;
        for (const Chart& surroundingChart : chartData.charts) {
            const int surroundingChartLabel = surroundingChart.label;

            for (unsigned int holeChartId : surroundingChart.holeCharts) {
                const Chart& holeChart = chartData.charts.at(holeChartId);

                if (!chartData.isExtreme.at(holeChart.id)) {
                    for (const int fId : holeChart.faces) {
                        association[fId] = surroundingChartLabel;
                        facesAffected++;

                        if (visibility(surroundingChartLabel, fId) == 0) {
                            facesNoLongerVisible++;
                        }
                    }
                }

                chartAffected++;
            }
        }

        if (chartAffected > 0) {
            //Get new chart data
            chartData = getChartData(mesh, association, minExtremes, maxExtremes);
        }

        std::cout << "Lost details for " << chartAffected << " hole charts. Faces affected: " << facesAffected << ". Faces no longer visible: " << facesNoLongerVisible << std::endl;
    }


    //Update association non-visible faces
    associationNonVisibleFaces.clear();
    for (unsigned int fId = 0; fId < nFaces; fId++){
        if (visibility(association[fId], fId) == 0) {
            associationNonVisibleFaces.push_back(fId);
        }
    }

    //The remaining directions are the new target directions
    std::vector<bool> usedDirections(directions.size(), false);
    std::vector<unsigned int> newTargetDirections;
    for (unsigned int fId = 0; fId < nFaces; fId++){
        usedDirections[association[fId]] = true;
    }
    //Set target directions
    for (unsigned int lId = 0; lId < directions.size(); ++lId) {
        if (usedDirections[lId]) {
            newTargetDirections.push_back(lId);
        }
    }

    targetDirections = newTargetDirections;
}

namespace internal {

/**
 * @brief Setup data cost
 * @param[in] Input mesh
 * @param[in] targetLabel Target labels
 * @param[in] freeCostAngle Angles lower than the values, are not considered in data term
 * @param[in] dataSigma Sigma of the gaussian function for calculating data term
 * @param[in] fixExtremes Fix extremes on the given directions
 * @param[in] data Four axis fabrication data
 * @param[out] dataCost Data cost output
 */
void setupDataCost(
        const cg3::EigenMesh& mesh,
        const std::vector<unsigned int> targetLabels,
        const double freeCostAngle,
        const double dataSigma,
        const bool fixExtremes,
        const Data& data,
        std::vector<float>& dataCost)
{
    const std::vector<cg3::Vec3>& directions = data.directions;
    const cg3::Array2D<int>& visibility = data.visibility;
    const std::vector<unsigned int>& minExtremes = data.minExtremes;
    const std::vector<unsigned int>& maxExtremes = data.maxExtremes;

    const unsigned int nFaces = mesh.numberFaces();
    const unsigned int nLabels = targetLabels.size();

    const double freeCostDot = cos(freeCostAngle);

    #pragma omp parallel for
    for (unsigned int faceId = 0; faceId < nFaces; faceId++){
        for (unsigned int label = 0; label < nLabels; ++label) {
            const unsigned int& directionIndex = targetLabels[label];
            const cg3::Vec3& labelNormal = directions[directionIndex];

            double cost;

            //Visible
            if (visibility(directionIndex, faceId) == 1) {
                const cg3::Vec3 faceNormal = mesh.faceNormal(faceId);

                double dot = faceNormal.dot(labelNormal);

                //If the angle is greater than the limit angle
                if (dot < freeCostDot) {
                    double normalizedDot = (dot - freeCostDot)/freeCostDot;
                    cost = pow(normalizedDot, dataSigma);
                }
                else {
                    cost = 0;
                }
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

/**
 * @brief Setup data cost
 * @param[in] targetLabel Target labels
 * @param[in] compactness Compactness
 * @param[out] smoothCost Smooth cost output
 */
void setupSmoothCost(
        const std::vector<unsigned int> targetLabels,
        const double compactness,
        std::vector<float>& smoothCost)
{
    const unsigned int nLabels = targetLabels.size();

    #pragma omp parallel for
    for (unsigned int l1 = 0; l1 < nLabels; ++l1) {

        #pragma omp parallel for
        for (unsigned int l2 = 0; l2 < nLabels; ++l2) {
            double cost;

            if (l1 == l2) {
                cost = 0.f;
            }
            else {
                cost = compactness;
            }

            smoothCost[l1 * nLabels + l2] = cost;
        }
    }
}

float getSmoothTerm(
        int f1, int f2,
        int l1, int l2,
        void *extra_data)
{
    SmoothData* smoothData = (SmoothData*) extra_data;
    const cg3::EigenMesh& mesh = smoothData->mesh;
    float smoothSigma = smoothData->smoothSigma;
    float compactness = smoothData->compactness;

    //    float smoothSigma = 0.2;
    //    float compactness = 4;
    //    float epsilon = 0.05;

    //    if (l1 == l2)
    //        return 0.f;

    //    cg3::Vec3 faceNormal1 = mesh->faceNormal(f1);
    //    cg3::Vec3 faceNormal2 = mesh->faceNormal(f2);

    //    float dot = faceNormal1.dot(faceNormal2);
    //    float cost = exp(-0.5 * pow((dot - 1.f)/smoothSigma, 2.f));

    //    float smoothTerm = compactness * (cost + epsilon);

    if (l1 == l2)
        return 0.0;

    cg3::Pointi face1 = mesh.face(f1);
    cg3::Pointi face2 = mesh.face(f2);

    //TODO FARLO PIU' EFFICIENTE
    std::set<int> vertices1;
    vertices1.insert(face1.x());
    vertices1.insert(face1.y());
    vertices1.insert(face1.z());
    std::set<int> vertices2;
    vertices2.insert(face2.x());
    vertices2.insert(face2.y());
    vertices2.insert(face2.z());
    std::set<int> intersect;
    std::set_intersection(vertices1.begin(), vertices1.end(), vertices2.begin(), vertices2.end(), std::inserter(intersect,intersect.begin()));

    if (intersect.size() != 2) {
        std::cout << "Error intersection" << std::endl;
        exit(-5);
    }

    std::set<int>::iterator it = intersect.begin();
    cg3::Pointd v1 = mesh.vertex(*it);
    it++;
    cg3::Pointd v2 = mesh.vertex(*it);

    cg3::Vec3 dir = v1 - v2;
    dir.normalize();

    float directionCost = static_cast<float>(1 - (dir.x() * dir.x()));

    float smoothTerm = compactness + (directionCost * smoothSigma);

    return smoothTerm;
}

}

} //namespace cg3

#endif
