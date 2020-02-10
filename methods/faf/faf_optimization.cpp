/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_optimization.h"

#include "faf_charts.h"

#include <cg3/libigl/mesh_adjacencies.h>

#include <set>
#include <queue>

namespace FourAxisFabrication {

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
        cg3::EigenMesh& mesh,
        const bool relaxHoles,
        const bool loseHoles,
        const double minChartArea,
        Data& data)
{
    //Get fabrication data    
    const std::vector<cg3::Vec3d>& directions = data.directions;
    cg3::Array2D<int>& visibility = data.visibility;
    std::vector<unsigned int>& minExtremes = data.minExtremes;
    std::vector<unsigned int>& maxExtremes = data.maxExtremes;
    std::vector<unsigned int>& targetDirections = data.targetDirections;
    std::vector<int>& association = data.association;
    std::vector<unsigned int>& associationNonVisibleFaces = data.associationNonVisibleFaces;

    const unsigned int nFaces = mesh.numberFaces();

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

            for (const std::set<size_t>& holeChartsIds : surroundingChart.holeCharts) {
                for (unsigned int holeChartId : holeChartsIds) {
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

                assert(!queue.empty());
                if (queue.empty())
                    break;

                while (!queue.empty()) {
                    unsigned int fId = queue.front();
                    queue.pop();

                    if (visitedFaces.find(fId) != visitedFaces.end())
                        continue;

                    visitedFaces.insert(fId);

                    const std::vector<int>& adjacentFaces = ffAdj.at(fId);

                    //The best label for the face is one among the adjacent
                    //which has the less dot product with the normal
                    double maxDot = -1;
                    int bestLabel = -1;

                    for (const unsigned int adjId : adjacentFaces) {
                        int adjLabel = association[adjId];

                        if (chartLabel == adjLabel) {
                            //Add the not visited faces in the border to the queue
                            if (visitedFaces.find(adjId) == visitedFaces.end())
                                queue.push(adjId);
                        }
                        else {
                            cg3::Vec3d normal = mesh.faceNormal(fId);
                            double dot = normal.dot(directions[adjLabel]);

                            if (dot >= maxDot) {
                                maxDot = dot;
                                bestLabel = adjLabel;
                            }
                        }
                    }
                    assert(bestLabel != -1 && bestLabel != chartLabel);

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

            for (const std::set<size_t>& holeChartsIds : surroundingChart.holeCharts) {
                for (unsigned int holeChartId : holeChartsIds) {
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

    unsigned int newNumberFaces = mesh.numberFaces();

    //Update association non-visible faces
    associationNonVisibleFaces.clear();
    for (unsigned int fId = 0; fId < newNumberFaces; fId++){
        if (visibility(association[fId], fId) == 0) {
            associationNonVisibleFaces.push_back(fId);
        }
    }

    //The remaining directions are the new target directions
    std::vector<bool> usedDirections(directions.size(), false);
    std::vector<unsigned int> newTargetDirections;
    for (unsigned int fId = 0; fId < newNumberFaces; fId++){
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

} //namespace FourAxisFabrication
