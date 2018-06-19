#include "faf_charts.h"

#include <cg3/geometry/transformations.h>

#include <cg3/libigl/mesh_adjacencies.h>

namespace FourAxisFabrication {

/**
 * @brief Initialize data associated to the charts
 * @param[in] targetMesh Target mesh
 * @param[in] data Four axis fabrication data
 * @param[out] chartData Data of the charts
 */
ChartData getChartData(
        const cg3::EigenMesh& targetMesh,
        const std::vector<int>& association)
{
    const std::vector<std::vector<int>> faceFaceAdjacencies = cg3::libigl::getFaceFaceAdjacencies(targetMesh);

    return getChartData(targetMesh, association, faceFaceAdjacencies);
}

/**
 * @brief Initialize data associated to the charts
 * @param[in] targetMesh Target mesh
 * @param[in] data Four axis fabrication data
 * @param[in] faceFaceAdjacencies Face-face adjacencies of the mesh
 * @param[out] chartData Data of the charts
 */
ChartData getChartData(
        const cg3::EigenMesh& targetMesh,
        const std::vector<int>& association,
        const std::vector<std::vector<int>>& faceFaceAdjacencies)
{
    ChartData chartData;

    unsigned int nFaces = targetMesh.getNumberFaces();

    //Clear data
    chartData.faceChartMap.resize(nFaces);


    //Visited flag vector
    std::vector<bool> visited(nFaces, false);

    for (unsigned int fId = 0; fId < targetMesh.getNumberFaces(); fId++) {
        if (!visited[fId]) {
            int label = association[fId];

            //If a label has been assigned
            if (label > 0) {
                //Initialize chart data
                Chart chart;

                chart.id = chartData.charts.size();
                chartData.faceChartMap[fId] = chart.id;

                chart.label = label;

                //Stack for iterating on adjacent faces
                std::stack<int> stack;
                stack.push(fId);


                //Region growing algorithm to get all chart faces
                while (!stack.empty()) {
                    int faceId = stack.top();
                    stack.pop();

                    //Add face index to the chart
                    chart.faces.push_back(faceId);

                    const std::vector<int>& adjFaces = faceFaceAdjacencies[faceId];

                    //Add adjacent faces
                    for (int adjFace : adjFaces) {
                        int adjFaceLabel = association[adjFace];
                        //The adjacent face has the same label
                        if (adjFaceLabel == label) {
                            if (!visited[adjFace]) {
                                stack.push(adjFace);
                            }
                        }
                        //The adjacent face has a different label
                        //i.e. it is a face of the contour
                        else {
                            chart.adjacentFaces.insert(adjFace);
                            chart.adjacentLabels.insert(adjFaceLabel);
                        }
                    }

                    visited[faceId] = true;
                }

                //Add chart data
                chartData.charts.push_back(chart);
            }
        }
    }

    return chartData;
}

}
