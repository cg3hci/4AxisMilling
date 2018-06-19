#ifndef FAF_CHARTS_H
#define FAF_CHARTS_H

#include <vector>
#include <set>

#include <cg3/meshes/eigenmesh/eigenmesh.h>



namespace FourAxisFabrication {

/**
 * @brief Struct to represent attributes and data structures of a chart
 */
struct Chart {
    size_t id;

    int label;

    std::vector<int> faces;
    std::set<int> vertices;

    std::set<int> adjacentFaces;
    std::set<int> adjacentLabels;
};

/**
 * @brief Struct all chart data
 */
struct ChartData {
    std::vector<size_t> faceChartMap;
    std::vector<Chart> charts;

    void clear() {
        charts.clear();
        faceChartMap.clear();
    }
};


ChartData getChartData(
        const cg3::EigenMesh& targetMesh,
        const std::vector<int>& association);

ChartData getChartData(
        const cg3::EigenMesh& targetMesh,
        const std::vector<int>& association,
        const std::vector<std::vector<int>>& faceFaceAdjacencies);

}

#endif // FAF_CHARTS_H
