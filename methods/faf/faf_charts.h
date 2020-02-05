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

    std::vector<unsigned int> faces;
    std::set<unsigned int> vertices;

    std::set<unsigned int> adjacentFaces;
    std::set<int> adjacentLabels;

    std::vector<unsigned int> borderVertices;
    std::vector<unsigned int> borderFaces;
    std::set<size_t> borderCharts;
    std::vector<std::vector<unsigned int>> holeVertices;
    std::vector<std::vector<unsigned int>> holeFaces;
    std::vector<std::set<size_t>> holeCharts;
};

/**
 * @brief Struct all chart data
 */
struct ChartData {
    std::vector<size_t> faceChartMap;
    std::vector<Chart> charts;
    std::vector<bool> isExtreme;
    std::map<std::pair<unsigned int, unsigned int>, std::array<int, 2>> edgeLabelMap;

    void clear() {
        charts.clear();
        faceChartMap.clear();
        isExtreme.clear();
    }
};


ChartData getChartData(
        const cg3::EigenMesh& targetMesh,
        const std::vector<int>& association,
        const std::vector<unsigned int>& minExtremes,
        const std::vector<unsigned int>& maxExtremes);

}

#endif // FAF_CHARTS_H
