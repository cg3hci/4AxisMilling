#include "faf_charts.h"

#include <cg3/geometry/transformations.h>

#include <cg3/libigl/mesh_adjacencies.h>

#include <cg3/meshes/dcel/dcel.h>

#include <unordered_map>
#include <unordered_set>

namespace FourAxisFabrication {

namespace internal {
bool getChartBorders(
        const ChartData& chartData,
        const Chart& chart,
        const std::unordered_map<unsigned int, std::vector<unsigned int>>& vNext,
        const std::unordered_map<unsigned int, std::vector<const cg3::Dcel::HalfEdge*>>& vHeMap,
        unsigned int vStart,
        unsigned int vCurrent,
        const bool isHoleChart,
        std::set<size_t>& currentBorderCharts,
        std::vector<unsigned int>& currentBorderFaces,
        std::vector<unsigned int>& currentBorderVertices,
        std::unordered_set<unsigned int>& visitedBorderVertex,
        int& currentHoleChartId);
} //namespace internal

/**
 * @brief Initialize data associated to the charts
 * @param[in] targetMesh Target mesh
 * @param[in] association Association of faces to the label
 * @param[in] minExtremes Min extremes
 * @param[in] maxExtremes Max extremes
 * @param[out] chartData Data of the charts
 */
ChartData getChartData(
        const cg3::EigenMesh& targetMesh,
        const std::vector<int>& association,
        const std::vector<unsigned int>& minExtremes,
        const std::vector<unsigned int>& maxExtremes)
{
    typedef cg3::Dcel Dcel;
    typedef cg3::Dcel::Face Face;
    typedef cg3::Dcel::HalfEdge HalfEdge;
    typedef cg3::Dcel::Vertex Vertex;

    //Create Dcel from mesh used for navigation
    Dcel dcel(targetMesh);

    std::unordered_set<unsigned int> extremeFaces;

    for (unsigned int f : minExtremes) {
        extremeFaces.insert(f);
    }

    for (unsigned int f : maxExtremes) {
        extremeFaces.insert(f);
    }

    //Result
    ChartData chartData;

    unsigned int nFaces = dcel.numberFaces();

    chartData.faceChartMap.resize(nFaces);

    //Visited flag vector
    std::vector<bool> visited(nFaces, false);

    //Half edges in the border for each chart
    std::vector<std::vector<const HalfEdge*>> borderHalfEdges;

    for (const Face* face : dcel.faceIterator()) {
        unsigned int startFaceId = face->id();

        if (!visited[startFaceId]) {
            int label = association[startFaceId];

            //Initialize chart data
            Chart chart;
            chart.id = chartData.charts.size();
            chart.label = label;

            bool isChartExtreme = false;

            //Half edges in the border of the chart
            std::vector<const HalfEdge*> chartBorderHalfEdges;

            //Stack for iterating on adjacent faces
            std::stack<unsigned int> stack;
            stack.push(startFaceId);

            //Region growing algorithm to get all chart faces
            while (!stack.empty()) {
                unsigned int currentFaceId = stack.top();
                stack.pop();

                if (extremeFaces.find(startFaceId) != extremeFaces.end())
                    isChartExtreme = true;

                if (!visited[currentFaceId]) {
                    visited[currentFaceId] = true;

                    Face* currentFace = dcel.face(currentFaceId);

                    //Add face index to the chart
                    chart.faces.push_back(currentFaceId);

                    //Add vertices
                    for (const Vertex* vertex : currentFace->incidentVertexIterator()) {
                        chart.vertices.insert(vertex->id());
                    }

                    //Add adjacent faces
                    for (const HalfEdge* he : currentFace->incidentHalfEdgeIterator()) {
                        const Face* adjFace = he->twin()->face();

                        unsigned int adjId = adjFace->id();
                        int adjLabel = association[adjId];

                        //If the adjacent face has the same label
                        if (adjLabel == label) {
                            if (!visited[adjId]) {
                                stack.push(adjId);
                            }
                        }
                        //If the adjacent face has a different label
                        //i.e. it is a face of the contour
                        else {
                            chartBorderHalfEdges.push_back(he);

                            chart.adjacentFaces.insert(adjId);
                            chart.adjacentLabels.insert(adjLabel);

                            //Set edge-label map
                            unsigned int fromId = he->fromVertex()->id();
                            unsigned int toId = he->toVertex()->id();

                            std::pair<unsigned int, unsigned int> edge(fromId, toId);
                            std::array<int, 2> labelArray;
                            labelArray[0] = label;
                            labelArray[1] = adjLabel;

                            chartData.edgeLabelMap.insert(std::make_pair(edge, labelArray));
                        }
                    }

                    chartData.faceChartMap[currentFaceId] = chart.id;
                }
            }

            //Add chart data
            chartData.charts.push_back(chart);
            chartData.isExtreme.push_back(isChartExtreme);

            borderHalfEdges.push_back(chartBorderHalfEdges);
        }

    }

    //Calculate borders and chart adjacencies
    for (Chart& chart : chartData.charts) {
        std::vector<const HalfEdge*>& chartBorderHalfEdges = borderHalfEdges.at(chart.id);

        if (!chartBorderHalfEdges.empty()) {
            size_t nVertices = chart.vertices.size();

            //Center of the chart
            cg3::Pointd chartCenter(0,0,0);
            for (const unsigned int& vId : chart.vertices) {
                const Vertex* vertex = dcel.vertex(vId);
                chartCenter += vertex->coordinate();
            }
            chartCenter /= nVertices;

            //Next map and set of vertices in the borders
            std::unordered_map<unsigned int, std::vector<unsigned int>> vNext;
            std::unordered_map<unsigned int, std::vector<const HalfEdge*>> vHeMap;
            std::set<unsigned int> remainingVertices;

            //Furthest vertex
            int furthestVertex = -1;
            double maxDistance = 0;

            for (const HalfEdge* he : chartBorderHalfEdges){
                const Vertex* fromV = he->fromVertex();
                const Vertex* toV = he->toVertex();
                const unsigned int fromId = fromV->id();
                const unsigned int toId = toV->id();

                //Create vectors in the map
                if (vNext.find(fromId) == vNext.end()) {
                    vNext.insert(std::make_pair(fromId, std::vector<unsigned int>()));
                    vHeMap.insert(std::make_pair(fromId, std::vector<const HalfEdge*>()));
                }

                //Fill maps
                vNext.at(fromId).push_back(toId);
                vHeMap.at(fromId).push_back(he);

                //Fill set of vertices
                remainingVertices.insert(fromId);

                //Get furthest point from center: it is certainly part of the external borders
                const cg3::Vec3 vec = fromV->coordinate() - chartCenter;
                double distance = vec.length();
                if (distance >= maxDistance) {
                    maxDistance = distance;
                    furthestVertex = static_cast<int>(fromId);
                }
            }
            assert(furthestVertex >= 0);

            unsigned int vStart;
            unsigned int vCurrent;


            //Get external borders
            vStart = static_cast<unsigned int>(furthestVertex);
            vCurrent = vStart;

            std::set<size_t> currentBorderCharts;
            std::vector<unsigned int> currentBorderFaces;
            std::vector<unsigned int> currentBorderVertices;
            std::unordered_set<unsigned int> visitedBorderVertex;

            int currentHoleId = -1;
            bool externalBorderSuccess = internal::getChartBorders(
                        chartData,
                        chart,
                        vNext,
                        vHeMap,
                        vStart,
                        vCurrent,
                        false,
                        currentBorderCharts,
                        currentBorderFaces,
                        currentBorderVertices,
                        visitedBorderVertex,
                        currentHoleId);

            if (!externalBorderSuccess)
                std::cout << "Error in detecting external borders." << std::endl;

            //Add adjacent chart
            chart.borderCharts = currentBorderCharts;
            chart.borderFaces = currentBorderFaces;
            chart.borderVertices = currentBorderVertices;

            for (unsigned int v : currentBorderVertices)
                remainingVertices.erase(v);


            //Get holes borders
            while (!remainingVertices.empty()) {
                vStart = *(remainingVertices.begin());
                vCurrent = vStart;

                std::set<size_t> currentHoleCharts;
                std::vector<unsigned int> currentHoleFaces;
                std::vector<unsigned int> currentHoleVertices;
                std::unordered_set<unsigned int> visitedHoleVertex;

                int currentHoleId = -1;
                bool holeBorderSuccess = internal::getChartBorders(
                            chartData,
                            chart,
                            vNext,
                            vHeMap,
                            vStart,
                            vCurrent,
                            true,
                            currentHoleCharts,
                            currentHoleFaces,
                            currentHoleVertices,
                            visitedHoleVertex,
                            currentHoleId);

                if (!holeBorderSuccess)
                    std::cout << "Error in detecting hole borders." << std::endl;

                if (currentHoleId == -1)
                    std::cout << "Error in detecting hole chart ids in borders." << std::endl;


                //Add adjacent chart
                chart.holeCharts.insert(static_cast<size_t>(currentHoleId));
                chart.holeFaces.push_back(currentHoleFaces);
                chart.holeVertices.push_back(currentHoleVertices);

                for (unsigned int v : currentHoleVertices)
                    remainingVertices.erase(v);
            }
        }
    }


    return chartData;
}


namespace internal {
bool getChartBorders(
        const ChartData& chartData,
        const Chart& chart,
        const std::unordered_map<unsigned int, std::vector<unsigned int>>& vNext,
        const std::unordered_map<unsigned int, std::vector<const cg3::Dcel::HalfEdge*>>& vHeMap,
        unsigned int vStart,
        unsigned int vCurrent,
        const bool isHoleChart,
        std::set<size_t>& currentBorderCharts,
        std::vector<unsigned int>& currentBorderFaces,
        std::vector<unsigned int>& currentBorderVertices,
        std::unordered_set<unsigned int>& visitedBorderVertex,
        int& currentHoleChartId)
{
    const cg3::Dcel::HalfEdge* he;

    do {
        if (visitedBorderVertex.find(vCurrent) != visitedBorderVertex.end() && !isHoleChart) {
            std::cout << "Error in detecting borders (already visited)." << std::endl;
            return false;
        }

        if (vNext.at(vCurrent).size() == 1) {
            he = vHeMap.at(vCurrent).at(0);

            unsigned int fId = he->face()->id();
            unsigned int adjId = he->twin()->face()->id();
            size_t adjChart = chartData.faceChartMap.at(adjId);

            if (currentHoleChartId == -1 && isHoleChart) {
                currentHoleChartId = static_cast<int>(adjChart);
            }

            currentBorderCharts.insert(adjChart);
            currentBorderFaces.push_back(fId);
            currentBorderVertices.push_back(vCurrent);
            visitedBorderVertex.insert(vCurrent);

            vCurrent = vNext.at(vCurrent).at(0);
        }
        else {
            size_t pos = 0;
            bool success = false;
            do {
                std::set<size_t> newCurrentBorderCharts;
                std::vector<unsigned int> newCurrentBorderFaces;
                std::vector<unsigned int> newCurrentBorderVertices;
                std::unordered_set<unsigned int> copyVisitedBorderVertex = visitedBorderVertex;

                he = vHeMap.at(vCurrent).at(pos);

                unsigned int fId = he->face()->id();
                unsigned int adjId = he->twin()->face()->id();
                size_t adjChart = chartData.faceChartMap.at(adjId);

                if (currentHoleChartId == -1 && isHoleChart) {
                    currentHoleChartId = static_cast<int>(adjChart);
                }

                int adjHoleChartId = -1;
                adjHoleChartId = static_cast<int>(adjChart);

                newCurrentBorderCharts.insert(adjChart);
                newCurrentBorderFaces.push_back(fId);
                newCurrentBorderVertices.push_back(vCurrent);
                copyVisitedBorderVertex.insert(vCurrent);

                if (!isHoleChart || adjHoleChartId == currentHoleChartId) {
                    unsigned int nextVCurrent = vNext.at(vCurrent).at(pos);

                    success = internal::getChartBorders(
                                chartData,
                                chart,
                                vNext,
                                vHeMap,
                                vStart,
                                nextVCurrent,
                                isHoleChart,
                                newCurrentBorderCharts,
                                newCurrentBorderFaces,
                                newCurrentBorderVertices,
                                copyVisitedBorderVertex,
                                currentHoleChartId);
                }

                if (success) {
                    currentBorderCharts.insert(newCurrentBorderCharts.begin(), newCurrentBorderCharts.end());
                    currentBorderFaces.insert(currentBorderFaces.end(), newCurrentBorderFaces.begin(), newCurrentBorderFaces.end());
                    currentBorderVertices.insert(currentBorderVertices.end(), newCurrentBorderVertices.begin(), newCurrentBorderVertices.end());
                    visitedBorderVertex = copyVisitedBorderVertex;

                    return true;
                }

                pos++;
                if (pos >= vHeMap.at(vCurrent).size()) {
                    std::cout << "Error in detecting borders (no path)." << std::endl;
                    return false;
                }
            }
            while (!success);
        }
    }
    while (vCurrent != vStart);

    return true;
}
} //namespace internal

}
