#include "faf_charts.h"

#include <cg3/geometry/transformations.h>

#include <cg3/libigl/mesh_adjacencies.h>

#include <cg3/meshes/dcel/dcel.h>

#include <unordered_map>

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
    typedef cg3::Dcel Dcel;
    typedef cg3::Dcel::Face Face;
    typedef cg3::Dcel::HalfEdge HalfEdge;
    typedef cg3::Dcel::Vertex Vertex;

    //Create Dcel from mesh used for navigation
    Dcel dcel(targetMesh);

    //Result
    ChartData chartData;

    unsigned int nFaces = dcel.getNumberFaces();

    chartData.faceChartMap.resize(nFaces);

    //Visited flag vector
    std::vector<bool> visited(nFaces, false);

    //Half edges in the border for each chart
    std::vector<std::vector<const HalfEdge*>> borderHalfEdges;

    for (const Face* face : dcel.faceIterator()) {
        unsigned int startFaceId = face->getId();

        if (!visited[startFaceId]) {
            int label = association[startFaceId];

            //Initialize chart data
            Chart chart;

            chart.id = chartData.charts.size();

            chart.label = label;

            //Half edges in the border of the chart
            std::vector<const HalfEdge*> chartBorderHalfEdges;

            //Stack for iterating on adjacent faces
            std::stack<int> stack;
            stack.push(startFaceId);


            //Region growing algorithm to get all chart faces
            while (!stack.empty()) {
                int currentFaceId = stack.top();
                stack.pop();

                visited[currentFaceId] = true;

                Face* currentFace = dcel.getFace(currentFaceId);

                //Add face index to the chart
                chart.faces.push_back(currentFaceId);

                //Add vertices
                for (const Vertex* vertex : currentFace->incidentVertexIterator()) {
                    chart.vertices.insert(vertex->getId());
                }

                //Add adjacent faces
                for (const HalfEdge* he : currentFace->incidentHalfEdgeIterator()) {
                    const Face* adjFace = he->getTwin()->getFace();

                    unsigned int adjId = adjFace->getId();
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
                    }
                }

                chartData.faceChartMap[currentFaceId] = chart.id;
            }

            //Add chart data
            chartData.charts.push_back(chart);

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
                const Vertex* vertex = dcel.getVertex(vId);
                chartCenter += vertex->getCoordinate();
            }
            chartCenter /= nVertices;

            //Next map and set of vertices in the borders
            std::unordered_map<unsigned int, unsigned int> vNext;
            std::unordered_map<unsigned int, const HalfEdge*> vHeMap;
            std::set<unsigned int> remainingVertices;

            //Furthest vertex
            int furthestVertex = -1;
            double maxDistance = 0;

            for (const HalfEdge* he : chartBorderHalfEdges){
                const Vertex* fromV = he->getFromVertex();
                const Vertex* toV = he->getToVertex();
                const unsigned int fromId = fromV->getId();
                const unsigned int toId = toV->getId();

                //Fill maps
                vNext[fromId] = toId;
                vHeMap[fromId] = he;

                //Fill set of vertices
                remainingVertices.insert(fromId);

                //Get furthest point from center: it is certainly part of the external borders
                const cg3::Vec3 vec = fromV->getCoordinate() - chartCenter;
                double distance = vec.getLength();
                if (distance >= maxDistance) {
                    maxDistance = distance;
                    furthestVertex = fromId;
                }
            }
            assert(furthestVertex >= 0);

            unsigned int vStart;
            unsigned int vCurrent;

            //Get external borders
            vStart = (unsigned int) furthestVertex;
            vCurrent = vStart;
            do {
                //Add adjacent chart
                unsigned int adjId = vHeMap.at(vCurrent)->getTwin()->getFace()->getId();
                chart.borderCharts.insert(chartData.faceChartMap.at(adjId));

                chart.borderVertices.push_back(vCurrent);

                remainingVertices.erase(vCurrent);

                vCurrent = vNext.at(vCurrent);
            }
            while (vCurrent != vStart);

            //Get holes
            while (!remainingVertices.empty()) {
                vStart = *(remainingVertices.begin());
                vCurrent = vStart;

                std::vector<unsigned int> hole;

                do {
                    //Add adjacent hole chart
                    unsigned int adjId = vHeMap.at(vCurrent)->getTwin()->getFace()->getId();
                    chart.holeCharts.insert(chartData.faceChartMap.at(adjId));

                    hole.push_back(vCurrent);

                    remainingVertices.erase(vCurrent);

                    vCurrent = vNext.at(vCurrent);
                }
                while (vCurrent != vStart);

                chart.holeVertices.push_back(hole);
            }
        }
    }


    return chartData;
}

}
