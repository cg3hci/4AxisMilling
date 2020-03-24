/**
 * @author Stefano Nuvoli
 */
#include "faf_smoothlines.h"

#include "faf_charts.h"

#include <cg3/libigl/mesh_adjacencies.h>
#include <cg3/vcglib/curve_on_manifold.h>

#include <set>

namespace FourAxisFabrication {

namespace internal {
void reassignLabelsAfterLineSmoothing(
        const cg3::EigenMesh& mesh,
        const std::set<std::pair<cg3::Point3d, cg3::Point3d>>& newEdgesCoordinates,
        const std::vector<cg3::Vec3d>& directions,
        std::vector<int>& association,
        cg3::Array2D<int>& visibility);
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
void smoothLines(
        cg3::EigenMesh& mesh,
        const bool smoothEdgeLines,
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

    //Get mesh adjacencies
    std::vector<std::vector<int>> ffAdj = cg3::libigl::faceToFaceAdjacencies(mesh);

    //Get chart data
    ChartData chartData = getChartData(mesh, association, minExtremes, maxExtremes);
    if (smoothEdgeLines) {
        std::vector<std::pair<cg3::Point3d, cg3::Point3d>> polylines;

        std::unordered_set<int> computedLabels;

        for (const Chart& chart : chartData.charts) {
            int label = chart.label;
            computedLabels.insert(label);

            for (const int adjLabel : chart.adjacentLabels) {
                if (computedLabels.find(adjLabel) != computedLabels.end())
                    continue;

                for (size_t i = 0; i < chart.borderVertices.size(); i++) {
                    std::array<int, 2>& labelArray = chartData.edgeLabelMap.at(
                                std::make_pair(
                                    chart.borderVertices[i],
                                    chart.borderVertices[(i+1) % chart.borderVertices.size()]
                                ));

                    if (labelArray[0] == label && labelArray[1] == adjLabel) {
                        polylines.push_back(std::make_pair(
                            mesh.vertex(chart.borderVertices[i]),
                            mesh.vertex(chart.borderVertices[(i+1) % chart.borderVertices.size()])
                        ));
                    }
                }

                for (size_t i = 0; i < chart.holeVertices.size(); i++) {
                    for (size_t j = 0; j < chart.holeVertices[i].size(); j++) {
                        std::array<int, 2>& labelArray = chartData.edgeLabelMap.at(
                                    std::make_pair(
                                        chart.holeVertices[i][j],
                                        chart.holeVertices[i][(j+1) % chart.holeVertices[i].size()]
                                    ));

                        if (labelArray[0] == label && labelArray[1] == adjLabel) {
                            polylines.push_back(std::make_pair(
                                mesh.vertex(chart.holeVertices[i][j]),
                                mesh.vertex(chart.holeVertices[i][(j+1) % chart.holeVertices[i].size()])
                            ));
                        }
                    }
                }
            }
        }

        unsigned int initialNumFaces = mesh.numberFaces();

        std::set<std::pair<cg3::Point3d, cg3::Point3d>> newEdgesCoordinates;
        mesh = cg3::vcglib::curveOnManifold(mesh, polylines, newEdgesCoordinates, 15, 15, 0.1, false, true);

        unsigned int newNumberFaces = mesh.numberFaces();

        visibility.conservativeResize(visibility.rows(), newNumberFaces);
        association.resize(newNumberFaces);
        for (unsigned int fId = initialNumFaces; fId < newNumberFaces; fId++) {
            association[fId] = -1;
            for (size_t lId = 0; lId < directions.size(); lId++) {
                visibility(lId, fId) = 0;
            }
        }

        internal::reassignLabelsAfterLineSmoothing(mesh, newEdgesCoordinates, data.directions, association, visibility);

        //Update min and max extremes
        int minLabel = targetDirections[targetDirections.size()-2];
        int maxLabel = targetDirections[targetDirections.size()-1];
        minExtremes.clear();
        maxExtremes.clear();
        for (size_t fId = 0; fId < newNumberFaces; fId++) {
            if (association[fId] == minLabel)
                minExtremes.push_back(fId);
            else if (association[fId] == maxLabel)
                maxExtremes.push_back(fId);
        }
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


namespace internal {
void reassignLabelsAfterLineSmoothing(
        const cg3::EigenMesh& mesh,
        const std::set<std::pair<cg3::Point3d, cg3::Point3d>>& newEdgesCoordinates,
        const std::vector<cg3::Vec3d>& directions,
        std::vector<int>& association,
        cg3::Array2D<int>& visibility)
{
    std::stack<unsigned int> stack;

    //Get mesh adjacencies
    std::vector<std::vector<int>> ffAdj = cg3::libigl::faceToFaceAdjacencies(mesh);

    assert(mesh.numberFaces() > 0);

    std::vector<bool> visited(mesh.numberFaces(), false);
    for (unsigned int i = 0; i < mesh.numberFaces(); i++) {
        if (visited[i])
            continue;

        std::vector<unsigned int> currentChartFaces;

        stack.push(i);
        while (!stack.empty()) {
            unsigned int fId = stack.top();
            stack.pop();

            if (visited[fId])
                continue;

            currentChartFaces.push_back(fId);

            visited[fId] = true;

            cg3::Point3i face = mesh.face(fId);
            std::set<cg3::Point3d> faceCoords;
            faceCoords.insert(mesh.vertex(face.x()));
            faceCoords.insert(mesh.vertex(face.y()));
            faceCoords.insert(mesh.vertex(face.z()));

            std::vector<int>& adjacentFaces = ffAdj[fId];
            for (int adjId : adjacentFaces) {
                cg3::Point3i adjFace = mesh.face(adjId);
                std::set<cg3::Point3d> adjFaceCoords;
                adjFaceCoords.insert(mesh.vertex(adjFace.x()));
                adjFaceCoords.insert(mesh.vertex(adjFace.y()));
                adjFaceCoords.insert(mesh.vertex(adjFace.z()));

                std::vector<cg3::Point3d> intersection;
                std::set_intersection(faceCoords.begin(), faceCoords.end(), adjFaceCoords.begin(), adjFaceCoords.end(), std::back_inserter(intersection));

                assert(intersection.size() == 2);
                std::pair<cg3::Point3d, cg3::Point3d> edge;
                edge.first = intersection[0];
                edge.second = intersection[1];
                if (edge.first < edge.second)
                    std::swap(edge.first, edge.second);

                if (newEdgesCoordinates.find(edge) == newEdgesCoordinates.end())
                    stack.push(adjId);
            }
        }

        std::vector<unsigned int> numFacesPerLabel(directions.size(), 0);
        for (unsigned int currentChartFace : currentChartFaces) {
            if (association[currentChartFace] >= 0) {
                numFacesPerLabel[association[currentChartFace]]++;
            }
        }
        size_t bestLabel = 0;
        for (size_t label = 1; label < directions.size(); label++) {
            if (numFacesPerLabel[label] >= numFacesPerLabel[bestLabel]) {
                bestLabel = label;
            }
        }
        for (unsigned int currentChartFace : currentChartFaces) {
            association[currentChartFace] = bestLabel;
            visibility(bestLabel, currentChartFace) = 1;
        }
    }
}
}

} //namespace FourAxisFabrication
