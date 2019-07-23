#include "faf_extremes.h"
#include "faf_various.h"

#include <vector>
#include <queue>
#include <utility>
#include <unordered_set>

#include <cg3/libigl/mesh_adjacencies.h>

namespace FourAxisFabrication {

/* ----- EXTREMES ----- */

/**
 * @brief Get min and max extremes of the mesh along the x direction.
 * The algorithm stops when the current triangle is not visible
 * by related the direction (-x for min and +x for max).
 * @param[in] mesh Input mesh
 * @param[in] heightFieldAngle Height field angle
 * @param[out] data Four axis fabrication data
 */
void selectExtremesOnXAxis(
        const cg3::EigenMesh& mesh,
        const double heightFieldAngle,
        Data& data)
{
    //Faces adjacencies
    const std::vector<std::vector<int>> ffAdj = cg3::libigl::faceToFaceAdjacencies(mesh);

    //Referencing output data
    std::vector<unsigned int>& minExtremes = data.minExtremes;
    std::vector<unsigned int>& maxExtremes = data.maxExtremes;

    selectExtremesOnXAxis(mesh, heightFieldAngle, ffAdj, minExtremes, maxExtremes);
}

/**
 * @brief Get min and max extremes of the mesh along the x direction.
 * The algorithm stops when the current triangle is not visible
 * by related the direction (-x for min and +x for max).
 * @param[in] mesh Input mesh
 * @param[in] heightFieldAngle Height field angle
 * @param[in] ffAdj Face-face adjacencies of the mesh
 * @param[out] minExtremes Min extremes
 * @param[out] maxExtremes Max extremes
 */
void selectExtremesOnXAxis(
        const cg3::EigenMesh& mesh,
        const double heightFieldAngle,
        const std::vector<std::vector<int>> ffAdj,
        std::vector<unsigned int>& minExtremes,
        std::vector<unsigned int>& maxExtremes)
{
    //Clearing current data (if any)
    minExtremes.clear();
    maxExtremes.clear();

    //Cos of height-field angle
    const double heightFieldAngleLimit = cos(heightFieldAngle);

    //Remember that the milling direction is the opposite if we want to
    //check the dot product. So (1,0,0) is the min, (-1,0,0) is the max.
    const cg3::Vec3d minDirection(-1,0,0);
    const cg3::Vec3d maxDirection(1,0,0);

    //Height-field on the extremes
    std::unordered_set<unsigned int> minHeightFieldSet;
    std::unordered_set<unsigned int> maxHeightFieldSet;

    //Initializing vector of face indices
    std::vector<unsigned int> fIndices(mesh.numberFaces());
    for (unsigned int i = 0; i < fIndices.size(); i++)
        fIndices[i] = i;

    //Order the vector by x-coordinate
    std::sort(fIndices.begin(), fIndices.end(), internal::XMinComparator(mesh));


    /* ----- MIN EXTREMES ----- */

    //Get min height-field faces
    size_t iMin = 0;
    while(mesh.faceNormal(fIndices[iMin]).dot(minDirection) >= heightFieldAngleLimit){
        minHeightFieldSet.insert(fIndices[iMin]);
        iMin++;
    }

    std::vector<bool> minVisited(mesh.numberFaces(), false);
    std::queue<unsigned int> minQueue;

    minQueue.push(fIndices[0]);

    while (!minQueue.empty()) {
        unsigned int fId = minQueue.front();
        minQueue.pop();

        if (!minVisited[fId]) {
            cg3::Point3i face = mesh.face(fId);

            minVisited[fId] = true;

            if (minHeightFieldSet.find(fId) != minHeightFieldSet.end()) {
                minExtremes.push_back(fId);

                const std::vector<int>& fAdj = ffAdj.at(fId);

                for (const int& adjId : fAdj) {
                    if (!minVisited[adjId]) {
                        minQueue.push(adjId);
                    }
                }

            }
        }
    }


    /* ----- MAX EXTREMES ----- */

    //Get max height-field faces
    size_t iMax = fIndices.size()-1;
    while(mesh.faceNormal(fIndices[iMax]).dot(maxDirection) >= heightFieldAngleLimit){
        maxHeightFieldSet.insert(fIndices[iMax]);
        iMax--;
    }

    std::vector<bool> maxVisited(mesh.numberFaces(), false);
    std::queue<unsigned int> maxQueue;

    maxQueue.push(fIndices[fIndices.size()-1]);

    while (!maxQueue.empty()) {
        unsigned int fId = maxQueue.front();
        maxQueue.pop();

        if (!maxVisited[fId]) {
            cg3::Point3i face = mesh.face(fId);

            maxVisited[fId] = true;

            if (maxHeightFieldSet.find(fId) != maxHeightFieldSet.end()) {
                maxExtremes.push_back(fId);

                const std::vector<int>& fAdj = ffAdj.at(fId);

                for (const int& adjId : fAdj) {
                    if (!maxVisited[adjId]) {
                        maxQueue.push(adjId);
                    }
                }

            }
        }
    }

}




}
