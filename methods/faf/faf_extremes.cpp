#include "faf_extremes.h"

#include <vector>
#include <queue>
#include <utility>

#include <cg3/libigl/mesh_adjacencies.h>

namespace FourAxisFabrication {

namespace internal {
/**
 * @brief Comparator for min x-coordinate values in a face
 * @param m Input mesh
 */
struct XMinComparator {
    const cg3::SimpleEigenMesh& m;
    inline XMinComparator(const cg3::SimpleEigenMesh& m) : m(m) {}
    inline bool operator()(unsigned int f1, unsigned int f2){
        const cg3::Pointi& ff1 = m.face(f1);
        const cg3::Pointi& ff2 = m.face(f2);
        double c1 = std::min(std::min(m.vertex(ff1.x()).x(), m.vertex(ff1.y()).x()), m.vertex(ff1.z()).x());
        double c2 = std::min(std::min(m.vertex(ff2.x()).x(), m.vertex(ff2.y()).x()), m.vertex(ff2.z()).x());
        return c1 < c2;
    }
};

}

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

    double levelSetMinX;
    double levelSetMaxX;

    selectExtremesOnXAxis(mesh, heightFieldAngle, ffAdj, minExtremes, maxExtremes, levelSetMinX, levelSetMaxX);
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
        std::vector<unsigned int>& maxExtremes,
        double& levelSetMinX,
        double& levelSetMaxX)
{
    //Clearing current data (if any)
    minExtremes.clear();
    maxExtremes.clear();

    //Cos of height-field angle
    const double heightFieldAngleLimit = cos(heightFieldAngle);

    //Remember that the milling direction is the opposite if we want to
    //check the dot product. So (1,0,0) is the min, (-1,0,0) is the max.
    const cg3::Vec3 minDirection(-1,0,0);
    const cg3::Vec3 maxDirection(1,0,0);

    //Height-field on the extremes
    std::set<unsigned int> minHeightFieldSet;
    std::set<unsigned int> maxHeightFieldSet;

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

    //Get the min x coordinate of the non-heightfield faces (it will be the level set)
    levelSetMinX = mesh.vertex(mesh.face(fIndices[iMin]).x()).x();
    while(iMin < fIndices.size()){
        cg3::Pointi face = mesh.face(fIndices[iMin]);

        levelSetMinX = std::min(levelSetMinX, mesh.vertex(face.x()).x());
        levelSetMinX = std::min(levelSetMinX, mesh.vertex(face.y()).x());
        levelSetMinX = std::min(levelSetMinX, mesh.vertex(face.z()).x());

        iMin++;
    }

    std::vector<bool> minVisited(mesh.numberFaces(), false);
    std::queue<unsigned int> minQueue;

    minQueue.push(fIndices[0]);

    while (!minQueue.empty()) {
        unsigned int fId = minQueue.front();
        minQueue.pop();

        if (!minVisited[fId]) {
            cg3::Pointi face = mesh.face(fId);

            minVisited[fId] = true;

            if (minHeightFieldSet.find(fId) != minHeightFieldSet.end()) {
                //If every face coordinate is lower the level set
                if (mesh.vertex(face.x()).x() <= levelSetMinX &&
                        mesh.vertex(face.y()).x() <= levelSetMinX &&
                        mesh.vertex(face.z()).x() <= levelSetMinX)
                {
                    minExtremes.push_back(fId);
                }

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
    int iMax = fIndices.size()-1;
    while(mesh.faceNormal(fIndices[iMax]).dot(maxDirection) >= heightFieldAngleLimit){
        maxHeightFieldSet.insert(fIndices[iMax]);
        iMax--;
    }

    //Get the max x coordinate of the non-selected faces (level set)
    levelSetMaxX = mesh.vertex(mesh.face(fIndices[iMax]).x()).x();
    while(iMax >= 0){
        cg3::Pointi face = mesh.face(fIndices[iMax]);

        levelSetMaxX = std::max(levelSetMaxX, mesh.vertex(face.x()).x());
        levelSetMaxX = std::max(levelSetMaxX, mesh.vertex(face.y()).x());
        levelSetMaxX = std::max(levelSetMaxX, mesh.vertex(face.z()).x());

        iMax--;
    }

    std::vector<bool> maxVisited(mesh.numberFaces(), false);
    std::queue<unsigned int> maxQueue;

    maxQueue.push(fIndices[fIndices.size()-1]);

    while (!maxQueue.empty()) {
        unsigned int fId = maxQueue.front();
        maxQueue.pop();

        if (!maxVisited[fId]) {
            cg3::Pointi face = mesh.face(fId);

            maxVisited[fId] = true;

            if (maxHeightFieldSet.find(fId) != maxHeightFieldSet.end()) {
                //If every face coordinate is lower the level set
                if (mesh.vertex(face.x()).x() >= levelSetMaxX &&
                        mesh.vertex(face.y()).x() >= levelSetMaxX &&
                        mesh.vertex(face.z()).x() >= levelSetMaxX)
                {
                    maxExtremes.push_back(fId);
                }

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
