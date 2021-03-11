#include "faf_details.h"

#include <cg3/algorithms/mesh_function_smoothing.h>
#include <cg3/algorithms/laplacian_smoothing.h>

#include <cg3/libigl/mesh_adjacencies.h>
#include <cg3/libigl/saliency.h>

namespace FourAxisFabrication {

void findDetails(
        Data& data,
        const bool unitScale,
        const unsigned int nRing,
        const unsigned int nScales,
        const double eps,
        const bool computeBySaliency,
        const double maxSmoothingIterations,
        const double laplacianSmoothingIterations)
{
    data.faceSaliency.clear();
    data.faceSaliency.resize(data.mesh.numberFaces(), 0.0);
    data.saliency.clear();
    data.saliency.resize(data.mesh.numberVertices(), 0.0);

    if (computeBySaliency) {
        cg3::EigenMesh scaledMesh = data.mesh;

        scaledMesh.updateBoundingBox();

        if (unitScale) {
            double scaleFactor = 1 / scaledMesh.boundingBox().diag();

            cg3::Vec3d translateVec = -scaledMesh.boundingBox().center();
            scaledMesh.translate(translateVec);

            const cg3::Vec3d scaleVec(scaleFactor, scaleFactor, scaleFactor);
            scaledMesh.scale(scaleVec);

            scaledMesh.updateBoundingBox();
        }

        //Compute vertex-vertex adjacencies
        std::vector<std::vector<int>> vvAdj = cg3::libigl::vertexToVertexAdjacencies(scaledMesh);

        //Compute saliency
        data.saliency = cg3::libigl::computeSaliencyMultiScale(scaledMesh, vvAdj, nRing, nScales, eps);
        for(unsigned int vId = 0; vId < scaledMesh.numberVertices(); vId++) {
            data.saliency[vId] /= nScales;
        }

        for (unsigned int it = 0; it < maxSmoothingIterations; it++) {
            std::vector<double> lastValues = data.saliency;

            for(unsigned int vId = 0; vId < scaledMesh.numberVertices(); vId++) {
                double maxValue = lastValues[vId];

                for(size_t j = 0; j < vvAdj[vId].size(); j++) {
                    unsigned int adjId = vvAdj[vId][j];

                    maxValue = std::max(maxValue, lastValues[adjId]);
                }

                data.saliency[vId] = maxValue;
            }
        }

        data.saliency = cg3::vertexFunctionLaplacianSmoothing(scaledMesh, data.saliency, laplacianSmoothingIterations, 0.5, vvAdj);

        for (size_t fId = 0; fId < scaledMesh.numberFaces(); fId++) {
            data.faceSaliency[fId] = (
                data.saliency[scaledMesh.face(fId).x()] +
                data.saliency[scaledMesh.face(fId).y()] +
                data.saliency[scaledMesh.face(fId).z()]
            ) / 3;
        }
    }
}

}
