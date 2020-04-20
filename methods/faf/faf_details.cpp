#include "faf_details.h"

#include <cg3/algorithms/gaussian_weighted_smoothing.h>
#include <cg3/algorithms/saliency.h>
#include <cg3/algorithms/laplacian_smoothing.h>

#include <cg3/libigl/mesh_adjacencies.h>

namespace FourAxisFabrication {

void findDetails(
        Data& data,
        const double limitValue,
        const bool unitScale,
        const unsigned int nRing,
        const unsigned int nScales,
        const bool computeBySaliency)
{
    data.detailFaces.clear();
    data.detailFaces.resize(data.mesh.numberFaces(), false);

    if (computeBySaliency) {
        std::vector<double> saliency;
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
        saliency = cg3::computeSaliencyMultiScale(scaledMesh, vvAdj, nRing, nScales);

        for (size_t fId = 0; fId < scaledMesh.numberFaces(); fId++) {
            if (saliency[scaledMesh.face(fId).x()] >= limitValue &&
                saliency[scaledMesh.face(fId).y()] >= limitValue &&
                saliency[scaledMesh.face(fId).z()] >= limitValue)
            {
                data.detailFaces[fId] = true;
            }
        }
    }
}

}
