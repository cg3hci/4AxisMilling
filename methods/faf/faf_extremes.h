#ifndef FAF_EXTREMES_H
#define FAF_EXTREMES_H

#include <cg3/meshes/eigenmesh/eigenmesh.h>

#include "faf_data.h"

namespace FourAxisFabrication {

/* Extremes selection */

void selectExtremesOnXAxis(
        const cg3::EigenMesh& mesh,
        const double heightFieldAngle,
        Data& data);

void selectExtremesOnXAxis(
        const cg3::EigenMesh& mesh,
        const double heightFieldAngle,
        const std::vector<std::vector<int>> ffAdj,
        std::vector<unsigned int>& minExtremes,
        std::vector<unsigned int>& maxExtremes,
        double& levelSetMinX,
        double& levelSetMaxX);

}


#endif // FAF_EXTREMES_H
