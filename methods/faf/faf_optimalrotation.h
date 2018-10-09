#ifndef FAF_OPTIMALROTATION_H
#define FAF_OPTIMALROTATION_H

#include <cg3/meshes/eigenmesh/eigenmesh.h>

#include "faf_data.h"

namespace FourAxisFabrication {

/* Optimal rotation */

void rotateToOptimalOrientation(
        cg3::EigenMesh& mesh,
        cg3::EigenMesh& smoothedMesh,
        const unsigned int nDirs,
        const double weightPower,
        const bool deterministic);

}

#endif // FAF_OPTIMALROTATION_H
