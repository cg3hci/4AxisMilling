#ifndef FAF_OPTIMALROTATION_H
#define FAF_OPTIMALROTATION_H

#include <cg3/meshes/eigenmesh/eigenmesh.h>

#include "faf_data.h"

namespace FourAxisFabrication {

/* Optimal rotation */

bool rotateToOptimalOrientation(
        cg3::EigenMesh& mesh,
        cg3::EigenMesh& smoothedMesh,
        const double stockLength,
        const double stockDiameter,
        const unsigned int nDirs,
        const double extremeWeight,
        const double BBweight,
        const bool deterministic);

}

#endif // FAF_OPTIMALROTATION_H
