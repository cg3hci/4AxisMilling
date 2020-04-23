#ifndef FAF_DETAILS_H
#define FAF_DETAILS_H

#include <cg3/meshes/eigenmesh/eigenmesh.h>
#include <cg3/libigl/curvature.h>

#include "faf_data.h"

namespace FourAxisFabrication {

void findDetails(
        Data& data,
        const bool unitScale = true,
        const unsigned int nRing = 5,
        const unsigned int nScales = 5,
        const double eps = 0.003,
        const bool computeBySaliency = true,
        const double maxSmoothingIterations = 5,
        const double laplacianSmoothingIterations = 20);

}


#endif // FAF_DETAILS_H
