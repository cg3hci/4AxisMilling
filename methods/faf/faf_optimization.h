/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#ifndef FAF_OPTIMIZATION_H
#define FAF_OPTIMIZATION_H

#include <cg3/meshes/eigenmesh/eigenmesh.h>

#include "faf_data.h"

namespace FourAxisFabrication {

void optimization(
        cg3::EigenMesh& mesh,
        const bool relaxHoles,
        const bool loseHoles,
        const double minChartArea,
        const bool smoothEdgeLines,
        Data& data);

}

#endif // FAF_OPTIMIZATION_H
