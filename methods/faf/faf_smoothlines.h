/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#ifndef FAF_SMOOTHLINES_H
#define FAF_SMOOTHLINES_H

#include <cg3/meshes/eigenmesh/eigenmesh.h>

#include "faf_data.h"

namespace FourAxisFabrication {

void smoothLines(
        cg3::EigenMesh& mesh,
        const bool smoothEdgeLines,
        Data& data);

}

#endif // FAF_SMOOTHLINES_H
