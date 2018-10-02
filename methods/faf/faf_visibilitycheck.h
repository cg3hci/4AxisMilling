/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#ifndef FAF_VISIBILITYCHECK_H
#define FAF_VISIBILITYCHECK_H

#include <vector>

#include <cg3/meshes/eigenmesh/eigenmesh.h>

#include "faf_data.h"

namespace FourAxisFabrication {

/* Get visibility */

void getVisibility(
        const cg3::EigenMesh& mesh,
        const unsigned int nDirections,
        const unsigned int resolution,
        const double heightfieldAngle,
        const bool includeXDirections,
        Data& data,
        const CheckMode checkMode);

}

#endif // FAF_VISIBILITYCHECK_H
