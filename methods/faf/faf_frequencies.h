/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#ifndef FAF_FREQUENCIES_H
#define FAF_FREQUENCIES_H

#include <vector>

#include <cg3/meshes/eigenmesh/eigenmesh.h>

#include "faf_data.h"

namespace FourAxisFabrication {

/* Restore frequencies */

void restoreFrequencies(
        const unsigned int iterations,
        const double heightfieldAngle,
        const cg3::EigenMesh& originalMesh,
        cg3::EigenMesh& smoothedMesh,
        Data& data);


void recheckVisibilityAfterRestore(
        Data& data,
        const double heightFieldAngle,
        const bool reassign,
        CheckMode checkMode = PROJECTION);

}

#endif // FAF_FREQUENCIES_H
