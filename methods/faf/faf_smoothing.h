/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#ifndef FAF_SMOOTHING_H
#define FAF_SMOOTHING_H

#include <vector>

#include <cg3/meshes/eigenmesh/eigenmesh.h>

#include "faf_data.h"

namespace FourAxisFabrication {

/* Restore frequencies */

void restoreFrequencies(
        const unsigned int iterations,
        const bool occlusionsCheck,
        const cg3::EigenMesh& originalMesh,
        cg3::EigenMesh& smoothedMesh,
        Data& data);

}

#endif // FAF_SMOOTHING_H
