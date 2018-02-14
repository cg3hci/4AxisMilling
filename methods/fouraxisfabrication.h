/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#ifndef FOURAXISFABRICATION_H
#define FOURAXISFABRICATION_H

#include <cg3/meshes/eigenmesh/eigenmesh.h>

#include "faf/faf_data.h"
#include "faf/faf_visibilitycheck.h"
#include "faf/faf_utilities.h"
#include "faf/faf_minimization.h"
#include "faf/faf_optimization.h"

namespace FourAxisFabrication {

/* Four axis fabrication */

void computeEntireAlgorithm(
        cg3::EigenMesh& mesh,
        cg3::EigenMesh& smoothedMesh,
        const unsigned int nOrientations,
        const bool deterministic,
        const unsigned int nDirections,
        const bool fixExtremeAssociation,
        const bool setCoverage,
        Data& data,
        CheckMode checkMode = PROJECTION);
}

#endif // FOURAXISFABRICATION_H
