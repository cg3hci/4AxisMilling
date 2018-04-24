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
        const bool fixExtremeAssociation,
        Data& data,
        CheckMode checkMode);


bool recheckVisibility(
        const cg3::EigenMesh& mesh,
        Data& data,
        CheckMode checkMode = PROJECTION);

}

#endif // FAF_VISIBILITYCHECK_H
