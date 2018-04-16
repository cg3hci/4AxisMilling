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

/* Visibility check */

void initializeDataForVisibilityCheck(
        const cg3::EigenMesh& mesh,
        const unsigned int nDirections,
        const bool fixExtremeAssociation,
        Data& data);

void getVisibility(
        const cg3::EigenMesh& mesh,
        const unsigned int nDirections,
        Data& data,
        CheckMode checkMode);


void detectNonVisibleFaces(
        Data& data);


}

#endif // FAF_VISIBILITYCHECK_H
