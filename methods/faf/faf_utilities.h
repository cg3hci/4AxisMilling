/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#ifndef FAF_UTILITIES_H
#define FAF_UTILITIES_H

#include <vector>

#include <cg3/meshes/eigenmesh/eigenmesh.h>

#include "faf_data.h"

namespace FourAxisFabrication {


/* Optimal rotation */

void rotateToOptimalOrientation(
        cg3::EigenMesh& mesh,
        cg3::EigenMesh& smoothedMesh,
        const unsigned int nOrientations,
        const bool deterministic);



/* Extremes */

void getExtremesOnXAxis(
        const cg3::EigenMesh &mesh,
        Data& data);


}

#endif // FAF_UTILITIES_H
