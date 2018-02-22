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
        const cg3::EigenMesh& originalMesh,
        const Data& data,
        cg3::EigenMesh& targetMesh,
        const int iterations = 5);

/* Distance computation */

#ifdef CG3_LIBIGL_DEFINED
double getHausdorffDistance(
        const cg3::SimpleEigenMesh& mesh1,
        const cg3::SimpleEigenMesh& mesh2);
#endif

}

#endif // FAF_SMOOTHING_H
