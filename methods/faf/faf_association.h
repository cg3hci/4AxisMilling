/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#ifndef FAF_ASSOCIATION_H
#define FAF_ASSOCIATION_H

#include <cg3/meshes/eigenmesh/eigenmesh.h>

#include "faf_data.h"

namespace FourAxisFabrication {

/* Get optimal association for each face */

void getAssociation(
        const cg3::EigenMesh& mesh,
        const double dataSigma,
        const double smoothSigma,
        const double compactness,
        const bool fixExtremes,
        Data& data);


}

#endif // FAF_ASSOCIATION_H
