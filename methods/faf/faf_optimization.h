/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#ifndef FAF_OPTIMIZATION_H
#define FAF_OPTIMIZATION_H

#include <cg3/meshes/eigenmesh/eigenmesh.h>

#include "faf_data.h"

namespace FourAxisFabrication {

/* Get optimal association for each face */

void getOptimizedAssociation(
        const cg3::EigenMesh& mesh,
        Data& data);


}

#endif // FAF_OPTIMIZATION_H
