/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#ifndef FAF_BOOLEAN_H
#define FAF_BOOLEAN_H

#include <vector>

#include <cg3/meshes/eigenmesh/eigenmesh.h>

#include "faf_data.h"

namespace FourAxisFabrication {

void cutComponents(
        const cg3::EigenMesh& mesh,
        Data& data);


}

#endif // FAF_BOOLEAN_H
