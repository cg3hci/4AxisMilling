/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#ifndef FAF_CUTTING_H
#define FAF_CUTTING_H

#include <vector>

#include <cg3/meshes/eigenmesh/eigenmesh.h>

#include "faf_data.h"

namespace FourAxisFabrication {

void cutComponents(
        Data& data,
        const bool cutComponents);


}

#endif // FAF_CUTTING_H
