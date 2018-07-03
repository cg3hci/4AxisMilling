/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#ifndef FAF_EXTRACTION_H
#define FAF_EXTRACTION_H

#include <vector>

#include <cg3/meshes/eigenmesh/eigenmesh.h>

#include "faf_data.h"

namespace FourAxisFabrication {

void extractResults(
        Data& data,
        const double stockLength,
        const double stockDiameter,
        const double stepHeight,
        const double stepWidth,
        const bool rotateMeshes);

}

#endif // FAF_EXTRACTION_H
