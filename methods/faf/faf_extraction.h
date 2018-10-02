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
        const double modelLength,
        const double stockLength,
        const double stockDiameter,
        const double supportHeight,
        const double firstLayerAngle,
        const double secondLayerAngle,
        const double millableHeight,
        const bool rotateMeshes);

}

#endif // FAF_EXTRACTION_H
