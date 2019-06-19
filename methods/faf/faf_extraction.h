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
        const double firstLayerAngle,
        const double firstLayerOffset,
        const double secondLayerStepWidth,
        const double secondLayerStepHeight,
        const double heightfieldAngle,
        const bool xDirectionsAfter,
        const bool rotateResults);

}

#endif // FAF_EXTRACTION_H
