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

void centerAndScale(
        Data& data,
        const bool scaleModel,
        const double modelLength);

void centerAndScale(
        cg3::EigenMesh& mesh,
        const bool scaleModel,
        const double modelLength);

void generateStock(
        Data& data,
        const double stockLength,
        const double stockDiameter);

void cutComponents(
        Data& data,
        const bool cutComponents);


}

#endif // FAF_CUTTING_H
