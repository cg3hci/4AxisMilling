#ifndef FAF_SMOOTHING_H
#define FAF_SMOOTHING_H

#include <cg3/meshes/eigenmesh/eigenmesh.h>

#include "faf_data.h"

namespace FourAxisFabrication {

void smoothing(
        Data& data,
        const int iterations,
        const float lambda,
        const float mu);

}


#endif // FAF_SMOOTHING_H
