#include "faf_smoothing.h"

#include <cg3/vcglib/smoothing.h>

namespace FourAxisFabrication {

void taubinSmoothing(
        Data& data,
        const int iterations,
        const float lambda,
        const float mu)
{
    data.smoothedMesh = cg3::vcglib::taubinSmoothing(data.mesh, iterations, lambda, mu);
}


}
