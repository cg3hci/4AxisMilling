#include "faf_smoothing.h"

#include <cg3/vcglib/smoothing.h>

namespace FourAxisFabrication {

void smoothing(
        Data& data,
        const int iterations,
        const float lambda,
        const float mu)
{
    //Smooth mesh
    data.smoothedMesh = cg3::vcglib::taubinSmoothing(data.mesh, iterations, lambda, mu);
}


}
