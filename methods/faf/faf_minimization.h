/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#ifndef FAF_MINIMIZATION_H
#define FAF_MINIMIZATION_H

#include "faf_data.h"

namespace FourAxisFabrication {

/* ----- MINIMIZATION OF THE FABRICATION DIRECTIONS ----- */

void getTargetDirections(
        const bool setCoverage,
        Data& data);


}

#endif // FAF_MINIMIZATION_H
