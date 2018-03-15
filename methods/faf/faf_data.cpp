/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_data.h"

namespace FourAxisFabrication {

/* ----- METHODS OF DATA FOR FOUR AXIS FABRICATION ----- */

void Data::clear() {
    minExtremes.clear();
    maxExtremes.clear();

    directions.clear();

    visibility.clear();

    nonVisibleFaces.clear();

    targetDirections.clear();

    association.clear();

    minResult.clear();
    maxResult.clear();
    fourAxisResult.clear();
    fourAxisResultAssociation.clear();
}

}
