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
    directionsAngle.clear();

    visibility.clear();

    nonVisibleFaces.clear();

    targetDirections.clear();

    association.clear();

    minComponent.clear();
    maxComponent.clear();
    fourAxisComponent.clear();
    fourAxisComponentAssociation.clear();

    results.clear();
    resultsAssociation.clear();
}

}
