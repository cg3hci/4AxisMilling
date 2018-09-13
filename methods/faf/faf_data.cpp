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
    angles.clear();

    visibility.clear();

    nonVisibleFaces.clear();

    targetDirections.clear();

    association.clear();
    associationNonVisibleFaces.clear();

    restoredMesh.clear();

    restoredMeshVisibility.clear();
    restoredMeshAssociation.clear();
    restoredMeshNonVisibleFaces.clear();

    minComponent.clear();
    maxComponent.clear();
    fourAxisComponent.clear();

    minComponentAssociation.clear();
    maxComponentAssociation.clear();
    fourAxisComponentAssociation.clear();

    minComponentNonVisibleFaces.clear();
    maxComponentNonVisibleFaces.clear();
    fourAxisComponentNonVisibleFaces.clear();

    surfaces.clear();
    surfacesAssociation.clear();
    minSurface.clear();
    maxSurface.clear();

    stocks.clear();

    results.clear();
    resultsAssociation.clear();
    minResult.clear();
    maxResult.clear();

    minSupport.clear();
    maxSupport.clear();
}

}
