/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_data.h"

namespace FourAxisFabrication {

/* ----- METHODS OF DATA FOR FOUR AXIS FABRICATION ----- */

Data::Data() {
    this->clear();
}

void Data::clear() {
    isMeshLoaded = false;
    isMeshOriented = false;
    areExtremesSelected = false;
    isVisibilityChecked = false;
    areTargetDirectionsFound = false;
    isAssociationComputed = false;
    isAssociationOptimized = false;
    areFrequenciesRestored = false;
    isVisibilityRecheckedAfterRestore = false;
    areComponentsCut = false;
    areResultsExtracted = false;

    originalMesh.clear();
    originalSmoothedMesh.clear();

    mesh.clear();
    smoothedMesh.clear();

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

    stocks.clear();

    results.clear();
    resultsAssociation.clear();
    minResult.clear();
    maxResult.clear();

    minSupport.clear();
    maxSupport.clear();
}

void Data::serialize(std::ofstream &binaryFile) const
{
    cg3::serializeObjectAttributes(
                "faf_data",
                binaryFile,
                isMeshLoaded,
                isMeshOriented,
                areExtremesSelected,
                isVisibilityChecked,
                areTargetDirectionsFound,
                isAssociationComputed,
                isAssociationOptimized,
                areFrequenciesRestored,
                isVisibilityRecheckedAfterRestore,
                areComponentsCut,
                areResultsExtracted,
                originalMesh,
                originalSmoothedMesh,
                mesh,
                smoothedMesh,
                minExtremes,
                maxExtremes,
                directions,
                angles,
                visibility,
                nonVisibleFaces,
                targetDirections,
                association,
                associationNonVisibleFaces,
                restoredMesh,
                restoredMeshVisibility,
                restoredMeshAssociation,
                restoredMeshNonVisibleFaces,
                minComponent,
                maxComponent,
                fourAxisComponent,
                minComponentAssociation,
                maxComponentAssociation,
                fourAxisComponentAssociation,
                minComponentNonVisibleFaces,
                maxComponentNonVisibleFaces,
                fourAxisComponentNonVisibleFaces,
                minResult,
                maxResult,
                stocks,
                results,
                resultsAssociation,
                minSupport,
                maxSupport);
}

void Data::deserialize(std::ifstream &binaryFile)
{
    cg3::deserializeObjectAttributes(
                "faf_data",
                binaryFile,
                isMeshLoaded,
                isMeshOriented,
                areExtremesSelected,
                isVisibilityChecked,
                areTargetDirectionsFound,
                isAssociationComputed,
                isAssociationOptimized,
                areFrequenciesRestored,
                isVisibilityRecheckedAfterRestore,
                areComponentsCut,
                areResultsExtracted,
                originalMesh,
                originalSmoothedMesh,
                mesh,
                smoothedMesh,
                minExtremes,
                maxExtremes,
                directions,
                angles,
                visibility,
                nonVisibleFaces,
                targetDirections,
                association,
                associationNonVisibleFaces,
                restoredMesh,
                restoredMeshVisibility,
                restoredMeshAssociation,
                restoredMeshNonVisibleFaces,
                minComponent,
                maxComponent,
                fourAxisComponent,
                minComponentAssociation,
                maxComponentAssociation,
                fourAxisComponentAssociation,
                minComponentNonVisibleFaces,
                maxComponentNonVisibleFaces,
                fourAxisComponentNonVisibleFaces,
                minResult,
                maxResult,
                stocks,
                results,
                resultsAssociation,
                minSupport,
                maxSupport);
}

}
