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
    isMeshScaledAndStockGenerated = false;
    isSaliencyComputed = false;
    isMeshSmoothed = false;
    isMeshOriented = false;
    areExtremesSelected = false;
    isVisibilityChecked = false;
    isAssociationComputed = false;
    isAssociationOptimized = false;
    isLineSmoothed = false;
    areFrequenciesRestored = false;
    areComponentsCut = false;
    areResultsExtracted = false;

    originalMesh.clear();
    originalSmoothedMesh.clear();

    mesh.clear();
    stock.clear();

    saliency.clear();
    faceSaliency.clear();

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

    fourAxisAssociation.clear();
    fourAxisVisibility.clear();
    fourAxisNonVisibleFaces.clear();

    minResult.clear();
    maxResult.clear();
    boxes.clear();
    stocks.clear();
    results.clear();
    resultsAssociation.clear();

    minSupport.clear();
    maxSupport.clear();
}

void Data::serialize(std::ofstream &binaryFile) const
{
    cg3::serializeObjectAttributes(
                "faf_data",
                binaryFile,
                isMeshLoaded,
                isMeshScaledAndStockGenerated,
                isSaliencyComputed,
                isMeshSmoothed,
                isMeshOriented,
                areExtremesSelected,
                isVisibilityChecked,
                isAssociationComputed,
                isAssociationOptimized,
                isLineSmoothed,
                areFrequenciesRestored,
                areComponentsCut,
                areResultsExtracted,
                originalMesh,
                originalSmoothedMesh,
                mesh,
                stock,
                saliency,
                faceSaliency,
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
                fourAxisAssociation,
                fourAxisVisibility,
                fourAxisNonVisibleFaces,
                minResult,
                maxResult,
                boxes,
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
                isSaliencyComputed,
                isMeshScaledAndStockGenerated,
                isMeshSmoothed,
                isMeshOriented,
                areExtremesSelected,
                isVisibilityChecked,
                isAssociationComputed,
                isAssociationOptimized,
                isLineSmoothed,
                areFrequenciesRestored,
                areComponentsCut,
                areResultsExtracted,
                originalMesh,
                originalSmoothedMesh,
                mesh,
                stock,
                saliency,
                faceSaliency,
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
                fourAxisAssociation,
                fourAxisVisibility,
                fourAxisNonVisibleFaces,
                minResult,
                maxResult,
                boxes,
                stocks,
                results,
                resultsAssociation,
                minSupport,
                maxSupport);
}

}
