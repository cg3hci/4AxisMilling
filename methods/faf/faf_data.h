/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#ifndef FAF_DATA_H
#define FAF_DATA_H

#include <vector>

#include <cg3/data_structures/arrays/array2d.h>

#include <cg3/geometry/point.h>

#include <cg3/meshes/eigenmesh/eigenmesh.h>

#include "faf_charts.h"

namespace FourAxisFabrication {

/* Check mode of the visibility */

enum CheckMode { PROJECTION, RAYSHOOTING, OPENGL };



/* Data for four axis fabrication */

class Data : cg3::SerializableObject {

public:

    Data();

    /* Flags */
    bool isMeshLoaded;
    bool isMeshOriented;
    bool areExtremesSelected;
    bool isVisibilityChecked;
    bool areTargetDirectionsFound;
    bool isAssociationComputed;
    bool isAssociationOptimized;
    bool areFrequenciesRestored;
    bool isVisibilityRecheckedAfterRestore;
    bool areComponentsCut;
    bool areResultsExtracted;

    /* Input meshes */
    cg3::EigenMesh originalMesh;
    cg3::EigenMesh originalSmoothedMesh;

    /* Mesh and smoothed mesh to be processed */
    cg3::EigenMesh mesh;
    cg3::EigenMesh smoothedMesh;

    /* Min/max extremes */

    std::vector<unsigned int> minExtremes;
    std::vector<unsigned int> maxExtremes;

    /* Directions */

    std::vector<cg3::Vec3> directions;
    std::vector<double> angles;

    /* Visibility */

    cg3::Array2D<int> visibility;
    std::vector<unsigned int> nonVisibleFaces;

    /* Target directions */

    std::vector<unsigned int> targetDirections;

    /* Associations */

    std::vector<int> association;
    std::vector<unsigned int> associationNonVisibleFaces;


    /* Frequencies restored data */

    cg3::EigenMesh restoredMesh;    
    cg3::Array2D<int> restoredMeshVisibility;
    std::vector<int> restoredMeshAssociation;
    std::vector<unsigned int> restoredMeshNonVisibleFaces;

    /* Extracted components */

    cg3::EigenMesh minComponent;
    cg3::EigenMesh maxComponent;
    cg3::EigenMesh fourAxisComponent;

    std::vector<int> minComponentAssociation;
    std::vector<int> maxComponentAssociation;
    std::vector<int> fourAxisComponentAssociation;

    std::vector<unsigned int> minComponentNonVisibleFaces;
    std::vector<unsigned int> maxComponentNonVisibleFaces;
    std::vector<unsigned int> fourAxisComponentNonVisibleFaces;


    /* Results */

    cg3::EigenMesh minResult;
    cg3::EigenMesh maxResult;
    std::vector<cg3::EigenMesh> stocks;
    std::vector<cg3::EigenMesh> results;
    std::vector<unsigned int> resultsAssociation;


    /* Supports */

    cg3::EigenMesh minSupport;
    cg3::EigenMesh maxSupport;


    /* Methods */

    void clear();


    // SerializableObject interface
    void serialize(std::ofstream &binaryFile) const;
    void deserialize(std::ifstream &binaryFile);
};

}

#endif // FAF_DATA_H
