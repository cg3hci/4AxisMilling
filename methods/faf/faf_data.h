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


/* Data for four axis fabrication */

struct Data {

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


    /* Chart data */
    ChartData chartData;


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

    /* Surface results */

    cg3::EigenMesh minSurface;
    cg3::EigenMesh maxSurface;
    std::vector<cg3::EigenMesh> surfaces;
    std::vector<unsigned int> surfacesAssociation;


    /* Results */

    cg3::EigenMesh minResult;
    cg3::EigenMesh maxResult;
    std::vector<cg3::EigenMesh> stocks;
    std::vector<cg3::EigenMesh> results;
    std::vector<unsigned int> resultsAssociation;


    /* Methods */

    void clear();
};


/* Check mode of the algorithm */

enum CheckMode { PROJECTION, RAYSHOOTING };

}

#endif // FAF_DATA_H
