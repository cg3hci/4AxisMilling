/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#ifndef FAF_DATA_H
#define FAF_DATA_H

#include <vector>

#include <cg3/data_structures/arrays/array2d.h>

#include <cg3/geometry/point.h>

namespace FourAxisFabrication {


/* Data for four axis fabrication */

struct Data {

    std::vector<unsigned int> minExtremes;
    std::vector<unsigned int> maxExtremes;

    std::vector<int> association;

    std::vector<cg3::Vec3> directions;

    cg3::Array2D<int> visibility;

    std::vector<unsigned int> nonVisibleFaces;

    std::vector<unsigned int> targetDirections;

    void clear();
};


/* Check mode of the algorithm */

enum CheckMode { PROJECTION, RAYSHOOTING };

}

#endif // FAF_DATA_H
