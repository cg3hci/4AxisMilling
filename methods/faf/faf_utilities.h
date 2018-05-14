/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#ifndef FAF_UTILITIES_H
#define FAF_UTILITIES_H


#include <cg3/geometry/point.h>
#include <cg3/geometry/2d/triangle2d.h>

#include <cg3/data_structures/trees/aabbtree.h>

#include <cg3/meshes/eigenmesh/simpleeigenmesh.h>

namespace FourAxisFabrication {

namespace internal {

/* Comparators */

struct BarycenterXComparator {
    const cg3::SimpleEigenMesh& m;
    BarycenterXComparator(const cg3::SimpleEigenMesh& m);
    bool operator()(unsigned int f1, unsigned int f2);
};

struct BarycenterZComparator {
    const cg3::SimpleEigenMesh& m;
    BarycenterZComparator(const cg3::SimpleEigenMesh& m);
    bool operator()(unsigned int f1, unsigned int f2);
};

bool triangle2DComparator(const cg3::Triangle2Dd& t1, const cg3::Triangle2Dd& t2);



/* AABB extractor functions */

double triangle2DAABBExtractor(
        const cg3::Triangle2Dd& triangle,
        const cg3::AABBValueType& valueType,
        const int& dim);




}

}

#endif // FAF_UTILITIES_H
