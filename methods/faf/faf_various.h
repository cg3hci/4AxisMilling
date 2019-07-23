/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#ifndef FAF_CUTTING_H
#define FAF_CUTTING_H

#include <vector>

#include <cg3/meshes/eigenmesh/eigenmesh.h>

#include "faf_data.h"

namespace FourAxisFabrication {

namespace internal {
/**
 * @brief Comparator for min x-coordinate values in a face
 * @param m Input mesh
 */
struct XMinComparator {
    const cg3::SimpleEigenMesh& m;
    inline XMinComparator(const cg3::SimpleEigenMesh& m) : m(m) {}
    inline bool operator()(unsigned int f1, unsigned int f2){
        const cg3::Point3i& ff1 = m.face(f1);
        const cg3::Point3i& ff2 = m.face(f2);
        double c1 = std::min(std::min(m.vertex(ff1.x()).x(), m.vertex(ff1.y()).x()), m.vertex(ff1.z()).x());
        double c2 = std::min(std::min(m.vertex(ff2.x()).x(), m.vertex(ff2.y()).x()), m.vertex(ff2.z()).x());
        return c1 < c2;
    }
};

}

/* Level set utilities */

void getMinAndMaxHeightfieldLevelSet(
        const cg3::EigenMesh& mesh,
        const double heightFieldAngle,
        const std::vector<std::vector<int>> ffAdj,
        double& levelSetMinX,
        double& levelSetMaxX);

/* Transformation utilities */

void centerAndScale(
        Data& data,
        const bool scaleModel,
        const double modelLength);

void centerAndScale(
        cg3::EigenMesh& mesh,
        const bool scaleModel,
        const double modelLength);

/* Stock */

void generateStock(
        Data& data,
        const double stockLength,
        const double stockDiameter);

/* Cut components */

void cutComponents(
        Data& data,
        const bool cutComponents);

}

#endif // FAF_CUTTING_H
