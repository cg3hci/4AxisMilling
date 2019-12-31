#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include <cg3/geometry/point2.h>
#include <cg3/meshes/eigenmesh/algorithms/eigenmesh_algorithms.h>

namespace FourAxisFabrication {

namespace internal {

std::vector<std::array<cg3::Point2d, 3>> triangulation(
                   std::vector<cg3::Point2d>& currentFirstLayerPoints2D,
                   std::vector<cg3::Point2d>& newLayerPoints2D,
                   std::set<cg3::Point2d>& newLayerPoints2DSet,
                   std::vector<cg3::Point2d>& newLayerPoints2Dvector);

void createSquare(std::vector<cg3::Point2d>& squarePoints2D, cg3::Point3d minCoord, cg3::Point3d maxCoord);

}

}

#endif // TRIANGULATION_H
