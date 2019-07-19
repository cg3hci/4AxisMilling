/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_split.h"
#include <cg3/cgal/skeleton.h>
#include <cg3/vcglib/smoothing.h>

namespace FourAxisFabrication {

std::vector<int> segmentation(
        const cg3::EigenMesh& mesh)
{
    std::vector<cg3::Point3d> points;
    std::vector<std::vector<size_t>> birthVertices;
    std::vector<std::vector<size_t>> polylines;

    points = cg3::cgal::skeleton(mesh, birthVertices, polylines);

    std::vector<int> association(mesh.numberVertices(), -1);

    int currentLabel = 0;
    for (const std::vector<size_t>& polyline : polylines) {
        for (const size_t polylineVId : polyline) {
            for (const size_t& birthVId : birthVertices[polylineVId]) {
                if (association[birthVId] == -1) {
                    association[birthVId] = currentLabel;
                }
                else {
                    association[birthVId] = -2;
                }
            }
        }

        currentLabel++;
    }

    return association;

}

} //namespace FourAxisFabrication
