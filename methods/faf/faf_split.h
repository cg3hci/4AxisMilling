/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#ifndef FAF_SPLIT_H
#define FAF_SPLIT_H

#include <cg3/meshes/eigenmesh/eigenmesh.h>

namespace FourAxisFabrication {

std::vector<int> segmentation(
        const cg3::EigenMesh& mesh);

}

#endif // FAF_SPLIT_H
