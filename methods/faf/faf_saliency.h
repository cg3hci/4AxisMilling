#ifndef FAF_SALIENCY_H
#define FAF_SALIENCY_H

#include <cg3/meshes/eigenmesh/eigenmesh.h>
#include <cg3/libigl/curvature.h>
#include <cg3/libigl/mesh_adjacencies.h>
#include "faf_data.h"
#include <cg3/viewer/drawable_objects/drawable_eigenmesh.h>
#include <algorithm>

namespace FourAxisFabrication {

void meshCurvature(cg3::DrawableEigenMesh& drawablePaintedMesh);
void colorMeanCurvature(cg3::DrawableEigenMesh& drawablePaintedMesh);

}


#endif // FAF_SALIENCY_H
