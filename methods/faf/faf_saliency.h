#ifndef FAF_SALIENCY_H
#define FAF_SALIENCY_H

#include <cg3/meshes/eigenmesh/eigenmesh.h>
#include <cg3/libigl/curvature.h>

#include <cg3/viewer/drawable_objects/drawable_eigenmesh.h> //Questo va cancellato (serve solo la eigenmesh)

namespace FourAxisFabrication {

//Queste vanno cancellate, colori dall'interfaccia
void colorByGaussianWeighted(cg3::DrawableEigenMesh& drawablePaintedMesh, const unsigned int nRing = 5);
void colorByMeanCurvature(cg3::DrawableEigenMesh& drawablePaintedMesh, const unsigned int nRing = 5);
void colorBySaliency(cg3::DrawableEigenMesh& drawablePaintedMesh, const unsigned int nRing = 5);

//Dovrai fare questa!
std::vector<bool> findDetailsBySaliency(
        const cg3::EigenMesh& mesh,
        const double limitValue,
        const unsigned int nRing = 5);
}


#endif // FAF_SALIENCY_H
