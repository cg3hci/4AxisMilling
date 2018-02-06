#ifndef FOURAXIS_H
#define FOURAXIS_H
#include <cg3/data_structures/arrays/arrays.h>
#include <cg3/meshes/eigenmesh/eigenmesh.h>

namespace FourAxisFabrication {

void findOptimalRotation(cg3::EigenMesh &m, cg3::EigenMesh& m2);

void cutExtremes(const cg3::EigenMesh& m, std::vector<unsigned int>& minExtreme, std::vector<unsigned int>& maxExtreme);

int maxYFace(std::vector<int> &list, const cg3::EigenMesh& mesh);
int minYFace(std::vector<int>& list, const cg3::EigenMesh& mesh);
void checkPlane(cg3::Array2D<int>& visibility, const cg3::EigenMesh& mesh, int indexPlane, int numberPlanes);
void checkVisibilityAllPlanes(const cg3::EigenMesh& mesh, cg3::Array2D<int> &visibility, int numberPlanes);
void minimizeNumberPlanes(std::vector<int> &survivedPlanes, const cg3::Array2D<int>& visibility);
#ifdef MULTI_LABEL_OPTIMIZATION_INCLUDED
std::vector<int> getAssociation(const std::vector<int> &survivedPlanes, cg3::Array2D<int> &visibility, const cg3::EigenMesh &mesh);
#endif

}

#endif // FOURAXIS_H
