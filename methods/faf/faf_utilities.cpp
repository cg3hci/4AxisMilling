/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_utilities.h"


namespace FourAxisFabrication {

namespace internal {


/* ----- COMPARATORS ----- */

/**
 * @brief Comparator for barycenter on x values
 * @param m Input mesh
 */
BarycenterXComparator::BarycenterXComparator(const cg3::SimpleEigenMesh& m) : m(m) {}
bool BarycenterXComparator::operator()(unsigned int f1, unsigned int f2){
    const cg3::Pointi& ff1 = m.face(f1);
    const cg3::Pointi& ff2 = m.face(f2);
    cg3::Pointd c1 = (m.vertex(ff1.x()) + m.vertex(ff1.y()) + m.vertex(ff1.z()))/3;
    cg3::Pointd c2 = (m.vertex(ff2.x()) + m.vertex(ff2.y()) + m.vertex(ff2.z()))/3;
    return c1 < c2;
}

/**
 * @brief Comparator for barycenter on z values
 * @param m Input mesh
 */
BarycenterZComparator::BarycenterZComparator(const cg3::SimpleEigenMesh& m) : m(m) {}
bool BarycenterZComparator::operator()(unsigned int f1, unsigned int f2){
    const cg3::Pointi& ff1 = m.face(f1);
    const cg3::Pointi& ff2 = m.face(f2);
    cg3::Pointd c1 = (m.vertex(ff1.x()) + m.vertex(ff1.y()) + m.vertex(ff1.z()))/3;
    cg3::Pointd c2 = (m.vertex(ff2.x()) + m.vertex(ff2.y()) + m.vertex(ff2.z()))/3;
    if (c1.z() < c2.z())
        return true;
    if (c2.z() < c1.z())
        return false;
    return c1 < c2;
}

/**
 * @brief Comparator for triangles
 * @param t1 Triangle 1
 * @param t2 Triangle 2
 * @return True if triangle 1 is less than triangle 2
 */
bool triangle2DComparator(const cg3::Triangle2Dd& t1, const cg3::Triangle2Dd& t2) {
    if (t1.v1() < t2.v1())
        return true;
    if (t2.v1() < t1.v1())
        return false;

    if (t1.v2() < t2.v2())
        return true;
    if (t2.v2() < t1.v2())
        return false;

    return t1.v3() < t2.v3();
}

/* ----- TRIANGLE OVERLAP AND AABB FUNCTIONS ----- */

/**
 * @brief Extract a 2D triangle AABB
 * @param[in] triangle Input triangle
 * @param[in] valueType Type of the value requested (MIN or MAX)
 * @param[in] dim Dimension requested of the value (0 for x, 1 for y)
 * @return Requested coordinate of the AABB
 */
double triangle2DAABBExtractor(
        const cg3::Triangle2Dd& triangle,
        const cg3::AABBValueType& valueType,
        const int& dim)
{
    if (valueType == cg3::AABBValueType::MIN) {
        switch (dim) {
        case 1:
            return (double) std::min(std::min(triangle.v1().x(), triangle.v2().x()), triangle.v3().x());
        case 2:
            return (double) std::min(std::min(triangle.v1().y(), triangle.v2().y()), triangle.v3().y());
        }
    }
    else if (valueType == cg3::AABBValueType::MAX) {
        switch (dim) {
        case 1:
            return (double) std::max(std::max(triangle.v1().x(), triangle.v2().x()), triangle.v3().x());
        case 2:
            return (double) std::max(std::max(triangle.v1().y(), triangle.v2().y()), triangle.v3().y());
        }
    }
    throw new std::runtime_error("Impossible to extract an AABB value.");
}





}





}
