/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_smoothing.h"


#ifdef CG3_LIBIGL_DEFINED
#include <cg3/libigl/mesh_distance.h>
#include <cg3/libigl/vertex_adjacencies.h>
#endif

#include <cg3/geometry/triangle.h>

#define BINARY_SEARCH_ITERATIONS 20

namespace FourAxisFabrication {


/* ----- INTERNAL FUNCTION DECLARATION ----- */

namespace internal {

bool validateMove(
        const cg3::EigenMesh& mesh,
        const Data& data,
        const int vId,
        const cg3::Pointd& newPoint,
        const std::vector<std::vector<int>>& vertexFaceAdjacencies);

std::vector<cg3::Vec3> computeDifferentialCoordinates(
        const cg3::EigenMesh& mesh,
        const std::vector<std::vector<int>>& vertexAdjacencies);

}

/* ----- RESTORE FREQUENCIES ----- */

/**
 * @brief Restore frequencies of a smoothed mesh.
 * Note that the meshes must have the same number of vertices and faces.
 * @param[in] originalMesh Original detailed mesh
 * @param[in] data Four axis fabrication data
 * @param[in] iterations Number of iterations of the algorithm
 * @param[out] targetMesh Target mesh
 */
void restoreFrequencies(
        const cg3::EigenMesh& mesh,
        const Data& data,
        const int iterations,
        cg3::EigenMesh& targetMesh)
{
    assert(mesh.getNumberVertices() == targetMesh.getNumberVertices());
    assert(mesh.getNumberFaces() == targetMesh.getNumberFaces());

    //Get vertex adjacencies
    const std::vector<std::vector<int>> vertexAdjacencies =
            cg3::libigl::getVertexAdjacencies(mesh);
    assert(vertexAdjacencies.size() == mesh.getNumberVertices());

    const std::vector<std::vector<int>> vertexFaceAdjacencies =
            cg3::libigl::getVertexFaceAdjacencies(mesh);
    assert(vertexFaceAdjacencies.size() == mesh.getNumberVertices());

    const std::vector<cg3::Vec3> differentialCoordinates =
            internal::computeDifferentialCoordinates(mesh, vertexAdjacencies);

    for(int i = 0; i < iterations; ++i) {
        for(unsigned int vId = 0; vId < targetMesh.getNumberVertices(); ++vId) {
            cg3::Pointd delta(0,0,0);

            const std::vector<int>& neighbors = vertexAdjacencies.at(vId);
            cg3::Pointd currentPoint = targetMesh.getVertex(vId);

            for(const int& neighborId : neighbors) {
                delta += targetMesh.getVertex(neighborId);
            }
            delta /= neighbors.size();

            cg3::Pointd newPoint = differentialCoordinates.at(vId) + delta;

            //Do binary search until the face normals do not violate the height-field condition
            int count = 0;
            while (!internal::validateMove(targetMesh, data, vId, newPoint, vertexFaceAdjacencies) &&
                   count < BINARY_SEARCH_ITERATIONS)
            {
                newPoint = 0.5 * (newPoint + currentPoint);

                count++;
            }

            if (count < BINARY_SEARCH_ITERATIONS) {
                targetMesh.setVertex(vId, newPoint);
            }
        }
    }
}

/* ----- DISTANCE COMPUTATION ----- */


#ifdef CG3_LIBIGL_DEFINED
double getHausdorffDistance(
        const cg3::SimpleEigenMesh& mesh1,
        const cg3::SimpleEigenMesh& mesh2)
{
    return cg3::libigl::hausdorffDistance(mesh1, mesh2);
}
#endif


/* ----- INTERNAL FUNCTION DEFINITION ----- */

namespace internal {

/**
 * @brief Compute differential coordinates for the vertices of a mesh
 * @param[in] mesh Input mesh
 * @param[in] vertexAdjacencies Vertex-vertex adjacencies
 * @return differentialCoordinates Vector of differential coordinates for each vertex
 */
std::vector<cg3::Vec3> computeDifferentialCoordinates(
        const cg3::EigenMesh& mesh,
        const std::vector<std::vector<int>>& vertexAdjacencies)
{
    //Resulting vector
    std::vector<cg3::Vec3> differentialCoordinates;
    differentialCoordinates.resize(mesh.getNumberVertices());

    #pragma omp parallel for
    for(unsigned int vId = 0; vId < mesh.getNumberVertices(); ++vId) {
        //Calculate differential coordinates for each point
        cg3::Pointd currentPoint = mesh.getVertex(vId);
        cg3::Vec3 delta(0,0,0);

        const std::vector<int>& neighbors = vertexAdjacencies.at(vId);
        for(const int& neighborId : neighbors) {
            delta += currentPoint - mesh.getVertex(neighborId);
        }

        delta /= neighbors.size();

        differentialCoordinates[vId] = delta;
    }

    return differentialCoordinates;
}

/**
 * @brief Validate move of a vertex.
 * @param[in] mesh Input mesh
 * @param[in] data Four axis fabrication data
 * @param[in] vId Vertex id
 * @param[in] newPos New position
 * @param[in] vertexFaceAdjacencies Vertex-face adjacencies of the mesh
 * @return True if the move is valid
 */
bool validateMove(
        const cg3::EigenMesh& mesh,
        const Data& data,
        const int vId,
        const cg3::Pointd& newPoint,
        const std::vector<std::vector<int>>& vertexFaceAdjacencies)
{
    const std::vector<int>& faces = vertexFaceAdjacencies.at(vId);
    for (const int& fId : faces) {
        const cg3::Pointi face = mesh.getFace(fId);

        cg3::Pointd p1, p2, p3;

        if (face.x() == vId)
            p1 = newPoint;
        else
            p1 = mesh.getVertex(face.x());

        if (face.y() == vId)
            p2 = newPoint;
        else
            p2 = mesh.getVertex(face.y());

        if (face.z() == vId)
            p3 = newPoint;
        else
            p3 = mesh.getVertex(face.z());

        cg3::Triangle3Dd triangle(p1, p2, p3);
        cg3::Vec3 faceNormal = triangle.normal();

        const cg3::Vec3& associatedDirection = data.association[fId];
        if (faceNormal.dot(associatedDirection) < 0)
            return false;
    }
    return true;
}

}

}






////::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

//bool Reconstruction::validate_move(const cinolib::Trimesh<> & m, const int vid, const int dir, const cinolib::vec3d & vid_new_pos)
//{
//    cinolib::vec3d vertex = m.vert(vid);
//    for(int tid : m.adj_v2p(vid))
//    {
//        cinolib::vec3d tri[3];
//        for(int offset=0; offset< 3; ++offset)
//        {
//            int nbr = m.poly_vert_id(tid, offset);
//            tri[offset] = (vid == nbr) ? vid_new_pos : m.vert(nbr);
//        }
//        cinolib::vec3d n = triangle_normal(tri[0], tri[1], tri[2]);

//        switch (dir)
//        {
//            case -1: return false; break;
//            case 0 :
//                if (n.dot(cinolib::vec3d( 1, 0, 0)) < FLIP_ANGLE)
//                    return false;
//                break;
//            case 1 :
//                if (n.dot(cinolib::vec3d( 0, 1, 0)) < FLIP_ANGLE)
//                    return false;
//                break;
//            case 2 :
//                if (n.dot(cinolib::vec3d( 0, 0, 1)) < FLIP_ANGLE)
//                    return false;
//                break;
//            case 3 :
//                if (n.dot(cinolib::vec3d(-1, 0, 0)) < FLIP_ANGLE)
//                    return false;
//                break;
//            case 4 :
//                if (n.dot(cinolib::vec3d( 0,-1, 0)) < FLIP_ANGLE)
//                    return false;
//                break;
//            case 5 :
//                if (n.dot(cinolib::vec3d( 0, 0,-1)) < FLIP_ANGLE)
//                    return false;
//                break;
//            default: assert(false);
//        }
//    }
//    return true;

//}

////::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

//void Reconstruction::differential_coordinates(const cinolib::Trimesh<> & m, std::vector<cinolib::vec3d> & diff_coords)
//{
//    assert(diff_coords.empty());
//    diff_coords.resize(m.num_verts());

//    #pragma omp parallel for
//    for(unsigned int vid=0; vid<m.num_verts(); ++vid)
//    {
//        double w    = 1.0 / double(m.vert_valence(vid));
//        cinolib::vec3d  curr = m.vert(vid);
//        cinolib::vec3d  delta(0,0,0);
//        for(int nbr : m.adj_v2v(vid))
//        {
//            delta += w * (curr - m.vert(nbr));
//        }
//        diff_coords[vid] = delta;
//    }
//}

////::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

//void Reconstruction::restore_high_frequencies_gauss_seidel(cinolib::Trimesh<>          & m_smooth,
//                                           const cinolib::Trimesh<>          & m_detail,
//                                           const std::vector< int > & hf_directions,
//                                           const int n_iters)
//{
//    std::vector<cinolib::vec3d> diff_coords;
//    differential_coordinates(m_detail, diff_coords);

//    for(int i=0; i<n_iters; ++i)
//    {
//        std::cerr << "iter " << i << std::endl;//<< "; nv: " << m_smooth.num_vertices() <<std::endl;

//        #pragma omp parallel for
//        for(unsigned int vid=0; vid<m_smooth.num_verts(); ++vid)
//        {
//            //std::cerr << vid << "\n";
//            cinolib::vec3d  gauss_iter(0,0,0);
//            double w = 1.0 / double(m_smooth.vert_valence(vid));
//            for(int nbr : m_smooth.adj_v2v(vid))
//            {
//                gauss_iter += w * m_smooth.vert(nbr);
//            }

//            cinolib::vec3d new_pos = diff_coords.at(vid) + gauss_iter;

//            // do binary search until the new pos does not violate the hf condition...
//            int count = 0;
//            while(!validate_move(m_smooth, vid, hf_directions.at(vid), new_pos) && ++count<5)
//            {
//                new_pos = 0.5 * (new_pos + m_smooth.vert(vid));
//            }

//            //if (count < 5) m_smooth.set_vertex(vid, new_pos);
//            if (count < 5) m_smooth.vert(vid) =  new_pos;
//        }
//    }
//}
