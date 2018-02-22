/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "faf_smoothing.h"


#ifdef CG3_LIBIGL_DEFINED
#include <cg3/libigl/mesh_distance.h>
#include <cg3/libigl/vertex_adjacencies.h>
#endif

namespace FourAxisFabrication {


/* ----- INTERNAL FUNCTION DECLARATION ----- */

namespace internal {

std::vector<cg3::Vec3> computeDifferentialCoordinates(const cg3::EigenMesh& mesh);

}

/* ----- RESTORE FREQUENCIES ----- */

/**
 * @brief Restore frequencies of a smoothed mesh.
 * Note that the meshes must have the same number of vertices and faces.
 * @param[in] originalMesh Original detailed mesh
 * @param[in] data Four axis fabrication data
 * @param[out] targetMesh Target mesh
 * @param[in] iterations Number of iterations (default is 5)
 */
void restoreFrequencies(
        const cg3::EigenMesh& originalMesh,
        const Data& data,
        cg3::EigenMesh& targetMesh,
        const int iterations)
{
    assert(originalMesh.getNumberVertices() == targetMesh.getNumberVertices());
    assert(originalMesh.getNumberFaces() == targetMesh.getNumberFaces());

    std::vector<cg3::Vec3> differentialCoordinates =
            internal::computeDifferentialCoordinates(originalMesh);

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


/* ----- RESTORE FREQUENCIES ----- */

namespace internal {

/**
 * @brief Compute differential coordinates for the vertices of a mesh
 * @param[in] mesh Input mesh
 * @param differentialCoordinates Vector of differential coordinates for each vertex
 */
std::vector<cg3::Vec3> computeDifferentialCoordinates(const cg3::EigenMesh& mesh)
{
    //Resulting vector
    std::vector<cg3::Vec3> differentialCoordinates;
    differentialCoordinates.resize(mesh.getNumberVertices());

    //Get vertex adjacencies
    std::vector<std::vector<int>> vertexAdjacencies = cg3::libigl::getVertexAdjacencies(mesh);

    #pragma omp parallel for
    for(unsigned int vId = 0; vId < mesh.getNumberVertices(); ++vId) {
        //Calculate differential coordinates for each point
        cg3::Pointd currentPoint = mesh.getVertex(vId);
        cg3::Vec3 delta(0,0,0);

        std::vector<int>& neighbors = vertexAdjacencies[vId];

        for(int neighborId : neighbors) {
            delta += currentPoint - mesh.getVertex(neighborId);
        }

        delta /= neighbors.size();

        differentialCoordinates[vId] = delta;
    }

    return differentialCoordinates;
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
