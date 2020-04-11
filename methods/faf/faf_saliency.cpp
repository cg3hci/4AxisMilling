#include "faf_saliency.h"

namespace FourAxisFabrication {

namespace internal {

cg3::Color computeColorByMeancurvature(double long floatBetweenZeroAndOne){
    cg3::Color color(0,0,0);

    if (floatBetweenZeroAndOne <= 0.5f)
    {
        //floatBetweenZeroAndOne = floatBetweenZeroAndOne * 2.0f - 1.0f;
        floatBetweenZeroAndOne *= 2.0f;
        color.setRed((unsigned int)(255 * (1.0f - floatBetweenZeroAndOne) + 0.5f));
        color.setGreen((unsigned int)(255 * (floatBetweenZeroAndOne) + 0.5f));
    }
    else
    {
        //floatBetweenZeroAndOne *= 2.0f;
        floatBetweenZeroAndOne = floatBetweenZeroAndOne * 2.0f - 1.0f;
        color.setGreen((unsigned int)(255 * (1.0f - floatBetweenZeroAndOne) + 0.5f));
        color.setBlue((unsigned int)(255 * (floatBetweenZeroAndOne) + 0.5f));
    }

    return color;
}

double expression(cg3::Point3d x, cg3::Point3d v, double sigma){
    return (-(std::pow(x.dist(v),2))) / (2 * std::pow(sigma,2));
}

}

void meshCurvature(cg3::DrawableEigenMesh& drawablePaintedMesh){

    bool meanCurvatureFalg = true;
    if(meanCurvatureFalg)
        colorMeanCurvature(drawablePaintedMesh);
    else {


        unsigned int nRing = 5;
        std::vector<double> meanCurvature = cg3::libigl::meanVertexCurvature(drawablePaintedMesh, nRing);
        std::vector<std::vector<int>> vvadjacenses = cg3::libigl::vertexToVertexAdjacencies(drawablePaintedMesh);
        std::vector<double> gaussian_wheighted(drawablePaintedMesh.numberVertices());
        double sigma = (drawablePaintedMesh.boundingBox().diag() * 0.003) * 2;

        for(unsigned int vertex_id = 0; vertex_id < drawablePaintedMesh.numberVertices(); vertex_id++){
            std::vector<int> stack;
            std::vector<bool> visited(drawablePaintedMesh.numberVertices(), false);
            long double numerator = 0;
            long double denominator = 0;
            stack.push_back(vertex_id);
            visited[vertex_id] = true;
            while(stack.size() > 0){
                int current_vertex = stack.back();
                stack.pop_back();
                double exp = internal::expression(drawablePaintedMesh.vertex(current_vertex), drawablePaintedMesh.vertex(vertex_id), sigma);
                numerator += meanCurvature[current_vertex] * exp;
                denominator += exp;

                for(unsigned int adj_vertex_id = 0; adj_vertex_id < vvadjacenses[current_vertex].size(); adj_vertex_id++){
                    if(!visited[vvadjacenses[current_vertex][adj_vertex_id]] && (drawablePaintedMesh.vertex(vertex_id).dist(drawablePaintedMesh.vertex(vvadjacenses[current_vertex][adj_vertex_id])) < (sigma * 2))){
                        stack.push_back(vvadjacenses[current_vertex][adj_vertex_id]);
                        visited[vvadjacenses[current_vertex][adj_vertex_id]] = true;
                    }
                }
            }
            if(denominator != 0)
                gaussian_wheighted[vertex_id] = numerator / denominator;
        }


        double min, max = gaussian_wheighted[0];
        for(unsigned int i = 0; i < gaussian_wheighted.size(); i++){
            if(gaussian_wheighted[i] > max)
                max = gaussian_wheighted[i];
            if(gaussian_wheighted[i] < min)
                min = gaussian_wheighted[i];
        }

        std::cout << "Min value: " << min << std::endl;
        std::cout << "Max value: " << max << std::endl;

        std::vector<double> gaussian_wheighted_norm = gaussian_wheighted;
        for(unsigned int value = 0; value < gaussian_wheighted_norm.size(); value++){
            gaussian_wheighted_norm[value] = (gaussian_wheighted_norm[value])/(max);
        }

        for(unsigned int face_id = 0; face_id < drawablePaintedMesh.numberFaces(); face_id++){

            double long floatBetweenZeroAndOne = (gaussian_wheighted[drawablePaintedMesh.face(face_id).x()] +
                                                  gaussian_wheighted[drawablePaintedMesh.face(face_id).y()] +
                                                  gaussian_wheighted[drawablePaintedMesh.face(face_id).z()] ) / 3;
            //std::cout << floatBetweenZeroAndOne << std::endl;
            cg3::Color color = internal::computeColorByMeancurvature(floatBetweenZeroAndOne);
            drawablePaintedMesh.setFaceColor(color, face_id);
        }

    }

}



void colorMeanCurvature(cg3::DrawableEigenMesh& drawablePaintedMesh){
    unsigned int nRing = 5;
    double scaleFactor = 1/drawablePaintedMesh.boundingBox().diag();
    cg3::DrawableEigenMesh scaledMesh(drawablePaintedMesh);
    const cg3::Vec3d scaleVec(scaleFactor, scaleFactor, scaleFactor);
    scaledMesh.scale(scaleVec);
    scaledMesh.updateBoundingBox();
    cg3::Vec3d translateVec = -scaledMesh.boundingBox().center();
    scaledMesh.translate(translateVec);
    scaledMesh.updateBoundingBox();

    std::vector<double> meanCurvature = cg3::libigl::meanVertexCurvature(drawablePaintedMesh, nRing);
    std::vector<double> meanCurvatureSorted = meanCurvature;
    std::vector<double> meanCurvatureNormalized = meanCurvature;

    std::sort(meanCurvatureSorted.begin(), meanCurvatureSorted.end());
    double min = meanCurvatureSorted.front();
    double max = meanCurvatureSorted.back();
    std::cout << min << std::endl;
    std::cout << max << std::endl;

    for(unsigned int value = 0; value < meanCurvature.size(); value++){
        meanCurvatureNormalized[value] = (meanCurvatureNormalized[value]-min)/(max-min);
    }

    for(unsigned int face_id = 0; face_id < drawablePaintedMesh.numberFaces(); face_id++){

        float floatBetweenZeroAndOne = (meanCurvatureNormalized[drawablePaintedMesh.face(face_id).x()] +
                                        meanCurvatureNormalized[drawablePaintedMesh.face(face_id).y()] +
                                        meanCurvatureNormalized[drawablePaintedMesh.face(face_id).z()] ) / 3;
        cg3::Color color = internal::computeColorByMeancurvature(floatBetweenZeroAndOne);
        drawablePaintedMesh.setFaceColor(color, face_id);
    }

}

void computeGaussianWeight(cg3::DrawableEigenMesh& drawablePaintedMesh){

}


}
