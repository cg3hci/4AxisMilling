#include "faf_saliency.h"

#include <cg3/algorithms/gaussian_weighted_smoothing.h>
#include <cg3/algorithms/saliency.h>

namespace FourAxisFabrication {

namespace internal {

//Da cancellare da qui, fare da interfaccia
cg3::Color computeColorByNormalizedValue(const double value);
//Da cancellare anche, da interfaccia
std::vector<double> normalizeWithVariance(const std::vector<double>& function, const double stdMultiplier = 1.5);

}

void colorByGaussianWeighted(cg3::DrawableEigenMesh& drawablePaintedMesh, const unsigned int nRing)
{
    std::vector<double> meanCurvature = cg3::libigl::meanVertexCurvature(drawablePaintedMesh, nRing);

    double sigma = (drawablePaintedMesh.boundingBox().diag() * 0.003) * 2;
    std::vector<double> gaussianWeighted = cg3::vertexGaussianWeightedSmoothing(drawablePaintedMesh, meanCurvature, sigma, sigma * 2);

    std::vector<double> gaussianWeightedNormalized = internal::normalizeWithVariance(gaussianWeighted);

    for(size_t fId = 0; fId < drawablePaintedMesh.numberFaces(); fId++) {

        double value = (gaussianWeightedNormalized[drawablePaintedMesh.face(fId).x()] +
                        gaussianWeightedNormalized[drawablePaintedMesh.face(fId).y()] +
                        gaussianWeightedNormalized[drawablePaintedMesh.face(fId).z()] ) / 3.0;

        cg3::Color color = internal::computeColorByNormalizedValue(value);

        drawablePaintedMesh.setFaceColor(color, fId);
    }
}

void colorByMeanCurvature(cg3::DrawableEigenMesh& drawablePaintedMesh, const unsigned int nRing)
{
    double scaleFactor = 1/drawablePaintedMesh.boundingBox().diag();
    cg3::DrawableEigenMesh scaledMesh(drawablePaintedMesh);
    const cg3::Vec3d scaleVec(scaleFactor, scaleFactor, scaleFactor);
    scaledMesh.scale(scaleVec);
    scaledMesh.updateBoundingBox();
    cg3::Vec3d translateVec = -scaledMesh.boundingBox().center();
    scaledMesh.translate(translateVec);
    scaledMesh.updateBoundingBox();

    std::vector<double> meanCurvature = cg3::libigl::meanVertexCurvature(drawablePaintedMesh, nRing);

    std::vector<double> meanCurvatureNormalized = internal::normalizeWithVariance(meanCurvature);

    for(size_t fId = 0; fId < drawablePaintedMesh.numberFaces(); fId++) {
        float value = (meanCurvatureNormalized[drawablePaintedMesh.face(fId).x()] +
                       meanCurvatureNormalized[drawablePaintedMesh.face(fId).y()] +
                       meanCurvatureNormalized[drawablePaintedMesh.face(fId).z()] ) / 3;

        cg3::Color color = internal::computeColorByNormalizedValue(value);

        drawablePaintedMesh.setFaceColor(color, fId);
    }

}

void colorBySaliency(cg3::DrawableEigenMesh& drawablePaintedMesh, const unsigned int nRing)
{
    double sigma = (drawablePaintedMesh.boundingBox().diag() * 0.003) * 4;

    std::vector<double> saliency = cg3::computeSaliency(drawablePaintedMesh, sigma, nRing);

    std::vector<double> saliencyNormalized = internal::normalizeWithVariance(saliency);

    for(size_t fId = 0; fId < drawablePaintedMesh.numberFaces(); fId++) {
        double value = (saliencyNormalized[drawablePaintedMesh.face(fId).x()] +
                        saliencyNormalized[drawablePaintedMesh.face(fId).y()] +
                        saliencyNormalized[drawablePaintedMesh.face(fId).z()] ) / 3.0;

        cg3::Color color = internal::computeColorByNormalizedValue(value);

        drawablePaintedMesh.setFaceColor(color, fId);
    }
}


namespace internal {

//Da cancellare da qui, fare da interfaccia
cg3::Color computeColorByNormalizedValue(const double value)
{
    cg3::Color color(0,0,0);

    assert(value >= 0 && value <= 1);

    if (value <= 0.5f) {
        double normalizedValue = value * 2.0;
        color.setRed(static_cast<unsigned int>(std::round(255 * (1.0 - normalizedValue))));
        color.setGreen(static_cast<unsigned int>(std::round(255 * normalizedValue)));
    }
    else {
        double normalizedValue = (value - 0.5) * 2.0;
        color.setGreen(static_cast<unsigned int>(std::round(255 * (1.0 - normalizedValue))));
        color.setBlue(static_cast<unsigned int>(std::round(255 * normalizedValue)));
    }

    return color;
}

std::vector<double> normalizeWithVariance(const std::vector<double>& function, const double stdMultiplier)
{
    std::vector<double> normalizedFunction(function.size());

//    double min = function[0];
//    double max = function[0];
//    for(size_t i = 0; i < function.size(); i++) {
//        if(function[i] > max)
//            max = function[i];
//        if(function[i] < min)
//            min = function[i];
//        for(size_t i = 0; i < function.size(); i++) {
//            normalizedFunction[i] = (function[i] - min) / (max - min);
//        }
//    }


    //We find the variance from 0
    double positiveSTD = 0;
    double negativeSTD = 0;

    size_t numPositive = 0;
    size_t numNegative = 0;
    for(size_t i = 0; i < function.size(); i++) {
        if (function[i] >= 0) {
            positiveSTD += (function[i] * function[i]);
            numPositive++;
        }
        else {
            negativeSTD += (function[i] * function[i]);
            numNegative++;
        }
    }
    positiveSTD /= numPositive;
    negativeSTD /= numNegative;

    positiveSTD = std::sqrt(positiveSTD) * stdMultiplier;
    negativeSTD = std::sqrt(negativeSTD) * stdMultiplier;

    for(size_t i = 0; i < function.size(); i++) {
        if (function[i] >= 0) {
            if (function[i] > positiveSTD)
                normalizedFunction[i] = 1;
            else
                normalizedFunction[i] = 0.5 + ((function[i] / positiveSTD) / 2.0);
        }
        else {
            if (-function[i] > negativeSTD)
                normalizedFunction[i] = 0;
            else
                normalizedFunction[i] = 0.5 - ((-function[i] / negativeSTD) / 2.0);
        }
    }

    return normalizedFunction;

}

}

}
