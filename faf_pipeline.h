#ifndef FAF_PIPELINE_H
#define FAF_PIPELINE_H

#include "methods/faf/faf_data.h"
#include "faf_parameters.h"

namespace FAFPipeline {

void scaleAndStock(
		FourAxisFabrication::Data& data,
		bool scaleModel,
		double modelLength,
		double stockLength,
		double stockDiameter);

void saliency(
		FourAxisFabrication::Data& data);

void smoothing(
		FourAxisFabrication::Data& data,
		unsigned int iterations);

void optimalOrientation(
		FourAxisFabrication::Data& data,
		double stockLength,
		double stockDiameter,
		unsigned int nOrientations);

void selectExtremes(
		FourAxisFabrication::Data& data);

void checkVisibility(
		FourAxisFabrication::Data& data,
		unsigned int nDirections);

void getAssociation(
		FourAxisFabrication::Data& data,
		double detailMultiplier,
		double compactness);

void optimizeAssociation(
		FourAxisFabrication::Data& data);

void smoothLines(
		FourAxisFabrication::Data& data);

void restoreFrequencies(
		FourAxisFabrication::Data& data);

void colorizeAssociation(
		FourAxisFabrication::Data& data);

void cutComponents(
		FourAxisFabrication::Data& data);

void extractResults(
		FourAxisFabrication::Data& data,
		double firstLayerAngle,
		bool minFirst,
		double stockLength,
		double stockDiameter);

void pipeline(
		FourAxisFabrication::Data& data,
		const FAFParameters& parmas);

}

#endif // FAF_PIPELINE_H
