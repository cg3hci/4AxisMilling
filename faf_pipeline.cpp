#include "faf_pipeline.h"

#include <cg3/libigl/mesh_distance.h>

#include "methods/faf/faf_various.h"

#include "methods/faf/faf_details.h"
#include "methods/faf/faf_smoothing.h"
#include "methods/faf/faf_optimalrotation.h"
#include "methods/faf/faf_extremes.h"
#include "methods/faf/faf_visibilitycheck.h"
#include "methods/faf/faf_association.h"
#include "methods/faf/faf_optimization.h"
#include "methods/faf/faf_smoothlines.h"
#include "methods/faf/faf_frequencies.h"
#include "methods/faf/faf_extraction.h"

//other default values
const double heightfieldAngle = 90.0 / 180.0 * M_PI;

//saliency
bool computeBySaliency = true;
bool unitScale = true;
unsigned int nRing = 5;
unsigned int nScales = 5;
double eps = 0.003;
unsigned int maxIterations = 5;
unsigned int laplacianIterations = 20;

//smoothing
float lambda = 0.9;
float mu = -0.89;

//optimal orientation
const double extremeWeight = 0.0;
const double BBWeight = 0.0;
const bool deterministic = true;

//check visibility
const unsigned int resolution = 16384;
const bool includeXDirections = false;
const FourAxisFabrication::CheckMode checkMode = FourAxisFabrication::PROJECTION;

//get association
const double dataSigma = 1.0;
const bool fixExtremes = true;

//optimize association
const bool relaxHoles = false;
const bool loseHoles = false;
const double minChartArea = 0.01;

//smooth lines
const bool smoothEdgeLines = true;

//restore frequencies
const unsigned int nIterations = 50;
const bool recheck = true;
const bool reassign = false;

//colorize
const int scatterColorMaxHue(240);
const int scatterColorSat(static_cast<int>(255 * 0.45));
const int scatterColorVal(static_cast<int>(255 * 0.9));
const cg3::Color defaultColor(128,128,128);
const cg3::Color minColor(200,60,60);
const cg3::Color maxColor(60,60,200);

//cut components
const bool cutComponents = false;

//extract results
const double firstLayerOffset = 3.0;
const unsigned int smoothingIterations = 5;
const double smoothingWeight = 0.8;
const double secondLayerStepWidth = 5;
const double secondLayerStepHeight = 15;
const bool secondLayerSideSubdivision = false;
const bool rotateResults = true;
const bool xDirectionsAfter = true;


void FAFPipeline::scaleAndStock(
		FourAxisFabrication::Data& data,
		bool scaleModel,
		double modelLength,
		double stockLength,
		double stockDiameter)
{
	FourAxisFabrication::centerAndScale(
				data,
				scaleModel,
				modelLength);

	FourAxisFabrication::generateStock(
				data,
				stockLength,
				stockDiameter);
}

void FAFPipeline::saliency(
		FourAxisFabrication::Data& data)
{
	FourAxisFabrication::findDetails(
				data,
				unitScale,
				nRing,
				nScales,
				eps,
				computeBySaliency,
				maxIterations,
				laplacianIterations);
}

void FAFPipeline::smoothing(
		FourAxisFabrication::Data& data,
		unsigned int iterations)
{
	FourAxisFabrication::smoothing(
				data,
				iterations,
				lambda,
				mu);
	data.isMeshSmoothed = true;
}

void FAFPipeline::optimalOrientation(
		FourAxisFabrication::Data& data,
		double stockLength,
		double stockDiameter,
		unsigned int nOrientations)
{
	data.isMeshOriented = FourAxisFabrication::rotateToOptimalOrientation(
				data.mesh,
				data.smoothedMesh,
				stockLength,
				stockDiameter,
				nOrientations,
				extremeWeight,
				BBWeight,
				deterministic);
	if (!data.isMeshOriented) {
		throw std::runtime_error("Error: model cannot fit on stock!");
	}
}

void FAFPipeline::selectExtremes(
		FourAxisFabrication::Data& data)
{
	FourAxisFabrication::selectExtremesOnXAxis(data.smoothedMesh, heightfieldAngle, data);
	data.areExtremesSelected = true;
}

void FAFPipeline::checkVisibility(
		FourAxisFabrication::Data& data,
		unsigned int nDirections)
{
	FourAxisFabrication::getVisibility(
				data.smoothedMesh,
				nDirections,
				resolution,
				heightfieldAngle,
				includeXDirections,
				data,
				checkMode);
	data.isVisibilityChecked = true;
}

void FAFPipeline::getAssociation(
		FourAxisFabrication::Data& data,
		double detailMultiplier,
		double compactness)
{
	FourAxisFabrication::getAssociation(
				data.smoothedMesh,
				dataSigma,
				detailMultiplier,
				compactness,
				fixExtremes,
				data);
	data.isAssociationComputed = true;
}

void FAFPipeline::optimizeAssociation(
		FourAxisFabrication::Data& data)
{
	FourAxisFabrication::optimization(
				data.smoothedMesh,
				relaxHoles,
				loseHoles,
				minChartArea,
				data);
	data.isAssociationOptimized = true;
}

void FAFPipeline::smoothLines(
		FourAxisFabrication::Data& data)
{
	FourAxisFabrication::smoothLines(
				data.smoothedMesh,
				smoothEdgeLines,
				data);
	data.isLineSmoothed = true;
}

void FAFPipeline::restoreFrequencies(
		FourAxisFabrication::Data& data)
{
	double haussDistance = cg3::libigl::hausdorffDistance(data.mesh, data.smoothedMesh);
	cg3::BoundingBox3 originalMeshBB = data.mesh.boundingBox();
	double haussDistanceBB = haussDistance/originalMeshBB.diag();
	std::cout << "Smoothed -> Haussdorff distance: " << haussDistance << " (w.r.t. bounding box: " << haussDistanceBB << ")" << std::endl;

	FourAxisFabrication::restoreFrequencies(nIterations, heightfieldAngle, data.mesh, data.smoothedMesh, data);
	haussDistance = cg3::libigl::hausdorffDistance(data.mesh, data.restoredMesh);
	originalMeshBB = data.mesh.boundingBox();
	haussDistanceBB = haussDistance/originalMeshBB.diag();
	std::cout << "Restored -> Haussdorff distance: " << haussDistance << " (w.r.t. bounding box: " << haussDistanceBB << ")" << std::endl;

	FourAxisFabrication::recheckVisibilityAfterRestore(recheck, resolution, heightfieldAngle, includeXDirections, reassign, data, checkMode);
	std::cout << "Non-visible triangles after recheck: " << data.restoredMeshNonVisibleFaces.size() << std::endl;
	data.areFrequenciesRestored = true;
}

void FAFPipeline::colorizeAssociation(
		FourAxisFabrication::Data& data)
{
	cg3::Color color;

	int subd = data.directions.size() > 2 ? scatterColorMaxHue / (data.targetDirections.size() - 2) : 2;

	//Set the color
	data.restoredMesh.setFaceColor(defaultColor);

	int minLabel = data.targetDirections[data.targetDirections.size()-2];
	int maxLabel = data.targetDirections[data.targetDirections.size()-1];

	//For each face of the drawable mesh
	for (unsigned int faceId = 0; faceId < data.restoredMesh.numberFaces(); faceId++) {
		//Get direction index associated to the current face
		int associatedDirectionIndex = data.association[faceId];

		//If it has an associated fabrication direction
		if (associatedDirectionIndex >= 0) {
			if (associatedDirectionIndex == minLabel) {
				color = minColor;
			}
			else if (associatedDirectionIndex == maxLabel) {
				color = maxColor;
			}
			else {
				//Find position in target directions to set the color
				auto it =
						std::find(data.targetDirections.begin(), data.targetDirections.end(), associatedDirectionIndex);

				int positionInTargetDirections = std::distance(data.targetDirections.begin(), it);

				color.setHsv(subd * positionInTargetDirections, scatterColorSat, scatterColorVal);
			}

			//Set the color
			data.restoredMesh.setFaceColor(color, faceId);
		}
	}
}

void FAFPipeline::cutComponents(
		FourAxisFabrication::Data& data)
{
	FourAxisFabrication::cutComponents(data, ::cutComponents);
	data.areComponentsCut = true;
}

void FAFPipeline::extractResults(
		FourAxisFabrication::Data& data,
		double firstLayerAngle,
		bool minFirst,
		double stockLength,
		double stockDiameter)
{
	firstLayerAngle = firstLayerAngle  / 180.0 * M_PI;

	FourAxisFabrication::extractResults(
				data,
				stockLength,
				stockDiameter,
				firstLayerAngle,
				firstLayerOffset,
				smoothingIterations,
				smoothingWeight,
				secondLayerStepWidth,
				secondLayerStepHeight,
				secondLayerSideSubdivision,
				heightfieldAngle,
				xDirectionsAfter,
				minFirst,
				rotateResults);
	data.areResultsExtracted = true;
}

void FAFPipeline::pipeline(
		FourAxisFabrication::Data& data,
		const FAFParameters& params)
{
	scaleAndStock(data, params.scaleModel, params.modelLength, params.stockLength, params.stockDiameter);
	saliency(data);
	smoothing(data, params.smoothIterations);
	optimalOrientation(data, params.stockLength, params.stockDiameter, params.nOrientations);
	selectExtremes(data);
	checkVisibility(data, params.nVisibilityDirections);
	getAssociation(data, params.detailMultiplier, params.compactness);
	optimizeAssociation(data);
	smoothLines(data);
	restoreFrequencies(data);
	colorizeAssociation(data);
	cutComponents(data);
	extractResults(data, params.firstLayerAngle, params.minFirst, params.stockLength, params.stockDiameter);
}
