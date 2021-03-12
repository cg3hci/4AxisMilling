#include "faf_pipeline.h"

#include <cg3/libigl/mesh_distance.h>
#include <cg3/utilities/timer.h>

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
	std::cout << "Scale and stock generation...\n";
	cg3::Timer t(std::string("Scale and stock generation"));
	FourAxisFabrication::centerAndScale(
				data,
				scaleModel,
				modelLength);

	FourAxisFabrication::generateStock(
				data,
				stockLength,
				stockDiameter);
	t.stopAndPrint();
	data.isMeshScaledAndStockGenerated = true;
}

void FAFPipeline::saliency(
		FourAxisFabrication::Data& data)
{
	std::cout << "Saliency details...\n";
	cg3::Timer t(std::string("Saliency details"));
	FourAxisFabrication::findDetails(
				data,
				unitScale,
				nRing,
				nScales,
				eps,
				computeBySaliency,
				maxIterations,
				laplacianIterations);
	t.stopAndPrint();
	data.isSaliencyComputed = true;
}

void FAFPipeline::smoothing(
		FourAxisFabrication::Data& data,
		unsigned int iterations)
{
	std::cout << "Prefiltering...\n";
	cg3::Timer t(std::string("Prefiltering"));
	FourAxisFabrication::smoothing(
				data,
				iterations,
				lambda,
				mu);
	t.stopAndPrint();
	data.isMeshSmoothed = true;
}

void FAFPipeline::optimalOrientation(
		FourAxisFabrication::Data& data,
		double stockLength,
		double stockDiameter,
		unsigned int nOrientations)
{
	std::cout << "Finding best axis...\n";
	cg3::Timer t(std::string("Finding best axis"));
	data.isMeshOriented = FourAxisFabrication::rotateToOptimalOrientation(
				data.mesh,
				data.smoothedMesh,
				stockLength,
				stockDiameter,
				nOrientations,
				extremeWeight,
				BBWeight,
				deterministic);
	t.stopAndPrint();
	if (!data.isMeshOriented) {
		throw std::runtime_error("Error: model cannot fit on stock!");
	}
}

void FAFPipeline::selectExtremes(
		FourAxisFabrication::Data& data)
{
	std::cout << "Select extremes...\n";
	cg3::Timer t(std::string("Select extremes"));
	FourAxisFabrication::selectExtremesOnXAxis(data.smoothedMesh, heightfieldAngle, data);
	t.stopAndPrint();
	data.areExtremesSelected = true;
}

void FAFPipeline::checkVisibility(
		FourAxisFabrication::Data& data,
		unsigned int nDirections)
{
	std::cout << "Visibility check...\n";
	cg3::Timer t(std::string("Visibility check"));
	FourAxisFabrication::getVisibility(
				data.smoothedMesh,
				nDirections,
				resolution,
				heightfieldAngle,
				includeXDirections,
				data,
				checkMode);
	t.stopAndPrint();
	data.isVisibilityChecked = true;
	std::cout << "Non-visible triangles: " << data.nonVisibleFaces.size() << std::endl;
}

void FAFPipeline::getAssociation(
		FourAxisFabrication::Data& data,
		double detailMultiplier,
		double compactness)
{
	std::cout << "Computing Segmentation...\n";
	cg3::Timer t(std::string("Computing Segmentation"));
	FourAxisFabrication::getAssociation(
				data.smoothedMesh,
				dataSigma,
				detailMultiplier,
				compactness,
				fixExtremes,
				data);
	t.stopAndPrint();
	data.isAssociationComputed = true;
}

void FAFPipeline::optimizeAssociation(
		FourAxisFabrication::Data& data)
{
	std::cout << "Charts optimization...\n";
	cg3::Timer t(std::string("Charts optimization"));
	FourAxisFabrication::optimization(
				data.smoothedMesh,
				relaxHoles,
				loseHoles,
				minChartArea,
				data);
	t.stopAndPrint();
	data.isAssociationOptimized = true;
}

void FAFPipeline::smoothLines(
		FourAxisFabrication::Data& data)
{
	std::cout << "Boundary smoothing...\n";
	cg3::Timer t(std::string("Boundary smoothing"));
	FourAxisFabrication::smoothLines(
				data.smoothedMesh,
				smoothEdgeLines,
				data);
	t.stopAndPrint();
	data.isLineSmoothed = true;
}

void FAFPipeline::restoreFrequencies(
		FourAxisFabrication::Data& data)
{
	double haussDistance = cg3::libigl::hausdorffDistance(data.mesh, data.smoothedMesh);
	cg3::BoundingBox3 originalMeshBB = data.mesh.boundingBox();
	double haussDistanceBB = haussDistance/originalMeshBB.diag();
	std::cout << "Smoothed -> Haussdorff distance: " << haussDistance << " (w.r.t. bounding box: " << haussDistanceBB << ")" << std::endl;

	std::cout << "Detail recovery...\n";
	cg3::Timer t(std::string("Detail recovery"));
	FourAxisFabrication::restoreFrequencies(nIterations, heightfieldAngle, data.mesh, data.smoothedMesh, data);
	t.stopAndPrint();

	haussDistance = cg3::libigl::hausdorffDistance(data.mesh, data.restoredMesh);
	originalMeshBB = data.mesh.boundingBox();
	haussDistanceBB = haussDistance/originalMeshBB.diag();
	std::cout << "Restored -> Haussdorff distance: " << haussDistance << " (w.r.t. bounding box: " << haussDistanceBB << ")" << std::endl;

	cg3::Timer tCheck("Recheck visibility after detail recovery");
	FourAxisFabrication::recheckVisibilityAfterRestore(recheck, resolution, heightfieldAngle, includeXDirections, reassign, data, checkMode);
	tCheck.stopAndPrint();
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

	std::cout << "Generating the fabrication sequence...\n";
	cg3::Timer t(std::string("Generating the fabrication sequence"));
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
	t.stopAndPrint();
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
	if (!params.justSegmentation){
		cutComponents(data);
		extractResults(data, params.firstLayerAngle, params.minFirst, params.stockLength, params.stockDiameter);
	}
}
