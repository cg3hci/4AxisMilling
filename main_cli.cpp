/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#include <iostream>
#include <filesystem>
#include <cg3/utilities/command_line_argument_manager.h>

#include <methods/faf/faf_data.h>

#include "faf_pipeline.h"

//strings used for intro and help
const std::string intro =
"This is the command line tool for the paper 'Automatic surface segmentation \n"
"for seamless fabrication using 4-axis milling machines'.";

const std::string help =
"Usage: \n"
"./fourAxisMilling --input=file.obj [--output=outDirectory] [parameters]";

FourAxisFabrication::Data getDataFromArguments(
		const cg3::CommandLineArgumentManager& clArguments,
		FAFParameters& params);

int main(int argc, char *argv[]) {
	cg3::CommandLineArgumentManager clArguments(argc, argv);

	FourAxisFabrication::Data data;
	FAFParameters params;
	try {
		data = getDataFromArguments(clArguments, params);
	}
	catch (const std::runtime_error& e) {
		if (std::string(e.what()) == std::string("ok")) {
			return 0;
		}
		else {
			std::cerr << e.what();
			return -1;
		}
	}

	params.print();

	//get the output dir and save
	std::string outputDir; //meant to be left empty if no outpurDir argument was given
	if (clArguments.exists("o")){
		outputDir = clArguments["o"];
	}
	if (clArguments.exists("output")){
		outputDir = clArguments["output"];
	}

	if (outputDir != ""){
		if (!std::filesystem::exists(outputDir)) {
			bool res = std::filesystem::create_directory(outputDir);
			if (!res)
				throw std::runtime_error("Cannot create " + outputDir);
		}
	}

	//run the algorithm...
	FAFPipeline::pipeline(data, params);

	data.restoredMesh.saveOnPly(outputDir + "/segmentation.ply");
	if (!params.justSegmentation) {
		unsigned int i = 0;
		for (const cg3::EigenMesh& s : data.stocks){
			s.saveOnPly(outputDir + "/stock_" + std::to_string(i++) + ".ply");
		}
		i = 0;
		for (const cg3::EigenMesh& s : data.results){
			s.saveOnPly(outputDir + "/result_" + std::to_string(i++) + ".ply");
		}
	}

	return 0;
}

/**
 * Fills all the data with the input parameters or default values.
 * Throws a std::runtime_error if some data is missing or invalid.
 */
FourAxisFabrication::Data getDataFromArguments(
		const cg3::CommandLineArgumentManager& clArguments,
		FAFParameters& params)
{
	FourAxisFabrication::Data data;
	if (clArguments.size() == 0 || clArguments.exists("h") || clArguments.exists("help")) {
		std::cout << intro << help;
		throw std::runtime_error("ok");
	}

	std::string inputFile;
	if (clArguments.exists("i")){
		inputFile = clArguments["i"];
	}
	if (clArguments.exists("input")){
		inputFile = clArguments["input"];
	}
	if (inputFile.empty()){
		throw std::runtime_error("Error: Input file not specified.\n" + help);
	}

	data.isMeshLoaded = data.originalMesh.loadFromFile(inputFile);
	if (!data.isMeshLoaded){
		throw std::runtime_error(
			"Error: impossible to load input file.\n"
			"Known input formats: OBJ, PLY.");
	}

	data.mesh = data.originalMesh;

	//manage other parameters
	const std::array<std::string, 12> strParams = {
		"model_height",
		"stock_length",
		"stock_diameter",
		"dont_scale_model",
		"n_smooth_iterations",
		"n_best_axis_dirs",
		"n_visibility_dirs",
		"saliency_factor",
		"compactness_term",
		"wall_angle",
		"max_first",
		"just_segmentation"
	};

	if (clArguments.exists(strParams[0])){
		params.modelLength = std::stod(clArguments[strParams[0]]);
	}
	if (clArguments.exists(strParams[1])){
		params.stockLength = std::stod(clArguments[strParams[1]]);
	}
	if (clArguments.exists(strParams[2])){
		params.stockDiameter = std::stod(clArguments[strParams[2]]);
	}
	if (clArguments.exists(strParams[3])){
		params.scaleModel = false;
	}
	if (clArguments.exists(strParams[4])){
		params.smoothIterations = std::stoi(clArguments[strParams[4]]);
	}
	if (clArguments.exists(strParams[5])){
		params.nOrientations = std::stoi(clArguments[strParams[5]]);
	}
	if (clArguments.exists(strParams[6])){
		params.nVisibilityDirections = std::stoi(clArguments[strParams[6]]);
	}
	if (clArguments.exists(strParams[7])){
		params.detailMultiplier = std::stod(clArguments[strParams[7]]);
	}
	if (clArguments.exists(strParams[8])){
		params.compactness = std::stod(clArguments[strParams[8]]);
	}
	if (clArguments.exists(strParams[9])){
		params.firstLayerAngle = std::stod(clArguments[strParams[9]]);
	}
	if (clArguments.exists(strParams[10])){
		params.minFirst = false;
	}
	if (clArguments.exists(strParams[10])){
		params.justSegmentation = true;
	}

	return data;
}

