/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#include <iostream>
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

	for (std::pair<std::string, std::string> p : clArguments){
		std::cerr << "Argument: " << p.first << "; Value: " << p.second << "\n";
	}

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

	//run the algorithm...
	FAFPipeline::pipeline(data, params);

	//get the output dir and save
	std::string outputDir; //meant to be left empty if no outpurDir argument was given
	if (clArguments.exists("o")){
		outputDir = clArguments["o"];
	}
	if (clArguments.exists("output")){
		outputDir = clArguments["output"];
	}

	data.fourAxisComponent.saveOnObj(outputDir + "/segmentation.obj");
	unsigned int i = 0;
	for (const cg3::EigenMesh& s : data.stocks){
		s.saveOnObj(outputDir + "/stock_" + std::to_string(i++) + ".obj");
	}
	i = 0;
	for (const cg3::EigenMesh& s : data.results){
		s.saveOnObj(outputDir + "/result_" + std::to_string(i++) + ".obj");
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
	const std::array<std::string, 13> str_params = {
		"model_length",
		"stock_length",
		"stock_diameter"
	};

	if (clArguments.exists(str_params[0])){
		params.modelLength = std::stod(clArguments[str_params[0]]);
	}
	if (clArguments.exists(str_params[1])){
		params.stockLength = std::stod(clArguments[str_params[1]]);
	}
	if (clArguments.exists(str_params[2])){
		params.stockDiameter = std::stod(clArguments[str_params[2]]);
	}

	return data;
}

