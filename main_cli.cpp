/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#include <iostream>
#include <cg3/utilities/command_line_argument_manager.h>

#include <methods/faf/faf_data.h>

const std::string intro =
"This is the command line tool for the paper 'Automatic surface segmentation \n"
"for seamless fabrication using 4-axis milling machines'.";

const std::string help =
"Usage: \n"
"./fourAxisMilling --input=file.obj [--output=outDirectory] [parameters]";

FourAxisFabrication::Data getDataFromArguments(const cg3::CommandLineArgumentManager& clArguments);

int main(int argc, char *argv[]) {
	cg3::CommandLineArgumentManager clArguments(argc, argv);

	for (std::pair<std::string, std::string> p : clArguments){
		std::cerr << "Argument: " << p.first << "; Value: " << p.second << "\n";
	}

	FourAxisFabrication::Data data;
	try {
		data = getDataFromArguments(clArguments);
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

	//get the output dir and save
	std::string outputDir; //meant to be left empty if no outpurDir argument was given
	if (clArguments.exists("o")){
		outputDir = clArguments["o"];
	}
	if (clArguments.exists("output")){
		outputDir = clArguments["output"];
	}

	return 0;
}

/**
 * Fills all the data with the input parameters or default values.
 * Throws a std::runtime_error if some data is missing or invalid.
 */
FourAxisFabrication::Data getDataFromArguments(
		const cg3::CommandLineArgumentManager& clArguments)
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

	bool loadOk = data.originalMesh.loadFromFile(inputFile);
	if (!loadOk){
		throw std::runtime_error(
			"Error: impossible to load input file.\n"
			"Known input formats: OBJ, PLY.");
	}


	return data;
}

