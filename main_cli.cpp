/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#include <iostream>
#include <cg3/utilities/command_line_argument_manager.h>

#include <methods/faf/faf_data.h>

const char* intro =
"This is the command line tool for the paper 'Automatic surface segmentation \n"
"for seamless fabrication using 4-axis milling machines'.\n";

const char* help =
"Usage: \n"
"./fourAxisMilling --input=file.obj --output=outDirectory [parameters]\n";

int main(int argc, char *argv[]) {
	cg3::CommandLineArgumentManager clArguments(argc, argv);

	for (std::pair<std::string, std::string> p : clArguments){
		std::cerr << "Argument: " << p.first << "; Value: " << p.second << "\n";
	}

	if (clArguments.size() == 0 || clArguments.exists("h") || clArguments.exists("help")) {
		std::cout << intro << help;
		return 0;
	}

	std::string inputFile;
	if (clArguments.exists("i")){
		inputFile = clArguments["i"];
	}
	if (clArguments.exists("input")){
		inputFile = clArguments["input"];
	}

	if (inputFile.empty()){
		std::cerr << "Error: Input file not specified.\n" << help;
	}

	return 0;
}
