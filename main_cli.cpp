/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#include <iostream>
#include <cg3/utilities/command_line_argument_manager.h>

int main(int argc, char *argv[]) {
	
	cg3::CommandLineArgumentManager clArguments(argc, argv);
	
	for (std::pair<std::string, std::string> p : clArguments){
		std::cerr << "Argument: " << p.first << "; Value: " << p.second << "\n";
	}
	
	return 0;
}
