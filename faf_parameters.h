#ifndef FAF_PARAMETERS_H
#define FAF_PARAMETERS_H

#include <iostream>

struct FAFParameters {
	bool scaleModel;
	double modelLength;
	double stockLength;
	double stockDiameter;
	unsigned int smoothIterations;
	unsigned int nOrientations;
	unsigned int nVisibilityDirections;
	double detailMultiplier;
	double compactness;
	double firstLayerAngle;
	bool minFirst;

	FAFParameters():
		scaleModel(true),
		modelLength(60.0),
		stockLength(100.0),
		stockDiameter(60.0),
		smoothIterations(500),
		nOrientations(2000),
		nVisibilityDirections(120),
		detailMultiplier(25.0),
		compactness(30.0),
		firstLayerAngle(25.0),
		minFirst(true)
	{
	}

	void print () const
	{
		std::cout << "Scale input mesh to stock: " << (scaleModel ? "true" : "false") << "\n";
		std::cout << "Model length: " << modelLength << "\n";
		std::cout << "Stock length: " << stockLength << "\n";
		std::cout << "Stock diameter: " << stockDiameter << "\n";
		std::cout << "Number smooth iterations: " << smoothIterations << "\n";
		std::cout << "Number of samples for best axis: " << nOrientations << "\n";
		std::cout << "Number of visibility drections to check: " << nVisibilityDirections << "\n";
		std::cout << "Fidelity term: " << detailMultiplier << "\n";
		std::cout << "Smoothness term: " << compactness << "\n";
		std::cout << "Walls angle: " << firstLayerAngle << "\n";
		std::cout << "Use -X as first extreme: " << (minFirst ? "true" : "false") << "\n";
	}
};

#endif // FAF_PARAMETERS_H
