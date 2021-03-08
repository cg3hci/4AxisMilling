#ifndef FAF_PARAMETERS_H
#define FAF_PARAMETERS_H

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
};

#endif // FAF_PARAMETERS_H
