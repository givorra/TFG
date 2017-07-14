#pragma once

#include "linear_regression.h"
#include <vector>
#include <boost/tuple/tuple.hpp>
#include <time.h>

using namespace std;

class Ransac
{
private:
	int _iterations;
	float _maxThresold;
	int _minInliers;
	vector<pair<float, float> > _inputXYPts;
	vector<pair<float, float> > _outputXYPts;

	int getRandomInt(int min, int max);

public:
	Ransac(int pIterations, float pMaxThresold, int pMinInliers);
	bool compute();

	int getIterations();
	float getMaxThresold();
	int getMinInliers();
	vector<pair<float, float> > getInputXYPts();
	vector<pair<float, float> > getOutputXYPts();

	void setIterations(int pIterations);
	void setMaxThresold(float pMaxThresold);
	void setMinInliers(int pMinInliers);
	void setInputXYPts(vector<pair<float, float> > pInputXYPts);

};