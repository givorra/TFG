#pragma once

#include "string.h"
#include <math.h>
#include "gnuplot-iostream.h"
#include "util.h"

using namespace std;

class LinearRegression
{
private:
	float _m;
	float _b;
	float _totalError;
	vector<pair<float, float> > _xyPts;

	void errorLinearRegression();

public:
	LinearRegression();
	bool compute();
	void plot(string pFileName);

	// Getters
	float getB();
	float getError();
	float getM();
	float getMeanError();

	// Setters
	void setXYPts(const vector<pair<float, float> >& pXYPts);
};