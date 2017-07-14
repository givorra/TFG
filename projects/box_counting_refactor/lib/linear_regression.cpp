#include "linear_regression.h"


LinearRegression::LinearRegression()
{
	_m = 0;
	_b = 0;
	_totalError = 0;
}

void LinearRegression::errorLinearRegression()
{
	_totalError = 0;
	// Calcula el sumatorio de la distancia de cada punto a la recta de regresion
	for(int i = 0; i < _xyPts.size(); i++)
	{
		_totalError += abs(_xyPts[i].second - (_m * _xyPts[i].first + _b));
	}
}

bool LinearRegression::compute()
{
	int n = _xyPts.size();
	float sumx=0,sumy=0,sumx2=0,sumy2=0,sumxy=0;
	float sxx,syy,sxy;

	_m = 0;
	_b = 0;
	if (n < 2)
		return false;

	/* Conpute some things we need */
	for (int i=0; i < n; i++)
	{
		sumx += _xyPts[i].first;
		sumy += _xyPts[i].second;
		sumx2 += (_xyPts[i].first * _xyPts[i].first);
		sumy2 += (_xyPts[i].second * _xyPts[i].second);
		sumxy += (_xyPts[i].first * _xyPts[i].second);
	}
	sxx = sumx2 - sumx * sumx / n;
	syy = sumy2 - sumy * sumy / n;
	sxy = sumxy - sumx * sumy / n;

	/*Infinite slope(m), non existant intercept (b)*/
	if (abs(sxx) == 0)
		return false;

	/* Calculate the slope (m) and intercept (b) */
	_m = sxy / sxx;
	_b = sumy / n - (_m) * sumx / n;

	errorLinearRegression();

#if DEBUG_MODE == 1
	cout << "# Results linear regression: m = " << _m << ", b = " << _b << ", error = " << _totalError << "\n";
#endif
	return true;
}

void LinearRegression::plot(string pFileName)
{
	Gnuplot gp;
	float rounded_m, rounded_b;
	string format = "png";
	pFileName = pFileName + "." + format;
	// Redondea valores a 4 decimales
	rounded_m = round(_m * 10000) / 10000;
	rounded_b = round(_b * 10000) / 10000;

	gp << "set terminal " << format << "\n";
	gp << "set fit logfile 'gnoplut_log.txt'\n";
	gp << "set output '"<< pFileName << "'\n";
	gp << "set grid\n";
	gp << "set xlabel 'Log(1/Leaf size)'\n";
	gp << "set ylabel 'Log(N Points)'\n";
	gp << "m = " << _m << "\n";
	gp << "b = " << _b << "\n";
	gp << "f(x) = m*x+b\n";
	gp << "set title '" << Util::getFileNameFromPath(pFileName) << "'\n";
	gp << "plot" << gp.file1d(_xyPts) << "with points title 'P', " 
		<< "f(x) title 'f(x) = " << Util::numberToString(rounded_m) << "x + " << Util::numberToString(rounded_b) << "\n";
}

// Getters
float LinearRegression::getB()
{
	return _b;
}

float LinearRegression::getError()
{
	return _totalError ;
}

float LinearRegression::getM()
{
	return _m;
}

float LinearRegression::getMeanError()
{
	return _totalError / _xyPts.size();
}

// Setters
void LinearRegression::setXYPts(const vector<pair<float, float> >& pXYPts)
{
	_xyPts = pXYPts;
}