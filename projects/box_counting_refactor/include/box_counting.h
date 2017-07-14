#pragma once

// Librerias propias
#include "util.h"
#include "linear_regression.h"

// Librerias PCL
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

// Otras librerias
#include <boost/tuple/tuple.hpp>
#include <vector>
#include "string.h"

using namespace std;

class BoxCounting
{
private:
	int _maxIterations;
	float _boxCountingInitialSizeDivisor;
	float _incrementBoxCountingSizeDivisor;

	MY_POINT_CLOUD::Ptr _inputCloudPtr;

	// Datos obtenidos
	vector<pair<float, float> > _xyPts;		// Puntos obtenidos en el conteo de cajas
	vector<pair<float, float> > _xyLogPts;	// Vector log log de _xyPts
	LinearRegression _linearRegression;

	MY_POINT_CLOUD::Ptr cloudToOriginCoordinates(MY_POINT_CLOUD::Ptr pCloudPtr);
	vector<pair<float, float> > getLogLogVector(vector<pair<float, float> > pXYPts);
	float meanNearestNeighbors(MY_POINT_CLOUD::Ptr pCloudPtr);

public:
	BoxCounting();

	bool compute();
	void plotLinearRegression(string pFileName);

	// Getters
	float getBoxCountingInitialSizeDivisor();
	float getFractalDimension();
	float getIncrementBoxCountingSizeDivisor();
	MY_POINT_CLOUD::Ptr getInputCloud();
	float getErrorLinearRegression();
	int getMaxIterations();

	// Setters
	void setBoxCountingInitialSizeDivisor(const float& pBoxCountingInitialSizeDivisor);
	void setIncrementBoxCountingSizeDivisor(const float& pIncrementBoxCountingSizeDivisor);
	void setInputCloud(const MY_POINT_CLOUD::Ptr pCloud);
	void setMaxIterations(const int& pMaxIterations);

};