#pragma once

// Librerias propias
#include "util.h"

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
	int maxIterations;
	float boxCountingInitialSizeDivisor;
	float incrementBoxCountingSizeDivisor;

	MY_POINT_CLOUD::Ptr inputCloudPtr;
	MY_POINT_CLOUD::Ptr inputLogLogCloudPtr;

	std::vector<std::pair<float, float> > boxCounting(string pResultsFile);
	std::vector<std::pair<float, float> > computeBoxCounting(float pMaxVoxelSize, float pMinVoxelSize);
	MY_POINT_CLOUD::Ptr cloudToOriginCoordinates(MY_POINT_CLOUD::Ptr pCloudPtr);
	std::vector<std::pair<float, float> > getLogLogVector(std::vector<std::pair<float, float> > xy_pts);
	void saveResults(string fresults, const std::vector<std::pair<float, float> > &xy_pts);
	float meanNearestNeighbors(MY_POINT_CLOUD::Ptr pCloudPtr);

public:
	BoxCounting();

	// Funcionalidad
	//std::vector<std::pair<string, std::vector<std::pair<float, float> > > > boxCountingDirectory(const string& pDirName);
	std::vector<std::pair<float, float> > boxCountingFile(const string& pFileName);
	//void boxCountingSubDirectories(const string& pDirName);

	// Setters
	void setInputCloud(MY_POINT_CLOUD::Ptr pCloud);

	// Getters
	MY_POINT_CLOUD::Ptr getInputCloud();
};