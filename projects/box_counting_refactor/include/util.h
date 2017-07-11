#pragma once

// Librerias PCL
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

// Librerias de terceros
#include <math.h>
#include "string.h"
#include "gnuplot-iostream.h"

using namespace std;

// Se define aqui el tipo de punto y PC de ese tipo para poder cambiarlo en cualquier momento
#define MY_POINT_TYPE pcl::PointXYZ						
#define MY_POINT_CLOUD pcl::PointCloud<MY_POINT_TYPE>
#define DEBUG_MODE 0

// Sirve tanto para ficheros como directorios
bool checkPathExist(const string& pPath);
string getFileNameFromPath(string path);
int linearRegression(std::vector<std::pair<float, float> > xy_pts, float& m, float& b, float& r);
bool loadPointCloud(const string& fileName, MY_POINT_CLOUD::Ptr cloud_out);
bool isNumber(const string& pString);
void plotXYgraph(string filename, std::vector<std::pair<float, float> > xy_pts);
void plotLinearRegression(string filename, std::vector<std::pair<float, float> > xy_pts, const float& m, const float& b);
