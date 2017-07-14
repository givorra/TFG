#pragma once

// Librerias PCL
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

// Librerias de terceros
#include "string.h"

using namespace std;

// Se define aqui el tipo de punto y PC de ese tipo para poder cambiarlo en cualquier momento
#define MY_POINT_TYPE pcl::PointXYZ						
#define MY_POINT_CLOUD pcl::PointCloud<MY_POINT_TYPE>
#define DEBUG_MODE 0

namespace Util
{
	// Sirve tanto para ficheros como directorios
	bool checkPathExist(const string& pPath);
	string getFileNameFromPath(string pPath);
	bool loadPointCloud(const string& pFileName, MY_POINT_CLOUD::Ptr pCloudOut);
	bool isNumber(const string& pString);
	string numberToString(double pNumber);
}