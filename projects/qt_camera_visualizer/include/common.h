#pragma once

#include <pcl/features/normal_3d.h>
#include <pcl/common/distances.h>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <limits>
#include <vector>
#include <QMainWindow>
#include <exception>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <limits>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/bilateral.h>
#include <fstream>


#define MY_POINT_TYPE pcl::PointXYZ
#define MY_POINT_NORMAL_TYPE
#define MY_POINT_CLOUD pcl::PointCloud<MY_POINT_TYPE>
#define DEBUG_MODE 1
#define CAMERA_MODE 1

MY_POINT_CLOUD::Ptr statisticalRemoveOutliers(MY_POINT_CLOUD::Ptr cloud_ptr);
MY_POINT_CLOUD::Ptr fastBilateralFilter(MY_POINT_CLOUD::Ptr cloud_ptr);

/*
	Tipo dato PoligonMesh
	Visualizador MeshLab
	Visualizar Poligonos en visualizador
	Poisson
	borrar inicializacion de visualizadores hasta que no se vallan a usar
	bilateral smoozing filtro. suaviza las normales asi no habrá tantos outliners

		Necesito q la nube de puntos sea organizada para bilateral, pero remove NAN me la deja desorganizada

	cierre (dilatacion-erosion)
	guardar secuencias de dilataciones (para la cerradura (dilatacion - erosion))
	Dataset modelnet 10
*/