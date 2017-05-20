
#ifndef VOXELGRID_H
#define VOXELGRID_H

#include <pcl/point_types.h>
#include "../lib/Cube.cpp"

using namespace std;

//
// Esta clase realiza el voxel grid y devuelve la densidad de puntos en cada voxel.
//
class VoxelGrid {

	private:
		// Tamaño del voxel
	    double leafSize;
	    // Tamaño del voxel grid
	    double voxelGridSize;

	public:
		VoxelGrid();
		void setLeafSize(double leafSize);
		double getLeafSize();
		void setVoxelGridSize(double voxelGridSize);
		double getVoxelGridSize();
		Cube filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
	    
};

#endif
