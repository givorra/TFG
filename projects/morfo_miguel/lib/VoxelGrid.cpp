#include "VoxelGrid.h"

VoxelGrid::VoxelGrid()
{
	leafSize = 0;
	voxelGridSize = 0;
}

void VoxelGrid::setLeafSize(double leafSize)
{
	this->leafSize = leafSize;
}

double VoxelGrid::getLeafSize()
{
	return leafSize;
}

void VoxelGrid::setVoxelGridSize(double voxelGridSize)
{
	this->voxelGridSize = voxelGridSize;
}

double VoxelGrid::getVoxelGridSize()
{
	return voxelGridSize;
}

Cube VoxelGrid::filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
	// Tamaño en voxels del lado del voxel grid
	int n = (int)(voxelGridSize/leafSize);

	// Cálculo de la posición de la nube de puntos
	// usado para saber la zona que ocupa y hacer
	// voxel grid en esa zona
	double Xmax = -std::numeric_limits<double>::max();
	double Ymax = -std::numeric_limits<double>::max();
	double Zmax = -std::numeric_limits<double>::max();
	double Xmin = std::numeric_limits<double>::max();
	double Ymin = std::numeric_limits<double>::max();
	double Zmin = std::numeric_limits<double>::max();
	for(int i = 0; i < cloud_ptr->size(); i++)
	{
		if(cloud_ptr->points[i].x < Xmin)
			Xmin = cloud_ptr->points[i].x;
		if(cloud_ptr->points[i].y < Ymin)
			Ymin = cloud_ptr->points[i].y;
		if(cloud_ptr->points[i].z < Zmin)
			Zmin = cloud_ptr->points[i].z;
		if(cloud_ptr->points[i].x > Xmax)
			Xmax = cloud_ptr->points[i].x;
		if(cloud_ptr->points[i].y > Ymax)
			Ymax = cloud_ptr->points[i].y;
		if(cloud_ptr->points[i].z > Zmax)
			Zmax = cloud_ptr->points[i].z;
	}

	double oX = Xmin;
	double oY = Ymin;
	double oZ = Zmin;
	int iM = 0;
	int jM = 0;
	int kM = 0;

	double maxDensity = -1 * numeric_limits<double>::max();

	Cube densityCube(n);

	for(double i = oX ; i < oX+getVoxelGridSize(); i = i+getLeafSize())
	{
		for(double j = oY; j < oY+getVoxelGridSize(); j = j+getLeafSize())
		{
			for(double k = oZ; k < oZ+getVoxelGridSize(); k = k+getLeafSize())
			{
				for(int l = 0; l < cloud_ptr->points.size(); l++)
				{
					if( ( cloud_ptr->points[l].x > i && cloud_ptr->points[l].x < i+getLeafSize() ) && // dentro de X
						( cloud_ptr->points[l].y > j && cloud_ptr->points[l].y < j+getLeafSize() ) && // dentro de y
						( cloud_ptr->points[l].z > k && cloud_ptr->points[l].z < k+getLeafSize() ) // dentro de Z
					  )
					{
						densityCube.set(iM,jM,kM, 1+densityCube.get(iM,jM,kM));
						//densityCube[iM][jM][kM] += 1;
					}

					if ( densityCube.get(iM,jM,kM) > maxDensity)
					{
						//maxDensity = densityCube[iM][jM][kM].size();
						maxDensity = densityCube.get(iM,jM,kM);
					}
				}
				kM++;
			}
			kM=0;
			jM++;
		}
		jM=0;
		iM++;
	}

	densityCube.setMaxDensity(maxDensity);

	return densityCube;
}