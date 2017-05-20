#pragma once

#include "common.h"

using namespace std;

//#define MY_POINT_TYPE pcl::PointXYZ

// Normal parameters
#define NORMAL_RADIUS_SEARCH 0.005


class MathMorph 
{
private:
        double size;
        double leafSize;
        //pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
        pcl::PointCloud<pcl::Normal>::Ptr computeNormals(MY_POINT_CLOUD::Ptr cloud_ptr);
public:
        MathMorph();
        MY_POINT_CLOUD::Ptr dilate (MY_POINT_CLOUD::Ptr cloud_ptr);
        MY_POINT_CLOUD::Ptr camera_dilate(MY_POINT_CLOUD::Ptr cloud_ptr, double size);
        MY_POINT_CLOUD::Ptr camera_erode(MY_POINT_CLOUD::Ptr cloud_ptr, double size);
        bool isInRange(MY_POINT_TYPE point, MY_POINT_TYPE searchPoint);
        MY_POINT_CLOUD::Ptr removeNan(MY_POINT_CLOUD::Ptr cloud);
};

/*
        Generar nube de puntos de un plano
        Replicar la imagen con color, replicar 
*/