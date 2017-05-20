#include "common.h"

//using namespace std;

MY_POINT_CLOUD::Ptr statisticalRemoveOutliers(MY_POINT_CLOUD::Ptr cloud_ptr)
{
    MY_POINT_CLOUD::Ptr cloud_out(new MY_POINT_CLOUD());
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<MY_POINT_TYPE> sor;
    sor.setInputCloud (cloud_ptr);
    sor.setMeanK (10);
    sor.setStddevMulThresh (2);
    sor.filter (*cloud_out);

    return cloud_out;
}

MY_POINT_CLOUD::Ptr fastBilateralFilter(MY_POINT_CLOUD::Ptr cloud_ptr)
{
  cout << "Fast Bilateral Filter size of input cloud: " << cloud_ptr->points.size() << "\n";

  MY_POINT_CLOUD::Ptr cloud_out(new MY_POINT_CLOUD());
  pcl::FastBilateralFilter<MY_POINT_TYPE> filter;

  cloud_out->width = cloud_ptr->width;
  cloud_out->height = cloud_ptr->height;

  filter.setInputCloud(cloud_ptr);
  filter.setSigmaS(4);
  filter.setSigmaR(0.1);
  filter.applyFilter(*cloud_out);


  cout << "Fast Bilateral Filter size of output cloud: " << cloud_out->points.size() << ", is organized = " << cloud_out->isOrganized() << "\n";

  return cloud_out;
}