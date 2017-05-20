#include "MathMorph.h"
#include <pcl/features/normal_3d.h>
#include <pcl/common/distances.h>

MathMorph::MathMorph(double sizei, double leafSizei)
{
    size=sizei;
    leafSize=leafSizei;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
}

pcl::PointCloud<PointType>::Ptr 
MathMorph::dilate2 (pcl::PointCloud<PointType>::Ptr cloud_ptr) 
{
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
    pcl::PointCloud<PointType>::Ptr cloud_out (new pcl::PointCloud<PointType>);
    PointType point;
    

    computeNormals(cloud_ptr);
    // Dilate the figure
    for (int i=0; i<cloud_ptr->size(); i++) {
        PointType point;
        point.x=cloud_ptr->points[i].x - size*cloud_normals->points[i].normal[0];
        point.y=cloud_ptr->points[i].y - size*cloud_normals->points[i].normal[1];
        point.z=cloud_ptr->points[i].z - size*cloud_normals->points[i].normal[2];
        cloud_out->points.push_back(point);
    }
    return cloud_out;
}

bool
MathMorph::isInRange (PointType point, PointType searchPoint) {
    return (fabs(point.x-searchPoint.x)<=leafSize/2 && fabs(point.y-searchPoint.y)<=leafSize/2 && fabs(point.z-searchPoint.z)<=leafSize/2);
}


pcl::PointCloud<PointType>::Ptr 
MathMorph::dilate (pcl::PointCloud<PointType>::Ptr cloud_ptr) 
{
    pcl::PointCloud<PointType>::Ptr cloud_out (new pcl::PointCloud<PointType>);
    pcl::KdTreeFLANN<PointType> kdtree;
    PointType point, minPt, maxPt, searchPoint;
    double cosine;
    int xRange, yRange, zRange, cont=0, cont2=0;
    bool found;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    Eigen::Vector4f p, q;

    // Compute normals to dilate
    computeNormals(cloud_ptr);

    // Calculate the bounding box (the size of the structured element is added)
    pcl::getMinMax3D (*cloud_ptr, minPt, maxPt);
    xRange = (maxPt.x+2*size-minPt.x)/leafSize;
    yRange = (maxPt.y+2*size-minPt.y)/leafSize;
    zRange = (maxPt.z+2*size-minPt.z)/leafSize;

    kdtree.setInputCloud (cloud_ptr);

    for (int x=0; x<=xRange; x++) {
        // Beginning with the lowest X and the leafside/2 is necessary to center the point in the voxel
        searchPoint.x=x*leafSize+(minPt.x-size)+leafSize/2;
        for (int y=0; y<=yRange+1; y++) {
            searchPoint.y=y*leafSize+(minPt.y-size)+leafSize/2; 
            for (int z=0; z<=zRange+1; z++) { 
                searchPoint.z=z*leafSize+(minPt.z-size)+leafSize/2; 
                if (kdtree.radiusSearch (searchPoint, size+2*leafSize, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
                    for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
                        PointType point;
                        point.x=cloud_ptr->points[pointIdxRadiusSearch[i]].x - size*cloud_normals->points[pointIdxRadiusSearch[i]].normal[0];
                        point.y=cloud_ptr->points[pointIdxRadiusSearch[i]].y - size*cloud_normals->points[pointIdxRadiusSearch[i]].normal[1];
                        point.z=cloud_ptr->points[pointIdxRadiusSearch[i]].z - size*cloud_normals->points[pointIdxRadiusSearch[i]].normal[2];
                        if (isInRange(point, searchPoint)) {
                            found=false;
                            cont2++;
                            for (int j=0; j<pointIdxRadiusSearch.size(); j++) {
                                if (j==i) continue;
                                p = cloud_normals->points[pointIdxRadiusSearch[j]].getNormalVector4fMap (); 
                                q = cloud_normals->points[pointIdxRadiusSearch[i]].getNormalVector4fMap (); 
                                cosine = p.dot(q);
                                if (cosine<0) { 
                                    cont++; 
                                    //cout << "cosine=" << cosine << " " <<p << endl << " " << q << endl;
                                    
                                }
                            }
                            if (!found) {
                                point.x=searchPoint.x;
                                point.y=searchPoint.y;
                                point.z=searchPoint.z;
                                cloud_out->points.push_back(point);
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
    cerr << cont << " " << cont2 <<endl;
    return cloud_out;
}


void
MathMorph::computeNormals(pcl::PointCloud<PointType>::Ptr cloud_ptr) 
{
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType> ());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    double centroid_x=0.0, centroid_y=0.0, centroid_z=0.0;
    // Compute normals
    
#if CAMERA_MODE != 1
    // Calculate centroid. This is made because normals must be calculated from inside the object. With 
    // a RGBD camera, it is not necessary
    // TODO: with pcl methods
    for (int i=0; i<cloud_ptr->size(); i++) {
        centroid_x += cloud_ptr->points[i].x;
        centroid_y += cloud_ptr->points[i].y;
        centroid_z += cloud_ptr->points[i].z;
    }
    centroid_x /= cloud_ptr->size();
    centroid_y /= cloud_ptr->size();
    centroid_z /= cloud_ptr->size();
#endif
    // Compute normals
    normal_estimation.setViewPoint(centroid_x, centroid_y, centroid_z);
    normal_estimation.setInputCloud (cloud_ptr);
    normal_estimation.setSearchMethod (tree);
    normal_estimation.setRadiusSearch (NORMAL_RADIUS_SEARCH);
    normal_estimation.compute(*cloud_normals);
    this->cloud_normals = cloud_normals;
}

/*
pcl::PointCloud<pcl::Normal>::Ptr
MathMorph::findNormals (pcl::PointCloud<PointType>::Ptr cloud_ptr) 
{
    pcl::NormalEstimation<PointType, pcl::Normal> normal_estimation;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());

    // Compute normals
    normal_estimation.setInputCloud(cloud_ptr);
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setRadiusSearch(NORMAL_RADIUS_SEARCH);
    normal_estimation.compute(*cloud_normals);
    return cloud_normals;
}


pcl::PointCloud<PointType>::Ptr 
MathMorph::dilateRGB (pcl::PointCloud<PointType>::Ptr cloud_ptr) {
    pcl::NormalEstimation<PointType, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointType>::Ptr cloud_out (new pcl::PointCloud<PointType>);
    
    // normals Compute
    normal_estimation.useSensorOriginAsViewPoint ();
    normal_estimation.setInputCloud (cloud_ptr);
    normal_estimation.setSearchMethod (tree);
    normal_estimation.setRadiusSearch (0.005);
    normal_estimation.compute (*cloud_normals);
    
    
}*/