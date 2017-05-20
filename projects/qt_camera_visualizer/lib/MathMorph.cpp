#include "MathMorph.h"

MathMorph::MathMorph()
{}
/*
MY_POINT_CLOUD::Ptr 
MathMorph::camera_dilate(MY_POINT_CLOUD::Ptr cloud_ptr, double size) 
{
    //pcl::search::KdTree<MY_POINT_TYPE>::Ptr tree (new pcl::search::KdTree<MY_POINT_TYPE> ());
    MY_POINT_CLOUD::Ptr cloud_out(new MY_POINT_CLOUD(*cloud_ptr));
    MY_POINT_CLOUD::Ptr cloud_out2(new MY_POINT_CLOUD());
   // MY_POINT_CLOUD::Ptr cloud(new MY_POINT_CLOUD(*currentPointCloud));
    //MY_POINT_TYPE point;
    

    //cout << "Dilate size: " << size << "\n";
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = computeNormals(cloud_out);
    // Dilate the figure
    //if(false)
    for (int i=0; i<cloud_ptr->size(); i++) 
    {
        cout << "Punto original: x=" << cloud_out->points[i].x << ", y=" << cloud_out->points[i].y << ", z=" << cloud_out->points[i].z << endl;
        cout << "Normal: x=" << cloud_normals->points[i].normal[0] << ", y=" << cloud_normals->points[i].normal[1] << ", z=" << cloud_normals->points[i].normal[2] << endl;
        cloud_out->points[i].x = cloud_out->points[i].x + size*cloud_normals->points[i].normal[0];
        cloud_out->points[i].y = cloud_out->points[i].y + size*cloud_normals->points[i].normal[1];
        cloud_out->points[i].z = cloud_out->points[i].z + size*cloud_normals->points[i].normal[2];
        //cout << "A punto x=" << cloud_out->points[i].x << ", y="<<cloud_out->points[i].y<<", z="<<cloud_out->points[i].z<<endl;
    }
    //cloud_out = removeNan(cloud_out);

    pcl::KdTreeFLANN<MY_POINT_TYPE> kdtree;
    kdtree.setInputCloud(cloud_ptr);
    int removedPoints = 0;
    MY_POINT_CLOUD::iterator itCloud;
    cout << "Revisando puntos sobrantes de la dilatacion...\n";
    //for(int i = 0; i < cloud_out->points.size(); i++)
    for(itCloud = cloud_out->points.begin(); itCloud != cloud_out->points.end(); ++itCloud)
    {
        int K = 1;  // Dont modify, the function only contemplated K=1
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        //cout << "punto x=" << cloud_out->points[i].x << ", y="<<cloud_out->points[i].y<<", z="<<cloud_out->points[i].z<<endl;
        //cout << "punto x=" << itCloud->x << ", y="<<itCloud->y<<", z="<<itCloud->z<<endl;
        double maxDistance = 0;
        if (//itCloud->x == std::numeric_limits<float>::quiet_NaN()
            //|| itCloud->y == std::numeric_limits<float>::quiet_NaN()
            //|| itCloud->z == std::numeric_limits<float>::quiet_NaN()
            //pcl_isfinite(itCloud->x)
            //|| pcl_isfinite(itCloud->y)
            //|| pcl_isfinite(itCloud->z)
            itCloud->x == itCloud->x
            && itCloud->y == itCloud->y
            && itCloud->z == itCloud->z)
            //&& kdtree.nearestKSearch(*itCloud, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 && pointNKNSquaredDistance[0] >= size)
        {
            if(kdtree.nearestKSearch(*itCloud, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
            {
                if(pointNKNSquaredDistance[0] > maxDistance)
                    maxDistance = pointNKNSquaredDistance[0];
                cout << "Distancia minima a figura original: " << pointNKNSquaredDistance[0] << endl;
                cout << "Punto origen: x=" << itCloud->x << ", y=" << itCloud->y << ", z=" << itCloud->z << endl;
                cout << "Punto vecino: x=" << cloud_ptr->points[pointIdxNKNSearch[0]].x << ", y=" << cloud_ptr->points[pointIdxNKNSearch[0]].y << ", z=" << cloud_ptr->points[pointIdxNKNSearch[0]].z << endl;
            }
            cout << "Max distance: " << maxDistance << endl;
            MY_POINT_TYPE point(*itCloud);
            /*point.x=itCloud->x;
            point.y=itCloud->y;
            point.z=itCloud->z;
            point.r=itCloud->r;
            point.g=itCloud->g;
            point.b=itCloud->b;
            cloud_out2->points.push_back(point);
            //cloud_out->erase(itCloud);
        }
            //cout << "vecino1\n";
    }
    cout << "Puntos cloud_out: " << cloud_out->size() << "\n";
    cout << "Puntos cloud_out2: " << cloud_out2->size() << "\n";
    return cloud_out2;
}
*/

MY_POINT_CLOUD::Ptr
removeOutliers(MY_POINT_CLOUD::Ptr cloud_ptr)
{
    MY_POINT_CLOUD::Ptr cloud_out(new MY_POINT_CLOUD());
    pcl::RadiusOutlierRemoval<MY_POINT_TYPE> outrem;
    // build the filter
    outrem.setInputCloud(cloud_ptr);
    outrem.setRadiusSearch(0.8);
    outrem.setMinNeighborsInRadius (2);
    // apply filter
    outrem.filter (*cloud_out);

    return cloud_out;
}

MY_POINT_CLOUD::Ptr 
MathMorph::camera_dilate(MY_POINT_CLOUD::Ptr cloud_ptr, double size) 
{
    cout << "Puntos cloud_ptr: " << cloud_ptr->size() << "\n";
/*
    if(cloud_ptr->isOrganized())
    {
        cloud_ptr = fastBilateralFilter(cloud_ptr);
    }

    cloud_out->width =  cloud_ptr->width;  
    cloud_out->height =  cloud_ptr->height; 
    cloud_out->is_dense =  cloud_ptr->is_dense; */
    MY_POINT_CLOUD::Ptr cloud_out(new MY_POINT_CLOUD()); 
    const float nan_point = std::numeric_limits<float>::quiet_NaN(); 
    cout << "Cloud width = " << cloud_ptr->width << ", height = " << cloud_ptr->height << "\n";
    float epsilon = 0.1;  // Error de desplazamiento

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = computeNormals(cloud_ptr);
    cout << "Computed normals" << cloud_ptr->size() << "\n";

    pcl::KdTreeFLANN<MY_POINT_TYPE> kdtree;
    kdtree.setInputCloud(cloud_ptr);
    cout << "KdTree with input cloud" << cloud_ptr->size() << "\n";
    int K = 1;  // Dont modify, the function only contemplated K=1
    // Dilate the figure
    for (int i=0; i<cloud_ptr->size(); i++) 
    {
        MY_POINT_TYPE dilatedPoint(cloud_ptr->points[i]);
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        //cout << "Punto original: x=" << cloud_out->points[i].x << ", y=" << cloud_out->points[i].y << ", z=" << cloud_out->points[i].z << endl;
        //cout << "Normal: x=" << cloud_normals->points[i].normal[0] << ", y=" << cloud_normals->points[i].normal[1] << ", z=" << cloud_normals->points[i].normal[2] << endl;
        dilatedPoint.x += size*cloud_normals->points[i].normal[0];
        dilatedPoint.y += size*cloud_normals->points[i].normal[1];
        dilatedPoint.z += size*cloud_normals->points[i].normal[2];
        
        if (pcl_isfinite (dilatedPoint.x) && 
            pcl_isfinite (dilatedPoint.y) &&
            pcl_isfinite (dilatedPoint.z))
        {
            if(kdtree.nearestKSearch(dilatedPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
            {
                float euclideanDilatedDistance = sqrt(pow(dilatedPoint.x - cloud_ptr->points[i].x, 2) + pow(dilatedPoint.y - cloud_ptr->points[i].y, 2) + pow(dilatedPoint.z - cloud_ptr->points[i].z, 2));
                if(euclideanDilatedDistance > (pointNKNSquaredDistance[0] - epsilon * euclideanDilatedDistance))
                {
                    /*
                    if(cloud_out->points.size() % 10 == 0)
                    {
                        MY_POINT_TYPE dilatedPoint2(cloud_ptr->points[i]);
                        dilatedPoint2.x -= 0.001;
                        cloud_out->points.push_back(dilatedPoint2);
                    }*/
                    cloud_out->points.push_back(dilatedPoint);
                }
                else
                {
                    dilatedPoint.x = dilatedPoint.y = dilatedPoint.z = nan_point;
                    cloud_out->points.push_back(dilatedPoint);
                }
                //cout << "Distancia minima a figura original: " << pointNKNSquaredDistance[0] << endl;
                //cout << "Punto origen: x=" << itCloud->x << ", y=" << itCloud->y << ", z=" << itCloud->z << endl;
                //cout << "Punto vecino: x=" << cloud_ptr->points[pointIdxNKNSearch[0]].x << ", y=" << cloud_ptr->points[pointIdxNKNSearch[0]].y << ", z=" << cloud_ptr->points[pointIdxNKNSearch[0]].z << endl;
            }
            else
            {
                dilatedPoint.x = dilatedPoint.y = dilatedPoint.z = nan_point;
                cloud_out->points.push_back(dilatedPoint);
            }
        }
        else
        {
            cloud_out->points.push_back(dilatedPoint);
        }

    }
    cout << "Dilated cloud\n";
/*
    cout << "Puntos cloud_ptr: " << cloud_ptr->size() << "\n";
    cout << "Puntos cloud_out: " << cloud_out->size() << "\n";
    cloud_out = statisticalRemoveOutliers(cloud_out);
    cout << "Puntos cloud_out inliers: " << cloud_out->size() << "\n";
    int size_cloud_ptr = cloud_ptr->points.size();
    int size_cloud_out = cloud_out->points.size();
    int numberCreatePoints = (size_cloud_ptr - size_cloud_out) * 0.8;
    

    for(int i = 0; i < numberCreatePoints; i++)
    {
        MY_POINT_TYPE pointToAdd(cloud_out->points[size_cloud_out/numberCreatePoints]);
        pointToAdd.x -= 0.001;
        cloud_out->points.push_back(pointToAdd);
    }
    cout << "Puntos cloud_out rellenada: " << cloud_out->size() << "\n";

    
    if(cloud_out->isOrganized())
    {
        cloud_out = fastBilateralFilter(cloud_out);
    }*/

    cout << "Puntos cloud_out: " << cloud_out->size() << "\n";
    return cloud_out;
}

MY_POINT_CLOUD::Ptr 
MathMorph::removeNan(MY_POINT_CLOUD::Ptr cloud)
{
    MY_POINT_CLOUD::Ptr cloud_out(new MY_POINT_CLOUD(*cloud));
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud_out, indices);
    cout << "Puntos cloud: " << cloud->size() << "\n";
    cout << "Puntos cloud_out: " << cloud_out->size() << "\n";

    return cloud_out;
}


MY_POINT_CLOUD::Ptr 
MathMorph::camera_erode(MY_POINT_CLOUD::Ptr cloud_ptr, double size) 
{
     MY_POINT_CLOUD::Ptr cloud_out(new MY_POINT_CLOUD());    
    float epsilon = 0.05;  // Error de desplazamiento

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = computeNormals(cloud_ptr);

    pcl::KdTreeFLANN<MY_POINT_TYPE> kdtree;
    kdtree.setInputCloud(cloud_ptr);
    int K = 1;  // Dont modify, the function only contemplated K=1
    // Dilate the figure
    for (int i=0; i<cloud_ptr->size(); i++) 
    {
        MY_POINT_TYPE dilatedPoint(cloud_ptr->points[i]);
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        //cout << "Punto original: x=" << cloud_out->points[i].x << ", y=" << cloud_out->points[i].y << ", z=" << cloud_out->points[i].z << endl;
        //cout << "Normal: x=" << cloud_normals->points[i].normal[0] << ", y=" << cloud_normals->points[i].normal[1] << ", z=" << cloud_normals->points[i].normal[2] << endl;
        dilatedPoint.x -= size*cloud_normals->points[i].normal[0];
        dilatedPoint.y -= size*cloud_normals->points[i].normal[1];
        dilatedPoint.z -= size*cloud_normals->points[i].normal[2];
        if (dilatedPoint.x == dilatedPoint.x
            && dilatedPoint.y == dilatedPoint.y
            && dilatedPoint.z == dilatedPoint.z)
        {
            if(kdtree.nearestKSearch(dilatedPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
            {
                float euclideanDilatedDistance = sqrt(pow(dilatedPoint.x - cloud_ptr->points[i].x, 2) + pow(dilatedPoint.y - cloud_ptr->points[i].y, 2) + pow(dilatedPoint.z - cloud_ptr->points[i].z, 2));
                if(euclideanDilatedDistance > (pointNKNSquaredDistance[0] - epsilon * euclideanDilatedDistance))
                    cloud_out->points.push_back(dilatedPoint);
                //cout << "Distancia minima a figura original: " << pointNKNSquaredDistance[0] << endl;
                //cout << "Punto origen: x=" << itCloud->x << ", y=" << itCloud->y << ", z=" << itCloud->z << endl;
                //cout << "Punto vecino: x=" << cloud_ptr->points[pointIdxNKNSearch[0]].x << ", y=" << cloud_ptr->points[pointIdxNKNSearch[0]].y << ", z=" << cloud_ptr->points[pointIdxNKNSearch[0]].z << endl;
            }
        }

    }
    cout << "Puntos cloud_ptr: " << cloud_ptr->size() << "\n";
    cout << "Puntos cloud_out: " << cloud_out->size() << "\n";
    cloud_out = statisticalRemoveOutliers(cloud_out);
    cout << "Puntos cloud_out inliers: " << cloud_out->size() << "\n";
    return cloud_out;
}

bool
MathMorph::isInRange (MY_POINT_TYPE point, MY_POINT_TYPE searchPoint) {
    return (fabs(point.x-searchPoint.x)<=leafSize/2 && fabs(point.y-searchPoint.y)<=leafSize/2 && fabs(point.z-searchPoint.z)<=leafSize/2);
}


MY_POINT_CLOUD::Ptr 
MathMorph::dilate (MY_POINT_CLOUD::Ptr cloud_ptr) 
{
    MY_POINT_CLOUD::Ptr cloud_out (new MY_POINT_CLOUD);
    pcl::KdTreeFLANN<MY_POINT_TYPE> kdtree;
    MY_POINT_TYPE point, minPt, maxPt, searchPoint;
    double cosine;
    int xRange, yRange, zRange, cont=0, cont2=0;
    bool found;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    Eigen::Vector4f p, q;

    // Compute normals to dilate
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = computeNormals(cloud_ptr);

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
                        MY_POINT_TYPE point;
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


pcl::PointCloud<pcl::Normal>::Ptr
MathMorph::computeNormals(MY_POINT_CLOUD::Ptr cloud_ptr) 
{
    pcl::search::KdTree<MY_POINT_TYPE>::Ptr tree(new pcl::search::KdTree<MY_POINT_TYPE> ());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<MY_POINT_TYPE, pcl::Normal> normal_estimation;
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
    normal_estimation.setViewPoint(centroid_x, centroid_y, centroid_z);
#endif
    // Compute normals
    normal_estimation.setInputCloud (cloud_ptr);
    normal_estimation.setSearchMethod (tree);
    normal_estimation.setRadiusSearch (NORMAL_RADIUS_SEARCH);
    normal_estimation.compute(*cloud_normals);
    return cloud_normals;
}

/*
pcl::PointCloud<pcl::Normal>::Ptr
MathMorph::findNormals (MY_POINT_CLOUD::Ptr cloud_ptr) 
{
    pcl::NormalEstimation<MY_POINT_TYPE, pcl::Normal> normal_estimation;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<MY_POINT_TYPE>::Ptr tree (new pcl::search::KdTree<MY_POINT_TYPE> ());

    // Compute normals
    normal_estimation.setInputCloud(cloud_ptr);
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setRadiusSearch(NORMAL_RADIUS_SEARCH);
    normal_estimation.compute(*cloud_normals);
    return cloud_normals;
}


MY_POINT_CLOUD::Ptr 
MathMorph::dilateRGB (MY_POINT_CLOUD::Ptr cloud_ptr) {
    pcl::NormalEstimation<MY_POINT_TYPE, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    MY_POINT_CLOUD::Ptr cloud_out (new MY_POINT_CLOUD);
    
    // normals Compute
    normal_estimation.useSensorOriginAsViewPoint ();
    normal_estimation.setInputCloud (cloud_ptr);
    normal_estimation.setSearchMethod (tree);
    normal_estimation.setRadiusSearch (0.005);
    normal_estimation.compute (*cloud_normals);
    
    
}*/