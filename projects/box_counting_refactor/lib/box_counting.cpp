#include "box_counting.h"

BoxCounting::BoxCounting()
{
	_maxIterations = numeric_limits<int>::max();
	_boxCountingInitialSizeDivisor = 2;
	_incrementBoxCountingSizeDivisor = 0.5;
}

vector<pair<float, float> > BoxCounting::applyRansac(int pIterations, float pMaxThresold, int pMinInliers)
{
	Ransac ransac(pIterations, pMaxThresold, pMinInliers);
	ransac.setInputXYPts(_xyLogPts);
	ransac.compute();

	return ransac.getOutputXYPts();
}

bool BoxCounting::compute()
{
    MY_POINT_TYPE min_pt, max_pt;								// X Y Z max y min de la nube
	float xSize, ySize, zSize;									// Tamaño del objeto en cada eje
	float maxSize, minSize;	
	float leafSize;
	float divisor;
	MY_POINT_CLOUD::Ptr cloud_filtered(new MY_POINT_CLOUD());	// Point Cloud donde se almacena la nube filtrada en cada iteracion
	pcl::VoxelGrid<MY_POINT_TYPE> sor;							// Filtro que realiza el box counting

    bool succes = false;

    // OBTENEMOS LOS PARAMETROS PARA EL CALCULO

	pcl::getMinMax3D(*_inputCloudPtr, min_pt, max_pt);

	// Se obtiene maximo en los tres ejes
	xSize = max_pt.x - min_pt.x;
    ySize = max_pt.y - min_pt.y;
    zSize = max_pt.z - min_pt.z;
    maxSize = fmaxf(fmaxf(xSize, ySize), zSize);

    // Se obtiene el minimo tamaño de voxel
	minSize = meanNearestNeighbors(_inputCloudPtr) * 2;

	divisor = _boxCountingInitialSizeDivisor;
	leafSize = maxSize / divisor;

    #if DEBUG_MODE == 1
    	cout << "##########################################################\n";
      	cout << "# BOX COUNTING DATA:\n";
      	cout << "#  - Cloud -> Max X = " << max_pt.x << "\tMin X = " << min_pt.x << "\tSize = " << xSize << "\n";
      	cout << "#  - Cloud -> Max Y = " << max_pt.y << "\tMin Y = " << min_pt.y << "\tSize = " << ySize << "\n";
      	cout << "#  - Cloud -> Max Z = " << max_pt.z << "\tMin Z = " << min_pt.z << "\tSize = " << zSize << "\n";
      	cout << "#  - Cloud Max Size = " << maxSize << "\n";
      	cout << "#  - Min Voxel Size = " << minSize << "\n";
    #endif

	// SE REALIZA EL CONTEO DE CAJAS haciendo uso de voxel grid
	sor.setInputCloud(_inputCloudPtr);							// Set de la nube sobre la que se aplica el filtro
	_xyPts.clear();

    for(float i = 0; i < _maxIterations && leafSize >= minSize; i++)
    {
		cloud_filtered->clear();

		sor.setLeafSize(leafSize, leafSize, leafSize);		// Leaf size en x, y, z 	*NOTA: Posibilidad de rectangulos y no cuadrados, sería factible?
		sor.filter(*cloud_filtered);						// Aplica filtro, conserva un solo punto por cada voxel

		// Guarda punto x,y obtenido en el conteo de cajas
		_xyPts.push_back(make_pair(leafSize, cloud_filtered->size()));

		divisor += _incrementBoxCountingSizeDivisor;
		leafSize = maxSize / divisor;
    }
	// Obtenemos vector de puntos (Log(1/x), Log(y))
	_xyLogPts = getLogLogVector(_xyPts);
	// Calculo de la recta de regresion por minimos cuadrados
	_linearRegression.setXYPts(_xyLogPts);
	succes = _linearRegression.compute();

	return succes;
}

MY_POINT_CLOUD::Ptr BoxCounting::cloudToOriginCoordinates(MY_POINT_CLOUD::Ptr pCloudPtr)
{
	MY_POINT_CLOUD::Ptr cloudPtrOut(new MY_POINT_CLOUD());

	MY_POINT_TYPE min_pt, max_pt, ptAux;
	pcl::getMinMax3D(*pCloudPtr, min_pt, max_pt);

	// Desplaza la nube al punto 0,0,0
	for(int i = 0; i < pCloudPtr->points.size(); i++)
	{
		ptAux.x = pCloudPtr->points[i].x - min_pt.x;
		ptAux.y = pCloudPtr->points[i].y - min_pt.y;
		ptAux.z = pCloudPtr->points[i].z - min_pt.z;

		cloudPtrOut->points.push_back(ptAux);
	}	

	return cloudPtrOut;
}

vector<pair<float, float> > BoxCounting::getLogLogVector(vector<pair<float, float> > pXYPts)
{
	vector<pair<float, float> > xyLogPts;
	// Obtenemos los puntos log log
	for(int i = 0; i < pXYPts.size(); i++)
	{
		xyLogPts.push_back(make_pair(log(1/pXYPts[i].first), log(pXYPts[i].second)));
	}
	return xyLogPts;
}

float BoxCounting::meanNearestNeighbors(MY_POINT_CLOUD::Ptr pCloudPtr)
{
	int K = 2;
    pcl::KdTreeFLANN<MY_POINT_TYPE> kdtree;
    kdtree.setInputCloud(pCloudPtr);
	vector<int> pointIdxNKNSearch(K);
    vector<float> pointNKNSquaredDistance(K);
    float mean = 0;
    for(int i = 0; i < pCloudPtr->points.size(); i++)
    {
		if(kdtree.nearestKSearch(i, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
        	mean += sqrt(pointNKNSquaredDistance[1]); // La posicion 0 corresponde al punto buscado ya que es el mas cercano a si mismo
        }
	}
	mean /= pCloudPtr->points.size();

#if DEBUG_MODE == 1
	cout << "Mean distance of nearest neighbors = " << mean << "\n";
#endif

	return mean;
}

void BoxCounting::plotLinearRegression(string pFileName)
{
	_linearRegression.plot(pFileName);
}

// ****************************** Getters

float BoxCounting::getBoxCountingInitialSizeDivisor()
{
	return _boxCountingInitialSizeDivisor;
}

float BoxCounting::getFractalDimension()
{
	return _linearRegression.getM();
}

float BoxCounting::getIncrementBoxCountingSizeDivisor()
{
	return _incrementBoxCountingSizeDivisor;
}

MY_POINT_CLOUD::Ptr BoxCounting::getInputCloud()
{
	return _inputCloudPtr;
}

float BoxCounting::getErrorLinearRegression()
{
	return _linearRegression.getMeanError();
}

int BoxCounting::getMaxIterations()
{
	return _maxIterations;
}

vector<pair<float, float> > BoxCounting::getXYPts()
{
	return _xyPts;
}

vector<pair<float, float> > BoxCounting::getXYLogPts()
{
	return _xyLogPts;
}

// **************************  Setters

void BoxCounting::setBoxCountingInitialSizeDivisor(const float &pBoxCountingInitialSizeDivisor)
{
	_boxCountingInitialSizeDivisor = pBoxCountingInitialSizeDivisor;
}

void BoxCounting::setIncrementBoxCountingSizeDivisor(const float& pIncrementBoxCountingSizeDivisor)
{
	_incrementBoxCountingSizeDivisor = pIncrementBoxCountingSizeDivisor;
}

void BoxCounting::setInputCloud(MY_POINT_CLOUD::Ptr pCloud)
{
	_inputCloudPtr = cloudToOriginCoordinates(pCloud);	// No recuerdo porque se ponia en el origen de coordenadas, investigar
}

void BoxCounting::setMaxIterations(const int& pMaxIterations)
{
	_maxIterations = pMaxIterations;
}
