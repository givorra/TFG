#include "box_counting.h"

BoxCounting::BoxCounting()
{
	maxIterations = std::numeric_limits<int>::max();
	boxCountingInitialSizeDivisor = 2;
	incrementBoxCountingSizeDivisor = 0.5;
}


/* @parm pCloud 	-> Cloud sobre la que se aplica el algoritmo
 * @parm pResultsFile	-> Fichero de salida donde se grabará el resultado del algoritmo
 * 
*/
std::vector<std::pair<float, float> > BoxCounting::boxCounting(string pResultsFile)
{
	std::vector<std::pair<float, float> > xy_pts;				// Vector de Tuplas que almacena el resultado de cada iteracion del box counting
	std::vector<std::pair<float, float> > xy_log_pts;			// Tupla que almacena ambos datos juntos

    MY_POINT_TYPE min_pt, max_pt;								// X Y Z max y min de la nube
	float xSize, ySize, zSize;									// Tamaño del objeto en cada eje
	float maxSize, minSize;												// Tamaño maximo de la nube en los tres ejes
	float leafSize;
	float increment;	
    float m, b, r;

	pcl::getMinMax3D(*inputCloudPtr, min_pt, max_pt);

	// Se obtiene maximo en los tres ejes
	xSize = max_pt.x - min_pt.x;
    ySize = max_pt.y - min_pt.y;
    zSize = max_pt.z - min_pt.z;
    maxSize = fmaxf(fmaxf(xSize, ySize), zSize);

    // Se obtiene el minimo tamaño de voxel
	minSize = meanNearestNeighbors(inputCloudPtr) * 2;

    #if DEBUG_MODE == 1
    	cout << "##########################################################\n";
      	cout << "# BOX COUNTING DATA:\n";
      	cout << "#  - Cloud -> Max X = " << max_pt.x << "\tMin X = " << min_pt.x << "\tSize = " << xSize << "\n";
      	cout << "#  - Cloud -> Max Y = " << max_pt.y << "\tMin Y = " << min_pt.y << "\tSize = " << ySize << "\n";
      	cout << "#  - Cloud -> Max Z = " << max_pt.z << "\tMin Z = " << min_pt.z << "\tSize = " << zSize << "\n";
      	cout << "#  - Cloud Max Size = " << maxSize << "\n";
      	cout << "#  - Min Voxel Size = " << minSize << "\n";
    #endif

    // Obtenemos el resultado del box counting
	xy_pts = computeBoxCounting(maxSize, minSize);

	xy_log_pts = getLogLogVector(xy_pts);
	linearRegression(xy_log_pts, m, b, r);
	//plotXYgraph(pResultsFile, xy_log_pts); 
	plotLinearRegression(pResultsFile + "_prueba", xy_log_pts, m, b);

	return xy_pts;
}
/*
std::vector<std::pair<string, std::vector<std::pair<float, float> > > > BoxCounting::boxCountingDirectory(const string& pDirName)
{
	std::vector<std::pair<string, std::vector<std::pair<float, float> > > > dir_results;

	if(checkPathExist(pDirName))
	{
		cout << "Se va a aplicar el algoritmo Box Counting sobre todos los PLY del directorio " << 
			 << "\nSalida > " <<  << "/[filename].out\n";

		string tmp_files = ".lista_fich";		// Fichero que contiene todos los ply del directorio

		// Hago una lista en un fichero con find>fich
		string cmd="find "++" -follow -type f \\( -iname \\*.ply -o -iname *.pcd \\) | sort > " + tmp_files;
		cout << cmd << "\n";
		system(cmd.c_str());

		fstream f;
		f.open(tmp_files.c_str());

		if(f.good())
		{
			string ply_file;
			getline(f, ply_file);

			while(!f.eof())
			{
				dir_results.push_back(std::make_pair(ply_file, boxCountingFile(ply_file)));
				//cout << "Procesado " << ply_file << "\n";
				getline(f, ply_file);
			}
			f.close();
			string fichero_resultados = pDirName + "results.txt";
			f.open(fichero_resultados.c_str(), ios::out);
			float m, b, r;
			if(f.good())
			{
				for(int i = 0; i < dir_results.size(); i++)
				{
					std::vector<std::pair<float, float> > xy_log_pts = getLogLogVector(dir_results[i].second);
					linearRegression(xy_log_pts, m, b, r);
					f << dir_results[i].first << " D = " << m << ", error = " << meanErrorLinearRegression(xy_log_pts, m, b) << "\n";
				}
				f.close();
			}
		}
	}
	else
		cerr << "ERROR: El directorio [" << pDirName << "] no existe o no es accesible\n";

	return dir_results;
}
*/
std::vector<std::pair<float, float> > BoxCounting::boxCountingFile(const string& pFileName)
{
	MY_POINT_CLOUD::Ptr tmp(new MY_POINT_CLOUD());
	std::vector<std::pair<float, float> > boxCountingOutput;
	string::size_type posExtension;
	string fresults;
	float m, b, r;

	if(loadPointCloud(pFileName, tmp))
	{
		setInputCloud(tmp);
		// Obtenemos nombre del fichero con los resultados
		posExtension = pFileName.find_last_of(".");	// Obtiene pos del ultimo '.' del nombre del fichero
		fresults = pFileName.substr(0, posExtension);	// Nombre del fichero sin extension, sera el nombre del fichero con los resultados

		boxCountingOutput = boxCounting(fresults);

		// Lineas de debug
		#if DEBUG_MODE == 1
			cout << "\n"
			 << "###########################################################\n"
			 << "# Box Counting on " << pFileName
			 << "\n# Output > " << fresults << "\n";
		#endif		 
	}
	return boxCountingOutput;
}

//void BoxCounting::boxCountingSubDirectories(const string& pDirName)
//{
//	std::vector<std::pair<string, std::vector<std::pair<float, float> > > > result;
//	string tmp_sub_dirs = ".lista_dirs";		// Fichero que contiene todos los subdirectorios del directorio recibido por parametro
//
//	// Hago una lista de los subdirectorios del directorio recibido por parámetro
//	string cmd="ls -d " + pDirName + "*/ | sort > " + tmp_sub_dirs;
//	system(cmd.c_str());
//
//	fstream f;
//	f.open(tmp_sub_dirs.c_str());
//
//	// Recorro el fichero creado
//	if(f.good())
//	{
//		string sub_dir;
//		getline(f, sub_dir);
//
//		while(!f.eof())
//		{
//			result = boxCountingDirectory(sub_dir);
//			result.clear();
//			getline(f, sub_dir);
//		}
//		f.close();
//	}
//	result.clear();
//}

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

std::vector<std::pair<float, float> > BoxCounting::computeBoxCounting(float pMaxVoxelSize, float pMinVoxelSize)
{
	float leafSize;
	float divisor, divisor_increment;
	std::vector<std::pair<float, float> > xy_pts;
	MY_POINT_CLOUD::Ptr cloud_filtered(new MY_POINT_CLOUD());	// Point Cloud donde se almacena la nube filtrada en cada iteracion
	pcl::VoxelGrid<MY_POINT_TYPE> sor;							// Filtro que realiza el box counting

	sor.setInputCloud(inputCloudPtr);								// Set de la nube sobre la que se aplica el filtro

	divisor = boxCountingInitialSizeDivisor;
	leafSize = pMaxVoxelSize / divisor;

    for(float i = 0; i < maxIterations && leafSize >= pMinVoxelSize; i++)
    {
		cloud_filtered->clear();

		sor.setLeafSize(leafSize, leafSize, leafSize);		// Leaf size en x, y, z 	*NOTA: Posibilidad de rectangulos y no cuadrados, sería factible?
		sor.filter(*cloud_filtered);						// Aplica filtro, conserva un solo punto por cada voxel

		xy_pts.push_back(std::make_pair(leafSize, cloud_filtered->size()));

		divisor += incrementBoxCountingSizeDivisor;
		leafSize = pMaxVoxelSize / divisor;
    }
	return xy_pts;
}

std::vector<std::pair<float, float> > BoxCounting::getLogLogVector(std::vector<std::pair<float, float> > xy_pts)
{
	std::vector<std::pair<float, float> > xy_log_pts;
	// Obtenemos los puntos log log
	for(int i = 0; i < xy_pts.size(); i++)
	{
		xy_log_pts.push_back(std::make_pair(log(1/xy_pts[i].first), log(xy_pts[i].second)));
	}
	return xy_log_pts;
}

MY_POINT_CLOUD::Ptr BoxCounting::getInputCloud()
{
	return inputCloudPtr;
}

float BoxCounting::meanNearestNeighbors(MY_POINT_CLOUD::Ptr pCloudPtr)
{
	int K = 2;
    pcl::KdTreeFLANN<MY_POINT_TYPE> kdtree;
    kdtree.setInputCloud(pCloudPtr);
	std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    float mean = 0;
    for(int i = 0; i < pCloudPtr->points.size(); i++)
    {
		if(kdtree.nearestKSearch(i, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
        	mean += sqrt(pointNKNSquaredDistance[1]);
        }
	}
	mean /= pCloudPtr->points.size();

#if DEBUG_MODE == 1
	cout << "Mean distance of nearest neighbors = " << mean << "\n";
#endif

	return mean;
}

void BoxCounting::saveResults(string fresults, const std::vector<std::pair<float, float> > &xy_pts)
{
  	fresults += ".out";
	ofstream f;
  	f.open(fresults.c_str());										// Open del fichero del fichero de salida para almacenar resultados

  	if(f.good())
  	{  		
	    for(int i = 0; i < xy_pts.size(); i++)
	    {
	      	f << xy_pts[i].first << ";" << xy_pts[i].second << ";" << log(xy_pts[i].first) << ";" << log(xy_pts[i].second)  << "\n";
	    }
	    f.close();
	}
	else
		cerr << "ERROR: No se ha podido crear el archivo de resultados" << fresults << "\n";
}

void BoxCounting::setInputCloud(MY_POINT_CLOUD::Ptr pCloud)
{
	inputCloudPtr = cloudToOriginCoordinates(pCloud);	// No recuerdo porque se ponia en el origen de coordenadas, investigar
}