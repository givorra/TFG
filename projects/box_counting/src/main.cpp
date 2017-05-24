#include <iostream>
#include <limits>
#include <time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <fstream>
#include <vector>
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
// Includes for gnuplot
#include <cmath>
#include <boost/tuple/tuple.hpp>
#include "gnuplot-iostream.h"

using namespace std;

// Se define aqui el tipo de punto y PC de ese tipo para poder cambiarlo en cualquier momento
#define MY_POINT_TYPE pcl::PointXYZ						
#define MY_POINT_CLOUD pcl::PointCloud<MY_POINT_TYPE>
#define DEBUG_MODE 0

// <HEADERS>

void infoParms();
bool checkPathExist(const string& path);
bool isNumber(const string& number);
bool loadPointCloud(const string& fileName, MY_POINT_CLOUD::Ptr cloud_out);
std::vector<std::pair<string, std::vector<std::pair<float, float> > > > boxCountingDirectory(const string& dir);
std::vector<std::pair<float, float> > boxCountingFile(const string& ply_file);
std::vector<std::pair<float, float> > boxCounting(MY_POINT_CLOUD::Ptr cloud_ptr, string fresults);
//void plotXYgraphDir(string filename, std::vector<std::pair<std::vector<std::pair<float, float> >, string > > xy_pts);
void plotXYgraph(string filename, std::vector<std::pair<float, float> > xy_pts);
void plotLinearRegression(string filename, std::vector<std::pair<float, float> > xy_pts, const float& m, const float& b);
string numberToString(int number);
int linearRegression(std::vector<std::pair<float, float> > xy_pts, float& m, float& b, float& r);
//void getBestLinearRegression(std::vector<std::pair<float, float> > xy_pts, float& best_m, float& best_b);
void getBestRange(std::vector<std::pair<float, float> > xy_pts, int& best_range_begin, int& best_range_end);
void cloudToOriginCoordinates(MY_POINT_TYPE min_pt, MY_POINT_CLOUD::Ptr cloud_ptr);
void saveResults(string fresults, const std::vector<std::pair<float, float> > &xy_pts);
std::vector<std::pair<float, float> > computeBoxCounting(int iterations, float first_leafSize, float last_leafSize, MY_POINT_CLOUD::Ptr cloud_ptr);
std::vector<std::pair<float, float> > getLogLogVector(std::vector<std::pair<float, float> > xy_pts);
float errorLinearRegression(const std::vector<std::pair<float, float> > &xy_pts, const float &m, const float &b);
std::vector<std::pair<float, float> > getBestPointsSet(std::vector<std::pair<float, float> > xy_pts);
string getFileNameFromPath(string path);
std::vector<std::pair<float, float> > deleteDuplicates(std::vector<std::pair<float, float> > xy_pts);
float meanNearestNeighbors(MY_POINT_CLOUD::Ptr cloud_ptr);
void boxCountingSubDirectories(const string& dir);
int getRandomInt(int min, int max);
std::vector<std::pair<float, float> > computeRansac(std::vector<std::pair<float, float> > xy_pts, int iterations, float maxThresold, int nMinInliers);
// </HEADERS>

// <CONST>

const string optionFile = "-f";		// Box Counting sobre fichero ply
const string optionDir 	= "-d";		// Box Counting sobre ficheros ply de un directorio
const string optionSubDir 	= "-s";		// Box Counting sobre lista de directorios
const int n_ranges = 5;

// </CONST>

int iterations;		// Iteraciones del algoritmo

string numberToString(int number)
{
	ostringstream ss;
	ss << number;
	return ss.str();
}

int main (int argc, char *argv[])
{
	if(argc == 4)
	{
		srand(time(NULL));
		string option 	= string(argv[1]);
		string path 	= string(argv[2]);
		string siterations = string(argv[3]);

		if(isNumber(siterations))
		{
			iterations = atoi(siterations.c_str());

			if(option == optionFile)
			{
				std::vector<std::pair<float, float> > xy_pts = boxCountingFile(path);	
				/*float m, b, r;
				getBestLinearRegression(xy_pts, m, b);*/
			}
			else if(option == optionDir)
			{
				std::vector<std::pair<string, std::vector<std::pair<float, float> > > > dir_results = boxCountingDirectory(path);
			}
			else if(option == optionSubDir)
			{
				boxCountingSubDirectories(path);
			}
			else
			{
				cerr << "ERROR: Parámetro ["<< option << "] desconocido\n";
				infoParms();
			}
		}
		else
		{
			cerr << "ERROR: El tercer parámetro (iteraciones) debe ser numérico\n";
			infoParms();
		}

	}
	else
	{
		cerr << "ERROR: Se deben recibir tres parámetros\n";
		infoParms();
	}
  	return 0;
}

void infoParms()
{
	cout << "Las opciones posibles de ejecucuón son:\n";
	cout << "  - Parm 1 [-f], Parm 2 [ply file name], Parm 3 [n iterations]  --> Realizar Box Counting para un objeto PLY\n";
	cout << "  - Parm 1 [-d], Parm 2 [directory path], Parm 3 [n iterations] --> Realizar Box Counting sobre todos los PLY de un directorio\n";
}

bool loadPointCloud(const string& fileName, MY_POINT_CLOUD::Ptr cloud_out)
{

  	if(checkPathExist(fileName))
  	{
    	string::size_type posExtension = fileName.find_last_of(".");	// Obtiene pos del ultimo '.' del nombre del fichero

    	if(posExtension != string::npos)								// Si ha encontrado algun punto...
    	{
      		//posExtension;
      		string extension = fileName.substr(posExtension, fileName.size()-posExtension);		// Se obtiene extension del archivo mediante sub string

      		if(extension == ".ply")
      		{
		        //if(ply_reader.read(fileName.c_str(), *tmp) < 0) 
		        if(pcl::io::loadPLYFile(fileName, *cloud_out) < 0)
		        {
		           cerr << "ERROR: No se ha podido leer fichero PLY [" << fileName << "]\n";
		        }
		        else
		        	return true;		// Nube cargada correctamente
      		}
      		else if(extension == ".pcd")      			
      		{
		        //if(ply_reader.read(fileName.c_str(), *tmp) < 0) 
		        if(pcl::io::loadPCDFile(fileName, *cloud_out) < 0)
		        {
		           cerr << "ERROR: No se ha podido leer fichero PCD [" << fileName << "]\n";
		        }
		        else
		        	return true;		// Nube cargada correctamente
      		}
      		else
      			cerr << "ERROR: Extension de fichero [" << extension << "] inválida, solo se admite .ply\n";
    	}
  	}
  	else
  		cerr << "ERROR: El fichero [" << fileName << "] no existe\n";

  return false; // Indica que no hay que procesar fichero
}

bool isNumber(const string& number)
{
	for(int i = 0; i < number.size(); i++)
	{
		if(number[i] < '0' || number[i] > '9')
		{
			return false;
		}
	}
	return true;
}

// Sirve tanto para ficheros como directorios
bool checkPathExist(const string& path) 
{
  	struct stat buffer;   
  	return (stat (path.c_str(), &buffer) == 0); 
}

std::vector<std::pair<string, std::vector<std::pair<float, float> > > > boxCountingDirectory(const string& dir)
{
	std::vector<std::pair<string, std::vector<std::pair<float, float> > > > dir_results;

	if(checkPathExist(dir))
	{
		cout << "Se va a aplicar el algoritmo Box Counting sobre todos los PLY del directorio " << dir
			 << "\nSalida > " << dir << "/[filename].out\n";

		string tmp_files = ".lista_fich";		// Fichero que contiene todos los ply del directorio

		// Hago una lista en un fichero con find>fich
		string cmd="find "+dir+" -follow -type f \\( -iname \\*.ply -o -iname *.pcd \\) | sort > " + tmp_files;
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
			string fichero_resultados = dir + "results.txt";
			f.open(fichero_resultados.c_str(), ios::out);
			float m, b, r;
			if(f.good())
			{
				for(int i = 0; i < dir_results.size(); i++)
				{
					std::vector<std::pair<float, float> > xy_log_pts = getLogLogVector(dir_results[i].second);
					linearRegression(xy_log_pts, m, b, r);
					f << dir_results[i].first << " D =" << m << " error = " << errorLinearRegression(xy_log_pts, m, b) << "\n";
				}
				f.close();
			}
		}
	}
	else
		cerr << "ERROR: El directorio [" << dir << "] no existe o no es accesible\n";

	return dir_results;
}

void boxCountingSubDirectories(const string& dir)
{
	std::vector<std::pair<string, std::vector<std::pair<float, float> > > > result;
	string tmp_sub_dirs = ".lista_dirs";		// Fichero que contiene todos los ply del directorio

	// Hago una lista en un fichero con find>fich
	string cmd="ls -d "+dir+"*/ | sort > " + tmp_sub_dirs;
	system(cmd.c_str());

	fstream f;
	f.open(tmp_sub_dirs.c_str());

	if(f.good())
	{
		string sub_dir;
		getline(f, sub_dir);

		while(!f.eof())
		{
			result = boxCountingDirectory(sub_dir);
			result.clear();
			getline(f, sub_dir);
		}
		f.close();
	}
	result.clear();
}

std::vector<std::pair<float, float> > boxCountingFile(const string& ply_file)
{
	

	MY_POINT_CLOUD::Ptr tmp(new MY_POINT_CLOUD());
	std::vector<std::pair<float, float> > cloud;

	if(loadPointCloud(ply_file, tmp))
	{
		string::size_type posExtension = ply_file.find_last_of(".");	// Obtiene pos del ultimo '.' del nombre del fichero

		string fresults = ply_file.substr(0, posExtension);	// File name without extension
#if DEBUG_MODE == 1
		cout << "\n"
		 << "###########################################################\n"
		 << "# Box Counting on " << ply_file
		 << "\n# Output > " << fresults << "\n";
#endif
		cloud = boxCounting(tmp, fresults);
	}
	return cloud;
}

/* @parm cloud_ptr 	-> Cloud sobre la que se aplica el algoritmo
 * @parm fresults	-> Fichero de salida donde se grabará el resultado del algoritmo
 * 
 * El algoritmo calcula el tamaño de la PC en cada eje y en base a este y al número de iteraciones
 * a realizar obtiene el incremento a aplicar al tamaño del voxel en cada iteracion
*/

std::vector<std::pair<float, float> > boxCounting(MY_POINT_CLOUD::Ptr cloud_ptr, string fresults)
{
	std::vector<std::pair<float, float> > xy_pts;				// Vector de Tuplas que almacena el resultado de cada iteracion del box counting
	std::vector<std::pair<float, float> > xy_log_pts;			// Tupla que almacena ambos datos juntos
	std::vector<std::pair<float, float> > xy_pts_without_duplicates;			// Tupla que almacena ambos datos juntos
	std::vector<std::pair<float, float> > xy_pts_best_set;
	std::vector<std::pair<float, float> > xy_pts_best_range;

    MY_POINT_TYPE min_pt, max_pt;								// X Y Z max y min del voxel que engloba el objeto
	float xSize, ySize, zSize;									// Tamaño del objeto en cada eje
	float selectedSize, maxSize;											// Tamaño seleccionado para calcular el incremento dividiendolo entre las iteraciones
	float leafSize;
	float increment;

	pcl::getMinMax3D(*cloud_ptr, min_pt, max_pt);
	
	// Movemos nube al origen de coordenadas (0,0,0)
	cloudToOriginCoordinates(min_pt, cloud_ptr);

	// Prueba con la separacion minima entre dos vecinos mas cercanos:: VER AL FINAL DEL DOCUMENTO
	// Aqui se calcula el incremento del leaf size para cada iteracion
	xSize = max_pt.x - min_pt.x;
    ySize = max_pt.y - min_pt.y;
    zSize = max_pt.z - min_pt.z;
    maxSize = fmaxf(fmaxf(xSize, ySize), zSize);

    #if DEBUG_MODE == 1
    	cout << "##########################################################\n";
      	cout << "# BOX COUNTING DATA:\n";
      	cout << "#  - Cloud -> Max X = " << max_pt.x << "\tMin X = " << min_pt.x << "\tSize = " << xSize << "\n";
      	cout << "#  - Cloud -> Max Y = " << max_pt.y << "\tMin Y = " << min_pt.y << "\tSize = " << ySize << "\n";
      	cout << "#  - Cloud -> Max Z = " << max_pt.z << "\tMin Z = " << min_pt.z << "\tSize = " << zSize << "\n";
      	cout << "#  - Cloud Max Size = " << maxSize << "\n";
    #endif

    // Obtenemos el resultado para leafsize desde 0 hasta el maximo tamaño de la figura/nube
	xy_pts = computeBoxCounting(iterations, 0, maxSize, cloud_ptr);

    //leafSize = maxSize/iterations;
    float m, b, r;

	xy_log_pts = getLogLogVector(xy_pts);
	linearRegression(xy_log_pts, m, b, r);
	plotXYgraph(fresults + "_normal", xy_log_pts); 

	// RANSAC
	/*
    std::vector<std::pair<float, float> > xy_pts_ransac;
    int ransac_iterations = 40;
    float ransac_maxThresold = errorLinearRegression(xy_log_pts, m, b) * 1.5 / xy_log_pts.size();
    int ransac_nMinInliers = xy_pts.size() * 0.75;

	xy_pts_ransac = computeRansac(xy_log_pts, ransac_iterations, ransac_maxThresold, ransac_nMinInliers);
	plotXYgraph(fresults + "_ransac", xy_pts_ransac);
	*/
	//plotXYgraph(fresults + "_xy_pts", xy_pts);
	//xy_log_pts = getLogLogVector(xy_pts);
	//saveResults(fresults, xy_pts);	
	//xy_pts_best_set = getBestPointsSet(xy_pts);
	//xy_log_pts = getLogLogVector(xy_pts_best_set);
	//plotXYgraph(fresults + "_best_set", xy_log_pts);

	// Mejor rango sin duplicados
	/*xy_pts_without_duplicates = deleteDuplicates(xy_pts);
	int best_range_begin, best_range_end;
	getBestRange(xy_pts_without_duplicates, best_range_begin, best_range_end);
	xy_pts_without_duplicates.erase(xy_pts_without_duplicates.begin() + best_range_end, xy_pts_without_duplicates.end());
	xy_pts_without_duplicates.erase(xy_pts_without_duplicates.begin(), xy_pts_without_duplicates.begin() + best_range_begin);
	xy_log_pts = getLogLogVector(xy_pts_without_duplicates);
	plotXYgraph(fresults + "_best_range_without_duplicate", xy_log_pts);
*/
	/*
	// Mejor rango nube original
	int best_range_begin, best_range_end;
	xy_pts_best_range = xy_pts;
	getBestRange(xy_pts_best_range, best_range_begin, best_range_end);
	xy_pts_best_range.erase(xy_pts_best_range.begin() + best_range_end, xy_pts_best_range.end());
	xy_pts_best_range.erase(xy_pts_best_range.begin(), xy_pts_best_range.begin() + best_range_begin);
	xy_log_pts = getLogLogVector(xy_pts_best_range);
	plotXYgraph(fresults + "_best_range", xy_log_pts);

	/*
	int best_range_begin, best_range_end;
	getBestRange(xy_pts, best_range_begin, best_range_end);
	std::vector<std::pair<float, float> > xy_pts_aux;// = xy_pts;
	//xy_pts_aux.erase(xy_pts_aux.begin() + best_range_end, xy_pts_aux.end());
	//xy_pts_aux.erase(xy_pts_aux.begin(), xy_pts_aux.begin() + best_range_begin);
	for(int i = best_range_begin; i <= best_range_end; i++)
		xy_pts_aux.push_back(xy_pts[i]);
	
	xy_log_pts = getLogLogVector(xy_pts_aux);
	plotXYgraph(fresults + "_normal_best", xy_log_pts); 

	xy_pts = computeBoxCounting(iterations, xy_pts[best_range_begin].first, xy_pts[best_range_end].first, cloud_ptr);	
	xy_log_pts = getLogLogVector(xy_pts);
	plotXYgraph(fresults + "_best", xy_log_pts); 

	getBestRange(xy_pts, best_range_begin, best_range_end);
	xy_pts.erase(xy_pts.begin(), xy_pts.begin() + best_range_begin);
	xy_pts.erase(xy_pts.begin() + best_range_end, xy_pts.end());
	xy_log_pts = getLogLogVector(xy_pts);
	plotXYgraph(fresults + "_best_best", xy_log_pts); 
*/


	return xy_pts;
}

std::vector<std::pair<float, float> > computeBoxCounting(int iterations, float first_leafSize, float last_leafSize, MY_POINT_CLOUD::Ptr cloud_ptr)
{

	std::vector<std::pair<float, float> > xy_pts;
	MY_POINT_CLOUD::Ptr cloud_filtered(new MY_POINT_CLOUD());	// Point Cloud donde se almacena la nube filtrada en cada iteracion
	pcl::VoxelGrid<MY_POINT_TYPE> sor;							// Filtro que realiza el box counting
	sor.setInputCloud(cloud_ptr);								// Set de la nube sobre la que se aplica el filtro
	float leafSize;

	//float meanNN = meanNearestNeighbors(cloud_ptr) * 2;
	//if(first_leafSize < meanNN)
	//{
	//	first_leafSize = meanNN;
	//}
	float increment = (last_leafSize-first_leafSize)/iterations;

	if(first_leafSize == 0)
		leafSize = increment;
	//else
	//	leafSize = first_leafSize;



	#if DEBUG_MODE == 1
    	cout << "##########################################################\n";
      	cout << "# computeBoxCounting parms:\n";
      	cout << "#  - iterations = " << iterations << "\n";
      	cout << "#  - first_leafSize2 = " << first_leafSize << "\n";
      	cout << "#  - meanNN = " << meanNN << "\n";
      	cout << "#  - last_leafSize = " << last_leafSize << "\n";
      	cout << "#  - leafSize = " << leafSize << "\n";
      	cout << "#  - increment = " << increment << "\n";
    #endif

    for(int i = 0; i < iterations; i++)							// Itera incrementando el leafsize
    {
		cloud_filtered->clear();

		sor.setLeafSize(leafSize, leafSize, leafSize);		// Leaf size en x, y, z 	*NOTA: Posibilidad de rectangulos y no cuadrados, sería factible?
		sor.filter(*cloud_filtered);						// Aplica filtro, conserva un solo punto por cada voxel

		xy_pts.push_back(std::make_pair(leafSize, cloud_filtered->size()));

		leafSize += increment;							// Incremento del leaf size
	}
	#if DEBUG_MODE == 1
      	cout << "#  - xy_pts.size = " << xy_pts.size() << "\n";
      	cout << "#  - last leaf size = " << leafSize - increment << "\n";
    	cout << "##########################################################\n";
    #endif
	return xy_pts;
}

void saveResults(string fresults, const std::vector<std::pair<float, float> > &xy_pts)
{
  	fresults += ".out";
	ofstream f;
  	f.open(fresults.c_str());										// Open del fichero del fichero de salida para almacenar resultados

  	if(f.good())
  	{  		
	    for(int i = 0; i < iterations; i++)
	    {
	      	f << xy_pts[i].first << ";" << xy_pts[i].second << ";" << log(xy_pts[i].first) << ";" << log(xy_pts[i].second)  << "\n";
	    }
	    f.close();
	}
	else
		cerr << "ERROR: No se ha podido crear el archivo de resultados" << fresults << "\n";
}

void cloudToOriginCoordinates(MY_POINT_TYPE min_pt, MY_POINT_CLOUD::Ptr cloud_ptr)
{
	// Desplaza la nube al punto 0,0,0
	for(int i = 0; i < cloud_ptr->points.size(); i++)
	{
		cloud_ptr->points[i].x -= min_pt.x;
		cloud_ptr->points[i].y -= min_pt.y;
		cloud_ptr->points[i].z -= min_pt.z;
	}	
}

void plotXYgraph(string filename, std::vector<std::pair<float, float> > xy_pts)
{
	/*
	xy_pts.clear();
	xy_pts.push_back(std::make_pair(1, 1));
	xy_pts.push_back(std::make_pair(1, 2));
	xy_pts.push_back(std::make_pair(2, 2));
	xy_pts.push_back(std::make_pair(3, 3));
	xy_pts.push_back(std::make_pair(4, 4));
	xy_pts.push_back(std::make_pair(5, 5));
	xy_pts.push_back(std::make_pair(6, 6));
	xy_pts.push_back(std::make_pair(7, 7));
	xy_pts.push_back(std::make_pair(8, 8));
	xy_pts.push_back(std::make_pair(2, 1));
	*/
	Gnuplot gp;
	string format = "png";

	gp << "set terminal " << format << "\n";
	gp << "set fit quiet\n";
	gp << "set title '" << getFileNameFromPath(filename) << "'\n";

	filename = filename + "." + format;
	
	gp << "set output '"<< filename << "'\n";
	gp << "set grid\n";
	gp << "set xlabel 'Log(1/Leaf size)'\n";
	gp << "set ylabel 'Log(N Points)'\n";
	gp << "f(x) = m*x+b\n";
	gp << "fit f(x)" << gp.file1d(xy_pts) << " via m,b\n";
	gp << "title_f(m,b) = sprintf('f(x) = %.2fx + %.2f', m, b)\n";
	gp << "plot" << gp.file1d(xy_pts) << "with points title 'P', f(x) title title_f(m,b)\n";

	//cout << "title_f(a,b) = sprintf('f(x) = %2fx + %.2f', a, b)\n";
}

void plotLinearRegression(string filename, std::vector<std::pair<float, float> > xy_pts, const float& m, const float& b)
{
	Gnuplot gp;
	string format = "png";
	filename = filename + "." + format;

	gp << "set terminal " << format << "\n";
	gp << "set fit logfile 'gnoplut_log.txt'";
	gp << "set output '"<< filename << "'\n";
	gp << "set grid\n";
	gp << "set xlabel 'Log(1/Leaf size)'\n";
	gp << "set ylabel 'Log(N Points)'\n";
	gp << "m = " << m << "\n";
	gp << "b = " << b << "\n";
	gp << "f(x) = m*x+b\n";
	//gp << "title_f(m,b) = sprintf('f(x) = %.2fx + %.2f', m, b)\n";
	gp << "set title '" << filename << "'\n";
	gp << "plot" << gp.file1d(xy_pts) << "with points title 'P', f(x) title 'f(x) = " << m << "x + " << b << "\n";

}


int linearRegression(std::vector<std::pair<float, float> > xy_pts, float& m, float& b, float& r)
{
	int n = xy_pts.size();
	float sumx=0,sumy=0,sumx2=0,sumy2=0,sumxy=0;
	float sxx,syy,sxy;

	m = 0;
	b = 0;
	r = 0;
	if (n < 2)
		return false;

	/* Conpute some things we need */
	for (int i=0; i < n; i++)
	{
		sumx += xy_pts[i].first;
		sumy += xy_pts[i].second;
		sumx2 += (xy_pts[i].first * xy_pts[i].first);
		sumy2 += (xy_pts[i].second * xy_pts[i].second);
		sumxy += (xy_pts[i].first * xy_pts[i].second);
	}
	sxx = sumx2 - sumx * sumx / n;
	syy = sumy2 - sumy * sumy / n;
	sxy = sumxy - sumx * sumy / n;

	/*Infinite slope(m), non existant intercept (b)*/
	if (abs(sxx) == 0)
		return false;

	/* Calculate the slope (m) and intercept (b) */
	m = sxy / sxx;
	b = sumy / n - (m) * sumx / n;

	/* Compute the regression coefficient */
	if (abs(syy) == 0)
		r = 1;
	else
		r = sxy / sqrt(sxx * syy);
#if DEBUG_MODE == 4
	cout << "# Results linear regression: m = " << m << ", b = " << b << ", error = " << errorLinearRegression(xy_pts, m, b) << "\n";
#endif
	return true;
}

void getBestRange(std::vector<std::pair<float, float> > xy_pts, int& best_range_begin, int& best_range_end)
{
	std::vector<std::pair<float, float> > xy_log_pts;

	float m = 0, b = 0, r = 0, best_r, best_b, best_m, best_error;
	std::vector<std::pair<float, float> > xy_pts_range, best_range;
	int n = xy_pts.size();
	int range_begin = 0;
	int range_end = n/2 - 1;	// Resta 1 porque empieza en indice 0
	int increment = n/20;

	best_m = 0;
	best_b = 0;
	best_r = 0;
	best_error = 9999999999;
#if DEBUG_MODE == 1
	cout << "# GET BEST RANGE LINEAR REGRESSION\n";
	cout << " - Número de muestras = " << n << "\n";
	cout << " - Inicio rango = " << range_begin << "\n";
	cout << " - Fin rango = " << range_end << "\n";
	cout << " - Incremento = " << increment << "\n";
#endif

	xy_log_pts = getLogLogVector(xy_pts);

	while(range_end <= n)
	{
		xy_pts_range.clear();
		for(int i = range_begin; i <= range_end; i++)
		{
			xy_pts_range.push_back(xy_log_pts[i]);
		}

		linearRegression(xy_pts_range, m, b, r);
		float error = errorLinearRegression(xy_pts_range, m, b);	// Se evita division por 0 si la regresion fuera perfecta

		//if(r > best_r)
		if(error < best_error && m > 1)
		{
			best_m = m;
			best_b = b;
			best_r = r;
			best_error = error;
			best_range_begin = range_begin;
			best_range_end = range_end;
		}

		range_begin += increment;
		range_end += increment;
	}
#if DEBUG_MODE == 1
	cout << "# Best Linear Regression: m = " << best_m << ", b = " << best_b << ", error = " << best_error << "\n";
	cout << "# Best leaf size: Begin = " << best_range_begin << ", End = " << best_range_end << "\n";
#endif
}

std::vector<std::pair<float, float> > getBestPointsSet(std::vector<std::pair<float, float> > xy_pts)
{
	std::vector<std::pair<float, float> > xy_log_pts;
	std::vector<std::pair<float, float> > xy_pts_out;

	const int divisorRango = 10;

	float m = 0, b = 0, r = 0, epsilon;
	std::vector<std::pair<float, float> > xy_pts_range, best_range;
	int n = xy_pts.size();
	int range_begin = 0;
	int range_end = n/divisorRango - 1;	// Resta 1 porque empieza en indice 0
	int increment = n/(divisorRango * 2);

	xy_log_pts = getLogLogVector(xy_pts);
	linearRegression(xy_log_pts, m, b, r);
	epsilon = errorLinearRegression(xy_log_pts, m, b) / divisorRango;	// Divido el error para que proporcional a el número de muestras

#if DEBUG_MODE == 1
	cout << "# GET BEST SET LINEAR REGRESSION\n";
	cout << " - Número de muestras = " << n << "\n";
	cout << " - Inicio rango = " << range_begin << "\n";
	cout << " - Fin rango = " << range_end << "\n";
	cout << " - Incremento = " << increment << "\n";
	cout << " - Error máximo = " << epsilon << "\n";
#endif

	while(range_end <= n)
	{
		xy_pts_range.clear();
		for(int i = range_begin; i <= range_end; i++)
		{
			xy_pts_range.push_back(xy_log_pts[i]);
		}

		linearRegression(xy_pts_range, m, b, r);
		float error = errorLinearRegression(xy_pts_range, m, b);	// Se evita division por 0 si la regresion fuera perfecta

#if DEBUG_MODE == 1
	cout << "# Resultados regresion lineal:  inicio rango = " << range_begin << ", fin rango = " << range_end << ", m = " << m << ", b = " << b << ", error = " << error << "\n";
#endif
		if(error <= epsilon && m >= 1.99)
		{
			for(int i = range_begin; i < range_end; i++)
			{
				if (std::find(xy_pts_out.begin(), xy_pts_out.end(), xy_pts[i]) == xy_pts_out.end())
				{
					xy_pts_out.push_back(xy_pts[i]);
				}
			}
		}
		range_begin += increment;
		range_end += increment;
	}

	return xy_pts_out;
}

std::vector<std::pair<float, float> > deleteDuplicates(std::vector<std::pair<float, float> > xy_pts)
{
	std::vector<std::pair<float, float> > xy_pts_out;
	bool repeated;

	for(int i = 0; i < xy_pts.size(); i++)
	{
		repeated = false;
		for(int j = 0; j < xy_pts_out.size() && !repeated; j++)
		{
			if(xy_pts[i].second == xy_pts_out[j].second)
				repeated = true;
		}
		if(!repeated)
		{
			xy_pts_out.push_back(xy_pts[i]);
		}
	}
#if DEBUG_MODE == 1
	cout << "# Elementos duplicados eliminados = " << xy_pts.size() - xy_pts_out.size() << "\n";
#endif
	return xy_pts_out;
}

float errorLinearRegression(const std::vector<std::pair<float, float> > &xy_pts, const float &m, const float &b)
{
	float error = 0;
	// Calcula el sumatorio de la distancia de cada punto a la recta de regresion
	for(int i = 0; i < xy_pts.size(); i++)
	{
		error += abs(xy_pts[i].second - (m*xy_pts[i].first+b));
		//cout << "Original y = " << xy_pts[i].second << ", Calculada y = " << m*xy_pts[i].first+b 
		//	<< ", Diferencia " << abs(xy_pts[i].second - (m*xy_pts[i].first+b)) << ", Error = " << error << "\n";
	}
	return error;
}

std::vector<std::pair<float, float> > getLogLogVector(std::vector<std::pair<float, float> > xy_pts)
{
	std::vector<std::pair<float, float> > xy_log_pts;
	// Obtenemos los puntos log log
	for(int i = 0; i < xy_pts.size(); i++)
	{
		xy_log_pts.push_back(std::make_pair(log(1/xy_pts[i].first), log(xy_pts[i].second)));
		//cout << "# xy_pts[i].first = " << xy_pts[i].first << ", xy_pts[i].second " << xy_pts[i].second << "\n";
		//cout << "# xy_log_pts[i].first = " << xy_log_pts[i].first << ", xy_log_pts[i].second " << xy_log_pts[i].second << "\n";
	}
	return xy_log_pts;
}

// A partir de la ruta de un archivo obtiene el nombre
string getFileNameFromPath(string path)
{
	string fileName = "";
	string::size_type posExtension = path.find_last_of("/");

	if(posExtension != string::npos)
	{
  		++posExtension;
  		fileName = path.substr(posExtension, path.size()-posExtension);	
  	}
	return fileName;
}


/*
 Dentro del mejor rango, volver a realizar el algoritmo

 Dimension en una grafica de las figuras generadas con diferentes iteraciones

 Transformacion pcl para rotar las figuras

 Hayar un metodo para elegir el numero de muestras que tiene que tener el rango
*/

float meanNearestNeighbors(MY_POINT_CLOUD::Ptr cloud_ptr)
{
	int K = 2;
    pcl::KdTreeFLANN<MY_POINT_TYPE> kdtree;
    kdtree.setInputCloud(cloud_ptr);
	std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    float mean = 0;
    for(int i = 0; i < cloud_ptr->points.size(); i++)
    {
    	//cout << "Search point = " << cloud_ptr->points[i].x << " " << cloud_ptr->points[i].y << " " << cloud_ptr->points[i].z << "\n";
		if(kdtree.nearestKSearch(i, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
        	mean += sqrt(pointNKNSquaredDistance[1]);
			/*
	        for (size_t j = 0; j < pointIdxNKNSearch.size (); j++)
	     		std::cout << "    "  <<   cloud_ptr->points[ pointIdxNKNSearch[j] ].x 
	                << " " << cloud_ptr->points[ pointIdxNKNSearch[j] ].y 
	                << " " << cloud_ptr->points[ pointIdxNKNSearch[j] ].z 
	                << " (squared distance: " << pointIdxNKNSearch[j] << " " << pointNKNSquaredDistance[j] << ")" << std::endl;
	                */
        }
	}
	mean /= cloud_ptr->points.size();

#if DEBUG_MODE == 1
	cout << "Mean distance of nearest neighbors = " << mean << "\n";
#endif

	return mean;
}



std::vector<std::pair<float, float> > computeRansac(std::vector<std::pair<float, float> > xy_pts, int iterations, float maxThresold, int nMinInliers)
{

#if DEBUG_MODE == 1
	cout << "# RANSAC\n";
#endif

	float m, b, r, error, best_error, t_maxThresold;
	int i = 0;
	int xy_pts_size = xy_pts.size();

	std::vector<std::pair<float, float> > out_xy_pts, best_xy_pts;
	best_error = std::numeric_limits<float>::max();

	t_maxThresold = maxThresold;

	for(int k = 0; k < 10 && best_xy_pts.size() == 0; k++)
	{
		while(i < iterations)
		{
			out_xy_pts.clear();

			int p1 = getRandomInt(0, xy_pts_size);
			int p2 = p1;
			while(p1 == p2)
				p2 = getRandomInt(0, xy_pts_size);

			out_xy_pts.push_back(xy_pts[p1]);
			out_xy_pts.push_back(xy_pts[p2]);
			linearRegression(out_xy_pts, m, b, r);

			for(int j = 0; j < xy_pts.size(); j++)
			{
				if(j != p1 && j != p2)
				{
					if(fabs((m * xy_pts[j].first + b) - xy_pts[j].second) <= t_maxThresold)
					{
						out_xy_pts.push_back(xy_pts[j]);
					}
				}
			}

			linearRegression(out_xy_pts, m, b, r);
			error = errorLinearRegression(out_xy_pts, m, b) / out_xy_pts.size();

			if(out_xy_pts.size() >= nMinInliers && error < best_error)
			{
				best_xy_pts = out_xy_pts;
				best_error = error;

	#if DEBUG_MODE == 1
		cout << "Bests inliers: best_error = " << best_error << ", n inliers = " << best_xy_pts.size() << "\n";
	#endif

			}
			i++;
		}
		t_maxThresold += 0.1 * maxThresold; // Incremento el 10 % el max thresold por si no ha llegado a ninguna solucion
	}
	

	return best_xy_pts;
}

int getRandomInt(int min, int max)
{
	return min + rand() % (max - min);
}