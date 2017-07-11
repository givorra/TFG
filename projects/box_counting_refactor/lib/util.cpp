#include "util.h"

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

// Sirve tanto para ficheros como directorios
bool checkPathExist(const string& pPath) 
{
  	struct stat buffer;   
  	return (stat (pPath.c_str(), &buffer) == 0); 
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

bool isNumber(const string& pString)
{
	//for(std::string::iterator it = pString.begin(); it != pString.end(); ++it)
	for(int i = 0; i < pString.size(); i++)
	{
		if(pString[i] < '0' || pString[i] > '9')
		{
			return false;
		}
	}
	return true;
}

bool loadPointCloud(const string& pFileName, MY_POINT_CLOUD::Ptr pCloudOut)
{
	if(checkPathExist(pFileName))
	{
		string::size_type posExtension = pFileName.find_last_of("."); // Obtiene pos del ultimo '.' del nombre del fichero

		if(posExtension != string::npos)                // Si ha encontrado algun punto...
		{
			string extension = pFileName.substr(posExtension, pFileName.size()-posExtension);   // Se obtiene extension del archivo mediante sub string

			if(extension == ".ply")
			{
				if(!(pcl::io::loadPLYFile(pFileName, *pCloudOut) < 0))
					return true;    // Nube cargada correctamente
				else
					cerr << "ERROR: No se ha podido leer fichero PLY [" << pFileName << "]\n";
			}
			else if(extension == ".pcd")            
			{
				if(!(pcl::io::loadPCDFile(pFileName, *pCloudOut) < 0))
					return true;    // Nube cargada correctamente
				else
					cerr << "ERROR: No se ha podido leer fichero PCD [" << pFileName << "]\n";
			}
			else
				cerr << "ERROR: Extension de fichero [" << extension << "] inválida, solo se admite formato PLY o PCD\n";
		}
	}
	else
		cerr << "ERROR: El fichero [" << pFileName << "] no existe\n";

	return false; // Indica que no hay que procesar fichero
}

void plotLinearRegression(string filename, std::vector<std::pair<float, float> > xy_pts, const float& m, const float& b)
{
	Gnuplot gp;
	string format = "png";
	filename = filename + "." + format;

	gp << "set terminal " << format << "\n";
	gp << "set fit logfile 'gnoplut_log.txt'\n";
	gp << "set output '"<< filename << "'\n";
	gp << "set grid\n";
	gp << "set xlabel 'Log(1/Leaf size)'\n";
	gp << "set ylabel 'Log(N Points)'\n";
	gp << "m = " << m << "\n";
	gp << "b = " << b << "\n";
	gp << "f(x) = m*x+b\n";
	//gp << "title_f(m,b) = sprintf('f(x) = %.2fx + %.2f', m, b)\n";
	gp << "set title '" << getFileNameFromPath(filename) << "'\n";
	gp << "plot" << gp.file1d(xy_pts) << "with points title 'P', f(x) title 'f(x) = " << (round(m * 10000) / 10000) << "x + " << (round(m * 10000) / 10000) << "\n";

}

void plotXYgraph(string filename, std::vector<std::pair<float, float> > xy_pts)
{
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
}