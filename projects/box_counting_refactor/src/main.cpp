#include <iostream>

// Librerias propias
#include "box_counting.h"
#include "util.h"

using namespace std;

// HEADERS
void infoParms();
BoxCounting boxCountingFile(const string& pFileName);
void boxCountingDirectory(const string& pDirName);
void boxCountingSubDirectories(const string& pDirName);

// CONSTS
const string optionFile = "-f";		// Box Counting sobre fichero ply
const string optionDir 	= "-d";		// Box Counting sobre ficheros ply de un directorio
const string optionSubDir = "-s";	// Box Counting sobre lista de directorios

int main(int argc, char *argv[])
{
	if(argc == 4)
	{
		srand(time(NULL));
		string option 	= string(argv[1]);
		string path 	= string(argv[2]);
		string siterations = string(argv[3]);

		if(Util::isNumber(siterations))
		{
			int iterations = atoi(siterations.c_str());

			if(option == optionFile)
			{
				boxCountingFile(path);
			}
			else if(option == optionDir)
			{
				boxCountingDirectory(path);
			}
			else if(option == optionSubDir)
			{
				//boxCountingSubDirectories(path);
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

BoxCounting boxCountingFile(const string& pFileName)
{
	BoxCounting bc;
	MY_POINT_CLOUD::Ptr tmp(new MY_POINT_CLOUD());
	string::size_type posExtension;
	string fresults;
	bool succes;

	if(Util::loadPointCloud(pFileName, tmp))
	{
		// Obtenemos nombre del fichero con los resultados
		posExtension = pFileName.find_last_of(".");	// Obtiene pos del ultimo '.' del nombre del fichero
		fresults = pFileName.substr(0, posExtension);	// Nombre del fichero sin extension, sera el nombre del fichero con los resultados

		bc.setInputCloud(tmp);
		succes = bc.compute();
		if(succes)
			bc.plotLinearRegression(fresults);

		// Lineas de debug
		#if DEBUG_MODE == 1
			cout << "\n"
			 << "###########################################################\n"
			 << "# Box Counting on " << pFileName
			 << "\n# Output > " << fresults << "\n";
		#endif		 
	}
	return bc;
}

void boxCountingDirectory(const string& pDirName)
{

	if(Util::checkPathExist(pDirName))
	{
		#if DEBUG_MODE == 1
			cout << "Se va a aplicar el algoritmo Box Counting sobre todos los ficheros '.ply' y '.pcd' del directorio " << pDirName << "\n";
		#endif

		fstream fresults, finput;
		string cmd, tmp_files, fichero_resultados;


		tmp_files = ".lista_fich";		// Fichero que contiene todos los ply del directorio
		fichero_resultados = pDirName + "results_bc.txt";

		// Hago una lista en un fichero con find>fich
		cmd ="find " + pDirName + " -follow -type f \\( -iname \\*.ply -o -iname *.pcd \\) | sort > " + tmp_files;
		cout << cmd << "\n";
		system(cmd.c_str());

		finput.open(tmp_files.c_str(), fstream::in);
		fresults.open(fichero_resultados.c_str(), fstream::out);

		if(!finput.good())
		{
			cerr << "ERROR: El fichero de lectura [" << tmp_files << "] no existe o no es accesible\n";
		}
		if(!fresults.good())
		{
			cerr << "ERROR: El fichero escritura [" << fichero_resultados << "] no existe o no es accesible\n";
		}
		else
		{
			BoxCounting bc;
			string ply_file;
			getline(finput, ply_file);

			while(!finput.eof())
			{
				bc = boxCountingFile(ply_file);
				fresults << ply_file << " D = " << bc.getFractalDimension() << ", error = " << bc.getErrorLinearRegression() << "\n";
				fresults.flush();

				getline(finput, ply_file);
			}
			finput.close();
			fresults.close();
		}
	}
}

void boxCountingSubDirectories(const string& pDirName)
{
	string tmp_sub_dirs = ".lista_dirs";		// Fichero que contiene todos los subdirectorios del directorio recibido por parametro

	// Hago una lista de los subdirectorios del directorio recibido por parámetro
	string cmd="ls -d " + pDirName + "*/ | sort > " + tmp_sub_dirs;
	system(cmd.c_str());

	fstream f;
	f.open(tmp_sub_dirs.c_str());

	// Recorro el fichero creado
	if(f.good())
	{
		string sub_dir;
		getline(f, sub_dir);

		while(!f.eof())
		{
			boxCountingDirectory(sub_dir);
			getline(f, sub_dir);
		}
		f.close();
	}
}