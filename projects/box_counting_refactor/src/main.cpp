#include <iostream>

// Librerias propias
#include "box_counting.h"
#include "util.h"

using namespace std;

// HEADERS
void infoParms();

// CONSTS
const string optionFile = "-f";		// Box Counting sobre fichero ply
const string optionDir 	= "-d";		// Box Counting sobre ficheros ply de un directorio
const string optionSubDir = "-s";		// Box Counting sobre lista de directorios

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
			int iterations = atoi(siterations.c_str());
			BoxCounting bc;

			if(option == optionFile)
			{
				std::vector<std::pair<float, float> > xy_pts = bc.boxCountingFile(path);
			}
			else if(option == optionDir)
			{
				//std::vector<std::pair<string, std::vector<std::pair<float, float> > > > dir_results = boxCountingDirectory(path);
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