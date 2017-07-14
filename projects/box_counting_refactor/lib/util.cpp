#include "util.h"

namespace Util
{
	// Sirve tanto para ficheros como directorios
	bool checkPathExist(const string& pPath) 
	{
	  	struct stat buffer;   
	  	return (stat (pPath.c_str(), &buffer) == 0); 
	}

	// A partir de la ruta de un archivo obtiene el nombre
	string getFileNameFromPath(string pPath)
	{
		string fileName = "";
		string::size_type posExtension = pPath.find_last_of("/");

		if(posExtension != string::npos)
		{
	  		++posExtension;
	  		fileName = pPath.substr(posExtension, pPath.size()-posExtension);	
	  	}
		return fileName;
	}

	bool isNumber(const string& pString)
	{
		//for(string::iterator it = pString.begin(); it != pString.end(); ++it)
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

	string numberToString(double pNumber)
	{
		ostringstream ss;
		ss << pNumber;
		return ss.str();
	}
}