#include "OFFReader.h"

// Global declarations
//int RETURN_OK = 0;
//int RETURN_ERROR = 1;


/*
struct vertex
{
	float x;
	float y;
	float z;
};

struct facade
{
	int v1;
	int v2;
	int v3;
};

//vertex *vertices;
facade *facades;
*/
// Parser
//void parse(string filename)

int OFFReader::read(string &fileName, pcl::PolygonMesh &mesh)
{
	vector<pcl::Vertices> vertices;
	int nv, nf;
	// Container holding last line read
	string readLine;
	// Containers for delimiter positions
	int delimiterPos_1, delimiterPos_2, delimiterPos_3, delimiterPos_4;

	// Open file for reading
	ifstream in(fileName.c_str());

	// Check if file is in OFF format
	getline(in,readLine);
	if (readLine != "OFF")
	{
		cout << "The file to read is not in OFF format." << endl;
		return -1;
	}

	// Read values for Nv and Nf
	getline(in,readLine);
	delimiterPos_1 = readLine.find(" ", 0);
	nv = atoi(readLine.substr(0,delimiterPos_1+1).c_str());
	delimiterPos_2 = readLine.find(" ", delimiterPos_1);
	nf = atoi(readLine.substr(delimiterPos_1,delimiterPos_2 +1).c_str());

	// Read the vertices
	//vertices = new vertex[nv];
pcl::Vertices v;
v.vertices.push_back(1.5);
cout << v.vertices[0] << endl;
/*
	for (int n=0; n<nv; n++)
	{
		pcl::Vertices vertice;
		getline(in,readLine);
		delimiterPos_1 = readLine.find(" ", 0);
		vertice[0] = atof(readLine.substr(0,delimiterPos_1).c_str());
		delimiterPos_2 = readLine.find(" ", delimiterPos_1+1);
		vertice[1] = atof(readLine.substr(delimiterPos_1,delimiterPos_2 ).c_str());
		delimiterPos_3 = readLine.find(" ", delimiterPos_2+1);
		vertice[2] = atof(readLine.substr(delimiterPos_2,delimiterPos_3 ).c_str());

		cout << vertices[0] << "\t" << vertices[1] << "\t" << vertices[2] << "\n";
	}
	// Read the facades
	facades = new facade[nf];

	for (int n=0; n<nf; n++)
	{
		getline(in,readLine);
		delimiterPos_1 = readLine.find(" ", 0);
		delimiterPos_2 = readLine.find(" ", delimiterPos_1+1);
		facades[n].v1 =
		atoi(readLine.substr(delimiterPos_1,delimiterPos_2 ).c_str());
		delimiterPos_3 = readLine.find(" ", delimiterPos_2+1);
		facades[n].v2 =
		atoi(readLine.substr(delimiterPos_2,delimiterPos_3 ).c_str());
		delimiterPos_4 = readLine.find(" ", delimiterPos_3+1);
		facades[n].v3 =
		atoi(readLine.substr(delimiterPos_3,delimiterPos_4 ).c_str());

		cout << facades[n].v1 << "\t" << facades[n].v2 << "\t" <<
		facades[n].v3 << "\t" << endl;
	}*/
		return 0;
}