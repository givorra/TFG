#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>

using namespace std;
// Se define aqui el tipo de punto y PC de ese tipo para poder cambiarlo en cualquier momento
#define POINT pcl::PointXYZ						
#define POINT_CLOUD pcl::PointCloud<POINT>
#define fractal_size 100
#define debug 0

// HEADERS
void infoParms();
void sierpinskiTetrahedronMesh(int iterations, pcl::PolygonMesh sierpinski_set);
void sierpinskiTetrahedronCloud(int iterations, POINT_CLOUD::Ptr sierpinski_set);
void sierpinskiPyramid(int iterations, POINT_CLOUD::Ptr sierpinski_set);
void savePointCloud(const string& filename, POINT_CLOUD::Ptr cloud);
void savePolygonMesh(const string& filename, pcl::PolygonMesh& mesh);
void makePolygonMesh();

string numberToString(int number)
{
	ostringstream ss;
	ss << number;
	return ss.str();
}

// main recibe por parámetro el número de iteraciones que realizará el algoritmo
int main(int argc, char *argv[])
{
	if(argc == 3)
	{
		string metodo = string(argv[1]);
		string siterations = string(argv[2]);
		int iterations = atoi(siterations.c_str());

		if(metodo == "tetra_pm")
		{
			pcl::PolygonMesh sierpinski_mesh;
			sierpinskiTetrahedronMesh(iterations, sierpinski_mesh);
		}
		else if(metodo == "tetra_pc")
		{
			POINT_CLOUD::Ptr sierpinski_cloud(new POINT_CLOUD());	
			sierpinskiTetrahedronCloud(iterations, sierpinski_cloud);
		}
		else if(metodo == "pyra_pc")
		{
			POINT_CLOUD::Ptr sierpinski_cloud(new POINT_CLOUD());	
			sierpinskiPyramid(iterations, sierpinski_cloud);
		}
		else
			infoParms();
	}
	else
		infoParms();
}

void sierpinskiTetrahedronMesh(int iterations, pcl::PolygonMesh sierpinski_mesh)
{
	POINT_CLOUD::Ptr pm_cloud(new POINT_CLOUD());
	POINT p1, p2, p3, p4;
	double h = (sqrt(6)/3) * fractal_size;

	p1.x = 0;
	p1.y = 0;
	p1.z = 0;
	p2.x = 0;
	p2.y = 0;
	p2.z = fractal_size;
	p3.x = (fractal_size*sqrt(3))/2;
	p3.y = 0;
	p3.z = fractal_size/2;

	p4.x = (p1.x + p2.x + p3.x)/3;
	p4.y = h;
	p4.z = (p1.z + p2.z + p3.z)/3;

	pm_cloud->points.push_back(p1);
	pm_cloud->points.push_back(p2);
	pm_cloud->points.push_back(p3);
	pm_cloud->points.push_back(p4);

	// Asigna la nube de puntos actual a la malla
	//cout << "Topcl 1\n";
	pcl::toPCLPointCloud2(*pm_cloud, sierpinski_mesh.cloud);

	pcl::Vertices triangle;
	triangle.vertices.push_back(0);
	triangle.vertices.push_back(1);
	triangle.vertices.push_back(2);
	sierpinski_mesh.polygons.push_back(triangle);
	triangle.vertices.clear();
	triangle.vertices.push_back(0);
	triangle.vertices.push_back(1);
	triangle.vertices.push_back(3);
	sierpinski_mesh.polygons.push_back(triangle);
	triangle.vertices.clear();
	triangle.vertices.push_back(1);
	triangle.vertices.push_back(2);
	triangle.vertices.push_back(3);
	sierpinski_mesh.polygons.push_back(triangle);
	triangle.vertices.clear();
	triangle.vertices.push_back(0);
	triangle.vertices.push_back(2);
	triangle.vertices.push_back(3);
	sierpinski_mesh.polygons.push_back(triangle);

	for(int i = 1; i <= iterations; i++)
	{
#if debug == 1 || debug == 2
		cout << "# Iteracion " << i << "...\n";
#endif
		pm_cloud->points.clear();
		pcl::fromPCLPointCloud2(sierpinski_mesh.cloud, *pm_cloud);
		pm_cloud->height = 0;
		pm_cloud->width = 0;
		int new_index_point[pm_cloud->points.size()][3];	// Tabla de equivalencias de los indices de cada punto en los nuevos tetraedros generados

		POINT_CLOUD::Ptr sierpinski_cloud_t(new POINT_CLOUD(*pm_cloud));

		for(int j = 0; j < pm_cloud->points.size(); j++)
		{
			/*
			int repetido = -1;
			for(int l = 0; l < sierpinski_cloud_t->points.size(); l++)
			{
				if(sierpinski_cloud_t->points[l].x == pm_cloud->points[j].x &&
					sierpinski_cloud_t->points[l].y == pm_cloud->points[j].y &&
					sierpinski_cloud_t->points[l].z == pm_cloud->points[j].z)
				{
					repetido = l;
					break;
				}
			}

			if(repetido == -1)
			{
				sierpinski_cloud_t->points.push_back(pm_cloud->points[j]);
			}*/
			POINT p1_aux, p2_aux, p3_aux;

			p1_aux.x = pm_cloud->points[j].x;
			p1_aux.y = pm_cloud->points[j].y;
			p1_aux.z = pm_cloud->points[j].z - fractal_size;

			p2_aux.x = pm_cloud->points[j].x + abs(p3.x-p2.x);
			p2_aux.y = pm_cloud->points[j].y;
			p2_aux.z = pm_cloud->points[j].z - abs(p3.z-p2.z);

			p3_aux.x = pm_cloud->points[j].x + p4.x;
			p3_aux.y = pm_cloud->points[j].y + h;
			p3_aux.z = pm_cloud->points[j].z - (fractal_size)/2;

#if debug == 2
			cout << "# Punto " << j << " = " << sierpinski_set->points[j] << "\n";
			cout << "# Punto " << j << "_1 = " << p1_aux << "\n";
			cout << "# Punto " << j << "_2 = " << p2_aux << "\n";
			cout << "# Punto " << j << "_3 = " << p3_aux << "\n";
#endif

			int repetido_p1_aux = -1;
			int repetido_p2_aux = -1;
			int repetido_p3_aux = -1;

			for(int l = 0; l < sierpinski_cloud_t->points.size(); l++)
			{
				// Si el punto generado ya se encuentra repetido, se guarda el indice del punto original para crear poligono
				if(repetido_p1_aux == -1 &&
					sierpinski_cloud_t->points[l].x == p1_aux.x &&
					sierpinski_cloud_t->points[l].y == p1_aux.y &&
					sierpinski_cloud_t->points[l].z == p1_aux.z)
				{
					repetido_p1_aux = l;
					continue;
				}

				if(repetido_p2_aux == -1 &&
					sierpinski_cloud_t->points[l].x == p2_aux.x &&
					sierpinski_cloud_t->points[l].y == p2_aux.y &&
					sierpinski_cloud_t->points[l].z == p2_aux.z)
				{
					repetido_p2_aux = l;
					continue;
				}
				if(repetido_p3_aux == -1 &&
					sierpinski_cloud_t->points[l].x == p3_aux.x &&
					sierpinski_cloud_t->points[l].y == p3_aux.y &&
					sierpinski_cloud_t->points[l].z == p3_aux.z)
				{
					repetido_p3_aux = l;
					continue;
				}
			}

			// Si los puntos generados no estan repetidos, se insertan y se guarda el indice
			if(repetido_p1_aux == -1)
			{
				repetido_p1_aux = sierpinski_cloud_t->points.size();
				sierpinski_cloud_t->points.push_back(p1_aux);
			}
			if(repetido_p2_aux == -1)
			{
				repetido_p2_aux = sierpinski_cloud_t->points.size();
				sierpinski_cloud_t->points.push_back(p2_aux);
			}
			if(repetido_p3_aux == -1)
			{
				repetido_p3_aux = sierpinski_cloud_t->points.size();
				sierpinski_cloud_t->points.push_back(p3_aux);
			}

			new_index_point[j][0] = repetido_p1_aux;
			new_index_point[j][1] = repetido_p2_aux;
			new_index_point[j][2] = repetido_p3_aux;
		}
		pm_cloud->points.clear();
		for(int j = 0; j < sierpinski_cloud_t->points.size(); j++)
		{
			sierpinski_cloud_t->points[j].x /= 2;
			sierpinski_cloud_t->points[j].y /= 2;
			sierpinski_cloud_t->points[j].z /= 2;
			pm_cloud->points.push_back(sierpinski_cloud_t->points[j]);
		}
		//cout << "Topcl 2\n";
		// Asigna la nube de puntos actual a la malla
		pcl::toPCLPointCloud2(*pm_cloud, sierpinski_mesh.cloud);

		int n_polygons = sierpinski_mesh.polygons.size();
		// Generamos nuevos poligonos, creando 2 mas por cada uno existente
		for(int j = 0; j < n_polygons; j++)
		{
			for(int k = 0; k < 3; k++)
			{
				triangle.vertices.clear();
				for(int l = 0; l < sierpinski_mesh.polygons[j].vertices.size(); l++)
				{
					triangle.vertices.push_back(new_index_point[sierpinski_mesh.polygons[j].vertices[l]][k]);
				}
				sierpinski_mesh.polygons.push_back(triangle);
			}
		}
		savePolygonMesh("../models/sierpinski_tetra_pm/sierpinski_mesh_" + numberToString(i) + ".ply", sierpinski_mesh);

#if debug == 1 || debug == 2
		cout << "# Tamaño sierpinski_set " << pm_cloud->points.size() << "...\n";
		//for(int j = 0; j < sierpinski_mesh.polygons.size(); j++)
			//cout << "# Poligonos: " << sierpinski_mesh.polygons[j] << "\n";
#endif
	}
#if debug == 2
		cout << "# Puntos insertados:\n";

		for(int j = 0; j < sierpinski_set->points.size(); j++)
		{
			cout << "# Punto " << j << " = " << sierpinski_set->points[j] << "\n";
		}
		cout << "# Tamaño sierpinski_set " << sierpinski_set->points.size() << "...\n";

		if(ply_writer.write("sierpinski_fin.ply", *sierpinski_set, false, false) < 0)
	  		cerr << "Error writing .ply file" << std::endl;
#endif

}


void sierpinskiTetrahedronCloud(int iterations, POINT_CLOUD::Ptr sierpinski_set)
{
	POINT p1, p2, p3, p4;
	double h = (sqrt(6)/3) * fractal_size;

	p1.x = 0;
	p1.y = 0;
	p1.z = 0;
	p2.x = 0;
	p2.y = 0;
	p2.z = fractal_size;
	p3.x = (fractal_size*sqrt(3))/2;
	p3.y = 0;
	p3.z = fractal_size/2;

	p4.x = (p1.x + p2.x + p3.x)/3;
	p4.y = h;
	p4.z = (p1.z + p2.z + p3.z)/3;

	sierpinski_set->points.push_back(p1);
	sierpinski_set->points.push_back(p2);
	sierpinski_set->points.push_back(p3);
	sierpinski_set->points.push_back(p4);


    pcl::PLYWriter ply_writer;

	for(int i = 1; i <= iterations; i++)
	{
#if debug == 1 || debug == 2
		cout << "# Iteracion " << i << "...\n";
#endif
		POINT_CLOUD::Ptr sierpinski_set_t(new POINT_CLOUD(*sierpinski_set));
		for(int j = 0; j < sierpinski_set->points.size(); j++)
		{
			/*
			bool repetido = false;
			for(int l = 0; l < sierpinski_set_t->points.size(); l++)
			{
				if(sierpinski_set_t->points[l].x == sierpinski_set->points[j].x &&
					sierpinski_set_t->points[l].y == sierpinski_set->points[j].y &&
					sierpinski_set_t->points[l].z == sierpinski_set->points[j].z)
				{
					repetido = true;
					break;
				}
			}

			if(!repetido)
				sierpinski_set_t->points.push_back(sierpinski_set->points[j]);
			*/
			POINT p1_aux, p2_aux, p3_aux;

			p1_aux.x = sierpinski_set->points[j].x;
			p1_aux.y = sierpinski_set->points[j].y;
			p1_aux.z = sierpinski_set->points[j].z - fractal_size;

			p2_aux.x = sierpinski_set->points[j].x + abs(p3.x-p2.x);
			p2_aux.y = sierpinski_set->points[j].y;
			p2_aux.z = sierpinski_set->points[j].z - abs(p3.z-p2.z);

			p3_aux.x = sierpinski_set->points[j].x + p4.x;
			p3_aux.y = sierpinski_set->points[j].y + h;
			p3_aux.z = sierpinski_set->points[j].z - (fractal_size)/2;

#if debug == 2
			cout << "# Punto " << j << " = " << sierpinski_set->points[j] << "\n";
			cout << "# Punto " << j << "_1 = " << p1_aux << "\n";
			cout << "# Punto " << j << "_2 = " << p2_aux << "\n";
			cout << "# Punto " << j << "_3 = " << p3_aux << "\n";
#endif

			bool repetido_p1_aux = false;
			bool repetido_p2_aux = false;
			bool repetido_p3_aux = false;

			for(int l = 0; l < sierpinski_set_t->points.size(); l++)
			{
				if(!repetido_p1_aux &&
					sierpinski_set_t->points[l].x == p1_aux.x &&
					sierpinski_set_t->points[l].y == p1_aux.y &&
					sierpinski_set_t->points[l].z == p1_aux.z)
				{
					repetido_p1_aux = true;
					continue;
				}

				if(!repetido_p2_aux &&
					sierpinski_set_t->points[l].x == p2_aux.x &&
					sierpinski_set_t->points[l].y == p2_aux.y &&
					sierpinski_set_t->points[l].z == p2_aux.z)
				{
					repetido_p2_aux = true;
					continue;
				}
				if(!repetido_p3_aux &&
					sierpinski_set_t->points[l].x == p3_aux.x &&
					sierpinski_set_t->points[l].y == p3_aux.y &&
					sierpinski_set_t->points[l].z == p3_aux.z)
				{
					repetido_p3_aux = true;
					continue;
				}
			}

			if(!repetido_p1_aux)
				sierpinski_set_t->points.push_back(p1_aux);
			if(!repetido_p2_aux)
				sierpinski_set_t->points.push_back(p2_aux);
			if(!repetido_p3_aux)
				sierpinski_set_t->points.push_back(p3_aux);

		}
		sierpinski_set->points.clear();
		for(int j = 0; j < sierpinski_set_t->points.size(); j++)
		{
			sierpinski_set_t->points[j].x /= 2;
			sierpinski_set_t->points[j].y /= 2;
			sierpinski_set_t->points[j].z /= 2;
			sierpinski_set->points.push_back(sierpinski_set_t->points[j]);
		}

		if(ply_writer.write("../models/sierpinski_tetra_pc/sierpinski_tetra_" + numberToString(i) + ".ply", *sierpinski_set, false, false) < 0)
      		cerr << "Error writing .ply file" << std::endl;

#if debug == 1 || debug == 2
		cout << "# Tamaño sierpinski_set " << sierpinski_set->points.size() << "...\n";
#endif

	}
#if debug == 2
		cout << "# Puntos insertados:\n";

		for(int j = 0; j < sierpinski_set->points.size(); j++)
		{
			cout << "# Punto " << j << " = " << sierpinski_set->points[j] << "\n";
		}
		cout << "# Tamaño sierpinski_set " << sierpinski_set->points.size() << "...\n";

		if(ply_writer.write("sierpinski_fin.ply", *sierpinski_set, false, false) < 0)
	  		cerr << "Error writing .ply file" << std::endl;
#endif

}

void sierpinskiPyramid(int iterations, POINT_CLOUD::Ptr sierpinski_set)
{
	POINT_CLOUD::Ptr sierpinski_set_t(new POINT_CLOUD());
	POINT p1, p2, p3, p4, p5;
	//double size = 1;
	double h = (sqrt(6)/3) * fractal_size;
	int increment = 1;

	p1.x = 0;
	p1.y = 0;
	p1.z = 0;

	p2.x = 0;
	p2.y = 0;
	p2.z = fractal_size;

	p3.x = fractal_size;
	p3.y = 0;
	p3.z = 0;

	p4.x = fractal_size;
	p4.y = 0;
	p4.z = fractal_size;

	double x = sqrt(pow(fractal_size, 2) - pow(fractal_size/2, 2));		// Apotema triangulo

	p5.x = fractal_size/2;
	p5.y = sqrt(pow(x, 2) - pow(fractal_size/2, 2));
	p5.z = fractal_size/2;

	sierpinski_set->points.push_back(p1);
	sierpinski_set->points.push_back(p2);
	sierpinski_set->points.push_back(p3);
	sierpinski_set->points.push_back(p4);
	sierpinski_set->points.push_back(p5);


    pcl::PLYWriter ply_writer;

	for(int i = 1; i <= iterations; i++)
	{
#if debug == 1 || debug == 2
		cout << "# Iteracion " << i << "...\n";
#endif
		sierpinski_set_t->points.clear();
		for(int j = 0; j < sierpinski_set->points.size(); j++)
		{
			bool repetido = false;
			for(int l = 0; l < sierpinski_set_t->points.size(); l++)
			{
				if(sierpinski_set_t->points[l].x == sierpinski_set->points[j].x &&
					sierpinski_set_t->points[l].y == sierpinski_set->points[j].y &&
					sierpinski_set_t->points[l].z == sierpinski_set->points[j].z)
				{
					repetido = true;
					break;
				}
			}

			if(!repetido)
				sierpinski_set_t->points.push_back(sierpinski_set->points[j]);
			POINT p1_aux, p2_aux, p3_aux, p4_aux;

			p1_aux.x = sierpinski_set->points[j].x + fractal_size;
			p1_aux.y = sierpinski_set->points[j].y;
			p1_aux.z = sierpinski_set->points[j].z;

			p2_aux.x = sierpinski_set->points[j].x;
			p2_aux.y = sierpinski_set->points[j].y;
			p2_aux.z = sierpinski_set->points[j].z - fractal_size;

			p3_aux.x = sierpinski_set->points[j].x + fractal_size;
			p3_aux.y = sierpinski_set->points[j].y;
			p3_aux.z = sierpinski_set->points[j].z - fractal_size;

			p4_aux.x = sierpinski_set->points[j].x + fractal_size/2;
			p4_aux.y = sierpinski_set->points[j].y + p5.y;
			p4_aux.z = sierpinski_set->points[j].z - fractal_size/2;

#if debug == 2
			cout << "# Punto " << j << " = " << sierpinski_set->points[j] << "\n";
			cout << "# Punto " << j << "_1 = " << p1_aux << "\n";
			cout << "# Punto " << j << "_2 = " << p2_aux << "\n";
			cout << "# Punto " << j << "_3 = " << p3_aux << "\n";
			cout << "# Punto " << j << "_3 = " << p4_aux << "\n";
#endif

			bool repetido_p1_aux = false;
			bool repetido_p2_aux = false;
			bool repetido_p3_aux = false;
			bool repetido_p4_aux = false;

			for(int l = 0; l < sierpinski_set_t->points.size(); l++)
			{
				if(sierpinski_set_t->points[l].x == p1_aux.x &&
					sierpinski_set_t->points[l].y == p1_aux.y &&
					sierpinski_set_t->points[l].z == p1_aux.z)
				{
					repetido_p1_aux = true;
				}

				if((p1_aux.x == p2_aux.x &&
					p1_aux.y == p2_aux.y &&
					p1_aux.z == p2_aux.z) ||
					(sierpinski_set_t->points[l].x == p2_aux.x &&
					sierpinski_set_t->points[l].y == p2_aux.y &&
					sierpinski_set_t->points[l].z == p2_aux.z))
				{
					repetido_p2_aux = true;
				}
				if((p1_aux.x == p3_aux.x &&
					p1_aux.y == p3_aux.y &&
					p1_aux.z == p3_aux.z) ||
					(p2_aux.x == p3_aux.x &&
					p2_aux.y == p3_aux.y &&
					p2_aux.z == p3_aux.z) ||
					(sierpinski_set_t->points[l].x == p3_aux.x &&
					sierpinski_set_t->points[l].y == p3_aux.y &&
					sierpinski_set_t->points[l].z == p3_aux.z))
				{
					repetido_p3_aux = true;
				}
				if((p1_aux.x == p4_aux.x &&
					p1_aux.y == p4_aux.y &&
					p1_aux.z == p4_aux.z) ||
					(p2_aux.x == p4_aux.x &&
					p2_aux.y == p4_aux.y &&
					p2_aux.z == p4_aux.z) ||
					(p3_aux.x == p4_aux.x &&
					p3_aux.y == p4_aux.y &&
					p3_aux.z == p4_aux.z) ||
					(sierpinski_set_t->points[l].x == p4_aux.x &&
					sierpinski_set_t->points[l].y == p4_aux.y &&
					sierpinski_set_t->points[l].z == p4_aux.z))
				{
					repetido_p4_aux = true;
				}
			}

			if(!repetido_p1_aux)
				sierpinski_set_t->points.push_back(p1_aux);
			if(!repetido_p2_aux)
				sierpinski_set_t->points.push_back(p2_aux);
			if(!repetido_p3_aux)
				sierpinski_set_t->points.push_back(p3_aux);
			if(!repetido_p4_aux)
				sierpinski_set_t->points.push_back(p4_aux);

		}
		sierpinski_set->points.clear();
		for(int j = 0; j < sierpinski_set_t->points.size(); j++)
		{
			sierpinski_set_t->points[j].x /= 2;
			sierpinski_set_t->points[j].y /= 2;
			sierpinski_set_t->points[j].z /= 2;
			sierpinski_set->points.push_back(sierpinski_set_t->points[j]);
		}

		if(ply_writer.write("../models/sierpinski_pyramid_pc/sierpinski_pyramid_" + numberToString(i) + ".ply", *sierpinski_set, false, false) < 0)
      		cerr << "Error writing .ply file" << std::endl;

#if debug == 1 || debug == 2
		cout << "# Tamaño sierpinski_set " << sierpinski_set->points.size() << "...\n";
#endif

	}
#if debug == 2
		cout << "# Puntos insertados:\n";

		for(int j = 0; j < sierpinski_set->points.size(); j++)
		{
			cout << "# Punto " << j << " = " << sierpinski_set->points[j] << "\n";
		}
		cout << "# Tamaño sierpinski_set " << sierpinski_set->points.size() << "...\n";

		if(ply_writer.write("sierpinski_pyramid_fin.ply", *sierpinski_set, false, false) < 0)
	  		cerr << "Error writing .ply file" << std::endl;
#endif

}

void infoParms()
{
	cerr << "ERROR: Debe introducir dos parámetros:\n";
	cerr << " - Parámetro 1: Método de generación [tetra_pm], [tetra_pc], [pyra_pc]\n";
	cerr << " - Parámetro 2: Iteraciones a realizar por el método especificado\n";
}

void savePointCloud(const string& filename, POINT_CLOUD::Ptr cloud)
{
	/*
	pcl::PLYWriter ply_writer;
	if(ply_writer.write(filename, *cloud, false, false) < 0)
		cerr << "Error writing .ply file" << std::endl;*/

	if(pcl::io::savePLYFile(filename, *cloud) < 0)
		cerr << "Error writing .ply file" << std::endl;

}

void savePolygonMesh(const string& filename, pcl::PolygonMesh& mesh)
{
	pcl::PLYWriter ply_writer;
	if(pcl::io::savePLYFile(filename, mesh) < 0)
		cerr << "Error writing .ply file" << std::endl;
}

/*
 * Ejemplo de como crear una malla poligonal
 */
void makePolygonMesh()
{
	POINT_CLOUD::Ptr pm_cloud(new POINT_CLOUD());
	pcl::PolygonMesh sierpinski_set;
	pcl::Vertices vertices;
	double h = (sqrt(6)/3) * fractal_size;
	POINT p1, p2, p3, p4;
	p1.x = 0;
	p1.y = 0;
	p1.z = 0;
	p2.x = 0;
	p2.y = 0;
	p2.z = fractal_size;
	p3.x = (fractal_size*sqrt(3))/2;
	p3.y = 0;
	p3.z = fractal_size/2;

	p4.x = (p1.x + p2.x + p3.x)/3;
	p4.y = h;
	p4.z = (p1.z + p2.z + p3.z)/3;

	pm_cloud->points.push_back(p1);
	pm_cloud->points.push_back(p2);
	pm_cloud->points.push_back(p3);
	pm_cloud->points.push_back(p4);

	pcl::toPCLPointCloud2(*pm_cloud, sierpinski_set.cloud);

	pcl::Vertices triangle;
	triangle.vertices.push_back(0);
	triangle.vertices.push_back(1);
	triangle.vertices.push_back(2);
	sierpinski_set.polygons.push_back(triangle);
	triangle.vertices.clear();
	triangle.vertices.push_back(0);
	triangle.vertices.push_back(1);
	triangle.vertices.push_back(3);
	sierpinski_set.polygons.push_back(triangle);
	triangle.vertices.clear();
	triangle.vertices.push_back(1);
	triangle.vertices.push_back(2);
	triangle.vertices.push_back(3);
	sierpinski_set.polygons.push_back(triangle);
	triangle.vertices.clear();
	triangle.vertices.push_back(0);
	triangle.vertices.push_back(2);
	triangle.vertices.push_back(3);
	sierpinski_set.polygons.push_back(triangle);
	triangle.vertices.clear();
	/*
	POINT_CLOUD::Ptr pc(new POINT_CLOUD());
	pcl::PolygonMesh pm;
	pcl::Vertices vertices;
	POINT p1, p2, p3;	// Tres vertices del poligono, será un triangulo
	p1.x = 0;
	p1.y = 0;
	p1.z = 0;
	p2.x = 100;
	p2.y = 0;
	p2.z = 0;
	p3.x = 50;
	p3.y = 100;
	p3.z = 0;

	pc->points.push_back(p1);
	pc->points.push_back(p2);
	pc->points.push_back(p3);

	//vertices.vertices.push_back(3); // Tamaño poligono
	vertices.vertices.push_back(0);
	vertices.vertices.push_back(1);
	vertices.vertices.push_back(2);
	pcl::toPCLPointCloud2(*pc, pm.cloud);
	pm.polygons.push_back(vertices);
	vertices.vertices.clear();*/

	savePolygonMesh("prueba_mesh.ply", sierpinski_set);
}
