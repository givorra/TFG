
#ifndef POOLING_H
#define POOLING_H


using namespace std;

//
// Clase que representa una matriz 3D de valores
//
class Pooling {

	private:
		// Tamaño de la region (zonas de regionSize * regionSize * regionSize)
		int regionSize;

	public:
		Pooling();
		void setRegionSize(int regionSize);
		int getRegionSize();
		// Realiza el proceso de pooling al cubo de entrada.
		// Devuelve un Cube donde en cada posición se representa
		// la media de las densidade de los voxels que caen
		// en cada region
		Cube filter(Cube originCube);
	    
};

#endif