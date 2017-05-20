#ifndef CUBE_H
#define CUBE_H

#include <vector>

using namespace std;

//
// Clase que representa una matriz 3D de valores
//
class Cube {

	private:
		vector< vector< vector< double > > > densityCube;
		double maxDensity;
		double minDensity;

	public:
		Cube();
		Cube(int n);
		double get(int i, int j, int k);
		void set(int i, int j, int k, double v);
		void setMaxDensity(double maxDensity);
		void setMinDensity(double minDensity);
		double getMaxDensity();
		double getMinDensity();
		int getSideSize();
	    
};

#endif