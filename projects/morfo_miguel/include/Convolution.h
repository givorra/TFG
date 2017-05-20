#ifndef CONVOLUTION_H
#define CONVOLUTION_H

#include <vector>
#include "Cube.h"
#include <random>

using namespace std;

//
// Clase que realiza el proceso de convolución
// La funcion de activación de una neurona es la siguiente :
//   f(x,y) = bias + sum_l( sum_m( w[l,m] * a[x+l][y+m] ) )
//
class Convolution {

	private:
		// Cantidad de desplazamiento a la hora de aplicar un filtro
		int stride;
		// Un Filtro
		Cube weights;
		// Bias
		double bias;
		// Inicializa los pesos random entre -1 y 1
		void initializeWeightsRandom();
		// Inicializa el bias con el valor c
		void initializeBiasConstant(int c);

	public:
		// El receptiveField marca cuantas neuronas de la capa anterior
		// están conectadas con una neurona de salida.
		Convolution(int receptiveFieldSize);
		void setStride(int stride);
		int getStride();
		// Realiza el proceso de convolución aplicando el filtro
		// deslizandolo por toda la imagen
		Cube filter(Cube cubeInput);
		void setWeights(Cube weights);
		Cube getWeights();
		void setBias(double bias);
		double getBias();
	    
};

#endif