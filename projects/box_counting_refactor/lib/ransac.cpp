#include "ransac.h"

Ransac::Ransac(int pIterations, float pMaxThresold, int pMinInliers)
{
	srand(time(NULL));
	_iterations = pIterations;
	_maxThresold = pMaxThresold;
	_minInliers = pMinInliers;
}

bool Ransac::compute()
{
	#if DEBUG_MODE == 1
		cout << "# RANSAC\n";
	#endif

	vector<pair<float, float> > tOutputXYPts;
	LinearRegression linearRegression;
	int xyPtsSize = _inputXYPts.size();

	_outputXYPts.clear(); // Se limpia el vector de resultados por si ha habido una ejecucion previa

	for(int i = 0; i < _iterations; i++)
	{
		tOutputXYPts.clear();

		// Calculamos los indices de los dos puntos escogidos al azar
		int p1 = getRandomInt(0, xyPtsSize);
		int p2 = p1;
		while(p1 == p2)
			p2 = getRandomInt(0, xyPtsSize);

		tOutputXYPts.push_back(_inputXYPts[p1]);
		tOutputXYPts.push_back(_inputXYPts[p2]);

		// Calculamos recta que pasa por los dos puntos obtenidos
		linearRegression.setXYPts(tOutputXYPts);
		linearRegression.compute();

		tOutputXYPts.clear(); // Limpio para no tener que comprobar que el punto actual sea uno de los dos anteriores

		// Se almacenan los puntos que cumplen las restricciones
		for(int j = 0; j < _inputXYPts.size(); j++)
		{
			// Se calcula el error de cada punto y si cumple la restriccion se almacena
			if(fabs((linearRegression.getM() * _inputXYPts[j].first + linearRegression.getB()) - _inputXYPts[j].second) <= _maxThresold)
			{
				tOutputXYPts.push_back(_inputXYPts[j]);
			}
		}

		// El criterio es quedarse con el conjunto de consenso más grande que cumpla las restricciones establecidas
		// Opcionalmente, se podría modificar el código para elegir la que menos error cometa, independientemente de si tiene menos muestras
		if(tOutputXYPts.size() >= _minInliers && tOutputXYPts.size() > _outputXYPts.size())
		{
			_outputXYPts = tOutputXYPts;
		}
	}

	return _outputXYPts.size() > 0;
}

int Ransac::getIterations()
{
	return _iterations;
}

float Ransac::getMaxThresold()
{
	return _maxThresold;
}

int Ransac::getMinInliers()
{
	return _minInliers;
}

vector<pair<float, float> > Ransac::getInputXYPts()
{
	return _inputXYPts;
}

vector<pair<float, float> > Ransac::getOutputXYPts()
{
	return _outputXYPts;
}

int Ransac::getRandomInt(int min, int max)
{
	return min + rand() % (max - min);
}

void Ransac::setIterations(int pIterations)
{
	_iterations = pIterations;
}

void Ransac::setMaxThresold(float pMaxThresold)
{
	_maxThresold = pMaxThresold;
}

void Ransac::setMinInliers(int pMinInliers)
{
	_minInliers = pMinInliers;
}

void Ransac::setInputXYPts(vector<pair<float, float> > pInputXYPts)
{
	_inputXYPts = pInputXYPts;
}