#include <iostream>
#include <vector>
#include "string.h"
#include "gnuplot-iostream.h"

using namespace std;

int main (int argc, char *argv[])
{
	std::vector<std::pair<float, float> > xy_pts;
	xy_pts.push_back(std::make_pair(1, 18));
	xy_pts.push_back(std::make_pair(0.5, 41));
	xy_pts.push_back(std::make_pair(0.25, 105));
	
	Gnuplot gp;
	string format = "png";

	gp << "set terminal " << format << "\n";
	gp << "set fit quiet\n";
	gp << "set title 'Curva de Koch'\n";

	filename = "bc_koch." + format;
	
	gp << "set output '"<< filename << "'\n";
	gp << "set grid\n";
	gp << "set xlabel 'Log(1/Leaf size)'\n";
	gp << "set ylabel 'Log(N Points)'\n";
	gp << "f(x) = m*x+b\n";
	gp << "fit f(x)" << gp.file1d(xy_pts) << " via m,b\n";
	gp << "title_f(m,b) = sprintf('f(x) = %.2fx + %.2f', m, b)\n";
	gp << "plot" << gp.file1d(xy_pts) << "with points title 'P', f(x) title title_f(m,b)\n";
}