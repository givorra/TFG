#include <iostream>

using namespace std;

int main()
{
	int n=10;
	float x[10],y[10],a,b,r;
	//Se guardan los datos en x y en y,
	linearRegression(x,y,n,&a,&b,&r);

}
/*[sourcecode language='cpp']
Linear Regression
y(x) = a + b x, for n samples
The following assumes the standard deviations are unknown for x and y
Return a, b and r the regression coefficient
*/
//int linearRegression(double *x,double *y,int n,double *a,double *b,double *r)
int linearRegression(std::vector<std::pair<double, double> > xy_pts, double *m, double *b, double *r)
{
	int i;
	double sumx=0,sumy=0,sumx2=0,sumy2=0,sumxy=0;
	double sxx,syy,sxy;

	*a = 0;
	*b = 0;
	*r = 0;
	if (n < 2)
		return false;

	/* Conpute some things we need */
	for (i=0; i < n; i++)
	{
		sumx += xy_pts[i].first;
		sumy += xy_pts[i].second;
		sumx2 += (xy_pts[i].first * xy_pts[i].first);
		sumy2 += (xy_pts[i].second * xy_pts[i].second);
		sumxy += (xy_pts[i].first * xy_pts[i].second);
	}
	sxx = sumx2 - sumx * sumx / n;
	syy = sumy2 - sumy * sumy / n;
	sxy = sumxy - sumx * sumy / n;

	/*Infinite slope(b), non existant intercept (a)*/
	if (ABS(sxx) == 0)
		return(FALSE);

	/* Calculate the slope (b) and intercept (a) */
	*b = sxy / sxx;
	*a = sumy / n - (*b) * sumx / n;

	/* Compute the regression coefficient */
	if (ABS(syy) == 0)
		*r = 1;
	else
		*r = sxy / sqrt(sxx * syy);

	return true;
}