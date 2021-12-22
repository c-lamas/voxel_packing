#include "3dlib.h"
#include <iostream>

void ind2sub(vector<int> &output, vector<int> &matrixSize, int inputNumber)
{
	int nout = output.size();
	int lenmatrixSize = matrixSize.size();


	if (nout > 2)
	{
		vector < int > cumProds(lenmatrixSize);
		cumProds.assign(lenmatrixSize, 1);
		for (int i = 0; i < lenmatrixSize; ++i)
		{
			for (int k = 0; k <= i; ++k)
				cumProds[i] = cumProds[i]*matrixSize[k];
		}
		// cout << "( cumProds = ";
		// for (int j = 0; j < cumProds.size(); ++j)
		// 	cout << cumProds[j] << ",";
		// cout << " ) " << endl;

		for (int i = nout - 1 ; i > 1; --i)
		{
			int prod = cumProds[i - 1];
			int vi = ((inputNumber - 1) % (prod)) + 1;
			int vj = int(double((inputNumber - vi)/prod)) + 1;
			// cout << "i = " << i << endl;
			// cout <<  "prod =" << prod << endl;
			// cout <<  "vi =" << vi << endl;
			// cout <<  "vj =" << vj << endl;
			output[i] = vj;
			inputNumber = vi;
		}
	}

	if (nout >= 2)
	{
		double vi = ((inputNumber-1) % (matrixSize[0])) + 1;
		output[1] = (inputNumber - vi)/matrixSize[0] + 1;
		output[0] = vi;
	}
	else
	{
		output[0] = inputNumber;
	}
}



double dotprod(vector<double> a, vector<double> b)
{
	double result = 0;

	for (int i = 0; i < a.size(); i++)
		result += a[i] * b[i];

	return result;
}


void crossprod(vector<double> v1, vector<double> v2, vector<double> &result)
{
	result = { v1[1] * v2[2] - v1[2] * v2[1], v1[2] * v2[0] - v1[0] * v2[2], v1[0] * v2[1] - v1[1] * v2[0] };
}

void rotate_vertex(vector<double> &vertex, vector<double> angle)
{
	// First construct the rot matrix:
	double RotMat[3][3];
	double sinA, cosA, sinB, cosB, sinG, cosG;
	vector<double> rotatedVertex(3);
	// REFINE (store angle, when rotating many vertices by the same angle, this is redundant!)
	sinA = sin(angle[0]);
	sinB = sin(angle[1]);
	sinG = sin(angle[2]);
	cosA = cos(angle[0]);
	cosB = cos(angle[1]);
	cosG = cos(angle[2]);

	RotMat[0][0] = cosB*cosG;
	RotMat[0][1] = -cosB*sinG;
	RotMat[0][2] = sinB;

	RotMat[1][0] = sinA*sinB*cosG + cosA*sinG;
	RotMat[1][1] = -sinA*sinB*sinG + cosA*cosG;
	RotMat[1][2] = -sinA*cosB;

	RotMat[2][0] = -cosA*sinB*cosG + sinA*sinG;
	RotMat[2][1] = cosA*sinB*sinG + sinA*cosG;
	RotMat[2][2] = cosA*cosB;

	for (int i = 0; i < 3; ++i)
	{
		rotatedVertex[i] = 0;
		for (int j = 0; j < 3; ++j)
		{
			rotatedVertex[i] += RotMat[i][j] * vertex[j];
		}
	}

	vertex = rotatedVertex;

}

void cout_point3(vector<float> point)
{
	cout << "(" << point[0] << "," <<  point[1] << "," << point[2] << ")" << endl;
}
void cout_point3(vector<double> point)
{
	cout << "(" << point[0] << "," <<  point[1] << "," << point[2] << ")" << endl;
}
void cout_point3(vector<int> point)
{
	cout << "(" << point[0] << "," <<  point[1] << "," << point[2] << ")" << endl;
}
void cout_point3(int i, int j, int k)
{
	cout << "(" << i << "," <<  j << "," << k << ")" << endl;
}
void cout_point3(double x, double y, double z)
{
	cout << "(" << x << "," <<  y << "," << z << ")" << endl;
}

//extern const char * print_point3(vector<int> point)
//{
//	char * myString;
//	sprintf(myString, "(%d, %d, %d)", point[0], point[1], point[2]);
//	return myString;
//}
//
//extern const char * print_point3(vector<double> point)
//{
//	char * myString;
//	sprintf(myString, "(%.2f, %.2f, %.2f)", point[0], point[1], point[2]);
//	return myString;
//}


// From rosettacode.org:
// https://rosettacode.org/wiki/Evaluate_binomial_coefficients#C.2B.2B
// Factorial and Binomial coeff.
//int rc_factorial(int nValue)
//{
//	int result = nValue;
//	int result_next;
//	int pc = nValue;
//	do
//	{
//		result_next = result*(pc - 1);
//		result = result_next;
//		pc--;
//	} while (pc>2);
//	nValue = result;
//	return nValue;
//}
//
//int rc_evaluateBinomialCoefficient(int nValue, int nValue2)
//{
//	int result;
//	if (nValue2 == 1)return nValue;
//	result = (rc_factorial(nValue)) / (rc_factorial(nValue2)*rc_factorial((nValue - nValue2)));
//	nValue2 = result;
//	return nValue2;
//}