#include "3dlib.h"
#include <cmath>

// Compute the intersection between two triangles:
bool POLYTOPE::moller_intersection(int faceNumber, POLYTOPE target, int targetFaceNumber)
{
	// For clarity, define two matrices with the vertices of the triangles A and B (B comes from target, A comes from "this")
	vector<vector <double>> A, B;
	for (int i = 0; i < 3; i++)
	{
		A.push_back(vertices[faces[faceNumber][i]]);
		B.push_back(target.vertices[target.faces[targetFaceNumber][i]]);
	}

	
	// Do the same with the normals:
	vector<double> normalA, normalB;
	normalA = facenormals[faceNumber];
	normalB = target.facenormals[targetFaceNumber];

	// STEP 1:
	// Check if the points of B are all on one side of the plane created by A
	// Loop over the three vertices of B:
	vector<double> dotProdsA = { 0, 0, 0 };
	for (int vert = 0; vert < 3; vert++)
	{
		// Compute <normalA, (B[vert] - A0)>
		for (int i = 0; i < 3; i++)
			dotProdsA[vert] += normalA[i] * (B[vert][i] - A[0][i]);
	}

	// STEP 2:
	// Check if all of them are on the same side and reject if it happens:
	int difA = 0;
	if (dotProdsA[0] < 0)
	{
		if (dotProdsA[1] < 0)
		{
			if (dotProdsA[2] < 0)
				return false;
			else
				difA = 2;
		}
		else
		{
			if (dotProdsA[2] < 0)
				difA = 1;
		}
			
	}
	else if (dotProdsA[0] > 0)
	{
		if (dotProdsA[1] > 0)
		{
			if (dotProdsA[2] > 0)
				return false;
			else
				difA = 2;
		}
		else
		{
			if (dotProdsA[2] > 0)
				difA = 1;
		}
	}

	// STEPS 3 AND 4:
	// The same as 1 and 2, but interchanging the plane and the triangle:
	vector<double> dotProdsB = { 0, 0, 0 };
	for (int vert = 0; vert < 3; vert++)
	{
		// Compute <normalB, (A[vert] - B0)>
		for (int i = 0; i < 3; i++)
			dotProdsB[vert] += normalB[i] * (A[vert][i] - B[0][i]);
	}
	// Do the same test:
	int difB = 0;
	if (dotProdsB[0] < 0)
	{
		if (dotProdsB[1] < 0)
		{
			if (dotProdsB[2] < 0)
				return false;
			else
				difB = 2;
		}
		else
		{
			if (dotProdsB[2] < 0)
				difB = 1;
		}

	}
	else if (dotProdsB[0] > 0)
	{
		if (dotProdsB[1] > 0)
		{
			if (dotProdsB[2] > 0)
				return false;
			else
				difB = 2;
		}
		else
		{
			if (dotProdsB[2] > 0)
				difB = 1;
		}
		
	}

	// STEP 5:
	// Find the intersection between the triangle edges and the line where the two planes intersect.
	
	// Store the cross-product of the normals:
	vector<double> D(3);
	D[0] = normalA[1] * normalB[2] - normalA[2] * normalB[1];
	D[1] = normalA[2] * normalB[0] - normalA[0] * normalB[2];
	D[2] = normalA[0] * normalB[1] - normalA[1] * normalB[0];

	// Choose the largest of the absolute value of these numbers:
	int maxD = 1;
	if (abs(D[0]) > abs(D[1]))
		maxD = 0;
	if (abs(D[maxD]) < abs(D[2]))
		maxD = 2;

	// Compute now the parameters tA[0] and tA[1], that represent the t parameter of the edge intersection with the line L: t*D + O
	int p1 = 0;
	int p2 = 1;
	if (difA == 0)
		p1 = 2;
	if (difA == 1)
		p2 = 2;
	vector<double> tA(2);

	tA[0] = A[p1][maxD] + (A[difA][maxD] - A[p1][maxD])*(dotProdsA[p1]/(dotProdsA[p1] - dotProdsA[difA]));
	tA[1] = A[p2][maxD] + (A[difA][maxD] - A[p2][maxD])*(dotProdsA[p2] / (dotProdsA[p2] - dotProdsA[difA]));

	vector<double> tAalt(2);
	tAalt[0] = dotprod(A[p1], D) + (dotprod(A[difA], D) - dotprod(A[p1], D))*(dotProdsA[p1] / (dotProdsA[p1] - dotProdsA[difA]));
	tAalt[1] = dotprod(A[p2], D) + (dotprod(A[difA], D) - dotprod(A[p2], D))*(dotProdsA[p2] / (dotProdsA[p2] - dotProdsA[difA]));
	/* --- DEBUG ---*/
	/*
	printf("A[p1][maxD] = %f\n", A[p1][maxD]);
	printf("(A[difA][maxD] - A[p1][maxD] = %f\n", (A[difA][maxD] - A[p1][maxD]));
	printf("(dotProdsA[p1]/(dotProdsA[p1] - dotProdsA[difA]) = %f\n", (dotProdsA[p1] / (dotProdsA[p1] - dotProdsA[difA])));
	printf("\n");
	printf("dotProdsA[p1] = %f\n", dotProdsA[p1]);
	printf("dotProdsA[p1] = %f\n", dotProdsA[p1]);
	printf("dotProdsA[difA] = %f\n", dotProdsA[difA]);
	printf("(dotProdsA[p1] - dotProdsA[difA]) = %f\n", (dotProdsA[p1] - dotProdsA[difA]));
	*/


	

	if (tA[0] > tA[1])
	{
		double temp = tA[0];
		tA[0] = tA[1];
		tA[1] = tA[0];
	}

	// Same for triangle B
	p1 = 0;
	p2 = 1;
	if (difB == 0)
		p1 = 2;
	if (difB == 1)
		p2 = 2;
	vector<double> tB(2);

	tB[0] = B[p1][maxD] + (B[difB][maxD] - B[p1][maxD])*(dotProdsB[p1] / (dotProdsB[p1] - dotProdsB[difB]));
	tB[1] = B[p2][maxD] + (B[difB][maxD] - B[p2][maxD])*(dotProdsB[p2] / (dotProdsB[p2] - dotProdsB[difB]));

	vector<double> tBalt(2);
	tBalt[0] = dotprod(B[p1], D) + (dotprod(B[difB], D) - dotprod(B[p1], D))*(dotProdsB[p1] / (dotProdsB[p1] - dotProdsB[difB]));
	tBalt[1] = dotprod(B[p2], D) + (dotprod(B[difB], D) - dotprod(B[p2], D))*(dotProdsB[p2] / (dotProdsB[p2] - dotProdsB[difB]));

	if (tB[0] > tB[1])
	{
		double temp = tB[0];
		tB[0] = tB[1];
		tB[1] = tB[0];
	}

	// Finally check if the segments of A and B that intersect with L intersect between them:
	if ((tBalt[0] > tAalt[0] && tBalt[0] < tAalt[1]) || (tBalt[1] > tAalt[0] && tBalt[1] < tAalt[1]))
		return true;

	// The intersection tests did not stop previously, then the triangles intersect:
	printf("\t->Triangle by triangle intersection not found!\n");
	return false;
}						 
