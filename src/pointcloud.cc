#include "3dlib.h"

void POINTCLOUD::move_cloud(vector<float> mvec)
{
	for (int jj = 0; jj < 3; jj++)
	{
		// Move all the vertices:
		for (int ii = 0; ii < n_vertices; ii++)
		{
			vertices[ii][jj] += mvec[jj];
		}
	}

}
void POINTCLOUD::move_cloud(vector<double> mvec)
{
	for (int jj = 0; jj < 3; jj++)
	{
		// Move all the vertices:
		for (int ii = 0; ii < n_vertices; ii++)
		{
			vertices[ii][jj] += mvec[jj];
		}
	}

}


void POINTCLOUD::rotate_cloud(int ax, double alpha)
{
	// We want to move around the center, so we go first to the origin:
	/*
	vector<double> mdircen(3);
	for (int ii = 0; ii < 3; ii++)
		mdircen[ii] = -1 * bbox_centre[ii];

	move(mdircen);
	*/

	double cosA = cos(alpha);
	double sinA = sin(alpha);
	double RotMat[3][3];
	if (ax == 0) // Rotate fixing X
	{
		RotMat[0][0] = 1;
		RotMat[0][1] = 0;
		RotMat[0][2] = 0;

		RotMat[1][0] = 0;
		RotMat[1][1] = cosA;
		RotMat[1][2] = -1 * sinA;

		RotMat[2][0] = 0;
		RotMat[2][1] = sinA;
		RotMat[2][2] = cosA;
	}
	else if (ax == 1) // Rotate fixing Y
	{

		RotMat[0][0] = cosA;
		RotMat[0][1] = 0;
		RotMat[0][2] = sinA;

		RotMat[1][0] = 0;
		RotMat[1][1] = 1;
		RotMat[1][2] = 0;

		RotMat[2][0] = -1*sinA;
		RotMat[2][1] = 0;
		RotMat[2][2] = cosA;
	}
	else if (ax == 2) // Rotate fixing Z
	{
		RotMat[0][0] = cosA;
		RotMat[0][1] = -1 * sinA;
		RotMat[0][2] = 0;

		RotMat[1][0] = sinA;
		RotMat[1][1] = cosA;
		RotMat[1][2] = 0;
		
		RotMat[2][0] = 0;
		RotMat[2][1] = 0;
		RotMat[2][2] = 1;
	}
	double curr_vert;

	for (int ii = 0; ii < n_vertices; ii++)
	{
		for (int jj = 0; jj < 3; jj++)
		{
			


			// Multiply the vertices:
			curr_vert = 0;
			 0;
			for (int coord = 0; coord < 3; coord++)
			{
				curr_vert += vertices[ii][coord] * RotMat[jj][coord];
			}

			vertices[ii][jj] = curr_vert;
		}
	}

	/*
	// Multiply the coordinates of the bounding box:
	double curr_bboxmax, curr_bboxmin;
	for (int ii = 0; ii < 3; ii++)
	{
		// Multiply the vertices:
		curr_bboxmax = 0;
		curr_bboxmin = 0;
		for (int jj = 0; jj < 3; jj++)
		{
			curr_bboxmin += bboxmin[ii] * RotMat[ii][jj];
			curr_bboxmax += bboxmax[ii] * RotMat[ii][jj];
		}
		bboxmin[ii] = curr_bboxmin;
		bboxmax[ii] = curr_bboxmax;
	}

	// Move back to its position:
	move(bbox_centre);
	*/
}