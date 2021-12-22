#include "3dlib.h"

BOX::BOX()
{
	n_vertices = 8;
	n_faces = 6;

	// Predefine the faces:
	vector<int> aux(4);
	aux[1] = 0;
		// , 1, 4, 2 };
	faces.push_back(aux);
	faces.push_back({ 1, 5, 7, 4 });
	faces.push_back({ 3, 5, 7, 6 });
	faces.push_back({ 0, 3, 6, 2 });
	faces.push_back({ 2, 4, 7, 6 });
	faces.push_back({ 0, 1, 5, 3 });

	// Define the normals of the faces:
	facenormals.push_back({ 0, 0, -1 });
	facenormals.push_back({ 1, 0, 0 });
	facenormals.push_back({ 0, 0, 1 });
	facenormals.push_back({ -1, 0, 0 });
	facenormals.push_back({ 0, 1, 0 });
	facenormals.push_back({ 0, -1, 0 });
}

BOX BOX::duplicate()
{
	BOX copy;
	
	// Inherited properties:
	copy.n_vertices = 	n_vertices;
	copy.vertices	= 	vertices;

	// Box specific properties:
	copy.n_faces = n_faces;
	copy.faces = faces;
	copy.facenormals = facenormals;

	copy.centre = centre;

	copy.minpoint = &copy.vertices[0];
	copy.maxpoint = &copy.vertices[7];

	return copy;
}

void BOX::expand(vector<int> bboxmin, vector<int> bboxmax)
{
	vector<double> bboxminDB;
	vector<double> bboxmaxDB;
	for (int i = 0; i < bboxmin.size(); i++)
	{
		bboxmaxDB.push_back((double)bboxmax[i]);
		bboxminDB.push_back((double)bboxmin[i]);
	}
	expand(bboxminDB, bboxmaxDB);
}

void BOX::expand(vector<double> bboxmin, vector<double> bboxmax)
{


	vector<double> emptyVec;
	emptyVec.assign(3, -100);

	for (int ii = 0; ii < 8; ii++)
	{
		vertices.push_back(emptyVec);
	}
	// return;

	// -- Start setting the vertices where they should be -- //
	for (int ii = 0; ii < 3; ii++)
	{
		vertices[0][ii] = bboxmin[ii];
		vertices[7][ii] = bboxmax[ii];
	}


	// From this point, generate three more modifying one coordinate at a time:
	vector<double> CPointMin, CPointMax;
	CPointMin.resize(3);
	CPointMax.resize(3);

	for (int ii = 0; ii < 3; ii++)
	{
		CPointMin = bboxmin;
		CPointMax = bboxmax;
		CPointMin[ii] = bboxmax[ii];
		CPointMax[ii] = bboxmin[ii];
		for (int jj = 0; jj < 3; jj++)
		{
			vertices[ii + 1][jj] = CPointMin[jj];
			vertices[6 - ii][jj] = CPointMax[jj];
		}

	}

	// Save these values:
	maxpoint = &vertices[7];
	minpoint = &vertices[0];
}

void BOX::locate_at(vector<double> &mvec)
{
	vector<float> mvec2 = { (float)mvec[0], (float)mvec[1], (float)mvec[2] };
	locate_at(mvec2);
}
void BOX::locate_at(vector<float> &mvec)
{
	vector<double> gotoZero = { -1 * (*minpoint)[0], -1 * (*minpoint)[1], -1 * (*minpoint)[2] };
	move_cloud(gotoZero);
	move_cloud(mvec);
}

void BOX::rotate(vector<double> &angle)
{
	// First construct the rot matrix:
	double RotMat[3][3];
	double sinA, cosA, sinB, cosB, sinG, cosG;
	vector<double> rotatedVertex(3), rotatedNormal(3);
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


	// Rotate the vertices:
	for (int vert = 0; vert < 8; ++vert)
	{
		for (int i = 0; i < 3; ++i)
		{
			rotatedVertex[i] = 0;
			for (int j = 0; j < 3; ++j)
			{
				rotatedVertex[i] += RotMat[i][j] * vertices[vert][j];
			}
		}
		vertices[vert] = rotatedVertex;
	}

	// Rotate the normals:
	for (int fac = 0; fac < 6; ++fac)
	{
		for (int i = 0; i < 3; ++i)
		{
			rotatedNormal[i] = 0;
			for (int j = 0; j < 3; ++j)
			{
				rotatedNormal[i] += RotMat[i][j] * facenormals[fac][j];
			}
		}
		facenormals[fac] = rotatedNormal;
	}


}
