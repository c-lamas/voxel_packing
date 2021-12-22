#include "3dlib.h"

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>


//#define TOL 0.00000001

// vector<POLYTOPE*>  globalElements;
class PACKING_LAYOUT;
PACKING_LAYOUT * finalLayout;

POLYTOPE::POLYTOPE()
{
	n_vertices = 0;
	// readPLY(filename);
	// globalElements.push_back(this);
	/*bboxvertex.resize(8);
	for (int ii = 0; ii < 8; ii++)
		bboxvertex[ii].resize(3);*/
}

void POLYTOPE::place_at(vector<float> mvec)
{
	vector<double> move_front(3);
	vector<double> move_back(3);
	for (int i = 0; i < 3; i++)
	{
		move_front[i] = (double)mvec[i];
		move_back[i] = -1* (* box.minpoint)[i];
	}
	move(move_back);
	move(move_front);
	voxel.corner = {(double) mvec[0], (double) mvec[1], (double) mvec[2] };
}

void POLYTOPE::place_at(vector<double> mvec)
{
	vector<double> move_back(3);
	for (int i = 0; i < 3; i++)
	{
		move_back[i] = -1* (* box.minpoint)[i];
	}
	move(move_back);
	move(mvec);
	voxel.corner = mvec;
}

void POLYTOPE::move(vector<double> mvec, bool onlyVoxel)
{
	// move_cloud(mvec);
	box.move_cloud(mvec);
	voxel.move(mvec);
}
void POLYTOPE::move(vector<double> mvec)
{
	move_cloud(mvec);
	box.move_cloud(mvec);
	voxel.move(mvec);
}

void POLYTOPE::rotate(int ax, double alpha)
{
	rotate_cloud(ax, alpha);
	box.rotate_cloud(ax, alpha);
	voxel.rotAngle[ax] += alpha;
}

void POLYTOPE::update_from_voxel()
{
	vector<double> bboxmin(3);
	bboxmin[0] = 0;
	bboxmin[1] = 0;
	bboxmin[2] = 0;

	vector<double> bboxmax(3);
	bboxmax[0] = voxel.resolution*voxel.width;
	bboxmax[1] = voxel.resolution*voxel.height;
	bboxmax[2] = voxel.resolution*voxel.depth;

	box.expand(bboxmin, bboxmax);

	modelname = voxel.name;
}

void POLYTOPE::voxelize(double resolution)
{
// Debug variables:
bool verboseDebug = false;

cout << "Starting voxelization..." << endl;


vector<int> ncubes;
ncubes.assign(3, -1);
voxel.corner.assign(3, -1);

// The corner of the voxel has to be related to the overall grid, defined by the "resolution" parameter:
// voxel.corner = {box.vertices[0][0], box.vertices[0][1], box.vertices[0][2]};
voxel.corner = {	floor(box.vertices[0][0] / resolution)*resolution, 
					floor(box.vertices[0][1] / resolution)*resolution,
					floor(box.vertices[0][2] / resolution)*resolution };

voxel.resolution = resolution;

for (int i = 0; i < 3; ++i)
{
	//ncubes[i] = int( (vector[7][i] - vector[0][i]) / resolution );
	ncubes[i] = (int) ( (box.vertices[7][i] - box.vertices[0][i]) / resolution );
	ncubes[i] += 2;
}
//cout << "ncubes = " << ncubes[0] << ", " << ncubes[1] << ", " << ncubes[2] << endl;
// printf("ncubes = %d, %d, %d\n", ncubes[0], ncubes[1], ncubes[2]);

// bool * raster = new bool[ncubes[0] * ncubes[1] * ncubes[2]]; 

vector<bool> tempBool;
tempBool.assign(ncubes[2], false);

vector<vector <bool>> tempBool2;
tempBool2.assign(ncubes[1], tempBool);

voxel.map.assign(ncubes[0], tempBool2);

// Step 1: go triangle by triangle and find the shell:
vector<double> minVals(3), maxVals(3), triVoxelLengths(3), minValsVoxelCoord(3);
minVals = voxel.corner;
maxVals = minVals;

vector<double> currentVert(3);
vector<int> currentVertVox(3);






/*
		LOOP OVER THE TRIANGLES
*/
// Define the variables to use inside the loop:
vector<int> minValsVox(3), maxValsVox(3), boxSteps(3);
double dotProdVal = 0.;
for (int currentFace = 0; currentFace < n_faces; ++currentFace)
{

	// Determine the bounding box of the triangle:
	for (int triVert = 0; triVert < 3; ++triVert)
	{
		// For clarity, this is the vertex where we are now:
		currentVert = vertices[faces[currentFace][triVert]];

		// Determine the bounding box of the triangle:
		for (int coord = 0; coord < 3; ++coord)
		{
			if (minVals[coord] > currentVert[coord])
				minVals[coord] = currentVert[coord];
			else if (maxVals[coord] < currentVert[coord])
				maxVals[coord] = currentVert[coord];
		}
	} // End of loop over the three vertices of the triangle

	// Convert the triangle bounding box extrema to voxel coordinates:
	for (int i = 0; i < 3; ++i)
	{
		minValsVox[i] = (int)((minVals[i] - voxel.corner[i]) / resolution);
		maxValsVox[i] = (int)((maxVals[i] - voxel.corner[i]) / resolution);
		boxSteps[i] = maxValsVox[i] - minValsVox[i];
		if (boxSteps[i] == 0)
			boxSteps[i] = 1;
	}







	// Create a matrix to store the dot products:
	vector<vector< vector<bool> > > dotProdSigns(boxSteps[0] + 2, vector<vector<bool>>(boxSteps[1] + 2, vector<bool>(boxSteps[2] + 2, 1)));
	vector<double> currentGridPoint(3);
	// Loop over the grid, compute the dot products and save them in dotProdSigns:
	for (int i = 0; i < boxSteps[0] + 2; ++i)
	{
		for (int j = 0; j < boxSteps[1] + 2; ++j)
		{
			for (int k = 0; k < boxSteps[2] + 2; ++k)
			{
				// Corresponding grid point:
				currentGridPoint[0] = minVals[0] + i * resolution;
				currentGridPoint[1] = minVals[1] + j * resolution;
				currentGridPoint[2] = minVals[2] + k * resolution;
				if (i == 3 && j == 2 && k == 1)
					cout << "oooh\n";

				if (i == 2 && j == 2 && k == 0)
					cout << "my god\n";

				// Compute of:
				// The normal of the triangle and the vector from vertex 0 of the triangle and the currentGridPoint
				dotProdVal = 0.;
				for (int coord = 0; coord < 3; ++coord)
					dotProdVal += (currentGridPoint[coord] - vertices[faces[currentFace][0]][coord])*facenormals[currentFace][coord];

				if (dotProdVal < -TOL)
					dotProdSigns[i][j][k] = false;
				else if (dotProdVal > TOL)
					dotProdSigns[i][j][k] = true;
				else
				{
					dotProdSigns[i][j][k] = false;
					voxel.map[minValsVox[0] + i][minValsVox[1] + j][minValsVox[2] + k] = true;
					if (minValsVox[0] + i + minValsVox[1] + j + minValsVox[2] + k == 15)
						cout << "setting the 0\n!" << endl;
				}

			} // loop k
		} // loop j
	} // loop i


	// Loop over the grid again, and see which cubes intersect with the triangle:
	for (int i = 0; i < boxSteps[0] + 1; ++i)
	{
		for (int j = 0; j < boxSteps[1] + 1; ++j)
		{
			for (int k = 0; k < boxSteps[2] + 1; ++k)
			{
				// Now check if any of the 8 vertices has a different sign:
				if ((dotProdSigns[i][j][k] ^ dotProdSigns[i + 1][j][k]) ||
					(dotProdSigns[i][j][k] ^ dotProdSigns[i + 1][j + 1][k]) ||
					(dotProdSigns[i][j][k] ^ dotProdSigns[i + 1][j + 1][k + 1]) ||
					(dotProdSigns[i][j][k] ^ dotProdSigns[i][j + 1][k]) ||
					(dotProdSigns[i][j][k] ^ dotProdSigns[i][j + 1][k + 1]) ||
					(dotProdSigns[i][j][k] ^ dotProdSigns[i][j][k + 1]) ||
					(dotProdSigns[i][j][k] ^ dotProdSigns[i + 1][j][k + 1]))
				{
					// cout << "Found a shell intersection!" << endl;
					voxel.map[minValsVox[0] + i][minValsVox[1] + j][minValsVox[2] + k] = true;
					if (minValsVox[0] + i + minValsVox[1] + j + minValsVox[2] + k == 15)
						cout << "setting the 0\n!" << endl;
				}
			} // loop k
		} // loop j
	} // loop i

} // End of loop over the mesh faces

/*
	END OF LOOP OVER THE FACES
*/







// We finished the process, mark that it is initialised:
voxel.init = true;
cout << "Voxelization finished!" << endl;
}



void POLYTOPE::readPLY(string filename)
{

	cout << "Reading the file \"" << filename << "\"..." << '\n';
	string currentWord;
	currentWord.reserve(80);
	ifstream in(filename);
	in >> currentWord;
	
	// Check that is a ply file:
	if (currentWord.compare("ply") != 0)
	{
		cout << "It's not a ply file" << '\n';
//		return;
	}
	// else
		// cout << "It's a ply file" << '\n';

	bool inHeader = true;
	string keyword[5];
	keyword[0].assign("element");
	keyword[1].assign("end_header");
	keyword[2].assign("format");
	keyword[3].assign("property");
	keyword[4].assign("comment");
	
	// Initialise the vertices and faces counter:
	n_vertices = -1;
	n_faces = -1;

	// Keep track of the found keywords:
	bool keywordFound = false;

	// So far, we only use the properties to find the normals
	// We track them with the following variables:
	string pType;
	int pCount = 0;
	int NormalsAt = 0;
	
	while (inHeader)
	{
		in >> currentWord;
		// see if there is any keyword here:
		for (int ii = 0; ii < 5; ii++)
		{
			keywordFound = (currentWord.compare(keyword[ii]) == 0);
			if (keywordFound)
			{
				switch (ii){
				// Keyword: element
				case (0) :
					// read the next word (type of element)
					in >> currentWord;
					if (currentWord.compare("vertex") == 0)
					{
						// read the number of vertex!
						in >> n_vertices;
					}
					else if (currentWord.compare("face") == 0)
					{
						// read the number of faces!
						in >> n_faces;
					}
					else
						cout << "Warning: Unknown element type \"" << currentWord << "\"" << '\n';
					break;

				// Keyword: end_of_header
				case(1) :
					inHeader = false;
					cout << "Reached end of header." << '\n';
					break;

				// Keyword: format
				case(2) :
					break;

				// Keyword: property
				case(3) :
					in >> currentWord;
					in >> pType;
					if (pType.compare("nx") == 0)
					{
						NormalsAt = pCount;
					}
					pCount++;
					break;

				// Keyword: comment
				case(4) :
					// skip till the end of line!
					break;

				default:
					cout << "Warning: found a keyword (\"" << keyword[ii] << "\") but no action taken." << '\n';


				}
				break;
			}
		}

	} // while inHeader

	// --- Read the vertices --- //

	// At this point, we know the size of the matrix:
	int n_rows = n_vertices;

	// Resize the element variables (rows)
	vertextrack.resize(n_vertices);
	facenormals.resize(n_faces);
	vertexnormals.resize(n_vertices);

	// Bounding box:
	vector<double> bboxmax, bboxmin;
	bboxmax.assign(3, -100);
	bboxmin.assign(3, -100);
	// bbox_centre.assign(3, -100);

	// Resize the vertex atribute:
	vector<vector<double> > vertices(n_vertices, std::vector<double>(3));

	double tempcoordinate = -1;
	for (int n_vert = 0; n_vert < n_vertices; n_vert++)
	{
		// Loop over the 3 coordinates:
		for (int jj = 0; jj < 3; jj++)
		{

			// retrieve the number:
			in >> tempcoordinate;

			// add it to the corresponding point in the matrix:
			vertices[n_vert][jj] = tempcoordinate;

			/*
			// Create the bounding box:
			if (n_vert == 1)
			{
				bboxmin[jj] = tempcoordinate;
				bboxmax[jj] = tempcoordinate;
			}
			else
			{
				if (tempcoordinate < bboxmin[jj])
					bboxmin[jj] = tempcoordinate;
				else if (tempcoordinate > bboxmax[jj])
					bboxmax[jj] = tempcoordinate;
			}
			*/

		} // End of loop over the 3 coordinates

		// Create the centre of the bbox:
		/*
		for (int ii = 0; ii < 3; ii++)
			bbox_centre[ii] = bboxmin[ii] + (bboxmax[ii] - bboxmin[ii]) / 2;
			*/

		if (NormalsAt > 0)
		{
			// Move till we find the normals:
			for (int jj = 3; jj < NormalsAt; jj++)
				in >> tempcoordinate;
			// Store them!
			for (int jj = 0; jj < 3; jj++)
			{
				in >> tempcoordinate;
				vertexnormals[n_vert].push_back(tempcoordinate);
			}
		}

	} // End of loop over the lines with vertices

	// --- Read the faces and compute its normals --- //

	// Resize the "matrix" to fit the number of rows (the num. of columns is variable)
	vector<vector<int> > faces(n_faces, std::vector<int>(0));

	string line;
	int tempnum = -1;
	// get first line (it will be empty):
	getline(in, line);

	// Vars to compute the normals:
	vector<double> v1, v2;

	for (int ii = 0; ii < n_faces; ii++)
	{
		getline(in, line);
		
		// Create a stream from this line:
		// we need sstream for this!
		istringstream row(line);

		// push back the rest of the line in the matrix:
		while (row >> tempnum)
		{
			faces[ii].push_back(tempnum);
			// At the same time, we keep track of which vertices are in which faces:
			vertextrack[tempnum].push_back(ii);
		}
			

		//

		/* The following code should be replaced by:
		while (determinant == 0)
			determinant of 3 vertices
		this will make sure no aligned points are taken and remove the for.
		*/
		// Compute the normal of the face and save it in facenormals
		//for (int fels = 0; fels < faces[ii].size(); fels++)
		//{
		//	
		//}
		if (NormalsAt == 0)
		{
		for (int coord = 0; coord < 3; coord++)
		{
			v1.push_back(vertices[faces[ii][1]][coord] - vertices[faces[ii][0]][coord]);
			v2.push_back(vertices[faces[ii][2]][coord] - vertices[faces[ii][1]][coord]);
		}
		// Compute the normal:
		facenormals[ii] = { v1[1] * v2[2] - v1[2] * v2[1], v1[2] * v2[0] - v1[0] * v2[2], v1[0] * v2[1] - v1[1] * v2[0] };
		}


	} // End of loop over the lines with face info
	if (NormalsAt == 0)
	{
		vector<double> finalnormal(3);
		double venorm = -1;
		// Loop again over the vertices, to average the normals of the adjacent faces:
		for (int ii = 0; ii < n_vertices; ii++)
		{
			// Loop over the adjacent faces of that vertex
			for (int adface = 0; adface < vertextrack[ii].size(); adface++)
			{
				for (int coord = 0; coord < 3; coord++)
				{
					finalnormal[coord] += facenormals[vertextrack[ii][adface]][coord];
				}
			} // end of loop over the adjacent faces of that vertex

			// Normalise and save the normal vector of this vertex:
			venorm = -1 * sqrt(powf(finalnormal[0], 2) + powf(finalnormal[1], 2) + powf(finalnormal[2], 2));
			vertexnormals[ii] = { finalnormal[0] / venorm, finalnormal[1] / venorm, finalnormal[2] / venorm };
		}
	}

	cout << "Reached end of ply file." << '\n' << '\n';
	// So far, the faces and vertex variables are local copies, not member of the object
	// using the -> operator, we can fix this:
	/* This might be improved, as at this point, the vertices and faces are duplicated in memory! */
	this->faces = faces;
	this->vertices = vertices;

	// Find all the vertices of the bounding box:
	//expand_bbox();
	box.expand(bboxmin, bboxmax);

}







// Detect if a point is inside the bounding box of this polytope:
bool POLYTOPE::in_bbox(vector<double> point)
{
	vector<double> maxPoint = box.vertices[7];
	vector<double> minPoint = box.vertices[0];
	int CoordInside = 0;
	for (int ii = 0; ii < 3; ii++)
	{
		if (point[ii] <= maxPoint[ii] && point[ii] >= minPoint[ii])
			CoordInside++;
	}
	if (CoordInside == 3)
		return true;
	else
		return false;
}

bool POLYTOPE::intersect(POLYTOPE target)
{

	bool findIntersection = true;
	// double inPlane;
	
	double prop_cos_n_px;

	// STEP 1:
	// check if their bounding boxes intersect:

	// Loop over all the faces of the bounding box:
	for (int fc = 0; fc < 6; fc++)
	{
		findIntersection = false;
		// Our point p is: vertices[faces[fc][0]]

		// Loop over the vertices of the second bounding box:
		for (int pt = 0; pt < 8; pt++)
		{
			// Initialize the variable proportinal to the cos of n and px:
			prop_cos_n_px = 0;

			// Choose one point from the face (P) and the point we are dealing with (X)
			// and construct the vector PX
			// We do not need to construct it, directly make the scalar product, based on this: // px[i] = target.box.vertices[pt][i] - (*p)[i];
			// prop_cos_n_px should be divided by the px modulus, but as this is positive and we only want to check the sign, we can skip that
			for (int i = 0; i < 3; i++)
				prop_cos_n_px += (target.box.vertices[pt][i] - box.vertices[box.faces[fc][0]][i]) * box.facenormals[fc][i];
				
			//printf("Face number %d\n", fc);
			//printf("Cos. value %f\n\n", prop_cos_n_px);

			// Now check

			if (prop_cos_n_px < 0)
			{
				/*
				cout << "\n --- Found intersection (bounding box) ---\n" << endl;
				printf("Face number %d, ", fc);
				// printf("Cos. value %f\n", prop_cos_n_px);
				printf("Face number %d, ", fc);
				printf("Conflicting point (%f, %f, %f)\n", target.box.vertices[pt][0], target.box.vertices[pt][1], target.box.vertices[pt][2]);
				*/

				findIntersection = true;
				break;
			}

		}

		if (!findIntersection)
			break;

	}

	if (!findIntersection)
	{
		cout << "\n\t->Intersection NOT found (bounding box)\n" << endl;
		return false;
	}
	else
	{
		cout << "\n\t->Found intersection (bounding box)\n" << endl;
	}
		


	// --- Convex Hull intersection --- //
	/*
	// To be completed

	if (!findIntersection)
		return false;
	*/

	// --- Face by face intersection --- //
	cout << "\n\t->Checking face to face intersection...\n" << endl;

	// Loop over the faces of this element:

	findIntersection = false;

	for (int currentFace = 0; currentFace < n_faces; currentFace++)
	{
		if ((currentFace % n_faces/100) == 0)
			printf("\t%d / %d\n", currentFace, n_faces);

		// Loop over the faces of the target element:
		for (int currentTargetFace = 0; currentTargetFace < target.n_faces; currentTargetFace++)
		{
			findIntersection = moller_intersection(currentFace, target, currentTargetFace);
			
			if (findIntersection)
			{
				// Start printing:
				printf("\t\t*** faces %d and %d intersect ***\n", currentFace, currentTargetFace);
				printf("\t\tElement 1, Face %d vertices:\n", currentFace);
				for (int printfac = 0; printfac < 3; printfac++)
				{
					printf("\t\t\t(%f, %f, %f)\n", vertices[faces[currentFace][printfac]][0], vertices[faces[currentFace][printfac]][1], vertices[faces[currentFace][printfac]][2]);
				}
				printf("\t\tElement 2, Face %d vertices:\n", currentTargetFace);
				for (int printfac = 0; printfac < 3; printfac++)
				{
					printf("\t\t\t(%f, %f, %f)\n",
						target.vertices[target.faces[currentTargetFace][printfac]][0],
						target.vertices[target.faces[currentTargetFace][printfac]][1],
						target.vertices[target.faces[currentTargetFace][printfac]][2]);
				}
				// End of printing
				break;
			}
				

		} // end of loop over the target faces

		if (findIntersection)
			break;

	} // end of loop over the source faces
	

	return findIntersection;

	/*
	// Loop over the faces and see if any of them intersect!
	for (int currentFace = 0; currentFace < n_faces; currentFace++)
	{ 
		findIntersection = false;
		// Our point p is: vertices[faces[fc][0]]


		for (int currentTargetFace = 0; currentTargetFace < target.n_faces; currentTargetFace++)
		{
			// Loop over the vertices of the currentTargetFace:
			for (int pt = 0; pt < 3; pt++)
			{
				// Initialize the variable proportinal to the cos of n and px:
				prop_cos_n_px = 0;

				for (int i = 0; i < 3; i++)
					prop_cos_n_px += (target.vertices[target.faces[currentTargetFace][pt]][i] - vertices[faces[currentFace][0]][i]) * facenormals[currentFace][i];

				if (prop_cos_n_px < 0)
				{
					// Start printing:
					printf("\t\t*** faces %d and %d intersect ***\n", currentFace, currentTargetFace);
					printf("\t\tElement 1, Face %d vertices:\n", currentFace);
					for (int printfac = 0; printfac < 3; printfac++)
					{
						printf("\t\t\t(%f, %f, %f)\n", vertices[faces[currentFace][printfac]][0], vertices[faces[currentFace][printfac]][1], vertices[faces[currentFace][printfac]][2]);
					}
					printf("\t\tElement 2, Face %d vertices:\n", currentTargetFace);
					for (int printfac = 0; printfac < 3; printfac++)
					{
						printf("\t\t\t(%f, %f, %f)\n", 
							target.vertices[target.faces[currentTargetFace][printfac]][0],
							target.vertices[target.faces[currentTargetFace][printfac]][1],
							target.vertices[target.faces[currentTargetFace][printfac]][2]);
					}
					// End of printing

					findIntersection = true;
					break;
				}

			} // end of loop over the 3 vertices of the currentTargetFace
			if (findIntersection)
				break;

		} // end of loop over the target elemement faces

		if (findIntersection)
		{
			printf("\t\tFace %d intersects with something, enough checks.\n", currentFace);
			findIntersection = true;
			break;
		}

	} // end of loop over this object faces

	if (findIntersection)
	{
		cout << "\n\t->Found intersection (faces) ---\n" << endl;
	}
	else
		cout << "\n\t->Intersection NOT found (faces) ---\n" << endl;

	return findIntersection;

	*/
}


POLYTOPE POLYTOPE::duplicate()
{
	// Create a new polytope and copy all the features from the model:
	POLYTOPE copy;
	copy.n_vertices = n_vertices;
	copy.n_faces = n_faces;
	copy.vertices = vertices;
	copy.faces = faces;
	copy.facenormals = facenormals;
	copy.vertexnormals = vertexnormals;
	copy.vertextrack = vertextrack;
	copy.box = box.duplicate();
	copy.voxel = voxel;
	copy.modelname = modelname;
	return copy;
}
/*
void POLYTOPE::expand_bbox()
{

	// If it is already initialised, do not modify it at all:
//	if (bboxvertex.size() > 0)
	//	return;

	// Otherwise, resize it first:
	//bboxvertex.resize(8);
	vector<double> emptyVec;
	emptyVec.assign(3, -100);

	for (int ii = 0; ii < 8; ii++)
	{
		box.vertices.push_back(emptyVec);
	}
	// return;

	// -- Start setting the vertices where they should be -- //
	for (int ii = 0; ii < 3; ii++)
	{
		box.vertices[0][ii] = box.vertices[ii];
		box.vertices[7][ii] = box.vertices[ii];
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
			bboxvertex[ii + 1][jj] = CPointMin[jj];
			bboxvertex[6 - ii][jj] = CPointMax[jj];
		}

	}
	

}
*/

NFV::NFV(int pfix, int pmov, vector<POLYTOPE*> pc)
{
	NFV::NFV(pfix, pmov, pc, false);
}

NFV::NFV(int pfix, int pmov, vector<POLYTOPE*> pc, bool fromFile) 
{ 
	// Generate NFV of i and j
	// pfix is the piece wich stays fixed and pmov is the one that moves around
	// NFV(vfix,vmov) records the valid positions of vmov when vfix is in (0,0)

	bool verbose = true;
	
	if (fromFile)
		verbose = false;

	// Start measuring the time:
	clock_t reloj, reloj2;
	if (!fromFile)
		reloj = clock();
	
	// Point to the voxels i and j for clarity later on:
	VOXEL * vfix = &(pc[pfix]->voxel);
	VOXEL * vmov = &(pc[pmov]->voxel);

	// Set the properties of NFV:
	idx_i = pfix;
	idx_j = pmov;

	size_i = vfix->get_grid_size(0) + vmov->get_grid_size(0) - 1;
	size_j = vfix->get_grid_size(1) + vmov->get_grid_size(1) - 1;
	size_k = vfix->get_grid_size(2) + vmov->get_grid_size(2) - 1;

	// Start the loops for intersections:
	density = 0.0;

	if (!fromFile) // If from file, this is set
		nPoints = 0;

	vector<int> aux_vec(3);

	if (!fromFile) // If reading from a file, the points are already there
	{
		for (int i = 1 - vmov->width; i < 1 - vmov->width + size_i; i++)
		{
			for (int j = 1 - vmov->height; j < 1 - vmov->height + size_j; j++)
			{
				for (int k = 1 - vmov->depth; k < 1 - vmov->depth + size_k; k++)
				{
					// See if vmov placed in (i,j,k) intersects vfix placed in (0,0,0)
					// vmov->corner[0] = vmov->resolution * i;
					// vmov->corner[1] = vmov->resolution * j;
					// vmov->corner[2] = vmov->resolution * k;
					vmov->cornerInd[0] = i;
					vmov->cornerInd[1] = j;
					vmov->cornerInd[2] = k;
					
					
					if (!vfix->voxel_voxel_intersection(*vmov, { 0, 0, 0 }, { i, j, k }))
					{
						// This is a valid point, we need to add it:
						// aux_vec[0] = i;
						// aux_vec[1] = j;
						// aux_vec[2] = k;
						// validPoints.push_back(aux_vec);
						add_point(i, j, k);


						// Debug:
						// cout << "Added point: ";
						// cout_point3({ i, j, k });
						// cout << endl;

						// density += 1.0;
					}
					
				}
			}
		}
	}



	// Calculate the density:
	if (!fromFile)
		density = (float) nPoints / (float) (size_i*size_j*size_k);

	// if (ypoints.size() > 0)
		// cout << ypoints[0].value << ", " << ypoints[1].value << ", " << ypoints[2].value << ", " << ypoints[3].value << ", " << ypoints[4].value << endl;

	// Finally, sort the nodes so they are in increasing size:
	if (!fromFile)
	{
		std::sort(ypoints.begin(), ypoints.end(), &NFVYNODE::sort_less_than);
		for (int yidx = 0; yidx < ypoints.size(); yidx++)
		{
			// std::sort(ypoints[yidx].xpoints.begin(), ypoints[yidx].xpoints.end(), ypoints[yidx].xpoints[0].sort_less_than);
			std::sort(ypoints[yidx].xpoints.begin(), ypoints[yidx].xpoints.end(), &NFVXNODE::sort_less_than);
			for (int xidx = 0; xidx < ypoints[yidx].xpoints.size(); xidx++)
			{
				std::sort(ypoints[yidx].xpoints[xidx].zpoints.begin(), ypoints[yidx].xpoints[xidx].zpoints.end(), &NFVXNODE::sort_less_than_zpoints);
			}
		}

	}
	// Show some info:
	if (verbose)
	{
		cout << endl << endl;
		if (!fromFile)
			cout << "No-Fit Voxel created for pieces " << pfix << " and " << pmov << "!" << endl;
		else
			cout << "No-Fit Voxel created (from file) for pieces " << pfix << " and " << pmov << "!" << endl;
		cout << "Found " << nPoints << " valid positions";
		cout << " which represents a density of " << density * 100 << "%" << endl;
	}

	// if (ypoints.size() > 0)
		// cout << ypoints[0].value << ", " << ypoints[1].value << ", " << ypoints[2].value << ", " << ypoints[3].value << ", " << ypoints[4].value << endl;

	// Set the creation time:
	if (!fromFile)
	{
		reloj2 = clock();
		creationTime = ((float)reloj2 - (float)reloj) / CLOCKS_PER_SEC;
	}
	else
		creationTime = -1; // To be read from file maybe...

	if (verbose && !fromFile)
		cout << "Creation time was: " << creationTime << " s." << endl;
	// Done!
}
void NFV::add_point(int i_val, int j_val, int k_val) { 
	
	// Look if there is a j_val in the Y-points:
	bool yIsThere = false;
	bool xIsThere = false;
	bool zIsThere = false;
	NFVYNODE * pointToY = NULL;
	NFVXNODE * pointToX = NULL; 

	for (int j = 0; j < ypoints.size(); j++)
	{
		if ( ypoints[j].value == j_val)
		{
			yIsThere = true;
			pointToY = &ypoints[j];
			break;
		}
	}
	if (!yIsThere)
	{
		NFVYNODE aux_node(j_val);
		ypoints.push_back(aux_node);
		pointToY = &ypoints.back();
	}

	// Y is cared for, let's check X
	for (int i = 0; i < pointToY->xpoints.size(); i++)
	{
		if ( pointToY->xpoints[i].value == i_val)
		{
			xIsThere = true;
			pointToX = &pointToY->xpoints[i];
			break;
		}
	}

	if (!xIsThere)
	{
		NFVXNODE aux_node_x(i_val);
		pointToY->xpoints.push_back(aux_node_x);
		pointToX = &pointToY->xpoints.back();
	}

	// X is cared for, check Z:
	for (int k = 0; k < pointToX->zpoints.size(); k++)
	{
		if (pointToX->zpoints[k] == k_val)
		{
			zIsThere = true;
			break;
		}
	}

	if (!zIsThere)
		pointToX->zpoints.push_back(k_val);


	// Finally:
	nPoints++;
}

int NFV::closest_point(int &i, int &j, int &k)
{
	int maxDistPar = -1;
	return closest_point(i, j, k, maxDistPar);
}

int NFV::closest_point(int &i, int &j, int &k, int &maxDist)
{
	bool sanityCheck = true;
	// Check the closest valid point to the given i,j,k
	// Returns the point in i,j,k!!!!
	// First, check if this point is valid:
	vector<int> closestPoint(3);
	closestPoint.assign(3, 0);
	int minDisp = maxDist;
	if (maxDist < 0) // Do all the pre-process
	{
		// cout << "WARNING: closest_point pre-process might be incorrect!!!!" << endl;
		if (has_point(i, j, k))
		{
			i = 0;
			j = 0;
			k = 0;
			return 0;
		}
		
	// 1 - Check the closest edge
	// 2 - Generate point in that direction (0, ceil(bound_size/2), 0) (debug : sure about ceil?)
	// 3 - Generate a cube of side bound_size/2, to test any closer point
	// 4 - Loop the cube with has_point statements, return the first found.

		// 1 - Check the closest edge
		// i
		int bestDir = 0;
		minDisp = ceil(size_i) - i;
		int proposedDisp = int(ceil(size_i)) + i;
		if (proposedDisp < minDisp)
		{
			minDisp = proposedDisp;
		}
		// j
		proposedDisp = int(ceil(size_j)) + j;
		if (proposedDisp < minDisp)
		{
			minDisp = proposedDisp;
			bestDir = 1;
		}
		proposedDisp = int(ceil(size_j)) + j;
		if (proposedDisp < minDisp)
		{
			minDisp = proposedDisp;
			bestDir = 1;
		}
		// k
		proposedDisp = int(ceil(size_k)) + k;
		if (proposedDisp < minDisp)
		{
			minDisp = proposedDisp;
			bestDir = 2;
		}
		proposedDisp = int(ceil(size_k)) + k;
		if (proposedDisp < minDisp)
		{
			minDisp = proposedDisp;
			bestDir = 2;
		}

		// So far, this is our best bet:
		closestPoint[bestDir] = minDisp;
		maxDist = minDisp;


		// REMOVE INITIAL CHECK!!!
		minDisp = min(size_i,size_j);
		minDisp = min(minDisp, size_k);
	}

	// 3 - Generate a cube of side 2*(minDisp - 1), centre in i,j,k
	int cubeRad = minDisp;
	// 4 - Loop the cube with has_point statements, return the first found.
	// debug : speed Faster if this loops in an outward spiral (we just return the first point)
	for (int ci = -cubeRad; ci < cubeRad; ci++)
	// for (int ci = 0; ci < size_i; ci++)
	{
		for (int cj = -cubeRad; cj < cubeRad; cj++)
		// for (int cj = 0; cj < size_j; cj++)
		{
			for (int ck = -cubeRad; ck < cubeRad; ck++)
			// for (int ck = 0; ck < size_k; ck++)
			{
				int pointDist = floor(sqrt(pow(ci, 2) + pow(cj, 2) + pow(ck, 2)));
				if (pointDist >= minDisp)
					continue; // We are no longer interested in this point

				if (has_point(i + ci, j + cj, k + ck))
				{
					// Found a closer match:
					closestPoint[0] = ci;
					closestPoint[1] = cj;
					closestPoint[2] = ck;
					minDisp = pointDist;
					if (minDisp < 2)
						break;
				}
			}

			if (minDisp < 2)
				break;
		}
		if (minDisp < 2)
			break;
	}

	// Return:
	i = closestPoint[0];
	j = closestPoint[1];
	k = closestPoint[2];

	if (sanityCheck)
	{
		// cout << "Sanity check on! (check_distance_is_one)" << endl;
		if (minDisp < -TOL)
			cout << "ERROR: Distance is less than one!!! minDisp = " << minDisp << ", initial estimate was " << maxDist << " ( closest_point() )" << endl;

	}
	return minDisp;
}

// Check if it contains a point or not:
bool NFV::has_point(int i_coord, int j_coord, int k_coord)
{
	return has_point(i_coord, j_coord, k_coord, false);
}

bool NFV::has_point(int i_coord, int j_coord, int k_coord, bool reverse)
{
	int multiplier = 1;

	// For the reverse nfv, coordinates change sign:
	if (reverse)
		multiplier = -1;
		
	// Prevent the function from looking in empty NFV's
	if (ypoints.size() == 0)
		return false;
	// Assume all nodes are sorted in increasing order!
	int j_node = -1;
	// START FROM J!!!!!! :)
	for (int j = 0; j < ypoints.size(); j++)
	{
		if (ypoints[j].value >= multiplier*j_coord)
		{
			if(ypoints[j].value == multiplier*j_coord) 
				j_node = j;

			break;
		}

// 		if (ypoints[j].value > j_coord)
// 			return false;
	}
	
	if (j_node == -1)
		return false;
	//vector<int> thePoint = { i_coord, j_coord, k_coord }; 
	//cout << "j_node " << j_node << endl;
	//cout << "ypoints.size() " << ypoints.size() << endl;
	//cout << "ypoints[j_node].value " << ypoints[j_node].value << endl;
	//cout << "i_coord " << i_coord << endl;
	//cout << "j_coord " << j_coord << endl;
	//cout << "k_coord " << k_coord << endl;
	//cout << "i_coord " << i_coord << endl;
	//cout << "j_coord " << j_coord << endl;
	//cout << "k_coord " << k_coord << endl;
	//cout << "Looking for " << print_point3(thePoint) << endl;
	//cout << ypoints[j_node].xpoints.size() << endl;



	// Coordinate y is there, search for x
	int i_node = -1;
	for (int i = 0; i < ypoints[j_node].xpoints.size(); i++)
	{
		if (ypoints[j_node].xpoints[i].value >= multiplier*i_coord)
		{
			if (ypoints[j_node].xpoints[i].value == multiplier*i_coord)
				i_node = i;

			break;
		}
		// if (ypoints[j_node].xpoints[i].value > i_coord)
			// return false;
	}

	if (i_node == -1)
		return false;

	// Coordinate y and y are there, search for z
	for (int k = 0; k < ypoints[j_node].xpoints[i_node].zpoints.size(); k++)
	{
		if (ypoints[j_node].xpoints[i_node].zpoints[k] >= multiplier*k_coord)
		{
			if (ypoints[j_node].xpoints[i_node].zpoints[k] == multiplier*k_coord)
				return true;
			else
				return false;
		}
		// if (ypoints[j_node].xpoints[i_node].zpoints[k] > k_coord)
			// return false;
	}

	// If we got here, we had x and y, but not z...
	return false;
}

bool NFV::has_point_reverse(int i, int j, int k)
{
	
	return has_point(i, j, k, true);
	// cout << "ERROR: FUNCTION YET TO BE CODED!!!!!" << endl;
	// exit(-1);
	// return false;
}

NFVYNODE::NFVYNODE(int value_par)
{
	value = value_par;
}

NFVXNODE::NFVXNODE(int value_par)
{
	value = value_par;
}

bool NFVXNODE::sort_less_than_zpoints(int i, int j)
{
	return i < j;
}
bool NFVXNODE::sort_less_than(NFVXNODE i, NFVXNODE j)
{
	return i.value < j.value;
}
bool NFVYNODE::sort_less_than(NFVYNODE i, NFVYNODE j)
{
	return i.value < j.value;
}
NFVSTRUCTURE::NFVSTRUCTURE()
{

}


NFVSTRUCTURE::NFVSTRUCTURE(vector<POLYTOPE> collection)
{
	vector<POLYTOPE*> collection_pointers;
	for (int i = 0; i < collection.size(); i++)
	{
		collection_pointers.push_back(&collection[i]);
	}
	generateNFVs(collection_pointers);
}

NFVSTRUCTURE::NFVSTRUCTURE(vector<POLYTOPE*> collection)
{
	generateNFVs(collection);
}

NFVSTRUCTURE::NFVSTRUCTURE(string filename, vector<POLYTOPE*> pc)
{
	deserialise(filename, pc);
}

void NFVSTRUCTURE::generateNFVs(vector<POLYTOPE*> collection)
{
// Generate the structure of NFVs:

	// Allocate memory for the pointers:
	vector<NFV*> aux_vec;
	NFV * NULLpointer;
	aux_vec.assign(collection.size(), NULLpointer);
	NFVaddresses.assign(collection.size(), aux_vec);
	cout << "NFVaddresses.size() = " << NFVaddresses.size() << endl;

	// Start generating and storing the NFV's
	for (int i = 0; i < collection.size(); i++)
	{
		for (int j = i; j < collection.size(); j++)
		{
			NFV auxNFV(i, j, collection, false);
			NFVvec.push_back(auxNFV);
		}
	}

	fix_addresses();
}

void NFVSTRUCTURE::fix_addresses()
{
	// Do the loop again and fix the pointers:
	int listIndex = 0;
	for (int i = 0; i < NFVaddresses.size(); i++)
	{
		for (int j = i; j < NFVaddresses[i].size(); j++)
		{
			NFVaddresses[i][j] = &NFVvec[listIndex];

			// The other NFV points to the same place:
			if (i != j)
				NFVaddresses[j][i] = &NFVvec[listIndex];

			listIndex++;
		}
	}
}

NFV * NFVSTRUCTURE::getNFV(int i, int j)
{
	// if (j > i)
		// cout << endl << "WARNING: getNFV(i,j) returns NFV(j,i) if j > i!" << endl;


	return NFVaddresses[i][j];
}

bool NFVSTRUCTURE::serialise(string filename)
{
	ofstream fi;
	fi.open(filename.c_str());

	// Header and version:
	fi << "NFVSTRUCTURE version 1" << endl;

	for (int i = 0; i < NFVaddresses.size(); i++)
	{
		for (int j = i; j < NFVaddresses[i].size(); j++)
		{
			NFV * toWrite = getNFV(i, j);
			fi << "START NFV " << i << "\t" << j << "\tPOINTS\t" << toWrite->nPoints << endl;
			for (int ypts = 0; ypts < toWrite->ypoints.size(); ypts++)
			{
				for (int xpts = 0; xpts < toWrite->ypoints[ypts].xpoints.size(); xpts++)
				{
					for (int zpts = 0; zpts < toWrite->ypoints[ypts].xpoints[xpts].zpoints.size(); zpts++)
					{
						// Write the three coordinates:
						fi << toWrite->ypoints[ypts].xpoints[xpts].value << "\t" << toWrite->ypoints[ypts].value;
						fi << "\t" << toWrite->ypoints[ypts].xpoints[xpts].zpoints[zpts] << endl;
					}

				}
			}
			fi << "END NFV" << endl;
		}
	}


	fi << "EOF" << endl;
	fi.close();
	return true;
}
bool NFVSTRUCTURE::deserialise(string filename, vector<POLYTOPE*> pc)
{
	string readString;
	int int1, int2, int3;

	ifstream fi(filename.c_str());

	// Some error handling:
	if (!fi.is_open())
	{
		cout << "ERROR: is_open() returned false for file \"" << filename.c_str() << "\"" << endl;
		return false;
	}

	// Read header and version:
	fi >> readString;
	cout << "Reading file format: " << readString << " version: ";
	fi >> readString;
	fi >> readString;
	cout << readString << endl;

	// Next thing we encounter should be START NFV i j POINTS nPoints
	int NFVread = 0;
	while (true)
	{
		fi >> readString;
		// If we reached end of file, exit
		if (readString.compare("EOF") == 0)
			break;

		// Otherwise, read the NFV, start with the header:
		ds_error(readString, "START");
		fi >> readString;
		ds_error(readString, "NFV");
		fi >> int1;
		fi >> int2;
		fi >> readString;
		ds_error(readString, "POINTS");
		int nfvpoints;
		fi >> nfvpoints;
		cout << "Reading NFV for " << int1 << " and " << int2 << ", which contains " << nfvpoints << " points." << endl;

		// Create NFV:
		NFV auxNFV(int1, int2, pc, true);

		// Read points:
		for (int pt = 0; pt < nfvpoints; pt++)
		{
			fi >> int1;
			fi >> int2;
			fi >> int3;
			auxNFV.add_point(int1, int2, int3);
		}
		NFVvec.push_back(auxNFV);

		// End of this NFV:
		fi >> readString;
		ds_error(readString, "END");

		fi >> readString;
		ds_error(readString, "NFV");

		NFVread++;
	}

	cout << "Reading finished. " << NFVread << " NFVs were read." << endl;
	fi.close();

	// Reading is finished, now sort out NVFSTRUCTURE parts:
	
	vector<NFV*> aux_vec;
	NFV * NULLpointer;
	aux_vec.assign(pc.size(), NULLpointer);
	NFVaddresses.assign(pc.size(), aux_vec);
	fix_addresses();
	


	return true;
}

void NFVSTRUCTURE::ds_error(string foundstring, string comparestring)
{
	// If they are the same, no problem:
	if (foundstring.compare(comparestring) == 0)
		return;


	cout << "ERROR: Reading NFV file." << endl;
	cout << "ERROR: Expected \"" << comparestring << "\" but found \"" << foundstring << "\" instead." << endl;
	exit(-1);
}


