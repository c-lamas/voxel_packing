#include "3dpack.h"
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iostream>
#include <random>
#include <iomanip>
#include "3dlib.h"

// std::default_random_engine randomNumGenerator;
std::default_random_engine randomNumGenerator(std::random_device{}());

PACKING_OPTIONS::~PACKING_OPTIONS()
{
	solutionFile.end();
	nfvfile.end();
	file.end();
	HCtype.end();
}
PACKING_OPTIONS::PACKING_OPTIONS()
{
	solutionFile.reserve(500);
	nfvfile.reserve(500);
	file.reserve(500);
	HCtype.reserve(500);


	// Set the defaults first:
	resolution = 0.1;
	method = 1;
	maxTime = 3600;
	tabuSize = 10;
	idOfRun = "";
	// How much we change the increase and decrease parameters in iterated tabu search, by default, disabled
	itsIterationMultiplier = 1;
	itsDecreaseMultiplier = 1;
	probChange = 0.5;
	sat0 = 0.05;
	satype = 25;
	doCompaction = true;
	maxIters = 100;
	maxKicks = 50;
	fitnessTypePar = 5; // Default fitness is bounding box
	fitnessString = "boundingbox"; // Default fitness is bounding box
	maxPerturbations = 50;
	initDelta = 1;
	deltaPumps = 3;
	HCtype = "closest";
	nfvfile = ":";
	solutionFile = ":";
	file = ":";
	oscillationMode = 0; // 0 - Random, 1 - By insert
	decPercentage = 0.1; // 0 - Random, 1 - By insert
	int1 = -1;
	int2 = -1;
	int3 = -1;
	double1 = -1.0;
	double2 = -1.0;
	double3 = -1.0;
	vLoadNFV = false;
	vleadString = ":";
	vfirstFile = 0;
	vlastFile = -1;
	modelBoxOverlap = 5;
	modelBoxNonOverlap = 2;
	modelMaxOverlapPairs = 2;
	randomSeed = -1;
}

CONTAINER::CONTAINER()
{
	// Initialise everything to -1, so it does not break but we know it's not valid.
	for (int i = 0; i < 3; i++)
	{
		sideSize[i] = -1;
		gridSize[i] = -1;
	}

	baseArea = -1.0;
	baseAreaVoxel = -1;
	openDimension = -2;
}

void CONTAINER::find_grid(double resolution)
{
	bool verbose = true;

	vector<string> dimensionName;
	dimensionName.push_back("x");
	dimensionName.push_back("y");
	dimensionName.push_back("z");

	baseAreaVoxel = 1;
	baseArea = 1.0;

	if (verbose == true)
		printf("\n \n");	

	for (int i = 0; i < 3; i++)
	{
		if (sideSize[i] > -1)
		{
			gridSize[i] = int(floor(double(sideSize[i]) / resolution));
			baseAreaVoxel = baseAreaVoxel * gridSize[i];
			baseArea = baseArea * sideSize[i];

			if (verbose == true)
				cout << "Resolution loss in " << dimensionName[i].c_str() << " was " << sideSize[i] - gridSize[i]*resolution << endl;	
				// printf("\nResolution loss in %s was %.2f", dimensionName[i], sideSize[i] - gridSize[i]*resolution);	
		}
	}
}

// bool NFVSTRUCTURE::nfv_has_point(int nfv1, int nfv2, int i, int j, int k)
bool NFVSTRUCTURE::nfv_has_point(int nfv1, int i1, int j1, int k1, int nfv2, int i2, int j2, int k2)
{
	vector<int> v1 = { i1, j1, k1 };
	vector<int> v2 = { i2, j2, k2 };
	
	return nfv_has_point(nfv1, v1 , nfv2, v2);
}

bool NFVSTRUCTURE::nfv_has_point(int &nfv1, vector<int> &placedHere, int &nfv2, vector < int > &point)
// bool NFVSTRUCTURE::nfv_has_point(int nfv1, int nfv2, vector < int > point)
{
	NFV * nfv;
	nfv = getNFV(nfv1, nfv2);

	// vector<int> pointToCheck = { point[0] - placedHere[0], point[1] - placedHere[1], point[2] - placedHere[2] };
	// cout << "Piece: " << nfv1 << " placed at ";
	// cout_point3(placedHere);
	// cout << endl;

	// cout << "Piece: " << nfv2 << " placed at ";
	// cout_point3(point);
	// cout << endl;

	// cout << "Point to check: ";
	// cout_point3(pointToCheck);
	// cout << endl;



	if (nfv1 <= nfv2)
		return nfv->has_point(point[0] - placedHere[0], point[1] - placedHere[1], point[2] - placedHere[2]);
	else
		return nfv->has_point_reverse(point[0] - placedHere[0], point[1] - placedHere[1], point[2] - placedHere[2]);
		// return nfv->has_point_reverse(point[0], point[1], point[2]);


}


/*                 PACKING LAYOUT                 */		

bool PACKING_LAYOUT::can_p_move_to(int p, int i, int j, int k)
{
	if (pieceIndices[p][0] + i < -TOL)
		return false;

	if (pieceIndices[p][1] + j < -TOL)
		return false;

	if (pieceIndices[p][2] + k < -TOL)
		return false;

	if (instance->container->openDimension != 0)
	{
		if (pieceIndices[p][0] + i > instance->maxPositions[instance->pieces[p]->index][0] + TOL)
			return false;
	}
	else if (pieceIndices[p][0] + i + instance->pieces[p]->voxel.get_grid_size(0) > UpperBoundH + TOL)
		return false;

	if (instance->container->openDimension != 1)
	{
		if (pieceIndices[p][1] + j > instance->maxPositions[instance->pieces[p]->index][1] + TOL)
			return false;
	}
	else if (pieceIndices[p][1] + j + instance->pieces[p]->voxel.get_grid_size(1) > UpperBoundH + TOL)
		return false;

	if (instance->container->openDimension != 2)
	{
		if (pieceIndices[p][2] + k > instance->maxPositions[instance->pieces[p]->index][2] + TOL)
			return false;
	}
	else if (pieceIndices[p][2] + k + instance->pieces[p]->voxel.get_grid_size(2) > UpperBoundH + TOL)
		return false;

	// None of the container borders is violated, then the piece can move:
	return true;

}

PACKING_LAYOUT::PACKING_LAYOUT()
{

}

PACKING_LAYOUT::PACKING_LAYOUT(INSTANCE &instance_parameter, PACKING_OPTIONS &packingOptions)
{
	instance = &instance_parameter;
	options = &packingOptions;
	feasible = true;
	complete = false;
	methodString = "Unknown";
	overlapCount = 0;
	resolution = packingOptions.resolution;
	fitnessType = options->fitnessTypePar;
	
	od = instance->container->openDimension;
	cd1 = (od + 1) % 3;
	cd2 = (od + 2) % 3;

	vector<float> auxVec_float;
	auxVec_float.assign(3, 0);

	pieceCoordinates.assign(instance->nPieces, auxVec_float);

	// Modifiers for BLB algorithms:
	placeFront.assign(instance->nPieces, false);
	placeRight.assign(instance->nPieces, false);
	
	// Unused at the moment:
	// rotationAngles.assign(instance->nPieces, auxVec_float);

	// Solution destroying methods:
	destroyNames.push_back("swap ov");
	destroyNames.push_back("r_insert");
	destroyNames.push_back("reinsert_overlaps");
	destroyNames.push_back("swap any");
	int nMoves = destroyNames.size();
	for (int i = 0; i < nMoves; i++)
		destroyScores.push_back(1.0 / float(nMoves));


	vector<int> auxVec_int;
	auxVec_int.assign(3, 0);
	pieceIndices.assign(instance->nPieces, auxVec_int);

	piecePlaced.assign(instance->nPieces, false);
	ignorePiece.assign(instance->nPieces, false);
	pieceCanMove.assign(instance->nPieces, true);
	lastAt.assign(instance->originalPieces.size(), 0);
	for (int i = 0; i < instance->nPieces; i++)
		placementOrder.push_back(i);

	elapsedTime = -1;
	relPenetrationDepth.assign(instance->nPairs, 0);
	relBoxOverlapPairVoxel.assign(instance->nPairs, 0);
	// Find bounds as well:
	find_simple_bounds_voxel(instance->voxelLowerBound, instance->voxelUpperBound, false, false);
}


void PACKING_LAYOUT::fix_overlaps_from_pairs()
{

int pair = -1;
overlapCount = 0;
overlaps.assign(instance->nPieces, false);
for (int p = 0; p < instance->nPieces; p++)
{
	for (int q = p + 1; q < instance->nPieces; q++)
	{
		pair++;
		if (relBoxOverlapPairVoxel[pair] > TOL)
		{
			overlaps[p] = true;
			overlaps[q] = true;
			overlapCount++;
		}
	}
}
}

void PACKING_LAYOUT::find_simple_bounds_voxel(int &lower, int &upper)
{
	find_simple_bounds_voxel(lower, upper, false);
}

void PACKING_LAYOUT::find_simple_bounds_voxel(int &lower, int &upper, bool saveSolution)
{
	find_simple_bounds_voxel(lower, upper, false, true);
}
void PACKING_LAYOUT::find_simple_bounds_voxel(int &lower, int &upper, bool saveSolution, bool doBLB)
{
	lower = instance->originalPieces[0]->voxel.get_grid_size(od);
	upper = instance->demands[0]*instance->originalPieces[0]->voxel.get_grid_size(od);
	for (int i = 1; i < instance->nPiecesOr; i++)
	{
		lower = max(instance->originalPieces[i]->voxel.get_grid_size(od), lower);
		upper += instance->demands[i]*instance->originalPieces[i]->voxel.get_grid_size(od);
	}
	if (doBLB)
	{
		if (!saveSolution)
		{
			PACKING_LAYOUT dummy = (*this);
			dummy.strip_voxel_packing_first_fit_nfv();
			upper = min(upper, dummy.highest_container_point());
		}
		else
		{
			strip_voxel_packing_first_fit_nfv();
			upper = min(upper, highest_container_point());
		}
	}

}
int PACKING_LAYOUT::solution_distance(PACKING_LAYOUT &anotherSolution)
{
	int overallDist = 0;
	for (int p = 0; p < instance->nPieces; p++)
	{
		int thisDist = abs(anotherSolution.pieceIndices[p][0] - pieceIndices[p][0]);
		thisDist = max(thisDist, abs(anotherSolution.pieceIndices[p][1] - pieceIndices[p][1]));
		thisDist = max(thisDist, abs(anotherSolution.pieceIndices[p][2] - pieceIndices[p][2]));
		overallDist = max(overallDist, thisDist);
	}
	//if (overallDist == 0)
	//	cout << "Solutions were equal!" << endl;
	//else
	//	cout << "Distance was " << overallDist << endl;
	return(overallDist);
}
bool PACKING_LAYOUT::is_feasible_voxel()
{
	relBoxOverlapPairVoxel.assign(instance->nPairs, 0.0);
	overlaps.assign(instance->nPieces, 0);
	overlapCount = 0; 
	int pair = -1;
	for (int p = 0; p < instance->nPieces; p++)
	{
		for (int q = p + 1; q < instance->nPieces; q++)
		{
			pair++;

			//// DEBUG PAIRS
			// cout << "(r) PAIR p = " << min(p, q) << " q = " << max(p, q) << " pairIdx = " << pair << endl;

			int minBox[3];
			int maxBox[3];
			int sideSize[3];
			bool noOverlap = false;
			for (int coord = 0; coord < 3; coord++)
			{
				minBox[coord] = max(pieceIndices[p][coord], pieceIndices[q][coord]);
				maxBox[coord] = min(pieceIndices[p][coord] + instance->pieces[p]->voxel.get_grid_size(coord),
					pieceIndices[q][coord] + instance->pieces[q]->voxel.get_grid_size(coord));

				sideSize[coord] = maxBox[coord] - minBox[coord];
				if (sideSize[coord] <= 0)
					noOverlap = true;
			}
			if (noOverlap)
				continue;
			else
			{
				if (allNFV->nfv_has_point(instance->pieces[p]->index, pieceIndices[p], instance->pieces[q]->index, pieceIndices[q]))
				{
					// fitness += (overlapVolume * nonOverlapWeight)/largestOverlap;
				}
				else
				{
					overlapCount++;
					overlaps[p] = true;
					overlaps[q] = true;
					double overlapVolume = sideSize[0] * sideSize[1] * sideSize[2];
					// fitness += (overlapVolume * overlapWeight)/largestOverlap;
					relBoxOverlapPairVoxel[pair] = overlapVolume / min(instance->pieces[p]->voxel.BoxVoxelVolume,
						instance->pieces[q]->voxel.BoxVoxelVolume);
				}
			}
		}
	}
	
	if (overlapCount == 0)
	{
		// Update lower bound
		if (highest_container_point() < instance->voxelLowerBound)
			instance->voxelLowerBound = maxHeightVoxels;
		
		return true;
	}
	else
		return false;
}

void PACKING_LAYOUT::deserialise(string filename)
{
	string readString;
	string keyword;
	int int1, int2, int3, aux_int;
	int nofPieces = 0;
	int nofPiecesOr = 0;
	bool npread = false;
	bool npreador = false;

	// Delete what is in the layout:
	vector< vector < int > > emptyInds;
	pieceIndices = emptyInds;


	ifstream fi(filename.c_str());

	// Some error handling:
	if (!fi.is_open())
	{
		cout << "ERROR: is_open() returned false for file \"" << filename.c_str() << "\"" << endl;
		exit(-1);
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
		fi >> keyword;
		if (keyword.compare("elapsedTime") == 0)
		{
			fi >> elapsedTime;
		}
		else if (keyword.compare("resolution") == 0)
		{
			fi >> resolution;
		}
		else if (keyword.compare("overlapCount") == 0)
		{
			fi >> overlapCount;
		}
		else if (keyword.compare("methodString") == 0)
		{
			fi >> readString;
			NFVSTRUCTURE::ds_error(readString, "<");
			methodString = "";
			while (true)
			{
				fi >> readString;
				if (readString.compare(">") == 0)
					break;
				
				methodString.append(readString);
				methodString.append(" ");
			}
		}
		else if (keyword.compare("nPieces") == 0)
		{
			fi >> nofPieces;
			npread = true;
		}
		else if (keyword.compare("nPiecesOr") == 0)
		{
			fi >> nofPiecesOr;
			npreador = true;
		}
		else if (keyword.compare("pieceIndices") == 0)
		{
			fi >> readString;
			NFVSTRUCTURE::ds_error(readString, "START");
			if (!npread)
			{
				cout << "ERROR: Number of pieces needs to be read first." << endl;
				exit(-1);
			}
			for (int line = 0; line < nofPieces; line++)
			{
				fi >> int1;
				fi >> int2;
				fi >> int3;
				pieceIndices.push_back({ int1, int2, int3 });
				piecePlaced[line] = true;
			}
		}
		else if (keyword.compare("instance_pieces") == 0)
		{
			fi >> int1;
			cout << "Pieces in the solution: " << endl;
			for (int line = 0; line < int1; line++)
			{
				fi >> readString;
				cout << readString << endl;
			}
		}
		else if (keyword.compare("instance_demands") == 0)
		{
			fi >> int1;
			cout << "Demands in the solution: " << endl;
			for (int line = 0; line < int1; line++)
			{
				fi >> int2;
				cout << int2 << endl;
			}
		}
		else if (keyword.compare("instance_order") == 0)
		{
			fi >> int1;
			for (int line = 0; line < int1; line++)
			{
				fi >> int2;
				instancePieceOrder.push_back(int2);
			}
			cout << "Sorting pieces with the found solution order... ";
			instance->sort_pieces_by(instancePieceOrder);
			cout << " Done." << endl;
		}
		else if (keyword.compare("EOF") == 0)
		{
			break;
		}
	}
	cout << "Reading finished!" << endl;

	if (options->vLoadNFV)
	{
		overlaps.assign(instance->nPairs, false);
		is_feasible_voxel();
	}
}

void PACKING_LAYOUT::serialise(string filename, int index) 
{
	// Just add the number and .psol
	std::stringstream streamfinal;
	streamfinal << filename << index << ".psol";
	string sa_final_solname = streamfinal.str();
	serialise(sa_final_solname);
}
void PACKING_LAYOUT::serialise(string filename)
{
	ofstream fi;
	fi.open(filename.c_str());

	// Header and version:
	fi << "PACKING_LAYOUT version 1" << endl;

	fi << "elapsedTime\t" << elapsedTime << endl;
	fi << "overlapCount\t" << overlapCount << endl;
	fi << "packingVoxels\t" << highest_container_point() << endl;
	fi << "packingHeight\t" << highest_container_point()*instance->pieces[0]->voxel.resolution << endl;
	fi << "resolution\t" << instance->pieces[0]->voxel.resolution << endl;
	// fi << "resolution\t" << instance->pieces[0]->voxel.resolution << endl;
	// fi << "instanceFile\t< " << this-> << " >" << endl;
	fi << "methodString\t< " << methodString << " >" << endl;
	fi << "nPieces\t" << instance->nPieces << endl;
	fi << "nPiecesOr\t" << instance->nPiecesOr << endl;
	fi << "pieceIndices\tSTART" << endl;
	for (int i = 0; i < instance->nPieces; i++)
	{
		fi << pieceIndices[i][0] << "\t";
		fi << pieceIndices[i][1] << "\t";
		fi << pieceIndices[i][2] << endl;
	}

	fi << "instance_pieces\t" << instance->nPiecesOr << endl;
	for (int i = 0; i < instance->nPiecesOr; i++)
	{
		fi << instance->originalPieces[i]->modelname << endl;
	}
	fi << "instance_demands\t" << instance->nPiecesOr << endl;
	for (int i = 0; i < instance->nPiecesOr; i++)
	{
		fi << instance->demands[i] << endl;
	}
	fi << "instance_order\t" << instance->nPieces << endl;
	for (int i = 0; i < instance->nPieces; i++)
	{
		fi << instance->pieces[i]->index << endl;
	}
	//fi << "pieceIndicesTypes\t" << instance->nPiecesOr << endl;
	//for (int i = 0; i < instance->nPiecesOr; i++)
	//{
	//	for (int j = 0; j < pieceIndicesType[i].size(); j++)
	//	{
	//		fi << pieceIndicesType[i][j][0] << "\t";
	//		fi << pieceIndicesType[i][j][1] << "\t";
	//		fi << pieceIndicesType[i][j][2] << endl;
	//	}
	//}

	fi << "placeFront\tSTART" << endl;
	for (int i = 0; i < instance->nPieces; i++)
	{
		if (placeFront[i])
			fi << "1\t";
		else
			fi << "0\t";
	}
	fi << endl;

	fi << "placeRight\tSTART" << endl;
	for (int i = 0; i < instance->nPieces; i++)
	{
		if (placeRight[i])
			fi << "1\t";
		else
			fi << "0\t";
	}
	fi << endl;

	fi << "placementOrder\tSTART" << endl;
	for (int i = 0; i < instance->nPieces; i++)
	{
		fi << placementOrder[i] << "\t";
	}

	fi << "realOrder\tSTART" << endl;
	for (int i = 0; i < instance->nPieces; i++)
	{
		fi << "\"" << instance->pieces[i]->modelname.c_str() << "\"\t";
	}
	fi << endl;


	fi << endl << "EOF";
	fi.close();
}

NFVSTRUCTURE PACKING_LAYOUT::generate_nfvs()
{
	// Reinitialise the NFV structure
	NFVSTRUCTURE allNFVSTRUCT(this->instance->originalPieces);

	// Just in case the pointers went mad...
	allNFV = &allNFVSTRUCT;
	allNFV->fix_addresses();
	return allNFVSTRUCT;
}

void PACKING_LAYOUT::reset()
{
	reset_after_index(0);
}

void PACKING_LAYOUT::reset_after_index(int init)
{
	complete = false;
	vector<int> aux_vec_i;
	vector<float> aux_vec_f;
	aux_vec_f.assign(3, 0.0);
	aux_vec_i.assign(3, 0);

	for (int i = init; i < piecePlaced.size(); i++)
	{
		piecePlaced[i] = false;
		pieceCoordinates[i] = aux_vec_f;
		pieceIndices[i] = aux_vec_i;
		// rotationAngles[i] = aux_vec_f;
	}

	elapsedTime = 0.0;
	lastAt.assign(instance->nPiecesOr, 0);
	maxHeight = 0;
	maxHeightVoxels = 0;
	methodString = "Unknown";
	// placeFront.assign(instance->nPieces, false);
	// placeRight.assign(instance->nPieces, false);

	// If we do not reset everything, get the values of lastAt correctly:
	for (int i = 0; i < init; i++)
	{
		lastAt[instance->pieces[i]->index] = pieceIndices[i][instance->container->openDimension];
	}
}

bool PACKING_LAYOUT::is_complete()
{
	for (int i = 0; i < piecePlaced.size(); i++)
	{
		if (!piecePlaced[i] || ignorePiece[i])
			return false;
	}
	return true;
}

void PACKING_LAYOUT::iterated_heuristics()
{

}

// Packing tools:
int PACKING_LAYOUT::slice_usage(int slice_idx)
{
	return slice_usage(slice_idx, instance->container->openDimension);
}

bool PACKING_LAYOUT::bboxes_intersectOR(int newPiece, int placedPiece, vector<int> proposedPoint, vector<int> placedPoint)
{
	// Check if min and max points of each coordinate lay outside the bounding box of placed piece. If this happen, there is no intersection
	for (int coord = 0; coord < 3; coord++)
	{
		// int np_coordsize = instance->pieces[newPiece]->voxel.get_grid_size(coord);
		// int pp_coordsize = instance->pieces[placedPiece]->voxel.get_grid_size(coord);
		if (
			// Check if both are smaller
			(
			(proposedPoint[coord] + instance->originalPieces[newPiece]->voxel.get_grid_size(coord) - 1 < placedPoint[coord]) // && 
			// (proposedPoint[coord] +  np_coordsize < pieceIndices[placedPiece][coord] + pp_coordsize) 
			)
			||  // OR
			// Check if both are larger
			(
			// (proposedPoint[coord] > pieceIndices[placedPiece][coord]) && 
			(proposedPoint[coord] > placedPoint[coord] + instance->originalPieces[placedPiece]->voxel.get_grid_size(coord) - 1)
			)
			)
		{
			// No intersection
			return false;
		}
	}
	// No coordinate seems to be separating the boxes, then they interstect:
	return true;
}


bool PACKING_LAYOUT::box_overlap_rel(int p, int q, double &relAmount)
{
	int minBox[3];
	int maxBox[3];
	int sideSize[3];
	bool noBoxOverlap = false;
	for (int coord = 0; coord < 3; coord++)
	{
		minBox[coord] = max(pieceIndices[p][coord], pieceIndices[q][coord]);
		maxBox[coord] = min(pieceIndices[p][coord] + instance->pieces[p]->voxel.get_grid_size(coord),
			pieceIndices[q][coord] + instance->pieces[q]->voxel.get_grid_size(coord));

		sideSize[coord] = maxBox[coord] - minBox[coord];
		if (sideSize[coord] <= 0)
			noBoxOverlap = true;
	}
	if (noBoxOverlap)
	{
		relAmount = 0;
		return false;
	}
	else
	{
		// Removed the scaling, as in large instances this can create numerical inestability...
		relAmount = double(sideSize[0] * sideSize[1] * sideSize[2]) / double(instance->largestVoxelBox);
		//relAmount = (sideSize[0] * sideSize[1] * sideSize[2]) / double(min(instance->pieces[p]->voxel.BoxVoxelVolume,
		//			instance->pieces[q]->voxel.BoxVoxelVolume));

		// If the boxes overlap, but the pieces don't, make this negative:
		if (allNFV->nfv_has_point(instance->pieces[p]->index, pieceIndices[p], instance->pieces[q]->index, pieceIndices[q]))
		{
			// The boxes overlap, the pieces don't, note this with a negative sign!
			relAmount = -1/10000000 * relAmount; // This seems dangerous...
			// relAmount = 0;
		}
		// cout << relAmount;

	}

	// If we do not exit earlier, there is overlap between the boxes!
	return true;
}
bool PACKING_LAYOUT::bboxes_intersect(int &newPiece, int &placedPiece, vector<int> &proposedPoint)
{
	// See if (voxel-wise) newPiece's bbox would intersect placedPiece when placing newPiece in proposedPoint
	// We assume placedPiece is located in pieceIndices[placedPiece] 

	// Check if min and max points of each coordinate lay outside the bounding box of placed piece. If this happen, there is no intersection
	for (int coord = 0; coord < 3; coord++)
	{
		// int np_coordsize = instance->pieces[newPiece]->voxel.get_grid_size(coord);
		// int pp_coordsize = instance->pieces[placedPiece]->voxel.get_grid_size(coord);
		if (
					// Check if both are smaller
				(
				(proposedPoint[coord]  + instance->pieces[newPiece]->voxel.get_grid_size(coord)- 1 < pieceIndices[placedPiece][coord]) // && 
				// (proposedPoint[coord] +  np_coordsize < pieceIndices[placedPiece][coord] + pp_coordsize) 
				)
				||  // OR
					// Check if both are larger
				(
				// (proposedPoint[coord] > pieceIndices[placedPiece][coord]) && 
				(proposedPoint[coord] > pieceIndices[placedPiece][coord] + instance->pieces[placedPiece]->voxel.get_grid_size(coord) - 1) 
				)
			)
		{
			// No intersection
			return false;
		}
	}
		// No coordinate seems to be separating the boxes, then they interstect:
		return true;
}

int PACKING_LAYOUT::slice_usage(int sliceIdx, int sliceCoord)
{
	// Performance can be improved here; loops should have const limits if possible!!!!

	// 1 - Loop over pieces
	// 2 - Discard if piece does not intersect that slice at all
	// 3 - Add the amount of true voxels of that piece in that slide
	// 4 - Return the sum

	int voxelsThere = 0;
	for (int pz = 0; pz < instance->pieces.size(); pz++)
	{
		int startValues[3];
		int endValues[3];
		for (int coord = 0; coord < 3; coord++)
		{
			if (sliceCoord == coord)
			{
				startValues[coord] = sliceIdx;
				endValues[coord] = sliceIdx;
			}
			else
			{
				startValues[coord] = 0;
				endValues[coord] = instance->pieces[coord]->voxel.get_grid_size(coord);
			}
		}
		
		// startValues[sliceCoord] = sliceIdx;

		if (pieceIndices[pz][sliceCoord] > sliceIdx)
		{
			// This piece is not going to overlap
			continue;
		}

		for (int i = startValues[0]; i < endValues[0]; i++)
		{
			for (int j = startValues[1]; j < endValues[1]; j++)
			{
				for (int k = startValues[2]; k < endValues[2]; k++)
				{
					if (instance->pieces[pz]->voxel.get_map_value(i, j, k))
						voxelsThere++;
				}
			}

		}
	}

	return voxelsThere;
}

int PACKING_LAYOUT::slice_free(int slice_idx)
{
	return 0;
}


int PACKING_LAYOUT::highest_container_point()
{
	return highest_container_point(instance->container->openDimension);
}

int PACKING_LAYOUT::highest_container_point(int open_dimension)
{
	int hPoint = 0;

	for (int i = 0; i < pieceIndices.size(); i++)
	{
		if (!piecePlaced[i])
			continue;
		int pieceHeight = pieceIndices[i][open_dimension] + instance->pieces[i]->voxel.get_grid_size(open_dimension);
		// cout << "(od = " << open_dimension << ") Min H for p = " << i << " is H = " << pieceHeight << endl;
		if (hPoint < pieceHeight)
			hPoint = pieceHeight;
	}

	// show_indices();
	// cout << "hPoint is " << hPoint << endl;
	maxHeight = double(hPoint)*resolution;
	maxHeightVoxels = hPoint;
	return hPoint;
}
bool PACKING_LAYOUT::get_piece_layout_value_with_pos(int pieceIndex, int i, int j, int k, int posx, int posy, int posz)
{
	return instance->pieces[pieceIndex]->voxel.get_map_value(i - posx,
		j - posy,
		k - posz);
}
bool PACKING_LAYOUT::get_piece_layout_value(int pieceIndex, int i, int j, int k)
{
	return instance->pieces[pieceIndex]->voxel.get_map_value(	i - pieceIndices[pieceIndex][0], 
														j - pieceIndices[pieceIndex][1], 
														k - pieceIndices[pieceIndex][2]);
}

int PACKING_LAYOUT::is_legal_position(vector<int> &targetPos, int pz) // Tells if the piece pz can be placed in targetPovector<int> &currentMove, int zpz); // Tells if the piezce pz can be lplaced in CcurrentPosMovetargetPos
{
	// Check if the piece is within the container:

	// It is whithin the container if its bounding box is
	for (int i = 0; i < 3; i++)
	{
		// Check the minimum (position cannot be negative)
		if (targetPos[i] < 0)
			return false;


		// Check the maximum
		// Do not check this for open dimension:
		if (i == instance->container->openDimension)
			continue;
		else
		{
			if (targetPos[i] > instance->container->gridSize[i] - instance->pieces[pz]->voxel.get_grid_size(i))
				return false;
		}
	}

	vector<int> maxPointPZ = targetPos;
	for (int coord = 0; coord < 3; coord++)
		maxPointPZ[coord] += instance->pieces[pz]->voxel.get_grid_size(coord) - 1;

	// Redo the last part (commented) to reuse voxel-voxel intersection:

	VOXEL * candidate = &(instance->pieces[pz]->voxel);
	for (int j = 0; j < instance->nPieces; j++)
	{
		VOXEL * placedV = &(instance->pieces[j]->voxel);
		if (pz == j)
			continue; // We don't care if we intersect with ourselves

		if (!piecePlaced[j])
			continue; // This is still in limbo

		// Check if the bounding boxes intersect:
		if (bboxes_intersect(pz, j, targetPos))
		{
			// Check if there is any intersection:
			if (placedV->voxel_voxel_intersection((*candidate), pieceIndices[j], targetPos))
				return false;
		}

	}

	// Nothing was wrong, the position is legal:
	return true;


	// Check if it intersects another piece:
	//for (int j = 0; j < instance->nPieces; j++)
	//{
	//	if (pz == j)
	//		continue; // We don't care if we intersect with ourselves

	//	if (!piecePlaced[j])
	//		continue; // This is still in limbo

	//	// See if their bounding boxes intersect at all:

	//	// Bbox of piece j is between pieceIndices[j] and maxPointJ:
	//	vector<int> maxPointJ = pieceIndices[j];
	//	for (int coord = 0; coord < 3; coord++)
	//		maxPointJ[coord] += instance->pieces[j]->voxel.get_grid_size(coord) - 1;

	//	BOX boxJ, boxPZ;
	//	boxJ.expand(pieceIndices[j], maxPointJ);
	//	boxPZ.expand(targetPos, maxPointPZ);

	//	bool thereIsBoxIntersection = false;
	//	vector<vector <double>> pointsInside;
	//	for (int corner = 0; corner < 8; corner++)
	//	{
	//		bool pointPZinJ = true;
	//		bool pointJinPZ = true;

	//		for (int coord = 0; coord < 3; coord++)
	//		{
	//			// Check if any point of pz is in j
	//			if (! ( 
	//				(boxPZ.vertices[corner][coord] >= (boxJ.minpoint->at(coord) - TOL)) && 
	//				(boxPZ.vertices[corner][coord] <= (boxJ.maxpoint->at(coord) + TOL)) 
	//				) )
	//			{
	//				pointPZinJ = false;
	//				// break;
	//			}



	//			// Check if any point of j is in pz
	//			if (! ( 
	//				(boxJ.vertices[corner][coord] >= (boxPZ.minpoint->at(coord) - TOL) )
	//				&& (boxJ.vertices[corner][coord] <= (boxPZ.maxpoint->at(coord) + TOL)) 
	//				) )
	//			{
	//				pointJinPZ = false;
	//				// break;
	//			}

	//			if ((!pointPZinJ) && (!pointJinPZ))
	//				break;

	//		} // for (int coord = 0; coord < 3; coord++)

	//		// If any of the two was there, add it:
	//		if (pointPZinJ)
	//			pointsInside.push_back(boxPZ.vertices[corner]);

	//		if (pointJinPZ)
	//			pointsInside.push_back(boxJ.vertices[corner]);

	//	} // for (int corner = 0; corner < 8; corner++)
	//	
	//	// Ok, now we know if there are points of one box inside the other.
	//	// If there is none, there is no intersection, go to next piece
	//	if (pointsInside.empty())
	//		continue;


	//	// If that was not the case set up the loop where we need to check the maps:
	//	// Find min and max of the box:
	//	vector<double> minCorner = pointsInside[0];
	//	vector<double> maxCorner = pointsInside[0];
	//	for (int corner = 1; corner < pointsInside.size(); corner++)
	//	{
	//		for (int coord = 0; coord < 3; coord++)
	//		{
	//			minCorner[coord] = min(minCorner[coord], pointsInside[corner][coord]);
	//			maxCorner[coord] = max(maxCorner[coord], pointsInside[corner][coord]);
	//		}
	//	}


	//	// Now, the big 3-dim loop:
	//	
	//	// PERFORMANCE WARNING, GET RID OF THE ROUNDS! BUT CAREFUL WITH DOUBLE - INT PROBLEMS
	//	for (int coord1 = round(minCorner[0]); coord1 < round(maxCorner[0] + 1); coord1++)
	//	{
	//		for (int coord2 = round(minCorner[1]); coord2 < round(maxCorner[1] + 1); coord2++)
	//		{
	//			for (int coord3 = round(minCorner[2]); coord3 < round(maxCorner[2] + 1); coord3++)
	//			{
	//				// If the two maps coincide, bad news...
	//				if ( get_piece_layout_value(j, coord1, coord2, coord3) &&
	//					 get_piece_layout_value_with_pos(pz, coord1, coord2, coord3, targetPos[0], targetPos[1], targetPos[2]) )
	//					return false;
	//			}
	//		}
	//	}

	//	// This piece did not intersect at all... let's go to the next one!

	//}



	//// It did not intersect anything if we reach this point, so it is legal then:
	//return true;
}
void PACKING_LAYOUT::report_on_screen()
{
	stringstream onStream;
	report_on_screen(onStream);
	cout << onStream.str().c_str();
}

void PACKING_LAYOUT::report_on_screen(stringstream &onStream)
{
	// Some variables:
	int containerGridVol = highest_container_point()*
		instance->container->gridSize[(instance->container->openDimension + 1) % 3] *
		instance->container->gridSize[(instance->container->openDimension + 2) % 3];
	// double containerRealVol = containerGridVol*pow(resolution, 3);
	double containerRealVol = double(highest_container_point()) * resolution *
		instance->container->sideSize[(instance->container->openDimension + 1) % 3] *
		instance->container->sideSize[(instance->container->openDimension + 2) % 3];

	onStream << "Solution resolution is: " << resolution << endl;

	onStream << "Container volume is (voxel): " << containerGridVol << " (" << instance->container->gridSize[0] <<
		" , " << instance->container->gridSize[1] << " , " << highest_container_point() << ")" << endl;

	onStream << "Container volume is (real): " << containerRealVol << " (" << instance->container->sideSize[0] <<
		" , " << instance->container->sideSize[1] << " , " << highest_container_point() * resolution  << ")" << endl;

	// cout << "Instance volume: " << instance->volume << " (" << instance->voxelVolume << ")" << endl;
	// cout << "Utilisation (real container): " << instance->volume / containerRealVol << endl;

	// cout << endl << "New figures: " << endl;
	onStream << "Instance volume: " << instance->voxel_volume_double() << " (" << instance->voxel_volume_int() << " voxels)" << endl;
	onStream << "Utilisation (real container): " << instance->voxel_volume_double() / containerRealVol << endl;
	onStream << "Utilisation (voxel container): " << double(instance->voxel_volume_int()) / double(containerGridVol) << endl << endl;
	onStream << endl << highest_container_point() * resolution;
}
void  PACKING_LAYOUT::show_indices()
{
	for (int i = 0; i < instance->nPieces; i++)
	{
		cout << "P" << i << " at ";
		cout_point3(pieceIndices[i]);
	}
}

void PACKING_LAYOUT::indices_to_indices_type()
{
	vector<int> aux_1;
	vector< vector <int> > aux_2;
	pieceIndicesType.assign(instance->nPiecesOr, aux_2);

	for (int i = 0; i < instance->nPieces; i++)
	{
		pieceIndicesType[instance->pieces[i]->index].push_back(pieceIndices[i]);
	}
}
void PACKING_LAYOUT::coordinates_from_indices(double resolution)
{

		for (int el = 0; el < pieceIndices.size(); el++)
		{
			for (int i = 0; i < 3; i++)
				pieceCoordinates[el][i] = float(pieceIndices[el][i]) * resolution;
		}
}


// Not needed!!!!
// void PACKING_LAYOUT::voxel_grid_starts_at(int pieceIdx, vector<int> &grid_index)
// {

// }

void PACKING_LAYOUT::initial_solution_first_fit(PACKING_OPTIONS packingOptions)
{
		// Initial solution can be given in solutionFile or needs to be calculated:
		if (packingOptions.solutionFile.compare(":") == 0)
		{
			cout << "Initial solution from First Fit BLB algorithm (Sorted by depth)..." << endl;
			std::sort(instance->pieces.begin(), instance->pieces.end(),  sort_piece_pointers_decreasing_depth);
			strip_voxel_packing_first_fit_nfv();
			cout << "Done." << endl;
		}
		else
		{
			cout << "Initial solution from file: " << packingOptions.solutionFile << endl;
			deserialise(packingOptions.solutionFile);
			report_on_screen();
			cout << "Done." << endl;
		}
}

void PACKING_LAYOUT::strip_voxel_packing_first_fit_nfv()
{
	bool verbose = false;
	// ************************************************************************//
	//								METHOD STEPS							   //
	// ************************************************************************//

	// Step 1 - Generate NFV structure:
	// Step 1b - Set the position of the first piece:

	// Step 2 - Start looping through the pieces

	// Step 3 - Find the valid positions of ref. point of piece with respect to container

	// Step 4 - Loop through container slices, going in openDimension direction
	// Step 4b - Loops through the valid points from Step 3

	// Step 5 - Check if point is in any bounding box (loop)
	// Step 5b - If yes, see if point is in NFV, otherwise loop to Step 4
	// Step 5c - If not, loop to Step 5 and if the point is in no bounding box, place it and loop to Step 2

	// ********************************************** // 
	// ********************************************** // 
	// ********************************************** // 

	// Set some initial variables:
	methodString = "Bottom Left Back (NFV)";

	/*	MEASURE TIME	*/
	clock_t packingClock;
	packingClock = clock();

	// time_t start, endt;
	// time(&start);

	// Keep the dimensions at hand, since we don't always minimise Y:
	int dimo = instance->container->openDimension;
	int dim1 = (instance->container->openDimension + 2) % 3;
	int dim2 = (instance->container->openDimension + 1) % 3;

// Step 1 - Generate NFV structure:
	// This is done now outside this algorithm!



	/*	MEASURE TIME	*/
	// clock_t packingClockNoNFV;



// Step 1b - Set the position of the first piece:
	// cout << "WARNING: Position of first piece needs to be set." << endl;

	if (!piecePlaced[0])
	{
		piecePlaced[0] = true;
		pieceIndices[0] = { 0, 0, 0 };
		if (placeRight[0])
			pieceIndices[0][dim1] = instance->container->gridSize[dim1] - instance->pieces[0]->voxel.get_grid_size(dim1);
		if (placeFront[0])
			pieceIndices[0][dim2] = instance->container->gridSize[dim2] - instance->pieces[0]->voxel.get_grid_size(dim2);
	}


// Step 2 - Start looping through the pieces
	for (int pind = 1; pind < instance->pieces.size(); pind++)
	{
		// bool isPiecePlaced = false;
		if (piecePlaced[pind])
			continue;
		// --- We are placing piece "pind" --- 

// Step 3 - Find the valid positions of ref. point of piece with respect to container
		int minX = 0;
		int minY = 0;
		int minZ = 0;

		// instance->container->gridSize[instance->container->openDimension] = 1000000;
		int maxDim1 = instance->container->gridSize[dim1] - instance->pieces[pind]->voxel.get_grid_size(dim1);
		int maxDim2 = instance->container->gridSize[dim2] - instance->pieces[pind]->voxel.get_grid_size(dim2);
		// int maxZ = instance->container->gridSize[2] - instance->pieces[1]->voxel.depth;
		// instance->container->gridSize[instance->container->openDimension] = -1;
		// cout << minX << " " << minY << " " << minZ << " max " << maxX << " " << maxY << " " << maxZ << endl;

		// Step 4 - Loop through container slices, going in openDimension direction
		vector<int> checkPoint = { minX, minY, minZ };
		int currentContainerHeight = highest_container_point();
		// cout << "New container height: " << currentContainerHeight << endl;
		for (int sliceInd = lastAt[instance->pieces[pind]->index]; sliceInd < currentContainerHeight + 2; sliceInd++)
		{
// Step 4b - Loops through the valid points from Step 3
			for (int d1idx = 0; d1idx <= maxDim1; d1idx++)
			{
				for (int d2idx = 0; d2idx <= maxDim2; d2idx++) // NECESSARY???????????, WE CAN CHECK FULL SLICES ON THE NFV
				{
					if (placeRight[pind])
						checkPoint[dim1] = maxDim1 - d1idx; 
					else
						checkPoint[dim1] = d1idx; 

					if (placeFront[pind])
						checkPoint[dim2] = maxDim2 - d2idx; 
					else
						checkPoint[dim2] = d2idx; 

					checkPoint[dimo] = sliceInd; 
					bool pointSuitable = false;

// Step 5 - Check if point is in any bounding box (loop)
					for (int placedPiecesIdx = 0; placedPiecesIdx < pind; placedPiecesIdx++)
					{
						if (bboxes_intersect(pind, placedPiecesIdx, checkPoint))
						{
							// cout << "Bboxes intersect for " << pind << " and " << placedPiecesIdx << endl;
	// Step 5b - If yes, see if point is in NFV, otherwise loop to Step 4
							if (!allNFV->nfv_has_point(instance->pieces[placedPiecesIdx]->index, pieceIndices[placedPiecesIdx], instance->pieces[pind]->index, checkPoint))
							{
								// cout << "NFV(" << pind << ", " << placedPiecesIdx << ") DOES NOT have the point ";
								// cout_point3(checkPoint);
								// cout << endl;
								pointSuitable = false;
								break;
							}
							else
							{
								// cout << "NFV(" << pind << ", " << placedPiecesIdx << ") does have the point ";
								// cout_point3(checkPoint);
								// cout << endl;
								pointSuitable = true;
							}

						}
						else
						{
	// Step 5c - If not, loop to Step 5 and if the point is in no bounding box, place it and loop to Step 2

							// cout << "Bboxes DO NOT intersect for " << pind << " and " << placedPiecesIdx << endl;
						// TODO
							pointSuitable = true;
						}
					} // end loop Step 5

					// We have been through all the pieces with this point, see if we can use it:
					if (pointSuitable)
					{
						// cout << "pieceIndices.size() " << pieceIndices.size() << endl;
						// cout << "pind = " << pind << endl;
						// cout << print_point3(pieceIndices[0]) << endl;
						// cout << print_point3(pieceIndices[1]) << endl;
						pieceIndices[pind] = checkPoint;
						// cout << "Piece " << pind << " (ID = " << instance->pieces[pind]->index << ") placed at: " << "(" << checkPoint[0] << ", " << checkPoint[1] << ", " << checkPoint[2] << ")" << endl;
						// cout << "Piece " << pz << " (ID = " << instance->pieces[pz]->index << ") placed at: " << print_point3(currentFeasible) << endl;
						// cout << "Piece " << pind << " placed at: " << print_point3(checkPoint) << endl;
						piecePlaced[pind] = true;

						// Update the minimum container height for this piece:
						lastAt[instance->pieces[pind]->index] = checkPoint[instance->container->openDimension];
						break;
					}
					else
						piecePlaced[pind] = false;

				} // end loop dim2
				if (piecePlaced[pind])
					break;
			} // end loop dim1
				if (piecePlaced[pind])
					break;
		} // end loop slices (open dimension)
		if (!piecePlaced[pind])
		{
			cout << endl << endl <<"ERROR: We have reached the end without placing the piece!" << endl;
			cout << "\tPiece number: " << pind << endl; 
			cout << "\tCurrent container height: " << currentContainerHeight << endl; 
			cout << "\tLast checked point: ";
			cout_point3(checkPoint);
			cout << endl << endl;
			exit(-1);
		}
				


	} // end loop through pieces!
	

	// time(&endt);
	// double dif = difftime(endt, start);
	// cout << dif;

	// elapsedTime = dif;

	// packingClockNoNFV = clock();
	// float elapsedTimeNONFV = (float) packingClockNoNFV / CLOCKS_PER_SEC;
	clock_t packingClockEnd = clock();
	elapsedTime = float(packingClockEnd - packingClock) / CLOCKS_PER_SEC;
	// if (verbose)
		// printf("Elapsed time (No NFV): %.2f s.\n\n", elapsedTimeNONFV);

	// if (verbose)
		// printf("Elapsed time: %.2f s. (%.2f s. NFV generation + %.2f s. Packing) \n", elapsedTimeNONFV + elapsedTime, elapsedTime, elapsedTimeNONFV );
	if (verbose)
		printf("Elapsed time: %.2f s.\n", elapsedTime);

	// elapsedTime = elapsedTimeNONFV + elapsedTime;

	if (verbose)
		cout << "Packing finished! Height: " << highest_container_point() << endl << endl;
}

void PACKING_LAYOUT::matheuristic(int moveLast, int moveLessLast)
{
	// Keep the dimensions at hand, since we don't always minimise Y:
	int dimo = instance->container->openDimension;
	int dim1 = (dimo + 2) % 3;
	int dim2 = (dimo + 1) % 3;

	// Parameters (to be read from file!!!)
	double timePerModel = 60.0;
	bool verbose = false;
	int moveOnlyLast = moveLast;
	int smallDelta = min(instance->container->gridSize[dim1], instance->container->gridSize[dim2]);
	smallDelta = (int)ceil(double(smallDelta) / 4.0);
	cout << "Less moving pieces can move at most " << smallDelta << " voxels away from their location." << endl;


	// Set some initial variables:
	methodString = "Constructive matheuristic";
	vector<int> aux_six(6);
	aux_six.assign(6, 0);

	// We do not want most pieces to interfere with the model:
	ignorePiece.assign(instance->nPieces, true);

	vector<vector<int > > boxDelta;
	boxDelta.assign(instance->nPieces, aux_six);

	int firstMovingSlice = 0;

	/*	MEASURE TIME	*/
	clock_t packingClock;
	packingClock = clock();



	// Place 1st piece somewhere:
	if (!piecePlaced[0])
	{
		piecePlaced[0] = true;
		pieceIndices[0] = { 0, 0, 0 };
	}
	ignorePiece[0] = false;

	for (int pind = 1; pind < instance->pieces.size(); pind++)
	{
		// From now on, this piece is important:
		ignorePiece[pind] = false;

		// Valid pos:
		int minX = 0;
		int minY = 0;
		int minZ = 0;
		int maxDim1 = instance->container->gridSize[dim1] - instance->pieces[pind]->voxel.get_grid_size(dim1);
		int maxDim2 = instance->container->gridSize[dim2] - instance->pieces[pind]->voxel.get_grid_size(dim2);

		// Use that info to set the boxDelta:
		boxDelta[pind][2 * dim1 + 1] = maxDim1;
		boxDelta[pind][2 * dim2 + 1] = maxDim2;

		// Fix the movement of pieces placed earlier, to avoid very large models:
		if (pind >= moveOnlyLast)
		{
			int fixedIdx = pind - moveOnlyLast;
			int maxDim1d = instance->container->gridSize[dim1] - instance->pieces[pind]->voxel.get_grid_size(dim1);
			int maxDim2d = instance->container->gridSize[dim2] - instance->pieces[pind]->voxel.get_grid_size(dim2);
			boxDelta[fixedIdx ][2 * dim1] = max(0, pieceIndices[fixedIdx][dim1]);
			boxDelta[fixedIdx ][2 * dim1 + 1] = min(pieceIndices[fixedIdx][dim1], maxDim1d);
			boxDelta[fixedIdx ][2 * dim2] = max(0, pieceIndices[fixedIdx][dim2] - smallDelta);
			boxDelta[fixedIdx ][2 * dim2 + 1] = min(pieceIndices[fixedIdx][dim2], maxDim2d);
			boxDelta[fixedIdx ][2 * dimo] = pieceIndices[fixedIdx][dimo];
			boxDelta[fixedIdx ][2 * dimo + 1] = pieceIndices[fixedIdx][dimo];

			//for (int lastpiece = 0; lastpiece < moveOnlyLast; lastpiece++)
			//{
			//	int pieceAffects = pieceIndices[pind - lastpiece][dimo] - instance->pieces[pind]->voxel.get_grid_size(dimo);
			//	if (firstMovingSlice < pieceAffects)
			//		firstMovingSlice = max(0, pieceAffects);
			//}

			if (pind >= (moveOnlyLast + moveLessLast))
			{
				int fixedIdx = pind - moveOnlyLast - moveLessLast;
				boxDelta[fixedIdx ][2 * dim1] = pieceIndices[fixedIdx][dim1];
				boxDelta[fixedIdx ][2 * dim1 + 1] = pieceIndices[fixedIdx][dim1];
				boxDelta[fixedIdx ][2 * dim2] = pieceIndices[fixedIdx][dim2];
				boxDelta[fixedIdx ][2 * dim2 + 1] = pieceIndices[fixedIdx][dim2];
				boxDelta[fixedIdx ][2 * dimo] = pieceIndices[fixedIdx][dimo];
				boxDelta[fixedIdx ][2 * dimo + 1] = pieceIndices[fixedIdx][dimo];
				pieceCanMove[fixedIdx] = false;
				for (int lastpiece = 0; lastpiece < moveOnlyLast; lastpiece++)
				{
					int pieceAffects = pieceIndices[pind - lastpiece][dimo] - instance->pieces[pind]->voxel.get_grid_size(dimo);
					if (firstMovingSlice < pieceAffects)
						firstMovingSlice = max(0, pieceAffects);
				}
			}
		}



		// Step 4 - Loop through container slices, going in openDimension direction
		vector<int> checkPoint = { minX, minY, minZ };
		int currentContainerHeight = highest_container_point();
		// cout << "New container height: " << currentContainerHeight << endl;
		for (int sliceInd = lastAt[instance->pieces[pind]->index]; sliceInd < currentContainerHeight + 2; sliceInd++)
		{
			// Move the boxDelta to this slice:
			boxDelta[pind][2 * dimo] = sliceInd;
			boxDelta[pind][2 * dimo + 1] = sliceInd;

			// Step 4b - Loops through the valid points from Step 3
			for (int d1idx = 0; d1idx < maxDim1; d1idx++)
			{
				for (int d2idx = 0; d2idx < maxDim2; d2idx++) // NECESSARY???????????, WE CAN CHECK FULL SLICES ON THE NFV
				{
					if (placeRight[pind])
						checkPoint[dim1] = maxDim1 - d1idx; 
					else
						checkPoint[dim1] = d1idx; 

					if (placeFront[pind])
						checkPoint[dim2] = maxDim2 - d2idx; 
					else
						checkPoint[dim2] = d2idx; 

					checkPoint[dimo] = sliceInd; 
					bool pointSuitable = false;

// Step 5 - Check if point is in any bounding box (loop)
					for (int placedPiecesIdx = 0; placedPiecesIdx < pind; placedPiecesIdx++)
					{
						if (bboxes_intersect(pind, placedPiecesIdx, checkPoint))
						{
	// Step 5b - If yes, see if point is in NFV, otherwise loop to Step 4
							if (!allNFV->nfv_has_point(instance->pieces[placedPiecesIdx]->index, pieceIndices[placedPiecesIdx], instance->pieces[pind]->index, checkPoint))
							{
								pointSuitable = false;
								break;
							}
							else
								pointSuitable = true;
						}
						else
						{
	// Step 5c - If not, loop to Step 5 and if the point is in no bounding box, place it and loop to Step 2
							pointSuitable = true;
						}
					} // end loop Step 5

					// We have been through all the pieces with this point, see if we can use it:
					if (pointSuitable)
					{
						pieceIndices[pind] = checkPoint;
						piecePlaced[pind] = true;

						// Update the minimum container height for this piece:
						lastAt[instance->pieces[pind]->index] = checkPoint[instance->container->openDimension];
						break;
					}
					else
						piecePlaced[pind] = false;

				} // end loop dim2
				if (piecePlaced[pind])
					break;
			} // end loop dim1

			if (!piecePlaced[pind] ) // Then solve a model!
			{
				// Do a mock placement first, to set correctly the initial slice:
				bool mockplacement = true;
				if (mockplacement)
				{

					vector<int> od_coord(moveOnlyLast + moveLessLast + 1);
					for (int lastpieces = 0; lastpieces < od_coord.size(); lastpieces++)
					{
						if (pind - lastpieces - 1 >= 0)
						{
							od_coord[lastpieces] = pieceIndices[pind - lastpieces - 1][dimo];
							pieceIndices[pind - lastpieces - 1][dimo] = -2*instance->pieces[pind - lastpieces - 1]->voxel.get_grid_size(dimo);
						}
					}
					strip_voxel_packing_first_fit_nfv();
					for (int lastpieces = 1; lastpieces < moveOnlyLast + moveLessLast + 1; lastpieces++)
					{
						// Restore the original coordinate:
						if (pind - lastpieces >= 0)
							pieceIndices[pind - lastpieces][dimo] = od_coord[lastpieces];
					}
					for (int finalPcs = pind + 1; finalPcs < instance->nPieces; finalPcs++)
						ignorePiece[finalPcs] = true;

					reset_after_index(pind + 1);
					int mockSlice = pieceIndices[pind][dimo];
					piecePlaced[pind] = false;
					
					if (sliceInd < mockSlice)
					{
						cout << "Moved directly up to slice: " << mockSlice << " (coming from " << sliceInd << ")" << endl;
						sliceInd = mockSlice - 1;
						continue;
					}

				}
				if (sliceInd >= firstMovingSlice)
				{
					// Before we go up one level, try to place it with the model:
					bool modify = false;


					// Solve the model:
					cout << "Need a model to place piece " << pind << " at " << sliceInd << endl;
					int currentH = highest_container_point();
					int targetH = instance->pieces[pind]->voxel.get_grid_size(dimo) + sliceInd;
					int modelH = max(currentH, targetH);
					// cout << "H of container is " << currentH << " new H will be " << targetH << " sending to model: " << modelH << endl;
					bool noObj = false;
					int oldPidx = pieceIndices[pind][dimo];
					if (!noObj)
						boxDelta[pind][2 * dimo + 1] = currentH;
					

					init_cplex(boxDelta,
						timePerModel,
						modify,
						modelH,
						pieceIndices,
						noObj);
					if (!noObj && pieceIndices[pind][dimo] != oldPidx)
						modify = true; // It was modified!

					if (modify)
					{
						lastAt[instance->pieces[pind]->index] = sliceInd;
						piecePlaced[pind] = true;
						cout << ">>> Placed piece " << pind << "!!! (model)" << endl;

						for (int i = pind + 1; i < instance->nPieces; i++)
							pieceIndices[i][instance->container->openDimension] = -1000;

						clock_t packingClockEndPiece = clock();
						elapsedTime = float(packingClockEndPiece - packingClock) / CLOCKS_PER_SEC;
						std::stringstream sstm;
						sstm << "MHL" << moveOnlyLast << "_p_" << pind << ".psol";
						string sa_solname = sstm.str();
						serialise(sa_solname);
						break;
					}
				}
				else
				{
					cout << "Not solving model." << endl;
					cout << "Slice ind: " << sliceInd << " first moving slice: " << firstMovingSlice << endl;
				}
			}
			else
			{
				cout << ">>> Placed piece " << pind << "!!! (heuristic)" << endl;
				clock_t packingClockEndPiece = clock();
				elapsedTime = float(packingClockEndPiece - packingClock) / CLOCKS_PER_SEC;
				for (int i = pind + 1; i < instance->nPieces; i++)
					pieceIndices[i][instance->container->openDimension] = -1000;
				std::stringstream sstm;
				sstm << "MHL" << moveOnlyLast << "_p_" << pind << ".psol";
				string sa_solname = sstm.str();
				serialise(sa_solname);
				break; // Go to next piece
			}
		}
	} // end of loop pieces

	clock_t packingClockEnd = clock();
	elapsedTime = float(packingClockEnd - packingClock) / CLOCKS_PER_SEC;

	std::stringstream sstmm;
	sstmm << "MH_" << highest_container_point() << "_last_" << moveOnlyLast << ".psol";
	string finalname = sstmm.str();
	serialise(finalname);

}

// Packing algorithms:
void PACKING_LAYOUT::strip_voxel_packing_sliding()
{
	bool verbose = true;
	packingType pType = PushAndSlide; // Or BottomLeftBack
	// packingType pType = BottomLeftBack; 
	if (pType == packingType::BottomLeftBack)
		methodString = "Bottom Left Back (Sliding)";
	else
		methodString = "Push and Slide";

	/*	MEASURE TIME	*/
	clock_t packingClockStart = clock();
	
	if (verbose == true)
		printf("\nVoxel strip packing started.\n");


	int UPPER_BOUND_SLICES = 100;

	// Place first piece in bottom (done, placed in 0,0,0 by default!)
	piecePlaced[0] = true;

	// int pieceTurn = 1;

	for (int pz = 1; pz < instance->nPieces; pz++)
	{
		vector<int> currentFeasible(3);
		for (int i = 0; i < 3; i++)
		{
			currentFeasible[i] = instance->container->gridSize[i] - instance->pieces[pz]->voxel.get_grid_size(i);
			// currentFeasible[i] = 0;
		}
		currentFeasible[instance->container->openDimension] = highest_container_point(instance->container->openDimension) + 1;

		// Piece is now placed on top of all others, ready to start travelling
		int noMoreMoves = 0;// We can move in 3 dimensions, when one dimension has no chance to move, increases noMoreMoves one. It is reseted to zero in every move.
		vector<int> targetPos(3);
		while (noMoreMoves < 2.5) 
		{
			targetPos = currentFeasible;

			for (int otherDirection = 0; otherDirection < 3; otherDirection++) // We want this to be 0, 1 and 2 only
			{
				// We start pushing "down" in the openDimension direction, then the other two:
				int currentDirection = (otherDirection + instance->container->openDimension) % 3;

				while (true)
				{
					bool moveMade = false;
					targetPos[currentDirection] -= 1;
					if (!is_legal_position(targetPos, pz))
					{
						// This went a bit too far let's undo it and go to the next coordinate:
						// cout << "FOUND ILLEGAL POSITION" << endl;
						// cout_point3(targetPos);
						// cout << endl;

						targetPos[currentDirection]++;
						currentFeasible = targetPos;
						if (!moveMade)
							noMoreMoves++;
						break;
					}
					else
					{
						// cout << "FOUND LEGAL POSITION";
						// cout_point3(targetPos);
						// cout << endl;
						noMoreMoves = 0;
						moveMade = true;
						if (pType == packingType::PushAndSlide)
						{
							// We want to push down again...
							otherDirection = -1;
							break;
						}
					}
				}

			}
		}
		// Ok, at this point the piece is placed:
		pieceIndices[pz] = currentFeasible;
		piecePlaced[pz] = true;
		cout << "Piece " << pz << " (ID = " << instance->pieces[pz]->index << ") placed at: ";
		cout_point3(currentFeasible);
		cout << endl;
	}


	




	if (verbose == true)
		printf("\nVoxel strip packing finished.\n");

	// Report the time:
	clock_t packingClock = clock();
	elapsedTime = (float) (packingClock - packingClockStart) / CLOCKS_PER_SEC;
	if (verbose == true)
		printf("Elapsed time: %.2f s.\n\n", elapsedTime);

}

void PACKING_LAYOUT::strategic_oscillation()
{
	int lob = 0;
	strategic_oscillation(lob);
}

void PACKING_LAYOUT::strategic_oscillation(int lob)
{
	highest_container_point(); // set maxHeightVoxels 
	int updateBy = (int) ceil(maxHeightVoxels*options->decPercentage);
	updateBy = min(updateBy, maxHeightVoxels - lob);
	cout << "Shrinking solution by " << updateBy << " voxels!" << endl;
	UpperBoundH = maxHeightVoxels - updateBy;
	cout << "New upper bound is: " << UpperBoundH << endl;

	bool randomizeInserts = true;
	if (options->oscillationMode == 0)
	{
		for (int p = 0; p < instance->nPieces; p++)
		{
			// if (!solutionInfeasible || justMoveDown)
			// {
				int ri = rand() % UpperBoundH;
				if (pieceIndices[p][od] > ri)
					pieceIndices[p][od] = max(0, pieceIndices[p][od] - updateBy);
				else // Adjust it not to go over the upper bound
					pieceIndices[p][od] = min(pieceIndices[p][od], max(0,UpperBoundH - instance->pieces[p]->voxel.get_grid_size(od)));
			// }
		}

	}
	else if(options->oscillationMode == 1)
	{
		cout << "Decreasing from " << maxHeightVoxels << " to at least " << UpperBoundH << " by insertion." << endl;
		decrease_by_insert(max(UpperBoundH, lob), randomizeInserts);
		UpperBoundH = highest_container_point();
		cout << "New height is: " << maxHeightVoxels << "." << endl;
	}
	else if (options->oscillationMode == 2) // Just throw the protruding pieces at random IFV positions
	{
		// std::default_random_engine randomNumGenerator;
		for (int p = 0; p < instance->nPieces; p++)
		{
			if (pieceIndices[p][od] + instance->pieces[p]->voxel.get_grid_size(od) >= UpperBoundH) // Protruding piece
			{
				int maxPoint = UpperBoundH - instance->pieces[p]->voxel.get_grid_size(od);
				if (maxPoint < 0)
				{
					cout << "ERROR: Maximum position allowed for piece " << p << " is " << maxPoint << endl;
					exit(-1);
				}

				std::uniform_int_distribution<int> integerDistribution(0, maxPoint);
				//cout << "-------------------------------------------" << endl;
				//cout << "od = " << od << " moved to " << pieceIndices[p][od] << endl;
				//report_piece(p);
				//cout << " MOVED TO:   " << endl;

				pieceIndices[p][od] =  integerDistribution(randomNumGenerator);
				for (int cddd = 0; cddd < 3; cddd++)
				{
					if (cddd == od)
						continue;
					// int cd = (od + cddd) % 3;
					std::uniform_int_distribution<int> integerDistributionCD(0, instance->maxPositions[instance->pieces[p]->index][cddd]);
					pieceIndices[p][cddd] = integerDistributionCD(randomNumGenerator);
				}

				//report_piece(p);
				//cout << "Maxpoint was: " << maxPoint << endl;
				//cout << "Height now is: " << pieceIndices[p][od] +instance->pieces[p]->voxel.get_grid_size(od) << endl;
				//cout << endl << endl;

			}
		}
	}
	cout << "Strategic oscillation finished. New height is: " << highest_container_point() << " (Upper bound is: " << UpperBoundH << ")" << endl;
}


int PACKING_LAYOUT::select_item(vector<float> &probabilities)
{
	bool sanityCheck = true; // debug : speed
	if (sanityCheck)
	{
		float sum = 0;
		for (int i = 0; i < probabilities.size(); i++)
			sum += probabilities[i];
		if (abs(sum - 1) > 1e-6)
		{
			cout << endl << endl << "ERROR: The probabilities vector sum is: " << sum << ", while it should be 1!!!!" << endl;
			cout << "(there is an offset of: " << abs(sum - 1)	<< endl << endl;
		}
	}

	// Generate random number between 0 and 1
	double randomNum = double(rand()) / double(RAND_MAX);
// 	cout << "randomNum = " << randomNum << endl;
	float sum = 0;
	for (int i = 0; i < probabilities.size() - 1; i++)
	{
		sum += probabilities[i];
		if (randomNum <= sum)
			return(i);
	}
	return(probabilities.size() - 1);

}
void PACKING_LAYOUT::decrease_n_by_perc(int n, float perc, vector<float> &probabilities)
{
	float initialProbN = probabilities[n];
	probabilities[n] = probabilities[n] * (1 - perc);
	float theRestIncreaseBy = (initialProbN - probabilities[n])/(probabilities.size() - 1);
	for (int i = 0; i < probabilities.size(); i++)
	{
		if (i == n)
			continue;
		probabilities[i] += theRestIncreaseBy;
	}
}

void PACKING_LAYOUT::decrease_all_but_n_by_perc(int n, float perc, vector<float> &probabilities)
{
	float totalSum = 0;
	for (int i = 0; i < probabilities.size(); i++)
	{
		if (i == n)
			continue;
		probabilities[i] = probabilities[i] * (1 - perc);
		totalSum += probabilities[i];
	}
	probabilities[n] = 1 - totalSum;
}

double PACKING_LAYOUT::decrease_by_insert(int decreaseTo, bool randomizeInserts)
{

	// serialise("before_insert.psol");
	// Determine which pieces will be inserted
	vector<int> piecesToInsert;
	for (int p = 0; p < instance->nPieces; p++)
	{
		instance->maxPositions[instance->pieces[p]->index][od] = decreaseTo - instance->pieces[p]->voxel.get_grid_size(od);
		if (pieceIndices[p][od] > instance->maxPositions[instance->pieces[p]->index][od])
		{
			piecesToInsert.push_back(p);
			// Get it out of the way:
			pieceIndices[p][od] = -10 * instance->pieces[p]->voxel.get_grid_size(od);
		}
	}

	// Insert them in the best position:
	// cout << "WARNING: Randomize order?" << endl;
	if (randomizeInserts)
		random_shuffle(piecesToInsert.begin(), piecesToInsert.end());

	vector<int> bestPoint = { 0, 0, 0 };
	for (int pidx = 0; pidx < piecesToInsert.size(); pidx++)
	{
		double bestFitness = -1;
		int p = piecesToInsert[pidx];
		// Go through all the container points:
		for (int ipos = 0; ipos < instance->maxPositions[instance->pieces[p]->index][0]; ipos++)
		{
			for (int jpos = 0; jpos < instance->maxPositions[instance->pieces[p]->index][1]; jpos++)
			{
				for (int kpos = 0; kpos < instance->maxPositions[instance->pieces[p]->index][2]; kpos++)
				{
					pieceIndices[p] = { ipos, jpos, kpos };
					double testFitness = full_fitness();
					if (testFitness > bestFitness + TOL)
					{
						bestFitness = testFitness;
						bestPoint = pieceIndices[p];
					}
				}
			}

		}
		pieceIndices[p] = bestPoint;

	}

	for (int p = 0; p < instance->nPiecesOr; p++)
		instance->maxPositions[p][od] = -1;

	// serialise("after_insert.psol");
	return 0.0;
}
void PACKING_LAYOUT::its()
{
	if (options->randomSeed < 0)
	{
		cout << "Random seed set to time(NULL), as options->randomSeed was " << options->randomSeed << endl;
		srand(time(NULL)); 
	}
	else
	{
		cout << "Random seed set to options->randomSeed = " << options->randomSeed << endl;
		srand(options->randomSeed); 
	}

	// srand(time(NULL)); // This is at the beginning of each method!
	cout << "Rand num: " << ((double) rand()/(RAND_MAX)) << endl;

	bool initialSolBLB = true;
	int lob = 0;
	int upb = 0;
	find_simple_bounds_voxel(lob, upb, initialSolBLB);

	if (initialSolBLB)
	{
		UpperBoundH = (int)ceil(upb*(1 - options->decPercentage));
		strategic_oscillation(lob);
	}

	cout << "Voxel bounds are: " << lob << ", " << upb << endl;
	its(lob, upb);
}
void PACKING_LAYOUT::its(int lob, int upb) // Iterated Tabu Search
{
	bool anyNoOverlap = false;
	cout << endl << endl << "  ***  ITERATED TABU SEARCH  ***  " << endl << endl << endl;
	PACKING_LAYOUT bestNoOverlap = (*this);

	// PARAMETERS:
	// double decreaseBy = 0.1;
	// bool randomizeInserts = true;
	// bool justMoveDown = false; // Otherwise, decrease by insert

	// Initial solution:
	/*
	bool initialSolAlgorithmically = false;
	if (initialSolAlgorithmically)
		initial_solution_first_fit((*options));
	else
	{
		for (int i = 0; i < instance->nPieces; i++)
		{
			pieceIndices[i] = { 0, 0, 0 };
			ignorePiece[i] = false;
			piecePlaced[i] = true;
		}
	}
	*/
	int dummyInt = 0;


	// Fitness debug:
	bool debugInfo = false;
	if (debugInfo)
	{
		cout << "Initial fitness is: " << full_fitness() << endl;
		cout << "Penetration depth values: " << endl;
		for (int i = 0; i < relPenetrationDepth.size(); i++)
		{
			cout << relPenetrationDepth[i] << ", ";
		}
		cout << endl;
	}



	// serialise("initial_solution.psol");
	if (is_feasible_voxel())
	{
		anyNoOverlap = true;
		bestNoOverlap = (*this);
		upb = min(upb, highest_container_point());
		cout << "Upper bound adjusted to: " << upb << endl;
		strategic_oscillation(lob);
		UpperBoundH = highest_container_point();
	}
	else
		UpperBoundH = (int) ceil((1 - options->decPercentage) * upb);

	// decrease_by_insert(max(lob, int(floor(highest_container_point()*(1 - decreaseBy)))), true);


	int trial = 0;
	int od = instance->container->openDimension;
	int it_counter = 0;
	
	clock_t InitialPackingClock = clock();
	clock_t packingClock = clock();
	elapsedTime = float(packingClock - InitialPackingClock) / CLOCKS_PER_SEC;
	double elapsedTimePrev = elapsedTime;
	int originalIterations = options->maxIters;
	double originalDecPercentage = options->decPercentage;
	while (true)
	{
		it_counter++;
		cout << "Tabu search iteration no. " << it_counter << " best feasible H is " << upb << " (lower bound: " << lob << ")" << endl;
		tabu_search(trial);
		// serialise("TS_", trial);
		bool solutionInfeasible = !is_feasible_voxel();

		// Exit if time exceeds our limit (before having the chance to save any improvement!)
		clock_t packingClock = clock();
		elapsedTime = float(packingClock - InitialPackingClock) / CLOCKS_PER_SEC;
		if (elapsedTime >= options->maxTime)
		{
			cout << "Time finished! " << elapsedTime << " seconds (returning to previous iteration, " << elapsedTimePrev << ")" << endl;
			elapsedTime = elapsedTimePrev;
			break;
			// return;
		}
		else
		{
			elapsedTimePrev = elapsedTime;
		}


		// Handle oscillation
		if (solutionInfeasible)
		{
			cout << "Iteration was INFEASIBLE, moving up and trying tabu search again..." << endl;
			UpperBoundH += 1;
			options->int2 = 3;
			highest_container_point();
			cout << "Fitness is: " << full_fitness() << endl;
			fix_overlaps_from_pairs();
		}
		else
		{
			cout << "Iteration was FEASIBLE, " << maxHeightVoxels << " voxels, real: " << maxHeight << endl; 
			options->decPercentage = originalDecPercentage;
			options->maxIters = originalIterations;
			cout << "Changing number of iterations and decreasing percentage: " << endl;
			cout << "Decreasing percentage: " << options->decPercentage << endl;
			cout << "Maximum iterations per height: " << options->maxIters << endl;

			serialise("its_zero_overlap_", highest_container_point());
			anyNoOverlap = true;
			bestNoOverlap = (*this);
			if (maxHeightVoxels < lob + TOL)
			{
				cout << endl;
				cout << "********************************" << endl;
				cout << "***     Optimal solution     ***" << endl;
				cout << "********************************" << endl;
				cout << "Height is the lower bound, found the optimal solution!!!";
				cout << " (H = " << maxHeightVoxels << ", real h = " << maxHeight << ")" << endl;
				cout << endl;
				break;
			}
		}

		if (!solutionInfeasible || (UpperBoundH >= upb))
		{
			if ((UpperBoundH >= upb))
			{
				cout << "\n +++ Going somewhere where we have been before +++ \n" << endl;
				options->decPercentage = (int) ceil(options->decPercentage * options->itsDecreaseMultiplier);
				options->maxIters = (int) ceil(options->maxIters * options->itsIterationMultiplier);
				cout << "Changing number of iterations and decreasing percentage: " << endl;
				cout << "Decreasing percentage: " << options->decPercentage << endl;
				cout << "Maximum iterations per height: " << options->maxIters << endl;
			}
			else
			{
				if (!solutionInfeasible)
				{
					cout << "Updating upper boundary from " << upb;
					upb = min(highest_container_point(), upb);
					cout << " to " << upb << " since a new solution has been found!" << endl;
				}

				cout << endl << "*** found a feasible solution ***" << endl;
				cout << "H = " << maxHeightVoxels << endl << endl;
			}

			// int updateBy = ceil(options->decPercentage*upb);
			// cout << "Shrinking solution by " << updateBy << " voxels!" << endl;
			// UpperBoundH -= updateBy;
			// Move down if any piece goes over the top:
			// if (solutionInfeasible & !justMoveDown)
				// cout << "Destroying everything..." << endl;

			strategic_oscillation(lob);

			highest_container_point();
			full_fitness();
			fix_overlaps_from_pairs();
		}

		trial++;
		if (trial >= options->maxKicks)
		{
			cout << "Max. number of kicks reached. Exiting!" << endl;
			break;
		}

	} // end of while(true)
	options->maxIters = originalIterations;
	options->decPercentage = originalDecPercentage;
	// Return to the best solution with no overlap!
	if (anyNoOverlap)
	{
		(*this) = bestNoOverlap;
		cout << "Best no overlap height is: " << highest_container_point() << endl;
	}
	else
	{
		cout << endl << endl << "No valid solution found!!!!" << endl << endl;
	}

}

void PACKING_LAYOUT::ivns()
{
	cout << "Running IVNS, last edited: " << __TIMESTAMP__ << endl;
	// cout << "WARNING: Lower and upper bounds are hard-coded!!!" << endl;
	int lob = 0;
	int upb = 0;
	find_simple_bounds_voxel(lob, upb);
	cout << "Voxel bounds are: " << lob << ", " << upb << endl;
	ivns(lob, upb);
}

void PACKING_LAYOUT::ivns(int lob, int upb) // Iterated Variable Neighbourhood Search
{
	// This is at the beggining of each method!
	if (options->randomSeed < 0)
	{
		cout << "Random seed set to time(NULL), as options->randomSeed was " << options->randomSeed << endl;
		srand(time(NULL)); 
	}
	else
	{
		cout << "Random seed set to options->randomSeed = " << options->randomSeed << endl;
		srand(options->randomSeed); 
	}

	// PARAMETERS:
	cout << "Rand num: " << ((double) rand()/(RAND_MAX)) << endl;
	bool silent = false;
	bool firstDescent = true;
	bool modelAsNeighbourhood = true;
	// int modelBoxOverlap = 10;
	// int modelBoxNonOverlap = 1;
	// int modelMaxOverlapPairs = 2;
	int MAX_TABU_ITERS = options->int2;
	// int MAX_TABU_ITERS = instance->nPairs;
	// int MAX_TABU_ITERS = ceil(instance->nPairs / 2);

	int	MAX_SWAPS_TO_TEST = int(options->double1);
	bool swapFromBest = true;
	bool placeAtZero = false;
	// bool decreaseByInsert = true;
	// bool insertRandomize = true;
	// double decreasePercentage = 0.75;
	// NEIGHBOURHOOD LIST:
	vector<NEIGHBOURHOOD*> neighbourhoodList;
	int currentNeighbourhood = 0;


	// int primaryN = 40;
	// int secondaryN = 30;

	NEIGHBOURHOOD ne0(int(options->sat0), this, int(options->double2)); // Cube, delta = 1
	cout << "N1 generated. Type " << options->sat0 << ", delta = " << int(options->double2) << endl;
	// NEIGHBOURHOOD ne1(40, this, 10); // Axis aligned, 10 cubes in each dir
	// cout << "N2 generated." << endl;

	NEIGHBOURHOOD ne2(int(options->satype), this, int(options->double3)); // Axis aligned, all container
	cout << "N2 generated. Type " << options->satype << ", delta = " << int(options->double3) << endl;

	// NEIGHBOURHOOD ne3(30, this, options->int3); // Insert in all container!
	// cout << "N4 generated." << endl;

	neighbourhoodList.push_back(&ne0);
	// neighbourhoodList.push_back(&ne1);
	neighbourhoodList.push_back(&ne2);
	// neighbourhoodList.push_back(&ne3);
	double move_gain = 0;
	vector<int> neighbourhood_improvements;
	neighbourhood_improvements.assign(3, 0);
	vector<double> neighbourhood_gains;
	neighbourhood_gains.assign(3, 0.0);
	NEIGHBOURHOOD * ne = neighbourhoodList[0];

	cout << "Neighbourhood list generated: " << endl;
	for (int i = 0; i < neighbourhoodList.size(); i++)
	{
		cout << "N" << i << ": " << "Type " << neighbourhoodList[i]->get_type() << ", parameter: " << neighbourhoodList[i]->get_parameter() << endl;
	}
	clock_t InitialPackingClock = clock();
	clock_t packingClock = clock();
	elapsedTime = float(packingClock - InitialPackingClock) / CLOCKS_PER_SEC;
	float elapsedTimePrev = elapsedTime;

	// Init some vars:
	if (is_feasible_voxel())
		UpperBoundH = max(lob, int(ceil(upb*options->decPercentage)));
	else
		UpperBoundH = upb - 1;

	bool modifyHeight = true; // Move height up and down
	bool stillImproving = true; // Control the first descent part
	bool increaseHeight = true;
	int heightIterations = -1;
	int overallIterations = -1;
	double overallBestF = -1;
	int tabuIters = -1;
	bool noOverlap = false;
	bool do_n30 = true;
	int modelLastRunH = -1;
	bool old_printing = false;
	
	// Printing aids:
	int colW = 8;
	const char separator = ' ';
	stringstream printStream;

	methodString = "Iterated Variable Neighbourhood Search";
	PACKING_LAYOUT bestSolution = (*this);
	PACKING_LAYOUT bestFeasibleSolution = (*this);
	PACKING_LAYOUT lastModel = (*this);
	NEIGHBOURHOOD swapN(50, this, -1); // This neighbourhood controls the swaps and tabu list of swaps
	swapN.enabledTabu = true;
	swapN.tabuSize = MAX_TABU_ITERS;
	// swapN.tabuSize = ceil(MAX_TABU_ITERS / 2);
	cout << "Swap neighbourhood has a tabu of: " << swapN.tabuSize << endl;

	// Check the maximum piece repetition:
	int nMaxEqualPieces = 1;
	for (int i = 0; i < instance->nPiecesOr; i++)
		nMaxEqualPieces = max(nMaxEqualPieces, instance->demands[i]);

	// Initial solution
	if (placeAtZero)
	{
		for (int i = 0; i < instance->nPieces; i++)
		{
			piecePlaced[i] = true;
			ignorePiece[i] = false;
		}
	}
	else
	{
		strip_voxel_packing_first_fit_nfv();
		highest_container_point();
		noOverlap = is_feasible_voxel();
		if (noOverlap)
			bestFeasibleSolution = (*this);
	}

	int originalIterations = options->maxIters;
	double originalDecPercentage = options->decPercentage;
	int heightModifications = 0;

	int hmods = 0;
	while (modifyHeight)
	{
		hmods++;
		cout << "Modifying height, number: " << hmods << endl;
		cout << "\tNeighbourhoods performance in last iteration: " << endl;
		for (int n_el = 0; n_el < neighbourhood_improvements.size(); ++n_el)
		{
			cout << "\t" << neighbourhood_improvements[n_el] << "(+ " << neighbourhood_gains[n_el] << " F)";
			neighbourhood_improvements[n_el] = 0;	 	
			neighbourhood_gains[n_el] = 0;	 	
		}
		cout << endl;

		swapN.clear_tabu_list();
		if (noOverlap)
		{
			increaseHeight = false;
			serialise("ivns_zero_overlap_", highest_container_point());
			//options->decPercentage = originalDecPercentage;
			//options->maxIters = originalIterations;
			//cout << "Changing number of iterations and decreasing percentage: " << endl;
			//cout << "Decreasing percentage: " << options->decPercentage << endl;
			//cout << "Maximum iterations per height: " << options->maxIters << endl;
			// cout << printStream.str();
			// break;
		}

		if (overallIterations > 1)
		{
			// debug : necessary to come back on ivns????
			// pieceIndices = bestSolution.pieceIndices;
			fix_overlaps_from_pairs();
			highest_container_point();
			cout << endl << endl << endl <<   "***********" << endl;
			cout << "***********" << endl;
			cout << "***********" << endl << endl;
			double currentF = full_fitness();
			cout << "Best feasible solution has H = " << bestFeasibleSolution.maxHeightVoxels;
			cout << " (real H = " << bestFeasibleSolution.maxHeight << ")" << endl;
			cout << "Best working solution has " << bestSolution.overlapCount << " overlaps with H " << bestSolution.maxHeightVoxels << endl;
			cout << "Expected fitness: " << overallBestF << " F = " <<	currentF;
			ne->set_fitness(currentF);
			tabuIters = 0;
		}

		if (increaseHeight && !(UpperBoundH >= (upb - 1)))
		{

			if (true)
			{
				cout << "Going back to best solution." << endl;
				cout << "Fitness was " << full_fitness_overlap() << endl;
				int currentHbeforeChange = highest_container_point();
				cout << "Height was " << currentHbeforeChange << endl;
				pieceIndices = bestSolution.pieceIndices;
				fix_overlaps_from_pairs();
				highest_container_point();
				cout << "Current fitness is now " << full_fitness_overlap() << endl;
				cout << "Current height is now " << highest_container_point();
				cout << "Height increased from " << UpperBoundH;
				UpperBoundH = currentHbeforeChange + 1;
			}
			else
			{
				cout << "Height increased from " << UpperBoundH;
				UpperBoundH++;	
			}
			heightModifications++;
			cout << " to " << UpperBoundH << "(modification no. " << heightModifications << ")"  << endl;
			cout << "The best feasible H we have is: " << bestFeasibleSolution.maxHeightVoxels << " (bounds: ";
			cout << lob << ", " << upb << ")" << endl;
		}
		else
		{
			//options->decPercentage = options->decPercentage * options->itsDecreaseMultiplier;
			//options->maxIters = options->maxIters * options->itsIterationMultiplier;
			//cout << "Changing number of iterations and decreasing percentage: " << endl;
			//cout << "Decreasing percentage: " << options->decPercentage << endl;
			//cout << "Maximum iterations per height: " << options->maxIters << endl;
			// if (!increaseHeight && (UpperBoundH >= (upb - 1)))
			cout << "Performing SO" << endl;
			strategic_oscillation(lob);
			heightModifications++;
			// Reduce height to find a better point
			// upb = highest_container_point();
			// UpperBoundH = ceil(decreasePercentage*upb);

			// if (decreaseByInsert)
			// {

				// serialise("before_insert.psol");
				// decrease_by_insert(UpperBoundH, insertRandomize);
				// serialise("after_insert.psol");
			// }
			// else
			// { // Just adjust the position to avoid going over the upper bound
				// for (int p = 0; p < instance->nPieces; p++)
					// pieceIndices[p][od] = min(pieceIndices[p][od], UpperBoundH - instance->pieces[p]->voxel.get_grid_size(od));
			// }

			// Update details so everything keeps working:
			overallBestF = full_fitness();
			highest_container_point();
			fix_overlaps_from_pairs();
			bestSolution = (*this);

		}

		if (UpperBoundH >= upb)
		{
			cout << "IVNS reached the upper bound height. Finish!" << endl;
			if (bestFeasibleSolution.is_feasible_voxel())
				(*this) = bestFeasibleSolution;
			return;
		}

		cout << endl << "New height is: " << UpperBoundH << endl;
		// Height is fixed now, resolve overlap:

		noOverlap = false;
		stillImproving = true;
		increaseHeight = true;
		/*TEMPORARY*/
		int move_idx = -1;
		/*TEMPORARY*/
		while (stillImproving)
		{
			// bool keepGoing = true;
			if (noOverlap)
				break;
			overallIterations++;
			// cout << "Calculating fitness and fixing overlaps... ";
			double bestF = full_fitness(); // debug : speed
			fix_overlaps_from_pairs();
			// cout << "Done." << endl;
			ne->cfit = bestF;
			int bestN = -1;
			int bestP = -1;

			// Populate overlap list:
			vector<int> overlapList;
			for (int p = 0; p < instance->nPieces; p++)
			{
				if (overlaps[p])
					overlapList.push_back(p);
			}
			// Randomise overlap list:
			if (firstDescent)
				random_shuffle(overlapList.begin(), overlapList.end());

			// Explore the full neighbourhood of each piece:
			bool didImprove = false;
			// cout << "Testing neighbours! (" << overlapList.size() << " overlapping pairs)" << endl;
			for (int idx = 0; idx < overlapList.size(); idx++)
			{
				int p = overlapList[idx];
				ne->change_piece_to(p);
				while (ne->next_neighbour(true))
				{
					if (ne->tfit > bestF + TOL)
					{
						bestF = ne->tfit;
						bestN = ne->cneig;
						bestP = p;
						didImprove = true;
						if (firstDescent)
							break;
					}
					// if (debugInfo)
					// {
					// cout << "\tn = " << ne.cneig << ", f = " << ne.tfit << ", BN  = " << bestN << ", BF = " << bestF << "; dir: ";
					// cout_point3(ne.dir_i, ne.dir_j, ne.dir_k);
					// }
				} // End for each neighbourhood point

				if (didImprove && firstDescent)
					break;
			} // End for each piece

			// Check the time before going any further 

			
			clock_t packingClock = clock();
			elapsedTime = float(packingClock - InitialPackingClock) / CLOCKS_PER_SEC;
			if (elapsedTime >= options->maxTime)
			{
				cout << "Time finished! " << elapsedTime << " seconds (returning to previous iteration, " << elapsedTimePrev << ")" << endl;
				elapsedTime = elapsedTimePrev;
				if (bestFeasibleSolution.is_feasible_voxel())
						(*this) = bestFeasibleSolution;
				return;
			}
			else
				elapsedTimePrev = elapsedTime;
			
			move_gain = bestF - ne->cfit;
			if (didImprove)
			{
				// cout << "Found a good move, expecting to go from " << ne->cfit << " to " << bestF;
				// Move to the new point:
				ne->change_piece_to(bestP);
				ne->cneig = bestN - 1;
				ne->rejectedMovements = 0;
				if (!ne->next_neighbour(false)) // This is a "real" move!
				{
					cout << "Move rejected!!! (bestN = " << bestN << ")" << endl;
					cout << "P = " << ne->get_piece() << " cneig = " << ne->cneig << endl;
				}
				else
				{
					neighbourhood_gains[currentNeighbourhood] += move_gain;
					neighbourhood_improvements[currentNeighbourhood]++;
					/* TEMPORARY - debug : animation 
					move_idx++;
					std::stringstream sstm;
					sstm << "ivns_long/vns_" << heightModifications << "_move_";
					string tabuSol = sstm.str();
					serialise(tabuSol.c_str(), move_idx);
					/* End of TEMPORARY */

					// Error check (seemed ok)
					// double thisFitness = full_fitness(); // Needed?? debug : speed
					// if (abs(thisFitness - ne->cfit) > 1e-6)
					// {
					// 	cout << "ERROR!!!!!!!! These things are supposed to be equal!" << endl;
					// 	cout << "full_fitness() = " << thisFitness << ", ne->cfit = " << ne->cfit << " (diff: " << abs(thisFitness - ne->cfit) << ")" << endl;
					// }

					// thisFitness = ne.cfit;
					/* Commented out to make this extra verbose... */
					// if (thisFitness  > overallBestF)
						// int colW = 8;
					if (old_printing)
					{
						/* code */
						printStream << setw(colW) << overallIterations;
						printStream << setw(colW) << overlapCount;
						printStream << setw(colW) << highest_container_point();
						printStream << setw(colW) << setprecision(5) << ne->cfit;
						printStream << setw(colW) << "n_" << ne->get_type() << "_p_" << ne->get_parameter();
						if (ne->cfit  > overallBestF)
							printStream << "\t\t*****";
						printStream << endl;
					}

						// cout << setw(colW) << ne.get_piece();
						// cout << setw(colW) << ne.cneig;
						// printStream << "It " << overallIterations << " ";
						// printStream << " H: " << highest_container_point() << " F: " << thisFitness << " O: " << overlapCount;
						// printStream << " N = " << ne.get_type();
					if (ne->cfit  > overallBestF)
					{
						bestSolution.pieceIndices = pieceIndices;
						overallBestF = ne->cfit;
					}

					/* End of verbose comments */
					

					// if (overallBestF >= 1 - TOL)
						// printStream << endl << endl << "PERFECT FITNESS!!!! :)" << endl << endl;
					if (overlapCount < 1)
					{
						// printStream << endl << endl << "PERFECT FITNESS!!!! :)" << endl << endl;
						cout << "********************************" << endl;
						cout << "***      Zero overlap!!!    ***" << endl;
						cout << "********************************" << endl;
						noOverlap = true;
						bestFeasibleSolution = (*this);
						upb = min(upb, highest_container_point());
						break;
					}
				}

				// cout << "Moving back to N30" << endl;
				// if (!do_n30)
				// {
					// cout << "Moving to n30" << endl;
					// do_n30 = true; // Return to N1
				// }

				// Return to first neighbourhood:
				currentNeighbourhood = 0;
				ne = neighbourhoodList[currentNeighbourhood];
				ne->change_piece_to(0); // Make sure the neighbourhood is reset
				continue;
				// keepGoing = false;
			}

			if (currentNeighbourhood < neighbourhoodList.size() - 1)
			{
				// cout << "Switching neighbourhood from " << currentNeighbourhood;
				// cout << " (" << neighbourhood_improvements[currentNeighbourhood] << " good moves, +";
				// cout << neighbourhood_gains[currentNeighbourhood] << " fitness) ";
				// cout <<  " to ";

				currentNeighbourhood++;
				ne = neighbourhoodList[currentNeighbourhood];
				ne->change_piece_to(0); // Make sure the neighbourhood is reset

				// cout << currentNeighbourhood << endl;
				continue;

				// keepGoing = false;
				// continue;
			}
			else // Solution did not improve and we are out of neighbourhoods...
			{
				// cout << "No more neighbourhoods, we were at " << currentNeighbourhood;
				// cout << " (" << neighbourhood_improvements[currentNeighbourhood] << " good moves, +";
				// cout << neighbourhood_gains[currentNeighbourhood] << " fitness) ";			
				// cout <<  " with no improvement. " << endl;

				// Reset neighbourhood, and go to model or tabu search
				currentNeighbourhood = 0;
				ne = neighbourhoodList[currentNeighbourhood];
				ne->change_piece_to(0); // Make sure the neighbourhood is reset
			}

			// Time to report (And maybe check time...)
			// printStream << endl;
			if (old_printing)
			{
				cout << printStream.str();
				printStream.clear();
				printStream.str(std::string() );
			}

			// Test the conditions for doing the model!
			bool doModel = modelAsNeighbourhood;

			if (modelLastRunH == UpperBoundH)
			{
				doModel = false;
				// cout << "We already ran the model it at this height, so skipping it." << endl;
			}

			if (doModel)
			{
				// Only do the model if we are at the last neighbourhood
				// doModel = (currentNeighbourhood == (neighbourhoodList.size() - 1));
				doModel = (doModel && (overlapCount <= options->modelMaxOverlapPairs));
				// if (!doModel)
					// cout << "Too much overlap for running a model." << endl;
				int SolDistance = solution_distance(lastModel);
				if (SolDistance < options->modelBoxNonOverlap)
				{
					doModel = false;
					// cout << "The distance between this and the last solution solved is not enough to run a model!" << endl;
				}
			
			}

			if (doModel)
			{

				// Update that we are running it, to not repeat this height
				modelLastRunH = UpperBoundH;
				// Return to first neighbourhood:
				currentNeighbourhood = 0;
				ne = neighbourhoodList[currentNeighbourhood];
				ne->change_piece_to(0); // Make sure the neighbourhood is reset

				cout << endl << "Running the model..." << endl;
				cout << endl << "There are " << overlapCount << " overlaps (The model runs at " << options->modelMaxOverlapPairs << ")" << endl;
				cout << endl << "Overlapping pieces can move " << options->modelBoxOverlap << " the rest, " << options->modelBoxNonOverlap << endl;
				// double maxModelTime = 30;
				double maxModelTime = options->maxTime - elapsedTime;
				cout << "The allowed time is: " << maxModelTime << " s." << endl;
				bool didItWork = false;
				vector<int> modelBoxVec(instance->nPieces);
				for (int p = 0; p < instance->nPieces; p++)
				{
					if (overlaps[p])
						modelBoxVec[p] = options->modelBoxOverlap;
					else
						modelBoxVec[p] = options->modelBoxNonOverlap;
				}
				vector< vector <int> >  oldIndices = pieceIndices;
				lastModel = (*this);
				preprocess(modelBoxVec, UpperBoundH);
				init_cplex(modelBoxVec, maxModelTime, didItWork, UpperBoundH, oldIndices, true);
				if (didItWork)
				{
					cout << endl << "********************************" << endl;
					cout << 		"***      Zero overlap!!!    ***" << endl;
					cout << 		"********************************" << endl;
					cout << endl << "Yay! The model did something, solution was modified!" << endl;
					full_fitness(-1, true);
					fix_overlaps_from_pairs();
					UpperBoundH = highest_container_point();
					cout << "Found a valid solution with height: " << UpperBoundH << endl;
					bestFeasibleSolution = (*this);
					upb = min(upb, highest_container_point());
					noOverlap = true;
					// keepGoing = false;
					continue; // A height modification will follow
				}
				else
				{
					cout << "No luck with the model :/" << endl;
					// Make sure next time we will be calling the tabu search:
					currentNeighbourhood = neighbourhoodList.size();
					// keepGoing = true;
				}
			}
			// else
				// cout << "Not doing model." << endl;

			// CONDITIONS FOR DOING TABU SEARCH!
			bool doTabu = true;
			if (tabuIters < MAX_TABU_ITERS)
			{
				/* This no longer applies with the shake, we can swap the same pieces, no problem
				// Check if it is worth checking for swaps or not
				if (overlapList.size() < nMaxEqualPieces)
				{
					doTabu = false; // Just do it if there are different pieces overlapping
					for (int pidx = 0; pidx < overlapList.size() - 1; pidx++)
					{
						for (int pidx2 = pidx + 1; pidx2 < overlapList.size(); pidx2++)
						{
							if (instance->pieces[overlapList[pidx]]->index != instance->pieces[overlapList[pidx2]]->index)
							{
								doTabu = true;
								break;
							}
						}
						if (doTabu)
							break;
					}
				}
				if (!doTabu)
					cout << "Tabu Search did not find any pieces of different types, not performing it!!!" << endl;
				*/
			}
			else
			{
				doTabu = false;
				if (tabuIters < MAX_TABU_ITERS) 
					cout << "Tabu search iterations have expired!" << endl << "Skipping it!" << endl;
			}

			// PERFORM TABU SEARCH
			if (doTabu)
			{
				if (old_printing)
					cout << endl << "Decided to do a swap." << endl;

				double fitness_before_tabu = full_fitness_overlap();
				// keepGoing = false;
				// cout << "Moving to N50" << endl;
				currentNeighbourhood = 0;
				ne = neighbourhoodList[0];
				// No neighbour improved for any piece, perform one move of the
				// Tabu Search with the swap neighbourhood, even if it is worse
				// if (!silent)
					// printStream << endl << endl << "Search hit local optimum. Changing to swap neighbourhood..." << endl;
				overallIterations++;
				tabuIters++;
				// cout << "Going to swap..." << endl;
				bool swapPossible = true;

				bool applyShake = true;
				// Swap from the best solution known:
				if (swapFromBest)
				{

					if (old_printing)
						cout << "Going back to best solution." << endl;
					pieceIndices = bestSolution.pieceIndices;

					if(!applyShake) // Otherwise it will be updated later
					{
						full_fitness();
						fix_overlaps_from_pairs();
						highest_container_point();
					}
				}

				if (applyShake)
				{
					/* APPLY A SHAKE OF THE PIECES!!!! */

					// Calculate how much to shake:
					double tabuProgress = double(tabuIters)/double(MAX_TABU_ITERS);

					bool shakeOverlappingOnly = true;
					int minShake = int(options->double2);
					int maxShake = 3*minShake;
					int maxShakePosition = max(minShake, int(floor(tabuProgress*double(maxShake))) );
					std::uniform_int_distribution<int> integerDistribution(-1*maxShakePosition, maxShakePosition);
					if (old_printing)
						cout << "Shaking pieces. We have applied " << int(tabuProgress*100) << "% of the kicks, so we shake ";
					if (old_printing)
					{		
						if(shakeOverlappingOnly)
							cout << "only the overlapping pieces ";
						else
							cout << "all the pieces ";
						if (old_printing)
							cout << "in a box of max side: " << maxShakePosition << endl;
					}
					// cout << endl << " -- Performing shake: " << endl;
					int maxShakes = instance->nPieces;
					if (shakeOverlappingOnly)
						maxShakes = overlapList.size();

					for (int opc = 0; opc < maxShakes; ++opc)
					{
						int pidx = opc;

						if (shakeOverlappingOnly)		
							pidx = overlapList[opc];

						// cout << "\tMoved piece " << pidx << " from ";
						// cout_point3(pieceIndices[pidx]);
						for (int idir = 0; idir < 3; ++idir)
						{
							int shakeMove = integerDistribution(randomNumGenerator);
							if (pieceIndices[pidx][idir] + shakeMove < 0)
								pieceIndices[pidx][idir] = 0;
							else
							{	
								int maxPosThisPiece = instance->container->gridSize[idir] - instance->pieces[pidx]->voxel.get_grid_size(idir);
								if (idir == od)
									maxPosThisPiece = UpperBoundH - instance->pieces[pidx]->voxel.get_grid_size(idir);
								pieceIndices[pidx][idir] = min(pieceIndices[pidx][idir] + shakeMove, maxPosThisPiece);
							}
						}
						// cout << "\t to ";
						// cout_point3(pieceIndices[pidx]);
						// cout << endl;
					}

					// Tidy up:
					full_fitness_overlap();
					fix_overlaps_from_pairs();
				} /* END OF SHAKE! */


				bestF = -1;
				bestN = -1;
				bestP = -1;
				bool testManySwaps = true;
				int totalSwapsTested = 0;
				if (testManySwaps)	
				{
					if (old_printing)
						cout << "Starting to test a maximum of " << MAX_SWAPS_TO_TEST << " swaps..." << endl;
					for (int p = 0; p < instance->nPieces; p++)
					{
						int	swapsTested = 0;
						swapN.change_piece_to(p);
						while (swapN.next_random_neighbour(true, overlapList))
						{
							swapsTested++;
							if (swapsTested > MAX_SWAPS_TO_TEST)
								break;

							// cout << "\tTest: p = " << swapN.get_piece() << ",  q = " << swapN.cneig << " - 1, f = " << swapN.tfit;
							
							if (swapN.tfit > bestF)
							{
								// cout << " *** ";
								bestF = swapN.tfit;
								bestN = swapN.cneig;
								bestP = swapN.get_piece();
							}
							swapN.change_piece_to(p);
							// cout << endl;
						}
						totalSwapsTested += swapsTested;
					}
				}
				else
				{
					std::uniform_int_distribution<int> integerDistributionB(0, overlapList.size() - 1);
					int pieceP =  integerDistributionB(randomNumGenerator);
					bestP = pieceP;
					swapN.change_piece_to(pieceP);
					swapN.next_random_neighbour(true, overlapList);
					bestF = swapN.tfit;
					bestN = swapN.cneig;
					totalSwapsTested = 1;
					if (old_printing)
						cout << "Accepting the first swap." << endl;
				}
				if (old_printing)
				{
					cout << "Done." << endl << "Tested " << totalSwapsTested << " swaps, best fitness was by " << endl;
					cout << "swapping p = " << bestP << " with " << bestN << " - 1, achieving a fitness of " << bestF << endl << endl;
				}

				// cout << "Found best swap bestF = " << bestF << " bestN " << bestN << " bestP " << bestP << endl;
				// Of all the moves, perform the best
				// cout << "Ind before swap:" << endl;
				// show_indices();
				swapN.change_piece_to(bestP);
				swapN.cneig = bestN - 1;
				if (old_printing)
					cout << "Attempting swap: bestN = " << bestN << ", bestP = " << bestP << " bestF = " << bestF << endl;
				if (bestP == -1 || !swapN.next_neighbour(false)) // This is a "real" move!
				{
					if (old_printing)
					{
						printStream << "SWAP rejected!!! (bestN = " << bestN << ")" << endl;
						printStream << "Swapping P = " << swapN.get_piece() << " and Q = " << swapN.cneig << endl;
					}
					swapPossible = false;
				}
				else
				{
						/* TEMPORARY - debug : animation 
						move_idx++;
						std::stringstream sstm;
						sstm << "ivns_long/vns_" << heightModifications << "_move_";
						string tabuSol = sstm.str();
						serialise(tabuSol.c_str(), move_idx);
						/* End of TEMPORARY */

					// Display overlap list and stuff:
					//  cout << "Overlap list was: " << endl << "[" << overlapList[0];
					//  for (int ovidx = 0; ovidx < overlapList.size(); ovidx++)
					//  	cout << ", " <<  overlapList[ovidx];
					//  cout << ")" << endl;

					// double tplim = 0.75;
					// double omt = 1 - tplim;
					// if (tabuProgress >= tplim)
					// {
					// 	shakeOverlappingOnly = false;
					// 	maxShakePosition = max(minShake, int(floor((tabuProgress/omt  + (1 - 1/omt))*double(maxShake))) );
					// }

					double thisFitness = full_fitness();
					if (old_printing)
					{						
						printStream << setw(colW) << overallIterations;
						printStream << setw(colW) << overlapCount;
						printStream << setw(colW) << highest_container_point();
						printStream << setw(colW) << setprecision(5) << ne->cfit;
						printStream << setw(colW) << "SWAP(" << swapN.get_piece() << ", " << swapN.cneig << ")";
					}	
						// cout << setw(colW) << ne.get_piece();
						// cout << setw(colW) << ne.cneig;
						// printStream << "It " << overallIterations << " ";
						// printStream << " H: " << highest_container_point() << " F: " << thisFitness << " O: " << overlapCount;
						// printStream << " N = " << ne.get_type();
						// if (thisFitness  > overallBestF)
						// {
							// printStream << "\t\t*****";
						// }
						// printStream << endl;
						
					if (thisFitness  > overallBestF)
					{
						// printStream << "Swapping P = " << swapN.get_piece() << " and Q = " << swapN.cneig << endl;
						// printStream << "It " << overallIterations << " ";
						// printStream << " H: " << highest_container_point() << " F: " << thisFitness << " O: " << overlapCount;
						if (old_printing)
							printStream << "\t\t*****";

						bestSolution.pieceIndices = pieceIndices;
						overallBestF = thisFitness;
						// if (overallBestF >= 1 - TOL)
						if (overlapCount < 1)
						{
							// printStream << endl << endl << "PERFECT FITNESS!!!! :)" << endl << endl;
							cout << "********************************" << endl;
							cout << "***      Zero overlap!!!    ***" << endl;
							cout << "********************************" << endl;
							noOverlap = true;
							break;
						}
					} 
					// printStream << " H: " << highest_container_point() << " F: " << full_fitness() << endl;
					swapN.add_tabu();
					if (old_printing)
					{
						printStream << endl;
						cout << printStream.str();
						printStream.clear();
						printStream.str(std::string() );
					}
					// cout << endl << "Ind after swap:" << endl;
					// show_indices();
				}


				if (!swapPossible)
				{
					if (old_printing)
					{
						printStream << endl << endl << "No more swaps are possible, modify height!" << endl;
						printStream << "(Fitness is " << full_fitness() << ")" << endl;
						printStream << "Cleaning tabu list..." << endl;
					}
					swapN.clear_tabu_list();
					if (!silent)
					{
						cout << printStream.str();
						printStream.clear();
						printStream.str(std::string() );

					}

					// cout << "Moving back to N30" << endl;
					do_n30 = true; // Return to N1

					break;

				}
				double fitness_after_tabu = full_fitness_overlap();
				neighbourhood_improvements[2]++;
				neighbourhood_gains[2] += fitness_after_tabu - fitness_before_tabu;
			}
			else
			{
				printStream << endl << endl << "Skipping tabu." << endl;
				printStream << "(Fitness is " << full_fitness() << ")" << endl;
				if (!silent && old_printing)
				{
					cout << printStream.str();
					printStream.clear();
					printStream.str(std::string() );

				}
				break;

			}

			// Check time again:
			packingClock = clock();
			elapsedTime = float(packingClock - InitialPackingClock) / CLOCKS_PER_SEC;
			if (elapsedTime >= options->maxTime)
			{
				cout << "Time finished! " << elapsedTime << " seconds (returning to previous iteration, " << elapsedTimePrev << ")" << endl;
				elapsedTime = elapsedTimePrev;
				break;
			}
			else
			{
				elapsedTimePrev = elapsedTime;
			}


			if (!silent)
			{
				cout << printStream.str();
				printStream.clear();
				printStream.str(std::string() );
			}
			// cout << "Calculating tabu and fixing overlaps." << endl;
			// full_fitness();
			if (old_printing)
				cout << "End of iteration." << endl;
		} // End of local search (stillImproving)

		
	} // End of height modifications
	options->maxIters = originalIterations;
	options->decPercentage = originalDecPercentage;

	// Return to best feasible solution!!!!
	(*this) = bestFeasibleSolution;

} // End of ivns

void PACKING_LAYOUT::tabu_search()
{
	tabu_search(0);
}
void PACKING_LAYOUT::tabu_search(int ils_iteration)
{
	clock_t packingClock = clock();
	tabu_search(ils_iteration, packingClock);
}

void PACKING_LAYOUT::tabu_search(int ils_iteration, clock_t &initialTimer)
{
	if (options->randomSeed < 0)
	{
		cout << "Random seed set to time(NULL), as options->randomSeed was " << options->randomSeed << endl;
		srand(time(NULL)); 
	}
	else
	{
		cout << "Random seed set to options->randomSeed = " << options->randomSeed << endl;
		srand(options->randomSeed); 
	}

	cout << "Rand num: " << ((double) rand()/(RAND_MAX)) << endl;
	// Info. on screen and files:
	bool createFile = false;
	bool silent = false;
	bool debugInfo = false;
	bool sanityCheck = false;
	bool saveAllMoves = false;
	clock_t packingClockEnd = clock();


	methodString = "Tabu search";
	// UpperBoundH = 115;
	// int increaseAfter = floor(options->maxIters / 4); // One time
	int increaseAfter = options->maxIters + 1;
	// Tabu list size is int1

	PACKING_LAYOUT bestSolution = (*this);
	std::default_random_engine randomNumGenerator(std::random_device{}());

	NEIGHBOURHOOD ne(40, this, -1);
	ne.enabledTabu = true;
	// ne.tabuSize = min(5*overlapCount, 25);
	// ne.tabuSize = min(10*overlapCount, 50);
	ne.tabuSize = options->tabuSize;
	cout << "Tabu size is: " << ne.tabuSize << endl;
	cout << "Upper bound of H is: " << UpperBoundH;
	cout << " (current H is: " << highest_container_point() << ")" << endl;
	if (sanityCheck)
	{
		for (int i = 0; i < instance->nPieces; i++)
		{
			report_piece(i);
			cout << "Placed : ";
			if (piecePlaced[i])
				cout << "True";
			else
				cout << "False";
			cout << endl; 
			piecePlaced[i] = true;
		}
	}

	
	ofstream fi;
	if (createFile)
	{
		std::stringstream streamForFile;
		streamForFile << "tabu_fitness" << ils_iteration << ".csv";
		string csvFilename = streamForFile.str();
		fi.open(csvFilename, std::ofstream::trunc);
	}
	// Header:
	int colW = 8;
	const char separator = ' ';
	stringstream printStream;
	if (!silent)
	{
		printStream << setw(7 * colW + 3) << setfill('-') << " " << endl;;
		printStream << setprecision(5);
		printStream << setw(colW) << setfill(separator) << "Iter";
		printStream << setw(colW) << setfill(separator) << "Overl";
		printStream << setw(colW) << setfill(separator) << "Height";
		printStream << setw(colW) << setfill(separator) << "Fitness";
		printStream << setw(colW) << setfill(separator) << "Piece";
		printStream << setw(colW) << setfill(separator) << "Neighb.";
		printStream << setw(colW) << setfill(separator) << "(Dir)";
		printStream << endl;
		printStream << setw(7 * colW + 3) << setfill('-') << " " << endl;;
		cout << printStream.str();
	}

	if (createFile)
		fi << printStream.str().c_str();

	vector<vector <int > > bestIndices; // All we need to know about the best solution!

	double absoluteBestF = full_fitness();
	
	int iter = 0;
	int nPrints = 0;
	bool isImproving = false;
	ne.cfit = full_fitness();
	while (iter < options->maxIters)
	{
		iter++;
		 // Every increaseAfter iterations, increase H
		if (iter % increaseAfter == 0)
		{
			int bound = UpperBoundH + 1;
			cout << ">> Updating the upper bound to " << bound << endl;
			cout << "Current fitness is: " << ne.cfit << " (from neighbour) " << full_fitness() << " (calculated)." << endl;
			// (*this) = bestSolution;
			cout << "Best solution fitness was: " << absoluteBestF << " (from solution: " << bestSolution.full_fitness() << ")" << endl;
			cout << "Copying indices..." << endl;
			pieceIndices = bestSolution.pieceIndices;
			coordinates_from_indices(resolution);
			ne.cfit = full_fitness();
			ne.tfit = -1;
			ne.pfit = -1;
			fix_overlaps_from_pairs();
			cout << "Done." << endl;
			cout << "Best solution fitness was: " << absoluteBestF << " (from current solution: " << full_fitness() << ")" << endl;
			UpperBoundH = bound;
			ne.clear_tabu_list();
			
		}
		// Select a random piece:
		// Only if we have not improved in the previous!
		// if (!isImproving)
		bool pickOldWay = false;
		int ri = -1;
		if (pickOldWay)
		{
			ri = (int) double(rand()) % instance->nPieces; // BIASED!!!!!!!!
			int k = 0;
			while (!overlaps[ri])
			{
				k++;
				if (k > 5 * instance->nPieces)
				{
					cout << "WARNING! Unable to find an overlapping piece! picking any..." << endl;
					break;
				}
				ri = (int) double(rand()) % instance->nPieces;
			}
		}
		else
		{
			// Better way of doing the previous (might be slower...)
			int overlappingPieces = std::count(overlaps.begin(), overlaps.end(), true);
			std::uniform_int_distribution<int> integerDistribution(0, overlappingPieces - 1);
			int chosenIndex = integerDistribution(randomNumGenerator);
			int overlapsFound = -1;
			for (int iidx = 0; iidx < instance->nPieces; iidx++)
			{
				if (overlaps[iidx])
				{
					overlapsFound++;
					if (overlapsFound == chosenIndex)
					{
						ri = iidx;
						break;
					}
					
				}
			}
		}

		// Select:
		ne.change_piece_to(ri);

		// Assume it is not going to improve...
		isImproving = false;

		// Explore its neighbourhood:
		int bestN = 0;
		double bestF = -1;

		if (debugInfo)
		{
			cout << endl << "-- Moving P = " << ne.get_piece() << "(height: " << instance->pieces[ne.get_piece()]->voxel.get_grid_size(od) << ") located at: ";
			cout_point3(pieceIndices[ne.get_piece()]);
		}

		ne.cneig = -1;
		while (ne.next_neighbour(true))
		{
			if (ne.tfit > bestF)
			{
				bestF = ne.tfit;
				bestN = ne.cneig;
			}
			if (debugInfo)
			{
				cout << "\tn = " << ne.cneig << ", f = " << ne.tfit << ", BN  = " << bestN << ", BF = " << bestF << "; dir: ";
				cout_point3(ne.dir_i, ne.dir_j, ne.dir_k);
			}
		}
		ne.rejectedMovements = 0; // This will be at a maximum now!
		if (debugInfo)
		{
			cout << "Best N was " << bestN << " with F = " << bestF << ", moving to:";
			cout_point3(ne.dir_i, ne.dir_j, ne.dir_k);
			cout << endl;
		}

		// Move to the best point found on this neighbourhood:

		if (bestF > -TOL) // Do not accept invalid fitness (-1)...
		{
			
			ne.cneig = bestN - 1;
			if (!ne.next_neighbour(false)) // This is a "real" move!
			{
				cout << "Move rejected!!! (bestN = " << bestN << ")" << endl;
				cout << "P = " << ne.get_piece() << " cneig = " << ne.cneig << endl;
				// cout << "Tabu list: " << endl;
				// ne.cout_tabu_list(ne.tabuSize <= 10);
				highest_container_point();
			}
			else
			{
				// int dummyInt = 0;
				// cout << "Expected fitness afterwards = " << bestF << " real: " << full_fitness_overlap(dummyInt) << endl;

				if (saveAllMoves)
				{
					std::stringstream sstm;
					sstm << "ivns_long/goingTabu" << "_ils_" << ils_iteration << "_";
					string tabuSol = sstm.str();
					serialise(tabuSol.c_str(), iter);
				}
			}

			// cout << "Attempted: " << bestN << "Found: " << ne.cneig << " (" << (bestN == ne.cneig) << ")" << endl;
			// fix_overlaps_from_pairs();
			// full_fitness_overlap(iter); // In case we are not calculating it correctly...
			// cout << "Neighbour fitness = " << ne.cfit << "; Real fitness: " << full_fitness_overlap(iter) << endl;

			// If we are improving, we keep with this piece!
			if (ne.cfit > ne.pfit)
				isImproving = true;
			// However, if we completely solved it's overlap, change it!
			if (!overlaps[ne.get_piece()]) 
			{
				// cout << "Found non-overlaping position :), P = " << ne.get_piece() << endl;
				isImproving = false;
			}

			// The file has ALL the movements!
			if (createFile)
				fi << iter << ", " << overlapCount << ", " << maxHeightVoxels << ", " << ne.cfit << ", " << ne.get_piece() << ", " << ne.cneig << endl;

			if (ne.cfit > absoluteBestF)
			{
				// Report our best fitness:
				if (!silent)
				{
					// cout << iter << ", " << overlapCount << ", " << highest_container_point() << ", " << ne.cfit << ", " << ne.get_piece() << ", " << ne.cneig << endl;
					cout << setw(colW) << iter;
					cout << setw(colW) << overlapCount;
					cout << setw(colW) << highest_container_point();
					cout << setw(colW) << setprecision(5) << ne.cfit;
					cout << setw(colW) << ne.get_piece();
					cout << setw(colW) << ne.cneig;

					int dirnum = int(ne.cneig) % 6;

					string dir_string = "+ x";

					if (dirnum == 1)
						dir_string = "- x";
					else if (dirnum == 2)
						dir_string = "+ y";
					else if (dirnum == 3)
						dir_string = "- y";
					else if (dirnum == 4)
						dir_string = "+ z";
					else if (dirnum == 5)
						dir_string = "- y";

					cout << setw(colW) << dir_string;
					cout << endl;
					nPrints++;
					if (nPrints > 30)
					{
						cout << printStream.str();
						nPrints = 0;
					}
				}

				// Only save if we are within time:
				packingClockEnd = clock();
				elapsedTime = float(packingClockEnd - initialTimer) / CLOCKS_PER_SEC;
				if (elapsedTime > options->maxTime)
				{
					cout << "Time up for TS!" << endl;
					break;
				}

				// Save the indices:
				absoluteBestF = ne.cfit;
				bestIndices = pieceIndices;
				bestSolution = (*this);

				// This is the best so far:
				// serialise("TS_best", iter);

				// if (ne.cfit > 1 - TOL && ne.cfit < 1 + TOL)
				if (overlapCount < 1)
				{
					cout << endl;
					cout << "********************************" << endl;
					cout << "***      Zero overlap!!!    ***" << endl;
					cout << "********************************" << endl;
					cout << endl;
					cout << "Fitness is " << ne.cfit << "! Exiting..." << endl;
					return;
				}

			}

			// Add the oposite move to the tabu list:
			ne.add_tabu();
			if (debugInfo)
				ne.cout_tabu_list(ne.tabuSize < 10);
		}
		else
		{
			// If the fitness was -1, it means the piece cannot go to any neighbour, likely a bug...
			cout << "WARNING: Piece trapped!!! P = " << ne.get_piece() << endl;
			ne.cout_tabu_list(ne.tabuSize <= 10);
		}

		if (debugInfo)
		{
			cout << endl << "xx New location of P = " << ne.get_piece() << " is: ";
			cout_point3(pieceIndices[ne.get_piece()]);
		}

		packingClockEnd = clock();
		elapsedTime = float(packingClockEnd - initialTimer) / CLOCKS_PER_SEC;
		if (elapsedTime > options->maxTime)
		{
			cout << "Reached time limit in Tabu Search, exit..." << endl;
			break;
		}

	} // End of iterations


	// Restore the best we have found:
	// pieceIndices = bestIndices;
	(*this) = bestSolution;

	packingClockEnd = clock();
	elapsedTime = float(packingClockEnd - initialTimer) / CLOCKS_PER_SEC;

	// Do updates on the solution:
	highest_container_point();
	is_feasible_voxel();
	int k = 0;
	coordinates_from_indices(resolution);

	cout << endl << endl;
	cout << "Tabu Search finished! (" << iter << " iterations, " << elapsedTime << "s.)" << endl;
	double temporaryFitnessValue = full_fitness();
	cout << "Best H = " << maxHeightVoxels << ", O = " << overlapCount << ", F = " << absoluteBestF << "(real = " << temporaryFitnessValue << " )" << endl;
	if (abs(absoluteBestF - temporaryFitnessValue) > TOL)
	{
		cout << endl << "WARNING: Fitness disagreements!!!!" << endl;
		cout << "absoluteBestF  = " << absoluteBestF << "; full_fitness() = ";
		cout << temporaryFitnessValue << "; ne.cfit = " << ne.cfit << "; ne.tfit = " << ne.tfit << endl;
		cout << endl << endl;
	}
	fix_overlaps_from_pairs();
}

void PACKING_LAYOUT::hill_climb_n_class()
{
	clock_t packingClock = clock();

	methodString = "Hill climbing (class)";
	NEIGHBOURHOOD ne(30, this, -1);
	
	bool silent = true;
	// Header:
	if (!silent)
		cout << "IT,    OV,    H,     F" << endl;


	int localOptIters = 0;
	int maxLocalOptIters = (int) ceil(0.2*options->maxIters); // After 100 iterations not improving, we are in a local optimum!
	int iter = 0;
	while (iter < options->maxIters)
	{
		iter++;

		// Select a random piece:
		int ri = (int) double(rand()) % instance->nPieces;
		int k = 0;
		while (!overlaps[ri])
		{
			k++;
			if (k > 5 * instance->nPieces)
			{
				cout << "WARNING! Unable to find an overlapping piece! picking any..." << endl;
				break;
			}
			ri = (int) double(rand()) % instance->nPieces;
		}
		// Select:
		ne.change_piece_to(ri);


		// Explore its neighbourhood:
		int bestN = 0;
		double bestF = 0;
		while (ne.next_neighbour(true))
		{
			if (ne.tfit >= bestF)
			{
				bestF = ne.tfit;
				bestN = ne.cneig;
			}
			// cout << "n = " << ne.cneig << ", f = " << ne.tfit << ", BN  = " << bestN << ", BF = " << bestF << endl;
		}
		// cout << "Best N was " << bestN << " with F = " << bestF << ", moving..." << endl;

		// Finished exploring the neighbourhood, move to the one we found was the best:
		if (ne.cfit > 1- TOL && ne.cfit < 1 + TOL)
		{
			cout << "Fitness is " << ne.cfit << "! Exiting..." << endl;
			return;
		}

		if (bestF < ne.cfit) // Nothing improved...
		{
			localOptIters++;
			if (localOptIters > maxLocalOptIters)
			{
				cout << "Converged! (Spend last " << localOptIters << " with F = " << ne.cfit << ")" << endl;
				break;
			}
			// else
			// {
				// cout << "REJ" << iter << ", " << overlapCount << ", " << highest_container_point() << ", " << bestF << "(worse than " << ne.cfit << ")" << endl;
			// }
		}
		else
		{
			localOptIters = 0;
			ne.cneig = bestN - 1;
			// if (bestN == 4)
			// {
				// cout << "BestN was " << bestN << endl;
				// cout << "Moving to.. " << ne.cneig << endl;
			// }
			ne.next_neighbour(false); // This is a "real" move!
			if (is_feasible_voxel())
			{
				cout << "Found a feasible solution!" << endl;
				break;
			}
			else
			{
				// Display iteration, fitness, etc...
				if (!silent)
					cout << iter << ", " << overlapCount << ", " << highest_container_point() << ", " << ne.cfit << endl;
			}

		}


	}

	clock_t packingClockEnd = clock();
	elapsedTime = float(packingClockEnd - packingClock) / CLOCKS_PER_SEC;

	cout << endl << endl;
	cout << "Hill Climbing finished! (" << iter << " iterations, " << elapsedTime << "s.)" << endl;
	cout << "Best H = " << maxHeightVoxels << ", O = " << overlapCount << ", F = " << ne.cfit << endl;

}
void PACKING_LAYOUT::ils()
{
	if (options->randomSeed < 0)
	{
		cout << "Random seed set to time(NULL), as options->randomSeed was " << options->randomSeed << endl;
		srand(time(NULL)); 
	}
	else
	{
		cout << "Random seed set to options->randomSeed = " << options->randomSeed << endl;
		srand(options->randomSeed); 
	}

	// Is this needed?
	// std::default_random_engine randomNumGenerator(std::random_device{}());
	// srand(time(NULL)); // This is at the beggining of each method!
	
	// bool switchToCompaction = false;
	// PACKING_LAYOUT bestSolution = (*this);
	int maxKicks = options->maxKicks;
	int bestH = highest_container_point();
	bool multistart = false;
	int number_of_changes = options->int1;
	if (number_of_changes < 0)
	{
		multistart = true;
	}
	cout << "Starting ILS for " << maxKicks << " kicks or " << options->maxTime << " seconds." << endl;
	cout << "Initial H = " << bestH << endl;
	clock_t InitialPackingClock = clock();
	clock_t PackingClock = clock();
	double soFarElapsedOld = 0;
	double soFarElapsed = 0;
	string hc_solname;

	// Real ILS:
	vector<POLYTOPE*> best_piece_vec = instance->pieces;
	vector<bool> best_right = placeRight;
	vector<bool> best_front = placeFront;

	for (int i = 0; i < maxKicks; i++)
	{
		cout << endl << " -- HC running for kick: " << i << " -- (best H so far: " << bestH << ")" << endl;

		// cout << "\t> Piece order:\t\n";
		// for (int i = 0; i < instance->nPieces; i++)
		// {
		// 	cout << instance->pieces[i]->index << "\t";
		// }
		// cout << "\n\tPR:\n";
		// for (int i = 0; i < instance->nPieces; i++)
		// {
		// 	cout << placeRight[i] << "\t";
		// }
		// cout << "\n\tPF:\n";
		// for (int i = 0; i < instance->nPieces; i++)
		// {
		// 	cout << placeFront[i] << "\t";
		// }
		// cout << endl << endl;


		// (Random orders and placements are done at the end, to start with the BLB solution) 

		// Find the local optimum around the current solution:
		hill_climb();
		// cout << "\n\t\tAFTER leaving HC, back in ILS:" << endl;
		// cout << "\n\t\t\tPR:\n\t\t\t";
		// for (int i = 0; i < instance->nPieces; i++)
		// {
		// 	cout << placeRight[i] << "\t";
		// }
		// cout << "\n\t\t\tPF:\n\t\t\t";
		// for (int i = 0; i < instance->nPieces; i++)
		// {
		// 	cout << placeFront[i] << "\t";
		// }
		// cout << endl << endl;

		if (options->doCompaction)
		{
			iterated_compaction(options->maxIters, 1, 2, 0, options->maxTime);
		}

		int foundH = highest_container_point();

		// Mind the time:
		soFarElapsedOld = soFarElapsed;
		PackingClock = clock();
		double elapsedTime = float(PackingClock - InitialPackingClock) / CLOCKS_PER_SEC;
		if (elapsedTime >= options->maxTime)
		{
			cout << "Elapsed time is: " << elapsedTime << " leaving on previous iteration (" << soFarElapsedOld << " s)" << endl;
			cout << "BestH is " << bestH << endl;
			elapsedTime = soFarElapsedOld; // Because last iteration does not count
			break;
		}
		else
			soFarElapsedOld = elapsedTime;

		cout << "Elapsed time is: " << elapsedTime << " (max allowed: " << options->maxTime << ")" << endl;

		// Check if this is the best solution:
		if (foundH < bestH + TOL)
		{

			cout << endl << endl << " ### ILS new best, H = " << foundH << " (better than " << bestH << ") ###" << endl << endl << endl;
			cout << "Real H for this is: " << maxHeight << endl;
			bestH = foundH;
			std::stringstream sstm;
			sstm << options->file << "_" << "HC_K_" << i << "H_" << foundH << ".psol";
			hc_solname = sstm.str();
			serialise(hc_solname);

			// Save this progress:
			best_piece_vec = instance->pieces;
			best_right = placeRight;
			best_front = placeFront;

			if (foundH <= instance->voxelLowerBound)
			{
				cout << "Found the optimal solution!!!" << endl;
				std::stringstream sstm_opt;
				sstm_opt << "OptimalSolution_" << "HC_K_" << i << "H_" << foundH << ".psol";
				hc_solname = sstm_opt.str();
				serialise(hc_solname);
				break;
			}
			// bestSolution = (*this);
			// cout << "Double check, best height is: " << bestSolution.highest_container_point() << endl;
		}
		else
		{
			cout << endl << endl << " ILS did not improve, H = " << foundH << " (worse than " << bestH << ")" << endl;
			cout << "real H for this is: " << maxHeight << endl;	
		}

		// Generate a random initial solution for next iteration
		reset();
		if (multistart)
		{
			// ENSURE RANDOMNESS!
			// std::random_shuffle(instance->pieces.begin(), instance->pieces.end(),randomNumGenerator);
			std::random_shuffle(instance->pieces.begin(), instance->pieces.end());
			random_placement_rules();
		}
		else
		{
			// Go back to best:
			instance->pieces = best_piece_vec;
			placeRight = best_right;
			placeFront = best_front;
			ils_kick(number_of_changes);
		}

		// Give an initial solution:
		strip_voxel_packing_first_fit_nfv();
	}

	// Return to best solution found:
	cout << "ILS finished!" << endl;
	this->deserialise(hc_solname);
	cout << "Best height was: " << highest_container_point() << endl;
	cout << "Real H: " << maxHeight << endl;
	cout << "Time taken was: " << elapsedTime << endl;
}

void PACKING_LAYOUT::hill_climb()
{
		int bestH = highest_container_point();
		int oldH;
		vector<bool> bestPlaceFront = placeFront;
		vector<bool> bestPlaceRight = placeRight;
		vector<bool> bestPlaceFrontInit = placeFront;
		vector<bool> bestPlaceRightInit = placeRight;
		int it = -1;
		bool foundOne;
		reset();
		while (true)
		{
			it++;
			oldH = bestH;
			foundOne = false;

			// See if any neighbour improves our solution (front):
			for (int i = 0; i < instance->nPieces; i++)
			{
				
				// placeFront = bestPlaceFrontInit;
				// placeRight = bestPlaceRightInit;

				placeFront[i] = !placeFront[i];
				strip_voxel_packing_first_fit_nfv();
				int foundH = highest_container_point();
				if (foundH < bestH)
				{
					cout << "\t\t> HC found that switching the value of FRONT for piece in pos " << i << " went from ";
					cout << bestH << " to " << foundH << endl;

					bestH = foundH;
					bestPlaceFront = placeFront;
					bestPlaceRight = placeRight;

					// If closest
					foundOne = true;
					break;
				}
				else
					placeFront[i] = !placeFront[i];

				reset_after_index(i);
				// reset();
			}

			if (foundOne)
				continue;

			// Check the other neighbours:
			for (int i = 0; i < instance->nPieces; i++)
			{
				if (foundOne)
					break;
				// reset();
				// placeFront = bestPlaceFrontInit;
				// placeRight = bestPlaceRightInit;

				placeRight[i] = !placeRight[i];
				strip_voxel_packing_first_fit_nfv();
				int foundH = highest_container_point();
				// int foundH = maxHeightVoxels;// highest_container_point();
				if (foundH < bestH)
				{
					cout << "\t\t> HC found that switching the value of RIGHT for piece in pos " << i << " went from ";
					cout << bestH << " to " << foundH << endl;
					bestH = maxHeightVoxels;
					bestPlaceFront = placeFront;
					bestPlaceRight = placeRight;
					foundOne = true;
				}
				else
				{
					placeRight[i] = !placeRight[i];
				}
				
				reset_after_index(i);
				// reset();
			}


			if (!foundOne) // (bestH == oldH)
			{
				cout << "Found a local optimum! (H = " << bestH << ", realH = " << (double) bestH * resolution << ")" << endl;
				break;
			}
			else // This means we have improved, move to the next point and check the neighbours
			{
				cout << "HC: Solution improved from " << oldH << " to " << bestH << " (it " << it << ")" << endl;

				// oldH = bestH;
				bestPlaceFrontInit = bestPlaceFront;
				bestPlaceRightInit = bestPlaceRight;
			}

		}

		// Make sure we are giving back the best solution:
		reset();
		placeRight = bestPlaceRight;
		placeFront = bestPlaceFront;
		strip_voxel_packing_first_fit_nfv();


		cout << "\t\tLeaving HC with:" << endl;
		cout << "\n\t\t\tPR:\n\t\t\t";
		for (int i = 0; i < instance->nPieces; i++)
		{
			cout << placeRight[i] << "\t";
		}
		cout << "\n\t\t\tPF:\n\t\t\t";
		for (int i = 0; i < instance->nPieces; i++)
		{
			cout << placeFront[i] << "\t";
		}
		cout << endl << endl;
}

void PACKING_LAYOUT::SA(int maxIter, double T0)
{
	// Default SA type is 0, which is simple constructive with different placement rules
	SA(maxIter, T0, 0);
}

void PACKING_LAYOUT::SA(int maxIter, double T0, int SAtype)
{
	cout << "Starting SA" << endl;
	cout << "SA Type:\t" << SAtype << endl;
	cout << "Max iters:\t" << maxIter << endl;
	cout << "divideBy:\t" << options->int1 << endl;
	// cout << "divideBy range:\t[" << options->int2 << ", " << options->int3 << "]" << endl;
	cout << "T0:\t\t" << T0 << endl;
	int k = 0;

	double f_zero = -RAND_MAX; // -Inf
	// Start with the proper SA fitness:
	if (SAtype == 25)
	{
		f_zero = full_fitness(k);
		ignorePiece.assign(instance->nPieces, false);
	}
	double f_star = f_zero;
	double f_old = f_zero;
	double f_best = f_zero;
	vector<double> f_values;
	f_values.push_back(f_zero);
	// We will be writting to a file:
	std::stringstream fileLine;
	string fileLineString;
	
	ofstream fi;
	fi.open("sa_run.csv", std::ofstream::trunc);

	double kickProb = 0.3;
	// cout << "Kick prob:\t" << kickProb << endl << endl;

	double t = T0;
	
	// double f_zero = double(highest_container_point()); // Current height.

	// int bestH = ch;
	int starting_H = highest_container_point();



	// We do not want to get any worse than this H:
	// UpperBoundH = starting_H;


	cout << "F_zero:\t\t" << f_zero << endl;
	cout << "Starting H:\t" << starting_H << endl;
	cout << "Upper bound H:\t" << UpperBoundH << endl;
	cout << "Overlap:\t" << overlapCount << endl;

	bool rej = false; // Control the output to the screen:
	double randomNum = 0;
	double criteria = 1;
	string typeStr;
	int nType = 1;

	PACKING_LAYOUT currentLayout = (*this);
	INSTANCE currentInstance = (*this->instance);

	PACKING_LAYOUT bestEverLayout = (*this);
	INSTANCE bestEverInstance = (*this->instance);

	bool serialiseall = false;
	bool dontSerialise = true;
	int k_bsol = 0;
	int k_acc = 0;

	// File header:
	fileLine << "Neigh, It, F, Dir, Best, H, Ov, Temp, Cri," << endl;
	fileLineString = fileLine.str();
	fi << fileLineString.c_str();
	cout << fileLineString;
	fileLineString.clear();
	fileLine.str(std::string() );
	fileLine.clear();

	while (k < maxIter)
	{
		if (f_zero >= 1 - TOL)
		{
			if (f_zero >= 1 + TOL)
			{
				cout << endl << "WARNING: Fitness " << f_zero << " is >= 1!!!" << endl;
			}
			else
			{
				cout << endl;
				cout << "********************************" << endl;
				cout << "***   Perfect fitness!! :D   ***" << endl;
				cout << "********************************" << endl;
				cout << endl;
				ignorePiece.assign(instance->nPieces, false);
				break;
			}
		}
		// Update temperature:
		t = (maxIter - k)*T0 / maxIter;
		k++;

		typeStr = "0, ";
		rej = false;
		bool isBest = false;
		fileLineString.clear();
		fileLine.str(std::string() );
		fileLine.clear();

		// Move to a neighbour:
		// 1 - Save the current instance and packing layout somewhere:
		// Both already saved in CurrentLayout + CurrentInstance

		// 2 - Chose a neighbourhood type:
		if (k > 0)
		{
			select_neighbourhood_type(nType, f_values, SAtype);

			// 3 - Go to a neighbour and get the fitness:
			random_neighbour(nType, f_star, k, f_values);


			// Acceptance criteria:
			criteria = exp((f_star - f_zero) / t);
		}

		randomNum = ((double)rand() / RAND_MAX);
		if (randomNum < criteria)
		{
			k_acc++;
			if (serialiseall)
			{
				k_acc;
				std::stringstream serialstream;
				k_bsol++;
				serialstream << "SAacc" << "_it_" << k_acc << ".psol";
				string sa_acc_solname = serialstream.str();
				serialise(sa_acc_solname);
			}


			if (f_star > f_zero)
			{

				typeStr = "1, ";

				// Check if this solution is as well the best so far:
				if (f_star > f_best)
				{
					// Save the point:
					bestEverLayout = (*this);
					bestEverInstance = (*this->instance);

					f_best = f_star;

					// Serialise and update the "isBest" parameter for the progress file:
					isBest = true;
					if (!dontSerialise)
					{
						std::stringstream sstm;
						k_bsol++;
						sstm << "SAbest" << "_it_" << k_bsol << ".psol";
						string sa_solname = sstm.str();
						serialise(sa_solname);
					}
				} 
			}
			else if (f_star < f_zero) // We might be accepting a worse solution
			{
				typeStr = "-1, ";
			}

			// Current fitness is the new fitness:
			f_zero = f_star;

			// Move to neighbour:
			// currentLayout = (*this);
			currentLayout.pieceIndices = pieceIndices;
			currentLayout.ignorePiece.assign(instance->nPieces, false);
			currentInstance = (*this->instance);

			// Write on file:
			fileLine << nType << ", " << k << ", " << f_star << ", " << typeStr.c_str();
			if (isBest)
				fileLine << "1, ";
			else
				fileLine << "0, ";
			fileLine << maxHeightVoxels << ", " << overlapCount << ", ";
		}
		else
		{

			// (*this) = currentLayout;
			pieceIndices = currentLayout.pieceIndices;
			ignorePiece.assign(instance->nPieces, false);
			(*this->instance) = currentInstance ;

			rej = true;
		}


		if (!rej) // If the solution was not rejected, add more info to the file:
		{
			rej = false;
			fileLine << t << ", " << min(double(1), criteria) << endl;
			fileLineString = fileLine.str();

			// Send to file:
			fi << fileLineString.c_str();

			// If is best, output on screen as well:
			if (isBest)
				cout << fileLineString.c_str();

		}
		// else
		// {
			// Debug, show everything...
			// cout << fileLineString.c_str();
		// }

	}

	// Finish:
	(*this) = bestEverLayout;
	(*this->instance) = bestEverInstance;
	allNFV->fix_addresses();
	cout << "SA finished. Best H = " << highest_container_point() << " (started from " << starting_H << ")" << endl;
	cout << "Best fitness = " << f_best << " (started from " << f_old << ")" << endl;
	cout << "Parameters were: " << endl;
	cout << "Max iters:\t" << maxIter << endl;
	cout << "T0:\t\t" << T0 << endl;
	cout << "Kick prob:\t" << kickProb << endl;

	// Clear stuff:
	fileLineString.end();
	fi.close();
}

void PACKING_LAYOUT::select_neighbourhood_type(int &nType, vector<double> &f_values, int &SAType)
{
	double	randomNum = ((double)rand() / RAND_MAX);
	double kickProb = 0.3;
	switch (SAType)
	{
	case 1:
		if (randomNum > kickProb)
			nType = 2;
		else
			nType = 3;
		break;
	case 25: // SA is a local search, allowing overlap and with an overlap-based measure as fitness
		nType = 25;
	default:
		break;
	}

}

int PACKING_LAYOUT::penetration_depth(int p, int q)
{
	// WARNING: THIS IS NOT ACCURATE IF ONE BBOX IS FULLY CONTAINED IN ANOTHER ONE
	bool debugpd = false;

	// We are assuming they do overlap...
	int pdepth = instance->container->gridSize[cd1] * instance->container->gridSize[cd2]; // Something big
	int minBox[3];
	int maxBox[3];
	int sideSize[3];

	// First, check the penetration depth with the bounding boxes:
	for (int coord = 0; coord < 3; coord++)
	{

	// bool noBoxOverlap = false;
		minBox[coord] = max(pieceIndices[p][coord], pieceIndices[q][coord]);
		maxBox[coord] = min(pieceIndices[p][coord] + instance->pieces[p]->voxel.get_grid_size(coord),
			pieceIndices[q][coord] + instance->pieces[q]->voxel.get_grid_size(coord));

		sideSize[coord] = maxBox[coord] - minBox[coord];

		if (sideSize[coord] <= 0)
			cout << "WARNING! Checking penetration depth of non-overlapping objects..." << endl;
			// noBoxOverlap = true;

		// If that is small, set it as new penetration depth 
		if (sideSize[coord] < pdepth)
			pdepth = sideSize[coord];
	}

	if (debugpd)
	{
		cout << "Checking penetration depth of objects p and q." << endl;
		cout << "Pieces p = " << p << " at: ";
		cout_point3(pieceIndices[p]);
		cout << "P dimensions: " << instance->pieces[p]->voxel.get_grid_size(0) << " x " << instance->pieces[p]->voxel.get_grid_size(1) << " x " << instance->pieces[p]->voxel.get_grid_size(2) << endl;
		cout << endl << " q = " << q;
		cout_point3(pieceIndices[q]);
		cout << "Q dimensions: " << instance->pieces[q]->voxel.get_grid_size(0) << " x " << instance->pieces[q]->voxel.get_grid_size(1) << " x " << instance->pieces[q]->voxel.get_grid_size(2) << endl;
		cout  << endl;
		cout << "After bounding box, penetration depth is: " << pdepth << endl;
	}
	
	// If it is 1 or 0, NFV is not going to do any better than that, exit
	if (pdepth < 2)
		return pdepth;

	// Check the NFV, this might take a while...
	NFV * nfv;
	int nfv1 = min(instance->pieces[p]->index, instance->pieces[q]->index);
	int nfv2 = max(instance->pieces[p]->index, instance->pieces[q]->index);
	nfv = allNFV->getNFV(nfv1, nfv2);
	bool pSmaller = (nfv1 == instance->pieces[p]->index);

	// Relative point, which we know is not in the NFV
	vector<int> a(3);
	for (int coord = 0; coord < 3; coord++)
	{
		if (pSmaller)
			a[coord] = pieceIndices[p][coord] - pieceIndices[q][coord];
		else
			a[coord] = pieceIndices[q][coord] - pieceIndices[p][coord];
	}

	if (debugpd)
	{
		cout << "Point to check distance in the NFV to is: ";
		cout_point3(a);
	}


	int tentative = 0;

	for (int j = 0; j < nfv->ypoints.size(); j++)
	{
		tentative = abs(nfv->ypoints[j].value - a[1]);
		if (tentative >= pdepth)
			break;
		for (int i = 0; i < nfv->ypoints[j].xpoints.size(); i++)
		{
			tentative = abs(nfv->ypoints[j].value - a[1]) + 
				abs(nfv->ypoints[j].xpoints[i].value - a[0]);

			if (tentative >= pdepth)
				break;

			for (int k = 0; k < nfv->ypoints[j].xpoints[i].zpoints.size(); k++)
			{
				tentative = abs(nfv->ypoints[j].value - a[1]) + 
					abs(nfv->ypoints[j].xpoints[i].value - a[0]) + 
					abs(nfv->ypoints[j].xpoints[i].zpoints[k] - a[2]);
				if (tentative >= pdepth)
					break;

				if (tentative < pdepth) // We found sth better
				{
					// cout << "Pdepth improved! now is " << tentative << " (was " << pdepth << ")" << endl;
					pdepth = tentative;
					if (debugpd)
					{
						cout << "Improved pdepth, now is: " << pdepth << endl;
						cout << "Came from the point: (" << nfv->ypoints[j].value << ", " << nfv->ypoints[j].xpoints[i].value << ", " << nfv->ypoints[j].xpoints[i].zpoints[k] << ")" << endl;
					}

				}
			}

		}
	}

	return pdepth;
}

double PACKING_LAYOUT::full_fitness()
{
	int dummy = -1;
	return full_fitness(dummy, true);
	
}
double PACKING_LAYOUT::full_fitness(bool updateThings)
{
	int dummy = -1;
	return full_fitness(dummy, updateThings);
}

double PACKING_LAYOUT::full_fitness(int updateOnlyP, bool updateThings)
{
	if (options->fitnessString.compare("closestpoint") == 0)
	{
		double fitness =  full_fitness_closest_point(updateOnlyP, updateThings);
		if (updateThings)
		{
			// debug : speed // This needs to be done directly with updateThings!!!!
			full_fitness_overlap();
			fix_overlaps_from_pairs();
		}
		return fitness;
	}
	else
	{
		// Classic overlap fitness
		if (updateThings)
		{
			fitness = full_fitness_overlap();
			return fitness;
		}
		else
		{
			// cout << endl << endl << "ERROR: Bounding box fitness for only one piece is not implemented! -- full_fitness()" << endl << endl;
			// return full_fitness_overlap();
			fitness = full_fitness_overlap();
			return fitness;
		}
		// else
			// return get_fitness(updateThings);
	}
}
double PACKING_LAYOUT::full_fitness_closest_point()
{
	return full_fitness_closest_point(-1);
}
double PACKING_LAYOUT::full_fitness_closest_point(int updateOnlyP)
{
	return(full_fitness_closest_point(updateOnlyP, true));
}
double PACKING_LAYOUT::full_fitness_closest_point(int updateOnlyP, bool updateThings)
{
	// cout << "------------------------------------------------" << endl;
	// cout << "pieceToUpdate=" << updateOnlyP << endl;
	// cout << "------------------------------------------------" << endl;
	// return 0;
	// Loop through all the pairs of pieces:
	double fitness = 0.0;
	bool sanityCheck = true;
	bool allowQuickVersion = true;

	bool onlyOnePiece = true;
	if (updateOnlyP < 0)
		onlyOnePiece = false;

	 if (updateThings && !updateOnlyP)
		 relPenetrationDepth.assign(instance->nPairs, 0);
	// else

	// double overlapWeight = -10;
	// double nonOverlapWeight = 0.1;

	// debug : DISABLED OVERLAPS AND STUFF!
	bool updateOverlaps = false;
	vector<bool> oldOverlaps;
	if (updateOverlaps)
	{
		oldOverlaps = overlaps;
		overlapCount = 0;
		overlaps.assign(instance->nPieces, false);
	}
	// int npairs = instance->nPairs; 

	// Dividing term (max displacement any piece might need:
	// debug : speed -> if this is the fitness we are going to use, this can be precalculated and stored in instance.
	int maxDiv = 0;
	for (int p = 0; p < instance->nPiecesOr; p++)
	{
		int thisPieceMin = instance->originalPieces[p]->voxel.get_grid_size(0);
		for (int sideofp = 1; sideofp < 3; sideofp++)
			thisPieceMin = min(thisPieceMin, instance->originalPieces[p]->voxel.get_grid_size(sideofp));
		maxDiv = max(maxDiv, thisPieceMin);
	}

	// Loop through all pairs:
	bool ignoredPiece = false;
	double oldValue = 0;
	int pair = -1;
	double pairContr = 0;
	double * ovValue;
	for (int p = 0; p < instance->nPieces - 1; p++)
	{
		for (int q = p + 1; q < instance->nPieces; q++)
		{
			pair++;
			if (updateThings)
				ovValue = &relPenetrationDepth[pair];
			else
			{
				pairContr = 0;
				ovValue = &pairContr;
			}

			if (onlyOnePiece)
			{
				if (((q != updateOnlyP) && (p != updateOnlyP)))
				{
					ignoredPiece = true;
					if (allowQuickVersion)
					{
						// Use saved value:
						fitness += pow(relPenetrationDepth[pair],2);
						continue;
					}
					else
						oldValue = relPenetrationDepth[pair];
				}
				else
					ignoredPiece = false;

			}
			// else

			// If we haven't moved these two and they did not overlap before, ignore the rest:
			// if (ignorePiece[p] && ignorePiece[q] && !oldOverlaps[p] && !oldOverlaps[q])
				// continue;

			// If the bounding boxes do not overlap, they contribute 0 to the fitness:
			// Next best scenario, pieces do not:
			double dummydouble = 0;
			// debug : speed // Probably box not necessary (unless arranging overlaps)
			bool box_overlaps = box_overlap_rel(p, q, dummydouble);

			//if (!box_overlaps && pieces_overlap)
			//{
			//}
			if (!box_overlaps)
			{
				(*ovValue) = 0;
				continue;
			}
			else
			{
				bool pieces_ok = allNFV->nfv_has_point(instance->pieces[p]->index, pieceIndices[p], instance->pieces[q]->index, pieceIndices[q]);
				if (pieces_ok)
				{
					(*ovValue) = 0;
					continue;
				}
			}

			// If we are here, the pieces do overlap.
			// Find the least quantity we can move p (within the container) so they do not overlap (upper boundary as closest feas. point)
			int bestDisp = maxDiv;
			vector<int> ptest(3);
			for (int coord = 0; coord < 3; coord++)
			{
				int leftDisp = (pieceIndices[p][coord] + instance->pieces[p]->voxel.get_grid_size(coord)) - pieceIndices[q][coord];
				if ((abs(leftDisp) < bestDisp))
				{
					// If it is better and feasible, this is our upper bound on an overlap-resolving movement
					//if ((((pieceIndices[p][coord] - leftDisp) > 0)) ||(pieceIndices[q][coord] + leftDisp) < instance->container->gridSize[coord])
						bestDisp = abs(leftDisp);
					
				}
				int rightDisp = (pieceIndices[q][coord] + instance->pieces[q]->voxel.get_grid_size(coord)) - pieceIndices[p][coord];
				if ((abs(rightDisp) < bestDisp))
				{
					//if ((((pieceIndices[p][coord] + rightDisp) < instance->container->gridSize[coord])) ||(pieceIndices[q][coord] - rightDisp) > 0)
					// If it is better and feasible, this is our upper bound on an overlap-resolving movement
					bestDisp = abs(rightDisp);
					
				}
			}

			// Loop through the cube:
			int cubeRad = bestDisp - 1;
			for (int ci = -cubeRad; ci < cubeRad; ci++)
			{
				for (int cj = -cubeRad; cj < cubeRad; cj++)
				{
					for (int ck = -cubeRad; ck < cubeRad; ck++)
					{
						int pointDist = (int) floor(sqrt(pow(ci, 2) + pow(cj, 2) + pow(ck, 2)));
						if (pointDist >= bestDisp)
							continue; // We are no longer interested in this point

						// Does the NFV have the point?
						vector<int> proposedPoint = pieceIndices[p];
						proposedPoint[0] += ci;
						proposedPoint[1] += cj;
						proposedPoint[2] += ck;

						if (allNFV->nfv_has_point(instance->pieces[p]->index, proposedPoint, instance->pieces[q]->index, pieceIndices[q]))
						{
							bestDisp = pointDist;
							if (bestDisp == 1)
								break;
						}
					}
					if (bestDisp == 1)
						break;
				}
				if (bestDisp == 1)
					break;
			}
			// vector<int> ptest(3);
			// ptest[coord] = pieceIndices[q][coord] - pieceIndices[p][coord];
			// NFV * pqnfv = allNFV->getNFV(instance->pieces[p]->index, instance->pieces[q]->index);
			(*ovValue) = bestDisp;
			//if ((ignoredPiece && sanityCheck) && abs(oldValue - (*ovValue)) > TOL)
			if (updateOnlyP && ignoredPiece  && (abs(oldValue - (*ovValue)) > TOL))
			{
				cout << "ovValue was " << oldValue;
				cout << ", and updated to " << (*ovValue) << endl;;
				cout << "Pair is p=" << p << ", q=" << q << ", pieceToUpdate=" << updateOnlyP << endl;
				// cout << "Pieces p = " << p << " and q = " << q << endl;
				report_piece(p);
				cout << endl;
				report_piece(q);
			}
			// }
			// else
			// {
				// (*ovValue) = 0;
			// }

			if (updateOverlaps && (*ovValue) > TOL) // debug : debug This is disabled at the moment!
			{
				overlaps[p] = true;
				overlaps[q] = true;
				overlapCount++;
			}
			if (sanityCheck)
			{
				if ( (*ovValue) > maxDiv)
					cout << "Warning - found a distance of " << (*ovValue) << " but maximum is " << maxDiv << "(initial estimate was: " << cubeRad + 1 << ")" << endl;
				// else
					// cout << "Pair p="<<p<<", q=" << q <<": Distance=" << (*ovValue) << " (max=" << maxDiv << ", initial est=" << cubeRad + 1 << ")" << endl;


			}
			fitness += pow((*ovValue),2);
		}
	}
	fitness = 1 - (fitness / (pow(maxDiv,2)*instance->nPairs));
	if (updateThings)
	{
		// debug : remove
		// This is only if we cannot fix overlap in this fitness!!!!!!!!
		full_fitness_overlap();
		fix_overlaps_from_pairs();
	}
	return(fitness);
}
double PACKING_LAYOUT::nfv_closest_point(int p, int q)
{
	return 0;
}

double PACKING_LAYOUT::full_fitness_overlap()
{
	int k = 0;
	return full_fitness_overlap(k);
}

double PACKING_LAYOUT::full_fitness_overlap(int &k)
{
	// if (relBoxOverlapPairVoxel.size() < 1)
		// relBoxOverlapPairVoxel.assign(instance->nPairs, 0);

	// if (relPenetrationDepth.size() < 1)
		// relPenetrationDepth.assign(instance->nPairs, 0);

	bool F1 = false;
	bool F2 = false; // U * Overlap measure
	bool F3 = false; // Only overlap!
	bool F4 = false; // alpha*U + beta*Overlap measure, alpha beta depend on k
	bool F5 = false; // Overlap (bounding box overlap, but bbox intersection no overlap of pieces is promoted)
	bool F6 = false; // Penetration depth (relative, divided by ? )

	double fi1 = 0.0;
	double fi2 = 0.0;
	double fi3 = 0.0;
	double fi4 = 0.0;
	double fi5 = 0.0;
	double fi6 = 0.0;

	switch (fitnessType)
	{
	case 1:
		F1 = true;
		break;
	case 2:
		F2 = true;
		break;
	case 3:
		F3 = true;
		break;
	case 4:
		F4 = true;
		break;
	case 5:
		F5 = true;
		break;
	case 6:
		F6 = true;
		break;
	default:
		break;
	}

	// Loop through all the pairs of pieces:
	double fitness = 0.0;

	double overlapWeight = -10;
	double nonOverlapWeight = 0.1;

	overlapCount = 0;
	vector<bool> oldOverlaps = overlaps;
	overlaps.assign(instance->nPieces, false);
	int npairs = instance->nPairs; 

	double largestOverlap = -1 * overlapWeight * instance->container->gridSize[cd1] * instance->container->gridSize[cd2] * highest_container_point();

	int pair = -1;
	for (int p = 0; p < instance->nPieces - 1; p++)
	{
		for (int q = p + 1; q < instance->nPieces; q++)
		{
			pair++;
			// If we haven't moved these two and they did not overlap before, ignore the rest:
			// if (ignorePiece[p] && ignorePiece[q] && !oldOverlaps[p] && !oldOverlaps[q])
				// continue;

			if (box_overlap_rel(p, q, relBoxOverlapPairVoxel[pair]) && relBoxOverlapPairVoxel[pair] > TOL)
			{
				overlapCount++;
				overlaps[p] = true;
				overlaps[q] = true;
				
				fi2 += relBoxOverlapPairVoxel[pair];

				// If this is the fitness, add the penetration depth
				if (F6)
				{
					if (relBoxOverlapPairVoxel[pair] > -TOL)
					{
						int pdpeth_p_q = penetration_depth(p, q);
						relPenetrationDepth[pair] = double(pdpeth_p_q) / double(instance->largestVoxelSide);
					}
					else
						relPenetrationDepth[pair] = 0.0;
					 // = contribution;
					// double contribution = double(relPenetrationDepth[pair]) / double(instance->largestVoxelSide);
					// if (contribution > 1)
					// {
						// cout << "full_fi cont::: " << contribution << endl;
					// }
					fi6 += relPenetrationDepth[pair];

				}
			}
			else
			{
				relPenetrationDepth[pair] = 0.0;
			}

			if (F5) // This adds (substracts) everything
			{
				fi5 += relBoxOverlapPairVoxel[pair];
			}

			//int minBox[3];
			//int maxBox[3];
			//int sideSize[3];
			//bool noOverlap = false;
			//for (int coord = 0; coord < 3; coord++)
			//{
			//	minBox[coord] = max(pieceIndices[p][coord], pieceIndices[q][coord]);
			//	maxBox[coord] = min(pieceIndices[p][coord] + instance->pieces[p]->voxel.get_grid_size(coord),
			//		pieceIndices[q][coord] + instance->pieces[q]->voxel.get_grid_size(coord));

			//	sideSize[coord] = maxBox[coord] - minBox[coord];
			//	if (sideSize[coord] <= 0)
			//		noOverlap = true;
			//}
			//if (noOverlap)
			//	continue;
			//else
			//{
			//	double overlapVolume = sideSize[0] * sideSize[1] * sideSize[2];
			//	if (allNFV.nfv_has_point(instance->pieces[p]->index, pieceIndices[p], instance->pieces[q]->index, pieceIndices[q]))
			//	{
			//		fitness += (overlapVolume * nonOverlapWeight)/largestOverlap;
			//	}
			//	else
			//	{
			//		overlapCount++;
			//		overlaps[p] = true;
			//		overlaps[q] = true;
			//		// fitness += (overlapVolume * overlapWeight)/largestOverlap;
			//		fi2 += overlapVolume / min(instance->pieces[p]->voxel.BoxVoxelVolume,
			//			instance->pieces[q]->voxel.BoxVoxelVolume);
			//	}
			//}
		}
	}


	// fitness = -1*highest_container_point();

	if (F2)
	{
		// cout << "pairs = " << npairs << " fi = " << fi2;
		fi2 = 1 - fi2 / npairs;
		// cout  << " (1 - fi2/pairs) =  " << fi2;
		int container_volume = instance->container->baseAreaVoxel * highest_container_point();
		double ut = double(instance->voxelVolume) / double(container_volume);
		double alpha, beta;
		beta = k / options->maxIters;
		alpha = 1 - beta;
		fitness = (alpha*ut + beta*overlapCount/npairs)*pow(fi2, 2);
		// cout << " ut = " << ut << " fitness2 = " << fitness << " overlaps = " << overlapCount << endl;
		// cout << "C_vol = " << container_volume << " I_vol = " << instance->voxelVolume << endl;
	}
	else if (F3)
	{
		fi3 = 1 - fi2 / npairs;
		fitness = fi3;
	}
	else if (F4)
	{
		fi4 = 1 - fi2 / npairs;
		int container_volume = instance->container->baseAreaVoxel * highest_container_point();
		double ut = double(instance->voxelVolume) / double(container_volume);
		double alpha, beta;
		beta = k / options->maxIters;
		alpha = 1 - beta;
		fitness = alpha*ut + beta*fi2;

	}
	else if (F5)
	{
		fi5 = 1 - fi5 / double(npairs);
		fitness = fi5;
	}
	else if (F6)
	{
		fitness = 1 - fi6 / double(npairs);
	}
	else
	{
		// Scale between 0 and 1: (now between -1 and 1)
		fitness = (fitness + 1) / 2;
		int h_upper_bound = 180;
		fitness = (fitness - highest_container_point() / h_upper_bound)/2;
	}

	return fitness;
}


void PACKING_LAYOUT::random_neighbour(int neighbourType, double &f_star, int &k, vector<double> &f_values)
{
	// Neighbourhood types:
	// 1 - Placement rule change (1 rule only)
	// 2 - Placement rule change (in both rules for one piece)
	// 3 - Swap 2 pieces in the sequence (conserve placement rule)
	// 25 - Randomly move the pieces around one fraction of their size, when they overlap

	// We will reset the solution as lie
	int resetFrom = 0;


	// Since some fitness evaluations are shared and simple, do the with a parameter (to be set in each category):
	int fEvaluation = -1;

	// We always need a random piece to apply things:
	int ri = (int) double(rand()) % instance->nPieces;

	if (neighbourType == 1) // 1 - Placement rule change (1 rule only)
	{
		double randomNum = ((double)rand() / RAND_MAX);
		if (randomNum <= 0.5)
			placeFront[ri] = !placeFront[ri];
		else
			placeRight[ri] = !placeRight[ri];

		resetFrom = ri;
		fEvaluation = 1;

	}
	else if (neighbourType == 2) // 2 - Placement rule change (in both rules for one piece)
	{
		// With equal probability, change right, front or both.

		int randChange = (int) double(rand()) % 3;
		if (randChange == 0)
			placeFront[ri] = !placeFront[ri];
		else if (randChange == 2)
			placeRight[ri] = !placeRight[ri];
		else
		{
			placeRight[ri] = !placeRight[ri];
			placeFront[ri] = !placeFront[ri];
		}

		resetFrom = ri;
		fEvaluation = 1;
	}
	else if (neighbourType == 3) // 3 - Swap 2 pieces in the sequence (conserve placement rule)
	{
		int ri2 = (int) double(rand()) % instance->nPieces;
		instance->swap_items(ri, ri2);
		resetFrom = min(ri, ri2);
		fEvaluation = 1;
	}
	else if (neighbourType == 25) // Move a random number in any direction. The max movement is 1 bounding box side
	{
		// Move the piece ri, but only if it overlaps!
		int pieceSelections = 0;
		ignorePiece.assign(instance->nPieces, true);
		 while (!overlaps[ri])
		 {
			pieceSelections++;
			ri = (int) double(rand()) % instance->nPieces;

			// To ensure we do not stay on this loop forever:
			 if (pieceSelections >= instance->nPieces * 5)
			 {
				cout << "WARNING: Unable to find a piece that is overlapping... moving a random one!" << endl;
				break;
			}
// 
		}
		ignorePiece[ri] = false;

		// int od = instance->container->openDimension;
		// int cd1 = (od + 1) % 3;
		// int cd2 = (od + 2) % 3;

		int divideBy = options->int1;
		int sizeOd = ceil(instance->pieces[ri]->voxel.get_grid_size(od)/divideBy);
		int sizeCd1 = ceil(instance->pieces[ri]->voxel.get_grid_size(cd1)/ divideBy);
		int sizeCd2 =  ceil(instance->pieces[ri]->voxel.get_grid_size(cd2) / divideBy);

		// Find how much to move in each direction:

		int manyMoves = 1;
		for (int i = 0; i < manyMoves; i++)
		{
			// ri = (int) double(rand()) % instance->nPieces;
			int m_od = (int) double(rand()) % sizeOd;
			int m_cd1 = (int) double(rand()) % sizeCd1;
			int m_cd2 = (int) double(rand()) % sizeCd2;

			// With more probability, moves are negative:
			double negProb = 0.5;
			double randomNum = ((double)rand() / RAND_MAX);
			if (randomNum <= negProb)
				m_od = -1 * m_od;

			randomNum = ((double)rand() / RAND_MAX);
			if (randomNum <= negProb)
				m_cd1 = -1 * m_cd1;

			randomNum = ((double)rand() / RAND_MAX);
			if (randomNum <= negProb)
				m_cd2 = -1 * m_cd2;


			// Make sure we do not go out of the container:
			if (m_od < 0)
				m_od = max(-1 * pieceIndices[ri][od], m_od);
			else // Height cannot be worse than the upper boundary:
				m_od = min(
				UpperBoundH - instance->pieces[ri]->voxel.get_grid_size(od) - pieceIndices[ri][od],
				m_od);


			if (m_cd1 < 0)
				m_cd1 = max(-1 * pieceIndices[ri][cd1], m_cd1);
			else
				m_cd1 = min(
				instance->container->gridSize[cd1] - instance->pieces[ri]->voxel.get_grid_size(cd1) - pieceIndices[ri][cd1],
				m_cd1);

			if (m_cd2 < 0)
				m_cd2 = max(-1 * pieceIndices[ri][cd2], m_cd2);
			else
				m_cd2 = min(
				instance->container->gridSize[cd2] - instance->pieces[ri]->voxel.get_grid_size(cd2) - pieceIndices[ri][cd2],
				m_cd2);

			//// Make sure we do not go out of the container from the large dimension:
			//if (m_od < 0)
			//	m_od = min(-1 * pieceIndices[ri][od], m_od);
			//if (m_cd1 < 0)
			//	m_cd1 = min(-1 * pieceIndices[ri][cd1], m_cd1);
			//if (m_cd2 < 0)
			//	m_cd2 = min(-1 * pieceIndices[ri][cd2], m_cd2);

			// Make the movement:
			pieceIndices[ri][od] += m_od;
			pieceIndices[ri][cd1] += m_cd1;
			pieceIndices[ri][cd2] += m_cd2;
		}

		 double newFitness =  full_fitness();

		 if ((f_star < newFitness + TOL) && (f_star > newFitness - TOL)) // Did not change!
		 {
			 // TODO: Make fitness better if H is lower!
		 }
		 else
			 f_star = newFitness;
	}
	if (fEvaluation == 1)
	{
		reset_after_index(resetFrom);
		strip_voxel_packing_first_fit_nfv();
		f_star = highest_container_point();
	}
}

void PACKING_LAYOUT::update_pair_index(int &p, int &pairIdx)
{
	// pairIdx should start at p - 1 before being updated in the loop
	pairIdx = p - 1;
}
void PACKING_LAYOUT::update_pair_index(int &p, int &q, int &pairIdx)
{
	// q is increasing 1 by 1
	if (q < p)
	{
		if (q < 1)
			pairIdx = p - 1;
		else
			pairIdx += instance->nPieces - q - 1;
	}
	else if (p == q )
	{
		if (p > 0)
			pairIdx += instance->nPieces - p - 1;
		else
			pairIdx = -1;
	}
	else
		pairIdx++;
}
void PACKING_LAYOUT::ils_kick(int number_of_changes)
{
	int p1, p2;
	POLYTOPE * p_aux;
	bool bool_aux;
	int MAX_TRIAL = instance->nPieces*10;
	for(int i = 0; i < number_of_changes; i++)
	{
		p1 = rand() % instance->nPieces;
		p2 = rand() % instance->nPieces;
		int trial = 0;
		while (instance->pieces[p1]->index == instance->pieces[p2]->index)
		{
			p2 = rand() % instance->nPieces;
			trial++;
			if (trial > MAX_TRIAL)
				break;
		}
		if (trial > MAX_TRIAL)
		{
			cout << endl << "WARNING: ILS_KICK cannot find two pieces to swap after " << MAX_TRIAL << " trials." << endl;
			continue;
		}
		cout << "\t\tSwitching " << p1 << " and " << p2 << "(types " << instance->pieces[p1]->index << " and " << instance->pieces[p2]->index << ")" << endl;
		bool_aux = placeRight[p1];
		placeRight[p1] = placeRight[p2];
		placeRight[p2] = bool_aux;

		p_aux = instance->pieces[p1];
		instance->pieces[p1] = instance->pieces[p2];
		instance->pieces[p2] = p_aux;

		bool_aux = placeRight[p1];
		placeRight[p1] = placeRight[p2];
		placeRight[p2] = bool_aux;

		bool_aux = placeRight[p1];
		placeFront[p1] = placeFront[p2];
		placeFront[p2] = bool_aux;
		
	}
	
}
void PACKING_LAYOUT::random_placement_rules()
{
	double randomNum = 0;
	// ENSURE RANDOMNESS!
	// srand (time(NULL));
	for (int randSearch = 0; randSearch < instance->nPieces; randSearch++)
	{
			randomNum = ((double)rand() / RAND_MAX);
			if (randomNum < 0.5)
				placeFront[randSearch] = !placeFront[randSearch];

			randomNum = ((double)rand() / RAND_MAX);
			if (randomNum < 0.5)
				placeRight[randSearch] = !placeRight[randSearch];
	}
}

void PACKING_LAYOUT::refine_positions()
{
	// Keep track if this was worth it:
	int piecesMoved = 0;
	int od = instance->container->openDimension; // For clarity, open dimension

	// This has to be decided later with a criteria, now we are looking at these two pieces:
	int p1, p2, p3, p4, p5;
	p2 = 8;
	p1 = 16;
	p3 = 21;
	p4 = 13;
	p5 = 7;

	vector<int> p1WasAt = pieceIndices[p1];
	vector<int> p2WasAt = pieceIndices[2];

	// Remove their lastAt's:
	lastAt[instance->pieces[p1]->index] = 0;
	lastAt[instance->pieces[p2]->index] = 0;
	lastAt[instance->pieces[p3]->index] = 0;

	// 1 - Identify if the pieces have potential to move when one is removed:
	piecePlaced[p1] = false;

	pieceIndices[p2][od] = -30;
	pieceIndices[p3][od] = -30;

	strip_voxel_packing_lowest_fit_nfv();

	int p1Potential = p1WasAt[od] - pieceIndices[p1][od];
	// int p2Potential = p2WasAt[od] - pieceIndices[p2][od];
	cout << "p1Potential is " << p1Potential << endl;
	// cout << "p2Potential is " << p2Potential << endl;

	int bestMove = 0;
	// int bestMaxHeight = highest_container_point();
	// int bestMaxHeight = pieceIndices[p1][od] + instance->pieces[p1]->voxel.get_grid_size(od);
	// bestMaxHeight = max(bestMaxHeight, pieceIndices[p2][od] + instance->pieces[p2]->voxel.get_grid_size(od));

	
	// bestMaxHeight = 10000;
	int bestMaxHeight = pieceIndices[p1][od] + instance->pieces[p1]->voxel.get_grid_size(od);
	bestMaxHeight = max(bestMaxHeight, pieceIndices[p2][od] + instance->pieces[p2]->voxel.get_grid_size(od));
	bestMaxHeight = max(bestMaxHeight, pieceIndices[p3][od] + instance->pieces[p3]->voxel.get_grid_size(od));

	int initialMax = bestMaxHeight;

	// cout << "DEBUGGGGGGG" << endl << endl;
	// p1Potential = 9;

	for (int goingDown = 0; goingDown < p1Potential; goingDown++)
	{
		// Remove p3 from the mix
		pieceIndices[p3][od] = -30;
		pieceIndices[p2][od] = -30;
		// pieceIndices[p4][od] = -30;
		pieceIndices[p5][od] = -30;

		lastAt[instance->pieces[p1]->index] = p1WasAt[od] - goingDown - 1;
		// lastAt[p1] = p1WasAt[od] - goingDown;
		piecePlaced[p1] = false;
		strip_voxel_packing_lowest_fit_nfv();
		cout << "P1 placed at " << pieceIndices[p1][od] << " (last at was: " << lastAt[instance->pieces[p1]->index] << ")" << endl;

		lastAt[instance->pieces[p2]->index] = 0;
		// lastAt[p2] = 0;
		piecePlaced[p2] = false;
		strip_voxel_packing_lowest_fit_nfv();
		cout << "P2 placed at " << pieceIndices[p2][od] << endl;


		// Correct p3
		piecePlaced[p3] = false;
		lastAt[instance->pieces[p1]->index] = 0;
		// lastAt[p3] = 0;
		strip_voxel_packing_lowest_fit_nfv();
		cout << "P3 placed at " << pieceIndices[p3][od] << endl;



		// See if we have improved our max height for the two pieces:
		int currentMaxHeight = pieceIndices[p1][od] + instance->pieces[p1]->voxel.get_grid_size(od);
		currentMaxHeight = max(currentMaxHeight, pieceIndices[p2][od] + instance->pieces[p2]->voxel.get_grid_size(od));
		currentMaxHeight = max(currentMaxHeight, pieceIndices[p3][od] + instance->pieces[p3]->voxel.get_grid_size(od));

		if (currentMaxHeight <= bestMaxHeight)
		{
			cout << currentMaxHeight << " < " << bestMaxHeight << " --- ";
			bestMaxHeight = currentMaxHeight;
			bestMove = goingDown;
		}
		else
		{
			cout << currentMaxHeight << " > " << bestMaxHeight << " --- ";
		}
	}
	cout << "The best was to move p1 " << bestMove << " voxels down." << endl;
	cout << "The max. height of the two is now " << bestMaxHeight << " (it was " << initialMax << " before)" << endl;

	// After we have checked everything, apply the best we found:
	// DEBUG: The coordinates can be saved and this can be done by hand... no need to repeat!
	pieceIndices[p3][od] = -30;
	pieceIndices[p2][od] = -30;
	// lastAt[p2] = p2WasAt[od] - bestMove;
	// piecePlaced[p2] = false;
	lastAt[instance->pieces[p1]->index] = p1WasAt[od] - bestMove;
	piecePlaced[p1] = false;
	strip_voxel_packing_lowest_fit_nfv();

	lastAt[instance->pieces[p2]->index] = -1;
	piecePlaced[p2] = false;
	strip_voxel_packing_lowest_fit_nfv();

	lastAt[instance->pieces[p3]->index] = -1;
	piecePlaced[p3] = false;
	strip_voxel_packing_lowest_fit_nfv();

	// lastAt[instance->pieces[p4]->index] = -1;
	// piecePlaced[p4] = false;
	// strip_voxel_packing_lowest_fit_nfv();

	lastAt[instance->pieces[p5]->index] = -1;
	piecePlaced[p5] = false;
	strip_voxel_packing_lowest_fit_nfv();

	// return;
	cout << "Placed the fifth, do the others" << endl;

	for (int i = 0; i < instance->nPieces; i++)
	{
		piecePlaced[i] = true;
	}
	cout << "Before moving things down, height is " << highest_container_point() << endl;

	// Finally, move all the possible pieces down if there is this possibility:
	bool thereAreMoves = true;
	while (thereAreMoves)
	{
		thereAreMoves = false;
		cout << "Refining..." << endl;
		for (int pc = 1; pc < instance->nPieces; pc++)
		{
			int currentOD = pieceIndices[pc][od];

			// Remove the piece:
			piecePlaced[pc] = false;
			lastAt[instance->pieces[pc]->index] = -1;

			// Fit the piece:
			strip_voxel_packing_lowest_fit_nfv();

			int diffOD = currentOD - pieceIndices[pc][od]; 
			if (diffOD > 0.9)
			{
				piecesMoved++;
				cout << "Piece " << pc << " moved down " << diffOD << " voxels." << endl; 
				thereAreMoves = true;
			}
			// else
			// {
				// cout << "Piece " << pc << " remains at " << pieceIndices[pc][od] << "\t"; 
			// }
		}
	} // End of while thereAreMoves

}


extern bool sort_piece_pointers_increasing(POLYTOPE * i, POLYTOPE * j)
{
	return (i->voxel.volume < j->voxel.volume);
}

extern bool sort_piece_pointers_decreasing(POLYTOPE * i, POLYTOPE * j)
{
	return (i->voxel.volume > j->voxel.volume);
}

extern bool sort_piece_pointers_decreasing_depth(POLYTOPE * i, POLYTOPE * j)
{
	return (i->voxel.depth > j->voxel.depth);
}

void PACKING_LAYOUT::strip_voxel_packing_lowest_fit_nfv()
{
	bool verbose = false;
	// ************************************************************************//
	//						LOWEST FIT - METHOD STEPS						   //
	// ************************************************************************//
	// See first fit, here we only change the order of a loop (first piece, then Y coordinate)

	// ********************************************** // 
	// ********************************************** // 
	// ********************************************** // 

	// Set some initial variables:
	methodString = "Bottom Left Back (NFV, lowest fit)";
	

	/*	MEASURE TIME	*/
	clock_t packingClock;



	// Keep the dimensions at hand, since we don't always minimise Y:
	int dimo = instance->container->openDimension;
	int dim1 = (instance->container->openDimension + 2) % 3;
	int dim2 = (instance->container->openDimension + 1) % 3;

// Step 1 - Generate NFV structure:
	// This is done now outside this algorithm!


	// Report the time:
	packingClock = clock();
	elapsedTime = (float) packingClock / CLOCKS_PER_SEC;

	/*	MEASURE TIME	*/
	clock_t packingClockNoNFV;



// Step 1b - Set the position of the first piece:
	// cout << "WARNING: Position of first piece needs to be set." << endl;
	bool isPartial = false;
	int placedPieces = 0;
	if (!piecePlaced[0])
	{
		piecePlaced[0] = true;
		pieceIndices[0] = { 0, 0, 0 };
	}
	else // This handles a partial solution!
	{
		isPartial = true;
		for (int pind = 1; pind < instance->pieces.size(); pind++)
		{
			if (piecePlaced[pind])
				placedPieces++;
		}

	}

// Step 2 - Start looping through the pieces
int currentContainerHeight = highest_container_point();
int pind = 0;
for (int sliceInd = 0; sliceInd < currentContainerHeight + 2; sliceInd++)
{
	// cout << "Placing at height " << sliceInd << ", remaining pieces: " << instance->nPieces - placedPieces << endl; 
	for (int pind = 1; pind < instance->nPieces; pind++)
	{
		if (piecePlaced[pind])
			continue;
		// If we have a piece of this type at a higher position, skip the type:
		 if (lastAt[instance->pieces[pind]->index] > sliceInd)
			 continue;

		// bool isPiecePlaced = false;
		// --- We are placing piece "pind" --- 

// Step 3 - Find the valid positions of ref. point of piece with respect to container
		int minX = 0;
		int minY = 0;
		int minZ = 0;

		// instance->container->gridSize[instance->container->openDimension] = 1000000;
		int maxDim1 = instance->container->gridSize[dim1] - instance->pieces[pind]->voxel.get_grid_size(dim1);
		int maxDim2 = instance->container->gridSize[dim2] - instance->pieces[pind]->voxel.get_grid_size(dim2);
		// int maxZ = instance->container->gridSize[2] - instance->pieces[1]->voxel.depth;
		// instance->container->gridSize[instance->container->openDimension] = -1;
		// cout << minX << " " << minY << " " << minZ << " max " << maxX << " " << maxY << " " << maxZ << endl;

		// Step 4 - Loop through container slices, going in openDimension direction
		vector<int> checkPoint = { minX, minY, minZ };
		// cout << "New container height: " << currentContainerHeight << endl;
// Step 4b - Loops through the valid points from Step 3
			for (int d1idx = 0; d1idx < maxDim1; d1idx++)
			{
				for (int d2idx = 0; d2idx < maxDim2; d2idx++) // NECESSARY???????????, WE CAN CHECK FULL SLICES ON THE NFV
				{
					checkPoint[dim1] = d1idx; 
					checkPoint[dim2] = d2idx; 
					checkPoint[dimo] = sliceInd; 
					bool pointSuitable = false;

// Step 5 - Check if point is in any bounding box (loop)
					for (int placedPiecesIdx = 0; placedPiecesIdx < instance->nPieces; placedPiecesIdx++)
					{
						
						if (!piecePlaced[placedPiecesIdx])
							continue;

						if (bboxes_intersect(pind, placedPiecesIdx, checkPoint))
						{
							// cout << "Bboxes intersect for " << pind << " and " << placedPiecesIdx << endl;
	// Step 5b - If yes, see if point is in NFV, otherwise loop to Step 4
							if (!allNFV->nfv_has_point(instance->pieces[placedPiecesIdx]->index, pieceIndices[placedPiecesIdx], instance->pieces[pind]->index, checkPoint))
							{
								// cout << "NFV(" << pind << ", " << placedPiecesIdx << ") DOES NOT have the point ";
								// cout_point3(checkPoint);
								// cout << endl;
								pointSuitable = false;
								break;
							}
							else
							{
								// cout << "NFV(" << pind << ", " << placedPiecesIdx << ") does have the point ";
								// cout_point3(checkPoint);
								// cout << endl;
								pointSuitable = true;
							}

						}
						else
						{
	// Step 5c - If not, loop to Step 5 and if the point is in no bounding box, place it and loop to Step 2

							// cout << "Bboxes DO NOT intersect for " << pind << " and " << placedPiecesIdx << endl;
						// TODO
							pointSuitable = true;
						}
					} // end loop Step 5

					// We have been through all the pieces with this point, see if we can use it:
					if (pointSuitable)
					{
						// cout << "pieceIndices.size() " << pieceIndices.size() << endl;
						// cout << "pind = " << pind << endl;
						// cout << print_point3(pieceIndices[0]) << endl;
						// cout << print_point3(pieceIndices[1]) << endl;
						pieceIndices[pind] = checkPoint;
						// cout << "Piece " << pind << " (ID = " << instance->pieces[pind]->index << ") placed at: " << "(" << checkPoint[0] << ", " << checkPoint[1] << ", " << checkPoint[2] << ")" << endl;
						// cout << "Piece " << pz << " (ID = " << instance->pieces[pz]->index << ") placed at: " << print_point3(currentFeasible) << endl;
						// cout << "Piece " << pind << " placed at: " << print_point3(checkPoint) << endl;
						piecePlaced[pind] = true;

						placedPieces++;
						if (!isPartial)
							placementOrder[placedPieces] = pind;

						// Update the minimum container height for this piece:
						lastAt[instance->pieces[pind]->index] = checkPoint[instance->container->openDimension];
						break;
					}
					else
						piecePlaced[pind] = false;

				} // end loop dim2
				if (piecePlaced[pind])
					break;
			} // end loop dim1
				// if (piecePlaced[pind])
					// break;



		} // end loop through pieces!



		//if (!piecePlaced[pind])
		//{
		//	cout << endl << endl <<"ERROR: We have reached the end without placing the piece!" << endl;
		//	cout << "\tPiece number: " << pind << endl; 
		//	cout << "\tCurrent container height: " << currentContainerHeight << endl; 
		//	cout << "\tLast checked point: ";
		//	cout_point3(checkPoint);
		//	cout << endl << endl;
		//	exit(-1);
		//}
				

	if (placedPieces >= instance->nPieces)
		break;

	currentContainerHeight = highest_container_point();
	} // end loop slices (open dimension)
	


	packingClockNoNFV = clock();
	float elapsedTimeNONFV = (float) packingClockNoNFV / CLOCKS_PER_SEC;
	// if (verbose)
		// printf("Elapsed time (No NFV): %.2f s.\n\n", elapsedTimeNONFV);

	if (verbose)
		printf("Elapsed time: %.2f s. (%.2f s. NFV generation + %.2f s. Packing) \n", elapsedTimeNONFV + elapsedTime, elapsedTime, elapsedTimeNONFV );

	elapsedTime = elapsedTimeNONFV + elapsedTime;

	if (verbose)
		cout << "Packing finished! Height: " << highest_container_point() << endl << endl;
}

void read_input(const char *filename, vector<string> &target_files, vector<int> &quantities, vector<double> &containerSizes)
{
	// Open the file:
	//char name_of_file[80];
	//strcpy(name_of_file, filename);

	ifstream in(filename);
	if (!in.is_open())
	{
		cout << "There was a problem opening the file: " << filename << endl;
		cout << "Exiting..." << endl;
		exit(-12432);
	}
	else
		cout << "Started reading from: " << filename << endl;

	// Define the variables for reading:
	string currentWord;
	currentWord.reserve(500);

	int currentQuant;

	in >> currentWord;

	if (currentWord.compare("CONTAINER") == 0)
	{
		double currentNum = 0.0;
		for (int i = 0; i < 3; i++)
		{
			in >> currentNum;
			containerSizes.push_back(currentNum);
		}
	}
	else
	{
		cout << "WARNING: There is not container information on this instance (\"" << filename << "\")" << endl;
		target_files.push_back(currentWord);
	}


	while (in >> currentWord)
	{
		// First is the name of the file with the 3d info:
		target_files.push_back(currentWord);

		// Then is the quantity of objects of this type:
		in >> currentQuant;
		quantities.push_back(currentQuant);
	}


}


INSTANCE::INSTANCE()
{

}

INSTANCE::INSTANCE(vector<string> target_files, vector<int> quantities, PACKING_OPTIONS &packingOptions)
{
	CONTAINER * container_par = NULL;
	INSTANCE(target_files, quantities, packingOptions, container_par);
}

INSTANCE::INSTANCE(vector<string> target_files, vector<int> quantities, PACKING_OPTIONS &packingOptions, CONTAINER * container_par)
{
	container = container_par;
	// Class member now!
	// vector<POLYTOPE> allElements; // vector with all the element types specified in the input file.
	int nFiles = target_files.size();
	allElements.resize(nFiles);
	for (int ii = 0; ii < nFiles; ii++)
	{

		 // Decide what to read, binvox at the moment!

		// allElements.back().readPLY(target_files[ii]);
		// allElements.back().read3DS(target_files[ii]);
		allElements[ii].voxel.resolution = packingOptions.resolution;
		allElements[ii].voxel.readBinvox(target_files[ii]);
		allElements[ii].index = ii;

		/*for (int icol = 0; icol < 3; icol++)
		allElements.back().colour.push_back(SCENE::colorScheme[ii % 5][icol]);*/


		//allElements.back().voxelize(resolution);
		// allElements.back().voxelizationExt(packingOptions.resolution);

	}

	volume = 0.0;
	voxelVolume = 0;
	largestVoxelBox = 0;
	for (int ii = 0; ii < target_files.size(); ii++)
	{
		allElements[ii].update_from_voxel();
		pieces.push_back(&allElements[ii]);

		// Save the original instance:
		originalPieces.push_back(&allElements[ii]);
		voxelVolume += allElements[ii].voxel.volume * quantities[ii];
		volume += allElements[ii].voxel.volumeDouble * quantities[ii];
		demands.push_back(quantities[ii]);
		for (int jj = 1; jj < quantities[ii]; jj++)
			pieces.push_back(&allElements[ii]);
			// volume += allElements[ii].voxel.volumeDouble;
			// voxelVolume += allElements[ii].voxel.volume;
			// allElements.push_back(allElements.back().duplicate());
		if (allElements[ii].voxel.BoxVoxelVolume > largestVoxelBox)
			largestVoxelBox = allElements[ii].voxel.BoxVoxelVolume;
	}

	nPieces = pieces.size();
	nPairs = nPieces * (nPieces - 1) / 2;
	nPiecesOr = originalPieces.size();

	// Determine the max. placement position of each piece [pOR, (x, y, z)] (minimum is 0)
	vector<int> maxPosP(3);
	maxPositions.assign(0, maxPosP);
	cout << "Maximum positions of piece types:" << endl;
	largestVoxelSide = -100;
	for (int por = 0; por < nPiecesOr; por++)
	{
		for (int coord = 0; coord < 3; coord++)
		{
			maxPosP[coord] = container->gridSize[coord] - originalPieces[por]->voxel.get_grid_size(coord);

			// Check also the largest side of box:
			largestVoxelSide = max(largestVoxelSide, originalPieces[por]->voxel.get_grid_size(coord));
		}
		maxPosP[container->openDimension] = -1;
		maxPositions.push_back(maxPosP);
		cout_point3(maxPosP);

	}

	cout << "The largest side of a bounding box is: " << largestVoxelSide;
	cout << "(Container is: " << container->gridSize[0] << " x " << container->gridSize[1] << " x " << container->gridSize[2] <<  ")" << endl;

}

void PACKING_LAYOUT::report_piece(int p)
{
	cout << "Piece: " << p << endl;
	cout << "Position: ";
	cout_point3(pieceIndices[p]);
	cout << "Dimensions: " << endl;
	for (int coord = 0; coord < 3; coord++)
		cout << "size[" << coord << "] = " << instance->pieces[p]->voxel.get_grid_size(coord) << "  ";
	cout << endl;
}

double INSTANCE::voxel_volume_double()
{
	return double(voxel_volume_int()) * pow(originalPieces[0]->voxel.resolution, 3);
}

int INSTANCE::voxel_volume_int()
{
	int vol = 0;
	for (int i = 0; i < pieces.size(); i++)
	{
		vol += pieces[i]->voxel.volume;
	}
	return vol;
}

void INSTANCE::sort_pieces_by(vector<int> sortingOrder)
{
	// Remove what is in the pieces vector:
	// POLYTOPE * null;
	pieces.assign(0, NULL); 

	for (int i = 0; i < sortingOrder.size(); i++)
	{
		pieces.push_back(originalPieces[sortingOrder[i]]);
	}
}

void INSTANCE::swap_items(int i, int j)
{
	int type_i = pieces[i]->index;
	pieces[i] = pieces[j];
	pieces[j] = originalPieces[type_i];
}


double NEIGHBOURHOOD::get_fitness(bool update)
{
	// Updates the fitness ONLY for a change of cp, if other piece is moved, this will not work!

	// If update is false, nothing changes, just return possible new fitness
	// If update is true, change cfit and the pair contributions as necessary.

	double newFitness;
	newFitness = cfit; // Fitness

	// Check the overlaps of the pieces with smaller / larger index than cp
	int pairIdx = 0;
	ly->update_pair_index(cp, pairIdx);
	for (int ii = 0; ii < ly->instance->nPieces; ii++)
	{
		ly->update_pair_index(cp, ii, pairIdx);
		if (cp == ii)
			continue;

		double pairContribution = 0;

		bool overlapOfBoxes = ly->box_overlap_rel(cp, ii, pairContribution);

		// If pieces do not overlap, contribution is zero
		if (!overlapOfBoxes)
			pairContribution = 0;

		// For most fitness, if pieces  have negative contribution, set that to zero
		if ((ly->fitnessType != 5) && (pairContribution < -TOL))
			pairContribution = 0;

		if (ly->fitnessType == 6)
		{
			double pairPDContribution = 0;
			if (!overlapOfBoxes || pairContribution < -TOL)
				pairPDContribution = 0;
			else
			{
				pairPDContribution = double(ly->penetration_depth(cp, ii)) / double(ly->instance->largestVoxelSide);
				if (pairPDContribution > 1 || pairPDContribution < 0)
					cout << "get_fi: " << pairPDContribution << endl;
			}

			double oldFit = newFitness;
			newFitness = newFitness + (- pairPDContribution + ly->relPenetrationDepth[pairIdx])/double(ly->instance->nPairs);
			if (newFitness > 1 + TOL)
			{
				double hardcfit = 0;
				for (int itemp = 0; itemp < ly->relPenetrationDepth.size(); itemp++)
				{
					hardcfit += ly->relPenetrationDepth[itemp];
				}
				cout << endl << endl << "WARNING: This movement would make the fitness over 1!!!!" << endl;
				cout << "Fitness was: " << oldFit << " and now is " << newFitness << endl;
				cout << "Brute force calculation is: " << 1 - (hardcfit / double(ly->instance->nPairs)) << endl;
				cout << "Movement of piece " << cp << " has been " << cneig << endl;
				cout << "Pieces p = " << cp << " at: ";
				cout_point3(ly->pieceIndices[cp]);
				cout << "P dimensions: " << ly->instance->pieces[cp]->voxel.get_grid_size(0) << " x " << ly->instance->pieces[cp]->voxel.get_grid_size(1) << " x " << ly->instance->pieces[cp]->voxel.get_grid_size(2) << endl;
				cout << endl << " q = " << ii;
				cout_point3(ly->pieceIndices[ii]);
				cout << "Q dimensions: " << ly->instance->pieces[ii]->voxel.get_grid_size(0) << " x " << ly->instance->pieces[ii]->voxel.get_grid_size(1) << " x " << ly->instance->pieces[ii]->voxel.get_grid_size(2) << endl;
				cout  << endl;
				double pdepthtest = double(ly->penetration_depth(cp, ii));
				cout << "Penetration depth is: " << pdepthtest << " (contribution: " << pdepthtest/double(ly->instance->largestVoxelSide) << ")" << endl;
				cout << "PairPDContribution is: " << pairPDContribution << endl;
				cout << "PDepth used to be: " << ly->relPenetrationDepth[pairIdx] << endl;
				int k_temp = 0;
				// ly->full_fitness_overlap(k_temp);
				// cout << "PDepth recalculated should have been: " << ly->relPenetrationDepth[pairIdx] << endl;
				// hardcfit = 0;
				// for (int itemp = 0; itemp < ly->relPenetrationDepth.size(); itemp++)
				// {
					// hardcfit += ly->relPenetrationDepth[itemp];
				// }
				// cout << "Brute force calculation after full_fitness...() is: " << 1 - (hardcfit / double(ly->instance->nPairs)) << endl;
			}

			if (update)
				ly->relPenetrationDepth[pairIdx] = pairPDContribution;
		}
		// if (overlapOfBoxes && (pairContribution > TOL)) // Boxes overlap and pieces as well
		// {
			// Nothing
		// }
		// else
		// {
			// pairContribution = 0;
		// }

		if (ly->fitnessType != 6)
			newFitness = newFitness + (- pairContribution + max(0.0, ly->relBoxOverlapPairVoxel[pairIdx]))/ly->instance->nPairs;

		// Save the pair contribution:
		if (update)
			ly->relBoxOverlapPairVoxel[pairIdx] = pairContribution;
	}

	if (update)
	{
		// Save this fitness to cfit and fix overlap:
		cfit = newFitness;
		ly->fix_overlaps_from_pairs();
	}

	// DEBUG
	//if (update)
	//{
	//	double realF = ly->full_fitness_overlap();
	//	double fDiff = realF - newFitness;
	//	if (fDiff > TOL || fDiff < -TOL)
	//		cout << "WARNING: Updated fitness yields " << newFitness << ", real is " << realF << endl;
	//}
	// END DEBUG

	return newFitness;
}

bool NEIGHBOURHOOD::next_random_neighbour(bool test, vector<int> &overlapList)
{
	std::uniform_int_distribution<int> integerDistributionOverlaps(0, overlapList.size() - 1);

	int maxTrials = 0;
	int maxTrialsLimit = 10000;
	bool can_find_other_type = false;
	// cout << endl << "Overlap list: " << endl;
	for (int i = 0; i < overlapList.size(); ++i)
	{
		if (ly->instance->pieces[overlapList[i]]->index != ly->instance->pieces[cp]->index)
		{
			can_find_other_type = true;
			// cout << overlapList[i] << ", ";
			break;
		}
	}	

	if (!can_find_other_type)
	{
		maxTrialsLimit = 0;
		// cout << "WARNING (NEXT RANDOM NEIGHBOUR) - Overlap list has only one type." << endl;
	}	
	// if (overlapList.size() < 2)
	// 	cout << "WARNING (NEXT RANDOM NEIGHBOUR) - Going into infinite loop..." << endl;


	// swapThis = overlapList[integerDistributionOverlaps(randomNumGenerator)];
	cneig = overlapList[integerDistributionOverlaps(randomNumGenerator)] - 1;
	//std::uniform_int_distribution<int> integerDistributionAll(0, pieceIndices.size() - 1);


	while (ly->instance->pieces[cp]->index == ly->instance->pieces[cneig + 1]->index)
	{
		maxTrials++;
		if ((!can_find_other_type) || (maxTrials >= maxTrialsLimit))
		{
			if (can_find_other_type)
				cout << "Warning: Next random neighbour cannot find a valid swap move." << endl;

			cneig = cp;
			break;
		}
		cneig = overlapList[integerDistributionOverlaps(randomNumGenerator)] - 1;
	}
	// cneig = 
	bool didchange = false;
	if (cp < cneig)
	{
		int dummy = cp;
		change_piece_to(cneig + 1);
		// cp = cneig + 1;
		cneig = dummy - 1;
		didchange = true;
	}

	// if (didchange)
		// cout << "Change: Trying piece " << cp << " and " << cneig;
	// else
	// cout << endl << "Trying piece " << cp << " and " << cneig;

	bool valid = next_neighbour(test);
	// cout << " F = " << tfit << endl;
	// cneig = cneig_old + 1;
	return(true);
}

bool NEIGHBOURHOOD::next_neighbour(bool test)
{
	bool symMove = false;
	int tamano = (n30_points.size() - 1);
	switch (ntype)
	{
		case 20: // N2, 6 points around:
			if (cneig >= 5)
				return false;

			// Search tabu list:
			if (enabledTabu)
			{
				for (int i = 0; i < tabuPiece.size(); i++)
				{
					// This piece is on the list
					if ((tabuPiece[i] == cp) && ((tabuMove[i] - 1) == cneig))
					{
						tfit = 0;
						cneig++;
						return(next_neighbour(test));
					}
				}
			}


			n20_next_neighbour(test);
			break;
		case 30: // N30 Cube neighbourhood
			// cout << cneig;
			// cout << " >= " << n30_points.size() << endl;
			// cout << (cneig >= (n30_points.size() - 1)) << endl;
			// cout << "Tamanho = " << tamano << " cneig >= tamano " << (cneig >= tamano) << endl;
			if (cneig >= tamano)
				return false;

			// Search tabu list:
			if (enabledTabu)
			{
				for (int i = 0; i < tabuPiece.size(); i++)
				{
					if ((tabuPiece[i] == cp))
					{
						// This piece is on the list
						vector<int> testPoint = n30_points[tabuMove[i]];
						vector<bool> agreeDir(3);
						agreeDir.assign(3, false);
						for (int coord = 0; coord < 3; coord++)
						{
							if (testPoint[coord] * n30_points[cneig + 1][coord] > 0)
								agreeDir[coord] = true;
						}

						if (!agreeDir[0] && !agreeDir[1] && !agreeDir[2]) // This point is trying to go back
						{
							tfit = 0;
							cneig++;
							return true;
						}
					}
				}
			}

			n30_next_neighbour(test);
			break;

		case 40: // Does not have a limit on points, but has a limit on rejections (6)
			if (rejectedMovements > 6)
				return false;

			// Find out which direction this is:
			cneig_mod_6 = cneig % 6;

			// Search tabu list: // Here, whole directions are banned, rather than points
			if (enabledTabu)
			{
				for (int i = 0; i < tabuPiece.size(); i++)
				{
					// This piece is on the list
					if ((tabuPiece[i] == cp) && ((tabuMove[i] - 1) == cneig_mod_6))
					{
						tfit = 0;
						cneig++;
						rejectedMovements++;
						return(next_neighbour(test));
					}
				}
			}

			n40_next_neighbour(test);
			break;

		case 50: // Limit is nPairs
			// cout << "Move: p = " << cp << " cneig = " << cneig << endl;
			if (cneig + 1 >= ly->instance->nPieces)
				return false;
			// CHECK FOR SYMMETRY
			symMove = false;
			// If the piece is the same
			if (cp <= cneig + 1)
				symMove = true;
			// Or the piece type is the same
			if (ly->instance->pieces[cp]->index == ly->instance->pieces[cneig + 1]->index)
				symMove = true;
			// Skip
			if (symMove)
			{
				// cout << "Move: p = " << cp << " cneig = " << cneig << endl;
				// cout << "Move is invalid, skip" << endl;
				tfit = 0;
				cneig++;
				rejectedMovements++;
				return(next_neighbour(test));
			}
			else
			{
				// cout << "Move: p = " << cp << " cneig = " << cneig << endl;
				// cout << "Move is valid, go ahead." << endl;
			}

			// Search tabu list: // Here, whole directions are banned, rather than points
			if (enabledTabu)
			{
			// CHECK TABU LIST
				for (int i = 0; i < tabuPiece.size(); i++)
				{
					// This piece is on the list
					if ( ((tabuPiece[i] == cp) && (tabuMove[i] == cneig + 1)) || ((tabuPiece[i] == cneig + 1) && (tabuMove[i] == cp)))
					{
						// cout << "WARNING: Tabu not checked properly!" << endl;
						tfit = 0;
						cneig++;
						rejectedMovements++;
						return(next_neighbour(test));
					}
				}
			}

			n50_next_neighbour(test);
			break;

		default:
			cout << endl << "ERROR: Trying to get a neighbour of type" << ntype << ", which is not implemented!" << endl << endl;
			break;
	}

	// We know how many neighbours each neighbourhood gives, 
	// so return false earlier if this is violated:
	return true;
}

bool NEIGHBOURHOOD::undo_neighbour()
{
	bool move_reverted = false;
	if (ntype == 25)
	{
		undo_move();
		move_reverted = true;
	}

	return move_reverted;
}

void NEIGHBOURHOOD::clear_tabu_list()
{
	tabuPiece.assign(0, 0);
	tabuMove.assign(0, 0);
	rejectedMovements = 0;
}
void NEIGHBOURHOOD::cout_tabu_list(bool horizontal)
{
	if (tabuPiece.size() == 0)
	{
		cout << "Tabu list is empty so far!!!" << endl;
		return;
	}

	if (horizontal)
		cout << endl << "P: ";
	else
		cout << endl << "- P -\t- N -" << endl;
	cout << tabuPiece[0];
	if (!horizontal)
		cout << "\t" << tabuMove[0];

	for (int i = 1; i < tabuPiece.size(); i++)
	{
		if (horizontal)
			cout << "\t";
		else
			cout << "\t" << tabuMove[i] << endl;

		cout << tabuPiece[i];
	}
	if (horizontal)
	{
		cout << endl << "N: ";
		cout << tabuMove[0];
		for (int i = 1; i < tabuMove.size(); i++)
		{
			cout << "\t" << tabuMove[i];
		}
	}
	cout << endl;
}

void NEIGHBOURHOOD::add_tabu()
{

	tabuPiece.push_front(cp);
	tabuMove.push_front(opposite_neighbour_to_tabu());

	if (tabuPiece.size() > tabuSize)
	{
		tabuPiece.pop_back();
		tabuMove.pop_back();
	}
	
}

int NEIGHBOURHOOD::opposite_neighbour_to_tabu()
{
	//	  currentN = 0 -> + x
	//	  currentN = 1 -> - x
	//	  current N = 2 -> + y
	//	  current N = 3 -> - y
	//	  current N = 4 -> + z
	//	  current N = 5 -> - z

	int testValue;
	int oneig = -1;
	if (ntype == 20 || ntype == 40)
	{
		if (ntype == 20)
		{
			testValue = cneig;
		}
		else if (ntype == 40)
		{
			testValue = cneig_mod_6;
		}
		switch (testValue)
		{
		case 0:
			oneig = 1;
			break;
		case 1:
			oneig = 0;
			break;
		case 2:
			oneig = 3;
			break;
		case 3:
			oneig = 2;
			break;
		case 4:
			oneig = 5;
			break;
		case 5:
			oneig = 4;
			break;
		default:
			cout << endl << "WARNING: Unkown opposite neighbour!!!!!" << endl << endl;
			break;
		}
	}
	else if (ntype == 30)
		oneig = cneig;
	else if (ntype == 50)
		oneig = cneig;

	return oneig;
}
bool NEIGHBOURHOOD::random_neighbour()
{
	return false;
}

void NEIGHBOURHOOD::change_parameter_to(int newPar)
{
	int	nPoint = 0;
	parameter = newPar;
	rejectedMovements = 0;
	cneig = -1;
	tfit = -1;
	if (ntype == 30)
	{
		vector<vector <int > > aux_aux;
		n30_points = aux_aux;
		int radMax = 1;
		if (parameter > 0)
		{
			radMax = parameter;
		}
		else
		{
			radMax = max(ly->UpperBoundH, ly->instance->container->gridSize[0]);
			for (int dims = 1; dims < 3; dims++)
				radMax = max(radMax, ly->instance->container->gridSize[dims]);
		}

		int radMin = -1*radMax;
		for (int i = radMin; i <= radMax; i++)
		{
			for (int j = radMin; j <= radMax; j++)
			{
				for (int k = radMin; k <= radMax; k++)
				{
					// No need to check (0, 0, 0) as the piece is there!
					if (i == 0 && j == 0 && k == 0) 
						continue; 

					// Add the point:
					vector<int> aux_point = { i, j, k };
					n30_points.push_back(aux_point);
					//n30_points[nPoint][0] = i;
					//n30_points[nPoint][1] = j;
					//n30_points[nPoint][2] = k;
					
					nPoint++;
				}
			}
		}
	}
}
void NEIGHBOURHOOD::change_piece_to(int p)
{
	cp = p;
	cneig = -1;
	tfit = -1;

	rejectedMovements = 0;
}

double NEIGHBOURHOOD::swap_to(bool test)
{
	int swapP = cp;
	int swapQ = cneig;
	// cout << "  ---  ACTUALLY SWAPPING: " << cp << " and " << cneig << endl;
	vector<int> p_next = ly->pieceIndices[swapQ];
	vector<int> q_next = ly->pieceIndices[swapP];
	// Save the previous coords in case it's only a test
	vector<int> p_old;
	vector<int> q_old;
	if (test)
	{
		p_old = ly->pieceIndices[swapP];
		q_old = ly->pieceIndices[swapQ];
	}
	// Make sure no piece goes over the height
	p_next[ly->od] = min(p_next[ly->od], max(ly->UpperBoundH - ly->instance->pieces[swapP]->voxel.get_grid_size(ly->od), 0));
	q_next[ly->od] = min(q_next[ly->od], max(ly->UpperBoundH - ly->instance->pieces[swapQ]->voxel.get_grid_size(ly->od), 0));

	// Or over the border
	p_next[ly->cd1] = max(0, min(p_next[ly->cd1], ly->instance->maxPositions[ly->instance->pieces[swapP]->index][ly->cd1]));
	p_next[ly->cd2] = max(0, min(p_next[ly->cd2], ly->instance->maxPositions[ly->instance->pieces[swapP]->index][ly->cd2]));

	q_next[ly->cd1] = max(0, min(q_next[ly->cd1], ly->instance->maxPositions[ly->instance->pieces[swapQ]->index][ly->cd1]));
	q_next[ly->cd2] = max(0, min(q_next[ly->cd2], ly->instance->maxPositions[ly->instance->pieces[swapQ]->index][ly->cd2]));

	// Change the coordinates:
	ly->pieceIndices[swapP] = p_next;
	ly->pieceIndices[swapQ] = q_next;

	// Find the fitness:
	tfit = ly->full_fitness(-1,false);

	if (test)
	{	// Reverse the change:
		ly->pieceIndices[swapP] = p_old;
		ly->pieceIndices[swapQ] = q_old;
		ly->full_fitness();
	}
	else
	{
		pfit = cfit; // Save the current one
		cfit = tfit; // Update to the next one
		ly->highest_container_point(); // Just in case, update the height
		ly->fix_overlaps_from_pairs();
	}

	return tfit;

}

double NEIGHBOURHOOD::test_move_to()
{
	// Move to the neighbour:
	ly->pieceIndices[cp][0] += dir_i;
	ly->pieceIndices[cp][1] += dir_j;
	ly->pieceIndices[cp][2] += dir_k;

	// Test the fitness there
	if (ly->options->fitnessString.compare("closestpoint") == 0)
		tfit = ly->full_fitness_closest_point(cp, false); // Only using one piece
	else
		tfit = get_fitness(false);

	// Move back:
	ly->pieceIndices[cp][0] -= dir_i;
	ly->pieceIndices[cp][1] -= dir_j;
	ly->pieceIndices[cp][2] -= dir_k;
	
	return tfit;
}

void NEIGHBOURHOOD::do_move_to()
{
	// Move to the neighbour:
	ly->pieceIndices[cp][0] += dir_i;
	ly->pieceIndices[cp][1] += dir_j;
	ly->pieceIndices[cp][2] += dir_k;

	// if (dir_k == 1)
	// {
		// cout << "MOVING to move upwards!!!" << endl;
	// }

	// Save values that will change, in case we need to return:
	pfit = cfit; // Fitness

	if (ly->options->fitnessString.compare("closestpoint") == 0)
	{
		cfit = ly->full_fitness_closest_point(-1, true);
		// UPDATES UNTIL THIS THING WORKS PROPERLY
		// debug : remove
		ly->full_fitness_overlap();
		ly->fix_overlaps_from_pairs();
	}
	else
		cfit = get_fitness(true);


	// prevLy.overlaps[cp] = ly->overlaps[cp];
	// prevLy.overlapCount = ly->overlapCount;


	// Save for debug purposes:
	//vector < double >  oldWholePairs = ly->relBoxOverlapPairVoxel;

	/// DEBUG:
	//vector <int > aux_pq = { -1, -1 };
	//vector < vector < int > > whichPairIs;
	//whichPairIs.assign(ly->instance->nPairs, aux_pq);
	/// 
	// end debug


	//int pairIdx = 0;
	//ly->update_pair_index(cp, pairIdx);
	//for (int ii = 0; ii < ly->instance->nPieces; ii++)
	//{
	//	ly->update_pair_index(cp, ii, pairIdx);
	//	if (cp == ii)
	//		continue;


	//	// DEBUG:
	//	//whichPairIs[pairIdx][0] = min(cp, ii);
	//	//whichPairIs[pairIdx][1] = max(cp, ii);
	//	//// end debug


	//	// Save the overlap info:
	//	// prevLy.relBoxOverlapPairVoxel[pairIdx] = ly->relBoxOverlapPairVoxel[pairIdx];
	//	double oldOverlapOfPair = ly->relBoxOverlapPairVoxel[pairIdx];

	//	// bool didTheyOverlap = (ly->relBoxOverlapPairVoxel[pairIdx] > TOL);
	//	bool overlapOfBoxes = ly->box_overlap_rel(cp, ii, ly->relBoxOverlapPairVoxel[pairIdx]);
	//	if (overlapOfBoxes && (ly->relBoxOverlapPairVoxel[pairIdx] > TOL)) // Boxes overlap and pieces as well
	//	{
	//		// if (!didTheyOverlap)
	//		// {
	//			// ly->overlapCount++;
	//			// ly->overlaps[cp] = true;
	//			// ly->overlaps[i] = true;
	//		// }
	//	}
	//	else
	//	{
	//		ly->relBoxOverlapPairVoxel[pairIdx] = 0;
	//		// if (didTheyOverlap) // Hooray! we solved an overlap!
	//			// ly->overlapCount--;
	//	}

	//	// Update the fitness with this new overlap measure:
	//	cfit = cfit + (- ly->relBoxOverlapPairVoxel[pairIdx] + oldOverlapOfPair)/ly->instance->nPairs;
	//	
	//}
	// Fitness updated here, done!

	// Update the overlap info:
	//ly->fix_overlaps_from_pairs();


	//bool seriousDebug = false;
	//if (seriousDebug)
	//{
	//	int k = 0;
	//	double realFitness = ly->full_fitness_overlap(k);
	//	if (cfit != realFitness)
	//	{
	//		cout << "Fitness disagree... ne.cfit = " << cfit << "; real = " << realFitness << endl;
	//		cout << "Current piece: " << cp << endl;
	//		int pair = -1;
	//		for (int p = 0; p < ly->instance->nPieces; p++)
	//		{
	//			for (int q = p + 1; q < ly->instance->nPieces; q++)
	//			{
	//				pair++;
	//				double diff = oldWholePairs[pair] - ly->relBoxOverlapPairVoxel[pair];
	//				if (diff > TOL || diff < -TOL)
	//				{
	//					cout << "PAIR " << pair << "(p = " << p << ", q = " << q << ") old: " << oldWholePairs[pair] << ", new: " << ly->relBoxOverlapPairVoxel[pair];
	//					cout << " (newpair, p= " << whichPairIs[pair][0] << ", q= " << whichPairIs[pair][1] << ")" << endl;
	//				}
	//			}
	//		}
	//		cout << endl;
	//		cfit = realFitness;
	//	}
		// else
			// cout << "Fitness do agree!" << endl;
	//}

}

void NEIGHBOURHOOD::undo_move()
{
	// Move to the neighbour:
	//ly->pieceIndices[cp][0] -= dir_i;
	//ly->pieceIndices[cp][1] -= dir_j;
	//ly->pieceIndices[cp][2] -= dir_k;

	//cfit = pfit; // Fitness
	//ly->overlaps[cp] = prevLy.overlaps[cp];
	//ly->overlapCount = prevLy.overlapCount;

	//// Revert overlap info:
	//int pairIdx = cp - 1;
	//int i = -1;
	//for (int ii = 0; ii < ly->instance->nPieces; ii++)
	//{
	//	if (ii == cp)
	//		continue;
	//	else
	//		i++;

	//	// return to previous overlap info:
	//	 ly->relBoxOverlapPairVoxel[pairIdx] = prevLy.relBoxOverlapPairVoxel[pairIdx];

	//	// Move to next pair:
	//	if (i < cp)
	//		pairIdx += ly->instance->nPieces - 1 - i;
	//	else
	//		pairIdx++;
	//}

}

void NEIGHBOURHOOD::n30_next_neighbour(bool test)
{
	// Cube neighbourhood
	cneig++;

	dir_i = n30_points[cneig][0];
	dir_j = n30_points[cneig][1];
	dir_k = n30_points[cneig][2];

	if (!ly->can_p_move_to(cp, dir_i, dir_j, dir_k)) // Movement not allowed...
	{
		tfit = -1;
		// next_neighbour(test);
		return;
	}

	if (test)
		test_move_to();
	else
		do_move_to();
}

void NEIGHBOURHOOD::n40_next_neighbour(bool test)
{
	cneig++;
	if (cneig_mod_6 < 5)
		cneig_mod_6++;
	else
		cneig_mod_6 = 0;

	int amount = floor(cneig / 6) + 1;
	
	// Choose direction and amount to move; defaults are case 0
	dir_i = n20_directions[cneig_mod_6][0] * amount;
	dir_j = n20_directions[cneig_mod_6][1] * amount;
	dir_k = n20_directions[cneig_mod_6][2] * amount;

	bool tooFar = false;
	if (parameter > 0 && amount > parameter)
		tooFar = true;


	if (tooFar || !ly->can_p_move_to(cp, dir_i, dir_j, dir_k))
	{
		// This movement is not allowed!
		// cfit = 0;
		tfit = 0;
		rejectedMovements++;
		next_neighbour(test);
		return;
	}
	else
		rejectedMovements = 0;
	//else
	//{
	//	if (dir_k == -1 && test)
	//	{
	//		cout << "N = " << cneig << "p = " << cp << " move (" << dir_i << ", " << dir_j << ", " << dir_k << ") from ";
	//		cout_point3(ly->pieceIndices[cp]);
	//		cout << endl;
	//	}
	//}


	if (test)
		test_move_to();
	else
		do_move_to();

}

void NEIGHBOURHOOD::n50_next_neighbour(bool test)
{
	cneig++;
	swap_to(test);

}
void NEIGHBOURHOOD::n20_next_neighbour(bool test)
{
	// Move piece "currentPiece" 1 voxel in one of the 6 directions:
	/* 

	currentN = 0 -> + x
	currentN = 1 -> - x
	current N = 2 -> + y
	current N = 3 -> - y
	current N = 4 -> + z
	current N = 5 -> - z

	*/
	cneig++;

	dir_i = n20_directions[cneig][0];
	dir_j = n20_directions[cneig][1];
	dir_k = n20_directions[cneig][2];

	// Choose direction and amount to move; defaults are case 0
	//dir_i = 0;
	//dir_j = 0;
	//dir_k = 0;

	//switch (cneig)
	//{
	//case 0:
	//	dir_i = 1;
	//	break;
	//case 1:
	//	dir_i = -1;
	//	break;
	//case 2:
	//	dir_j = 1;
	//	break;
	//case 3:
	//	dir_j = -1;
	//	break;
	//case 4:
	//	dir_k = 1;
	//	break;
	//case 5:
	//	dir_k= -1;
	//	break;
	//}

	if (!ly->can_p_move_to(cp, dir_i, dir_j, dir_k))
	{
		// This movement is not allowed!
		// cfit = 0;
		tfit = 0;
		next_neighbour(test);
		return;
	}
	//else
	//{
	//	if (dir_k == -1 && test)
	//	{
	//		cout << "N = " << cneig << "p = " << cp << " move (" << dir_i << ", " << dir_j << ", " << dir_k << ") from ";
	//		cout_point3(ly->pieceIndices[cp]);
	//		cout << endl;
	//	}
	//}

	if (test)
		test_move_to();
	else
		do_move_to();

}

void NEIGHBOURHOOD::set_type(int t)
{
	ntype = t;
}
int NEIGHBOURHOOD::get_parameter()
{
	return parameter;
}

int NEIGHBOURHOOD::get_type()
{
	return ntype;
}

int NEIGHBOURHOOD::get_piece()
{
	return cp;
}

void NEIGHBOURHOOD::set_fitness()
{
	cfit = ly->full_fitness();
}
void NEIGHBOURHOOD::set_fitness(double fitness)
{
	cfit = fitness;
}
// NEIGHBOURHOOD::NEIGHBOURHOOD(int neighbourhoodType, PACKING_LAYOUT * layout)
// {
	// NEIGHBOURHOOD(neighbourhoodType, layout, -1);
// }

NEIGHBOURHOOD::NEIGHBOURHOOD(int neighbourhoodType, PACKING_LAYOUT * layout, int givenParameter)
{
	parameter = givenParameter;
	ntype = neighbourhoodType; // Neighbourhood type
	ftype = -1; // Fitness type
	cp = 0; // Current piece
	ly = layout;
	cfit = -1;
	prevLy = (*layout);
	cneig = -1;
	pfit = cfit;
	dir_i = 0;
	dir_j = 0;
	dir_k = 0;

	rejectedMovements = 0;


	// Do not care much about this at this stage:
	upperBoundary = RAND_MAX;

	od = &ly->od;
	cd1 = &ly->od;
	cd2 = &ly->od;


	int nPoint = 0;
	int realDir = 0;
	vector <int > aux_zeros(3);
	aux_zeros.assign(3, 0);
	for (int coord = 0; coord < 3; coord++)
	{
		for (int dir = 0; dir < 2; dir++)
		{
			if (dir == 0)
				realDir = 1;
			else
				realDir = -1;

			n20_directions.push_back(aux_zeros);
			// n20_directions[nPoint][0] = 0;
			// n20_directions[nPoint][1] = 0;
			// n20_directions[nPoint][2] = 0;
			n20_directions[nPoint][coord] = realDir;

			nPoint++;
		}

	}

	// Neighbourhood points:
	change_parameter_to(parameter);

}

void PACKING_LAYOUT::vns_february(int lob, int upb)
{
	/*
				Algorithm steps

		// # 1 - explore_neighbourhoods_vns	
		// # 2 - decide_oscillation_vs_disruption
		// # 3a - disrupt
		// # 3b - oscillate_up
		// # 3c - oscillate_down

	*/

cout << "Starting vns_february()..." << endl;

// --- PARAMETERS (not loaded) --- //

	bool sanityCheck = true; // Extra tests
	bool silent = false; // Disable screen output
	bool placeAtZero = false; // Start with everything at (0,0,0)
	int  VNS_ITERS_MAX = options->maxIters;
	int  DISRUPTIONS_MAX = options->maxIters;
	bool createFile = true; // Create a csv with all runtime info

	// NEIGHBOURHOOD LIST:
	vector<NEIGHBOURHOOD*> neighbourhoodList;
	int currentNeighbourhood = 0;

	NEIGHBOURHOOD ne0(30, this, 1); // Cube, delta = 1
	cout << "N1 generated." << endl;
	// NEIGHBOURHOOD ne1(40, this, 10); // Axis aligned, 10 cubes in each dir
	// cout << "N2 generated." << endl;

	NEIGHBOURHOOD ne2(40, this, -1); // Axis aligned, all container
	cout << "N3 generated." << endl;

	NEIGHBOURHOOD ne3(30, this, 10); // Insert in all container!
	cout << "N4 generated." << endl;

	neighbourhoodList.push_back(&ne0);
	// neighbourhoodList.push_back(&ne1);
	neighbourhoodList.push_back(&ne2);
	neighbourhoodList.push_back(&ne3);


	//Disrupt phase - swap:
	// This moved to destroy_n()
	// bool chooseRandomPiecesToSwap = true; // Just swap any two random pieces 

	// Debugging stuff (to be removed)
	bool saveAllSteps = false;
	int serialisedSols = 0;

// --- Initializations --- //
	clock_t InitialPackingClock = clock();
	clock_t packingClock = clock();
	double workingFitness = 0;
	double fitnessBeforeDisruption = -1;
	double fitnessAfterDisruptionAndHC = -1;
	int nofDisruptions = 0;
	float percImprovement = 0.05;
	float percWorsening = 0.1;
	float percEqual = 0.05;
	int previousDisruption = -1;

	// Printing aids:
	int colW = 8;
	int nCols = 6;
	const char separator = ' | ';
	stringstream printStream;

	ofstream fi;
	if (createFile)
	{
		std::stringstream streamForFile;
		streamForFile << "vns_feb_" << "runtimeinfo" << ".csv";
		string csvFilename = streamForFile.str();
		fi.open(csvFilename, std::ofstream::trunc);
	}
	// Header:
	if (createFile)
	{
		fi << setprecision(5);
		fi << "Iter,";
		fi << "Time,";
		fi << "Overl,";
		fi << "Height,";
		fi << "UpbH,";
		fi << "Fitness,";
		fi << "VNS_iters,";
		fi << "VNS_movements,";
		for (int i = 0; i < destroyNames.size(); i++)
		{
			fi << "\"" << destroyNames[i].c_str() << " score\",";
		}
		fi << "Last_disruption,";
		fi << "disruption_parameter,";
		fi << endl;
		// cout << printStream.str();
	}


	methodString = "Variable Neighbourhood Search - February 2017 version";

	// initial_solution
	bool noOverlap = false;
	if (placeAtZero)
	{
		for (int i = 0; i < instance->nPieces; i++)
		{
			piecePlaced[i] = true;
			ignorePiece[i] = false;
		}
	}
	else
	{
		strip_voxel_packing_first_fit_nfv();
	}
	PACKING_LAYOUT bestSolution = (*this);

	// Upper bounds:
	if (is_feasible_voxel())
	{
		cout << "We start with a feasible solution of H = " << highest_container_point() << endl << "Strategic oscillation: " << endl;
		strategic_oscillation(lob);
		cout << "Done." << endl;
		// UpperBoundH = max(lob, int(ceil(upb*options->decPercentage)));
	}
	else
		UpperBoundH = upb - 1;

	cout << "Initial UppberBoundH is " << UpperBoundH << endl << endl;
	cout << "Initial fitness is " << full_fitness() << endl;

	
// Start algorithm!
if (!silent)
{
	printStream << setw(nCols * colW + 3*(nCols -1)) << setfill('-') << " " << endl;
	printStream << setprecision(5);
	// printStream << setw(colW) << setfill(separator) << "Iter";
	printStream << setw(colW) << setfill(separator) << "Time (s)";
	printStream << setw(colW) << setfill(separator) << "Overl";
	printStream << setw(colW) << setfill(separator) << "Height";
	printStream << setw(colW) << setfill(separator) << "Fitness";
//	printStream << setw(colW) << setfill(separator) << "Piece";
	printStream << setw(colW) << setfill(separator) << "Comment";
	// printStream << setw(colW) << setfill(separator) << "(Dir)";
	printStream << endl;
	printStream << setw(nCols * colW + 3*(nCols - 1)) << setfill('-') << " " << endl;;
	cout << printStream.str();
}
int alnsiters = 0;
while (true)
{
	packingClock = clock();
	elapsedTime = float(packingClock - InitialPackingClock) / CLOCKS_PER_SEC;
	// stop_criteria

// # 1 - explore_neighbourhoods_vns	
	// output_line:
	if (!silent && false) // No need for this, only repeats disruption thingy
	{
		cout << setw(colW) << setprecision(2) << elapsedTime;
		cout << setw(colW) << overlapCount;
		cout << setw(colW) << highest_container_point();
		// cout << setw(colW) << setprecision(5) << workingFitness;
		cout << setw(colW) << setprecision(5) << full_fitness();
		// cout << setw(colW) << setprecision(5) << full_fitness_closest_point(); // debug : remove!
		cout << setw(colW) << "New it";
		cout << endl;
	}
	int currentNeighbourhood = 0;
	NEIGHBOURHOOD * ne = neighbourhoodList[currentNeighbourhood % neighbourhoodList.size()];
	// NEIGHBOURHOOD ne(, this, -1);
	bool redoOverlapList = true;
	vector<int> overlapList;
	vector<int> emptyVec;
	int vnsiters = 0;
	int realMovements = 0;

	while (true) // Explore all neighbourhoods
	{
		vnsiters++;
		if (noOverlap)
			break;
		// double bestF = full_fitness_overlap(); // debug : speed
		// ne->cfit = bestF;
		int bestN = -1;
		int bestP = -1;

		// Re-do overlap List
		full_fitness(-1, true);
		fix_overlaps_from_pairs();
		ne->cfit = fitness;
		// if (redoOverlapList)
		if (true)
		{
			overlapList = emptyVec;
			for (int p = 0; p < instance->nPieces; p++)
			{
				if (overlaps[p])
					overlapList.push_back(p);
			}
			// debug : we always re-do overlap list!!!!!!!!
					// redoOverlapList = false;
		}

		/* ----------------------- FITNESS CHANGE ---------------- */
		/* ----------------------- FITNESS CHANGE ---------------- */
		/* ----------------------- FITNESS CHANGE ---------------- */
		if (overlapCount > 1)
		{
			int smallParam = 15;
			if (ne3.get_parameter() != smallParam)
			{
				cout << "--- Parameter is: " << smallParam << "(was " << ne3.get_parameter() << ", overlapCount is: " << overlapCount << ", " << overlapList.size() << " ---" << endl;
				ne3.change_parameter_to(smallParam);
			}
			// cout << "Swtiching to closest point fitness" << endl;
			// this->options->fitnessString = "closestpoint";
		}
		else
		{
			int largeParam = UpperBoundH;
			if (ne3.get_parameter() != largeParam)
			{
				cout << "*** Parameter is: " << largeParam << " (was " << ne3.get_parameter() << ", overlapCount is: " << overlapCount << ", " << overlapList.size() << " ***" << endl;
				// cout << "*** Parameter of last neighbourhood grows to 40 *** " << endl;
				ne3.change_parameter_to(largeParam);
			}
			// cout << "Swtiching to overlap fitness" << endl;
			// this->options->fitnessString = "overlap";
		}
		/* ----------------------- FITNESS CHANGE END ---------------- */
		/* ----------------------- FITNESS CHANGE END ---------------- */
		/* ----------------------- FITNESS CHANGE END ---------------- */




		// Test the neighbourhood of all pieces, select the best move of all of them
		bool didImprove = false;
		double bestF = this->fitness; // Freshly calculated above

		for (int idx = 0; idx < overlapList.size(); idx++)
		{
			int p = overlapList[idx];
			ne->change_piece_to(p);
			while (ne->next_neighbour(true))
			{
				if (ne->tfit > bestF + TOL)
				{
					bestF = ne->tfit;
					bestN = ne->cneig;
					bestP = p;
					didImprove = true;
				}
				//else
				//{
				//	cout << "Rejected move: bestF = " << bestF << "; ne->tfit = " << ne->tfit << endl;
				//}
				// if (debugInfo)
				// {
				// cout << "\tn = " << ne->cneig << ", f = " << ne->tfit << ", BN  = " << bestN << ", BF = " << bestF << "; dir: ";
				// cout_point3(ne->dir_i, ne->dir_j, ne->dir_k);
				// }
			} // End for each neighbourhood point
		} // End for each overlapping piece
		// cout << "Finished exploration. Current F = " << full_fitness_closest_point(-1, true) << ", aiming to bestF = " << bestF << endl;
		if (saveAllSteps)
		{
			serialisedSols++;
			std::stringstream sstm;
			sstm << options->file << "_" << "ALNS_progress_" << serialisedSols << ".psol";
			string hc_solname = sstm.str();
			serialise(hc_solname);
		}
		// Finished exploring the neighbourhood, check what happened
		if (didImprove)
		{
			// Move to the new point:
			ne->change_piece_to(bestP);
			ne->cneig = bestN - 1;
			if (!ne->next_neighbour(false)) // This is a "real" move!
			{
				printStream << "Move rejected!!! (bestN = " << bestN << ")" << endl;
				printStream << "P = " << ne->get_piece() << " cneig = " << ne->cneig << endl;
			}
			else
			{
				realMovements++;
				// cout << "Movement! f_overlap " << full_fitness_overlap() << ", f_closest " << full_fitness_closest_point() << " overlaps " << overlapCount << endl;
				// If we are here, we just performed a move!
				redoOverlapList = true; // debug : speed (This is only necessary sometines!)
				double thisFitness = ne->cfit;
				double correctFitness = full_fitness(-1, true); // Needed to test overlap later
				fix_overlaps_from_pairs();

				// cout << "Movement fitness: " << correctFitness << endl;

				// double correctFitness = full_fitness_overlap(); // Needed to test overlap later
				workingFitness = correctFitness;
				if (sanityCheck)
				{
					if (abs(thisFitness - correctFitness) >= TOL)
					{
						cout << "WARNING: ne->cfit (" << ne->cfit << ") and real fitness (" << correctFitness << ")  do not match!!!!" << endl;
						thisFitness = correctFitness;
					}
				}


				double currentClosest = full_fitness(-1, true);
				// cout << "Movement! f_overlap " << full_fitness_overlap() << ", f_closest " << currentClosest << " overlaps " << overlapCount << endl;
				// cout << "BestF was " << bestF;
				// cout << ", current closest_point f = " << currentClosest << endl;
				if (sanityCheck && abs(currentClosest - bestF) > TOL)
					cout << "WARNING: Fitness do not match (expected vs. real)" << endl;
			}

			if (overlapCount < 1)
			{
				cout << "********************************" << endl;
				cout << "***      Zero overlap!!!    ***" << endl;
				cout << "********************************" << endl;
				int foundH = highest_container_point();
				upb = foundH;
				// cout << "Height: " << foundH << endl;
				cout << " (H = " << foundH << ", real h = " << maxHeight << ")" << endl;

				noOverlap = true;
				// In theory, we never go to worst solutions
				bestSolution = (*this);
				// Serialise it!
				std::stringstream sstm;
				sstm << options->file << "_" << "VNS_feb_" << "H_" << foundH << ".psol";
				string hc_solname = sstm.str();
				serialise(hc_solname);
				break;
			}
			else
			{
				// Improved, move to first neighbourhood:
				ne = neighbourhoodList[0];
				currentNeighbourhood = 0;
			}

		}
		else
		{
			// No improvement. Either move to next neighbourhood or terminate.
			if (currentNeighbourhood >= neighbourhoodList.size() - 1)
				break; // Go to oscillation or disruption
			else
			{
				currentNeighbourhood++;
				ne = neighbourhoodList[currentNeighbourhood];
			}

		}

	} // End of "vns part" --
	// check_time
	packingClock = clock();
	elapsedTime = float(packingClock - InitialPackingClock) / CLOCKS_PER_SEC;

	// output_line:
	if (!silent)
	{
		cout << setw(colW) << setprecision(2) << elapsedTime;
		cout << setw(colW) << overlapCount;
		cout << setw(colW) << highest_container_point();
		// cout << setw(colW) << setprecision(5) << workingFitness;
		cout << setw(colW) << setprecision(5) << full_fitness();
		// cout << setw(colW) << setprecision(5) << full_fitness_closest_point(); // debug : remove!
		cout << setw(colW) << " end VNS (" << vnsiters << " its, " << realMovements << " movements)";
		cout << endl;
	}

// # 2 - decide_oscillation_vs_disruption
	bool disrupt = false;
	bool oscillateUp = false;
	bool oscillateDown = false;
	if (noOverlap)
		oscillateDown = true;
	else
	{
		// Oscillate up or disrupt
		// if (vnsiters >= VNS_ITERS_MAX)
		if (nofDisruptions >= DISRUPTIONS_MAX)
		{
			cout << endl << "We have performed " << nofDisruptions << " disruptions, stop them and go up." << endl;
			oscillateUp = true;
			vnsiters = 0;
		}
		else
		{
			disrupt = true;
			nofDisruptions++;
		}
	}


// # 3a - disrupt
	if (disrupt)
	{
		// Update scores:
		if (fitnessBeforeDisruption > -TOL) // Fitness is different from -1 (It's initialised)
		{
			// Update the scores:
			fitnessAfterDisruptionAndHC = fitness;
			double fitnessDiff = fitnessAfterDisruptionAndHC - fitnessBeforeDisruption;
			// cout << "Fitness diff = " << fitnessDiff << " for " << destroyNames[previousDisruption].c_str() << endl;
			// cout << "Fitness went from " << fitnessBeforeDisruption << " to " << fitnessAfterDisruptionAndHC << "(diff = " << fitnessDiff << ")" << endl;
			// cout << "All scores were: (" << destroyScores[0] << ", " << destroyScores[1] << ")" << endl;

			// Update the scores:
			if (fitnessDiff > TOL)
				decrease_all_but_n_by_perc(previousDisruption, percImprovement,destroyScores);
			else if (fitnessDiff < -TOL)
				decrease_n_by_perc(previousDisruption, percWorsening, destroyScores);
			else
				decrease_n_by_perc(previousDisruption, percEqual, destroyScores);

			// cout << "And are updated: (" << destroyScores[0] << ", " << destroyScores[1] << ")" << endl;

		}
		int disruption = select_item(destroyScores);
		previousDisruption = disruption;
		// cout << "Algorithm selected to perform: " << destroyNames[disruption].c_str() << "(Disruption no. " << disruption;
		// cout << ", score " << destroyScores[disruption] << ")" << endl << endl;
		
		// Perform the disruption:
		fitnessBeforeDisruption = fitness;
		destroy_n(disruption, overlapList);

		// Report what happened:
		// output_line
		if (!silent)
		{
			cout << setw(colW) << setprecision(2) << elapsedTime;
			cout << setw(colW) << overlapCount;
			cout << setw(colW) << highest_container_point();
			// cout << setw(colW) << setprecision(5) << workingFitness;
			cout << setw(colW) << setprecision(5) << full_fitness();
			// cout << setw(colW) << setprecision(5) << full_fitness_closest_point(); // debug : remove!
			cout << setw(colW) << destroyNames[disruption] << "(" << destroyInfo.c_str() << ")";
			cout << endl;
			destroyInfo = "-"; // Reset this var
		}
	}
	else
		nofDisruptions = 0;

	if (oscillateUp && UpperBoundH >= upb - 1)
	{
		cout << endl << "Do not go somewhere we have been before!!!!" << endl;
		oscillateUp = false;
		oscillateDown = true;
	}
// # 3b - oscillate_up
	if (oscillateUp)
	{
		cout << endl << "Oscillating up...";
		UpperBoundH++;
		cout << endl;
	}
// # 3c - oscillate_down
	if ( oscillateDown)
	{
		cout << endl << "Oscillating down...";
		strategic_oscillation(lob);
		cout << endl << "Done. New fitness is: " << full_fitness_overlap() << endl;
		noOverlap = false;
		cout << endl;
	}

	// Report information to file, if chosen:
	alnsiters++;
	if (createFile)
	{
		fi << setprecision(5);
		fi << alnsiters << ",";
		fi << elapsedTime << ",";
		fi << overlapCount << ",";
		fi << maxHeightVoxels << ",";
		fi << UpperBoundH << ",";
		fi << fitness << ",";
		fi << vnsiters << ",";
		fi << realMovements << ",";
		for (int i = 0; i < destroyNames.size(); i++)
		{
			fi << destroyScores[i] << ",";
		}
		if (oscillateDown)
			fi << "oscillate_down," << options->decPercentage;
		else if (oscillateUp)
			fi << "oscillate_up,1";
		else if (disrupt)
			fi << "\"" << destroyNames[previousDisruption] << "\"," << destroyInfo.c_str();
		else
			fi << "error,error";
		fi << endl;
	}

	// Now we have reported this, we can set to 0 again:
	realMovements = 0;
} // End of main while

// Post-processing:

// Return to best solution:

// Show results, stats and exit:
(*this) = bestSolution;
cout << endl << "vns_february() finished." << endl;
if (createFile)
	fi.close();

}

void PACKING_LAYOUT::destroy_n(int n, vector<int> &overlapList)
{
	if(n == 0 || n == 3) // SWAP (overlapping, or any):
	{
		bool chooseRandomPiecesToSwap = true; // Just swap any two random pieces 
		// Select two pieces to swap:
		int swapThis = 0;
		int withThis = 1;
		if (chooseRandomPiecesToSwap)
		{
			if (n == 0)
			{
				std::uniform_int_distribution<int> integerDistributionOverlaps(0, overlapList.size() - 1);
				if (overlapList.size() < 2)
					cout << "WARNING - Going into infinite loop..." << endl;
				swapThis = overlapList[integerDistributionOverlaps(randomNumGenerator)];
				withThis = overlapList[integerDistributionOverlaps(randomNumGenerator)];
				//std::uniform_int_distribution<int> integerDistributionAll(0, pieceIndices.size() - 1);
				while (swapThis == withThis)
					withThis = overlapList[integerDistributionOverlaps(randomNumGenerator)];

			}
			else if(n == 2)
			{

				std::uniform_int_distribution<int> integerDistributionOverlaps(0, overlapList.size() - 1);
				swapThis = overlapList[integerDistributionOverlaps(randomNumGenerator)];

				std::uniform_int_distribution<int> integerDistributionAll(0, instance->nPieces - 1);
				withThis = integerDistributionAll(randomNumGenerator);
				//std::uniform_int_distribution<int> integerDistributionAll(0, pieceIndices.size() - 1);
				while ((swapThis == withThis) || (instance->pieces[swapThis]->index == instance->pieces[withThis]->index))
					withThis = integerDistributionAll(randomNumGenerator);
			}
		}

		// cout << endl << "Swapping piece " << swapThis << " with " << withThis << "...";
		// Do this through neighbourhood -- debug : speed (this function can be made standalone)
		NEIGHBOURHOOD justForSwap(50, this, -1); // debug : speed (this can be declared earlier?)
		justForSwap.change_piece_to(swapThis);
		justForSwap.cneig = withThis;
		justForSwap.swap_to(false);
		char aux_char[50];
		sprintf(aux_char, "%d, %d", swapThis, withThis);
		destroyInfo = aux_char;
	}
	else if (n == 1) // Random Insert
	{
		int POINTS_TO_TRY = 10;
		int MAX_POINTS_TO_TRY = POINTS_TO_TRY*10;
		uniform_int_distribution<int> integerDistributionOverlaps(0, overlapList.size() - 1);
		int moveThis = overlapList[integerDistributionOverlaps(randomNumGenerator)];
		double bestBet = -1;
		vector<int> bestPoint = pieceIndices[moveThis]; // If nothing better is found, keep this point
		int bestTrial = -1;
		int generatedPoints = 0;
		for (int trial = 0; trial < POINTS_TO_TRY; trial++)
		{
			vector<int> moveHere(3);
			for (int coord = 0; coord < 3; coord++)
			{
				if (coord == od)
					uniform_int_distribution<int> integerDistributionOverlaps(0, UpperBoundH);
				else
				{
					uniform_int_distribution<int> integerDistributionOverlaps(0, instance->container->gridSize[coord]);
				}

				moveHere[coord] = (integerDistributionOverlaps(randomNumGenerator));

			}
			generatedPoints++;
			if (bboxes_intersect(moveThis, moveThis, moveHere) || !can_p_move_to(moveThis, moveHere[0], moveHere[1], moveHere[2]))
			{
				// We do not accept this point, too close.
				POINTS_TO_TRY++;
				if (POINTS_TO_TRY > MAX_POINTS_TO_TRY)
					break;
			}
			else
			{
				// Test the point:
				pieceIndices[moveThis] = moveHere;
				if (full_fitness(true) > bestBet + TOL)
				{
					bestBet = fitness;
					bestPoint = moveHere;
					bestTrial = trial;
				}
			}
		}
		if (bestBet < 0)
		{
			cout << "WARNING: Destroying the solution by random insert could no find any point in " << generatedPoints << " trials!" << endl;
			cout << "WARNING: Piece is not moved!" << endl;
			bestTrial = -2;
		}
		// if (bestBet > 0)
		pieceIndices[moveThis] = bestPoint;

		full_fitness(true);
		char aux_char[40];
		sprintf(aux_char, "%d -> (%d,%d,%d) trial %d/%d", moveThis, bestPoint[0], bestPoint[1], bestPoint[2], bestTrial, generatedPoints);
		destroyInfo = aux_char;
	}
	else if (n == 2) // Remove and re-insert all overlapping pieces
	{
		// Move all of them to negative numbers:
		for (int i = 0; i < overlapList.size(); i++)
			pieceIndices[overlapList[i]] = { -i*UpperBoundH, -i*UpperBoundH, -i*UpperBoundH };

		vector<int> overlapInserts = overlapList;
		random_shuffle(overlapInserts.begin(), overlapInserts.end());

		// Do the insert:
		for (int pidx = 0; pidx < overlapInserts.size(); pidx++)
		{
			vector<int> bestPoint = { 0, 0, 0 };
			double bestFitness = -1;
			int p = overlapInserts[pidx];
			// Go through all the container points:
			for (int ipos = 0; ipos < instance->maxPositions[instance->pieces[p]->index][0]; ipos++)
			{
				for (int jpos = 0; jpos < instance->maxPositions[instance->pieces[p]->index][1]; jpos++)
				{
					for (int kpos = 0; kpos < instance->maxPositions[instance->pieces[p]->index][2]; kpos++)
					{
						pieceIndices[p] = { ipos, jpos, kpos };
						double testFitness = full_fitness();
						if (testFitness > bestFitness + TOL)
						{
							bestFitness = testFitness;
							bestPoint = pieceIndices[p];
						}
					}
				}

			}
			pieceIndices[p] = bestPoint;
		}

		// What info to put?
		// char aux_char[50];
		// sprintf(aux_char, "%d, %d", swapThis, withThis);
		// destroyInfo = aux_char;

	}
	else
		cout << endl << endl << "WARNING: Destroy operator of type " << n << " is not implemented!!!" << endl << endl;
}





