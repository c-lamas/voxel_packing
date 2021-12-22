
// Include CPLEX:
#ifdef __linux__ 
	// #include </home/carlos/cplex1263/cplex/include/ilcplex/ilocplex.h>
	// #include </home/carlos/cplex1263/cplex/include/ilcplex/ilocplexi.h>
	#include </local/software/cplex/12.6.1/cplex/include/ilcplex/ilocplex.h>
	#include </local/software/cplex/12.6.1/cplex/include/ilcplex/ilocplexi.h>
#else
	#include <ilcplex\ilocplex.h>
#endif
#include "3dlib.h"
#include "3dpack.h"
#include <iostream>
#include <cmath>
#include <algorithm>
void PACKING_LAYOUT::preprocess_box(vector<vector<int>> &boxDelta, vector<int> &orDelta, int Hmax)
{
	bool silent = false;
	vector<int> aux_limits(6);
	boxDelta.assign(instance->nPieces, aux_limits);

	// Define the container limits:
	vector<int> containerLimits(3);
	for (int j = 0; j < 3; j++)
	{
		if (j == instance->container->openDimension)
			containerLimits[j] = Hmax;
		else
			containerLimits[j] = instance->container->gridSize[j];
	}


	// Adjust boxDelta accordingly:
	for (int p = 0; p < instance->nPieces; p++)
	{
		for (int coord = 0; coord < 3; coord++)
		{
			// Calculate how much will be adjusted (equivalent to moving the piece)
			int mp = containerLimits[coord] - instance->pieces[p]->voxel.get_grid_size(coord);
			int adjLow = max(0, 0 - (pieceIndices[p][coord] - orDelta[p]));
			int adjMax = max(0, (pieceIndices[p][coord] + orDelta[p]) - mp);
			boxDelta[p][2 * coord] = max(0, adjLow + pieceIndices[p][coord] - orDelta[p]);
			boxDelta[p][2 * coord + 1] = min(adjMax + pieceIndices[p][coord] + orDelta[p], mp);
		}
	}

}

void PACKING_LAYOUT::preprocess(vector<int> delta, int Hmax)
{
	bool silent = true;

	vector<int> containerLimits(3);

	for (int j = 0; j < 3; j++)
	{
		if (j == instance->container->openDimension)
			containerLimits[j] = Hmax;
		else
			containerLimits[j] = instance->container->gridSize[j];
	}
	if (!silent)
	{
		cout << "Maximum allowed height is: " << Hmax << endl;
		cout << "Container size (voxels) is: ";
		cout_point3(containerLimits);
		cout << endl;
		// Show indices before preprocess:
		cout << "Piece indices are: ";
		show_indices();
	}

	vector< vector<int> > oldIndices = pieceIndices;
	bool change = false;
	for (int i = 0; i < pieceIndices.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			if (delta[i] > pieceIndices[i][j])
			{
				// cout << "Exchanged (min)" << pieceIndices[i][j] << " by " << delta[i] << " piece: " << i << endl;
				pieceIndices[i][j] = delta[i];
				change = true;
			}
			//if (change)
			//{
			//	cout << "(Min) Piece i was at: ";
			//	cout_point3(oldIndices[i]);
			//	cout << "; now is at: ";
			//	cout_point3(pieceIndices[i]);
			//	cout << endl;
			//	change = false;
			//}


			int maxPos = containerLimits[j] - instance->pieces[i]->voxel.get_grid_size(j) - delta[i];
			// cout << "maxPos is " <<  maxPos << endl;
			// cout << "delta[i] is " << delta[i] << endl;
			// cout << " instance->pieces[i]->voxel.get_grid_size(j)is "  << instance->pieces[i]->voxel.get_grid_size(j) << endl;
			if (pieceIndices[i][j] > maxPos)
			{
				change = true;
				pieceIndices[i][j] = maxPos;
				// cout << "Exchanged (max)" << pieceIndices[i][j] << " by " << maxPos << " piece: " << i << endl;
			}

			// pieceCoordinates[i][j] = max((int)pieceCoordinates[i][j], (int)delta[i]);
			// pieceCoordinates[i][j] = min((int)pieceCoordinates[i][j], (int)containerLimits[j] - (int)instance->pieces[i]->voxel.get_grid_size(j) - (int)delta[i]);
		}

		if (change && !silent)
		{
			cout << "Piece i was at: ";
			cout_point3(oldIndices[i]);
			cout << "; now is at: ";
			cout_point3(pieceIndices[i]);
			cout << endl;
			cout << "(Piece dims: ";
			vector<int> sizepoint;
			sizepoint.push_back(instance->pieces[i]->voxel.width);
			sizepoint.push_back(instance->pieces[i]->voxel.height);
			sizepoint.push_back(instance->pieces[i]->voxel.depth);
			cout_point3(sizepoint);
			cout << "; delta was: " << delta[i] << ")" << endl;
			change = false;
		}
	}

	coordinates_from_indices(resolution);
	if (!silent)
	{
		serialise("solution_compaction_initial.psol");
		// Show indices after preprocess:
		cout << "After preproc: " << endl;
		show_indices();
	}

}

void PACKING_LAYOUT::iterated_compaction(int maxIters, int deltaZero, int oportunities, int mode, double maxTime)
{
		int H0 = highest_container_point();
		cout << "Iterated compaction (starting from H = " << H0 << ")" << endl;
		vector<int> delta;
		bool modify;
		int wastedOps = 0; // Number of times we increase the delta:

		for (int pt = 0; pt < instance->nPieces; pt++)
			delta.push_back(deltaZero);

		for (int ii = 0; ii < maxIters; ii++)
		{
			int H = highest_container_point();
			if (H <= instance->voxelLowerBound)
			{
				cout << "The solution is optimal! (matched lower bound)" << endl;
				break;
			}
			vector< vector <int> > oldIndices = pieceIndices;
			vector<vector<int>> boxDelta;
			preprocess_box(boxDelta, delta, H);
			// preprocess(delta, H);
			init_cplex(boxDelta, maxTime, modify, H, oldIndices);
			int newH = highest_container_point();

			if (!modify)
			{
				cout << "Finished iteration " << ii << " with H = " << newH << endl;

				std::stringstream sstm;
				sstm << "c_step_" << ii << "_H_" << newH << ".psol";
				string bestsolname = sstm.str();
				serialise(bestsolname.c_str());

				if (wastedOps >= oportunities)
				{
					cout << "Finished iterated compaction." << endl;
					cout << "Best H was: " << newH << endl;
					return;
					std::stringstream sstm;
					sstm << "final_c_step_" << ii << "_H_" << newH << ".psol";
					string bestsolname = sstm.str();
					serialise(bestsolname.c_str());
				}
				else
				{
					cout << endl << endl << "Solution did not improve (H = " << newH << ")" << endl;
					cout << "Iterations with no improvement: " << wastedOps << endl;

					wastedOps++;
					for (int pt = 0; pt < instance->nPieces; pt++)
						delta[pt]++;

					cout << "Delta increased." << endl;
				}
			}
			else
			{
				wastedOps = 0;
			}
		}
}

void PACKING_LAYOUT::expand_delta(vector<vector<int>> &boxDelta, vector<int> &orDelta)
{
	// Remove any content from boxDelta:
	vector<int> aux_limits(6);
	boxDelta.assign(instance->nPieces, aux_limits);

	for (int p = 0; p < instance->nPieces; p++)
	{
		for (int coord = 0; coord < 3; coord++)
		{
			boxDelta[p][2 * coord] = max(0,pieceIndices[p][coord] - orDelta[p]);
			// This one does not need to be adjusted if it comes from preprocess
			boxDelta[p][2 * coord + 1] = pieceIndices[p][coord] + orDelta[p]; 
		}
	}
	
}

void PACKING_LAYOUT::init_cplex(vector<int> delta, double time_limit, bool &modify, int &Hmax, vector<vector<int> > oldIndices)
{
	init_cplex(delta, time_limit, modify, Hmax, oldIndices, false);
}

void PACKING_LAYOUT::init_cplex(vector<vector<int> > boxDelta, double time_limit, bool &modify, int &Hmax, vector<vector<int> > oldIndices)
{
	init_cplex(boxDelta, time_limit, modify, Hmax, oldIndices, false);
}

void PACKING_LAYOUT::init_cplex(vector<int> delta, double time_limit, bool &modify, int &H, vector< vector <int> > oldIndices, bool noObj)
{
	// Old format delta...
	vector<vector<int>> boxDelta;
	expand_delta(boxDelta, delta);
	init_cplex(boxDelta, time_limit, modify, H, oldIndices, noObj);
}

void PACKING_LAYOUT::init_cplex(vector<vector<int> > boxDelta, double time_limit, bool &modify, int &Hmax, vector<vector<int> > oldIndices, bool noObj)
{
	if ((instance->voxelLowerBound >= Hmax) && is_feasible_voxel())
	{
		cout << "The solution provided is optimal! don't bother to solve!" << endl;
		serialise("optimal_solution.psol");
		exit(0);
	}
	// Start compaction:

	// delta: numer of cells allowed to move each one of the pieces (starting on 0)
	// Assumin we minimize k

	/// From class
	IloEnv env;
	IloModel model;
	IloCplex cplex;
	IloNumVarArray *vars;
	IloRangeArray *rngs;
	IloObjective *obj; // By default: minimize

	// track position of the variables
	int ****X_pos;
	int H_pos;
	int ntp; //number of types
	//// above is From class
	IloEnv environment;
	IloModel mod(environment);
	IloCplex cpl(mod);

	IloNumVarArray vars2(environment);
	IloRangeArray rngs2(environment);
	IloObjective obj2(environment); // By default: minimize

	env = environment;
	model = mod;
	cplex = cpl;
	vars = &vars2;
	rngs = &rngs2;
	obj = &obj2;

	// Save the open and closed dimensions:
	int od = instance->container->openDimension;
	int cd1 = (od + 1) % 3;
	int cd2 = (od + 2) % 3;

	bool silent = true;
	if (silent)
	{
		cplex.setParam(IloCplex::NumericalEmphasis, true);
		cplex.setParam(IloCplex::MIPDisplay, 0);
	}
	else
		cout << "Max. allowed height: " << Hmax << endl;


	// if (time_limit > 0) // Time limit negative disables this
	cplex.setParam(IloCplex::TiLim, time_limit);

	cplex.setParam(IloCplex::WriteLevel, 3);
	cplex.setParam(IloCplex::MIPDisplay, 3);
	// cplex.setParam(IloCplex::MIPInterval, 1);
	// cplex.setParam(IloCplex::CutsFactor, -1); // Desactivar cortes cplex

	ntp = instance->nPieces;
	X_pos = new int ***[ntp]; // piece types
	int bestPosH = 0;

	for (int p = 0; p < ntp; p++)
	{
		if (ignorePiece[p])
			continue;

		X_pos[p] = new int**[boxDelta[p][1] - boxDelta[p][0] + 1];
		for (int i = boxDelta[p][0]; i < boxDelta[p][1] + 1; i++)
		{
			X_pos[p][i - boxDelta[p][0]] = new int*[boxDelta[p][3] - boxDelta[p][2] + 1];
			for (int j = boxDelta[p][2]; j < boxDelta[p][3] + 1; j++)
			{
				X_pos[p][i - boxDelta[p][0]][j - boxDelta[p][2]] = new int[boxDelta[p][5] - boxDelta[p][4] + 1];
				for (int k = boxDelta[p][4]; k < boxDelta[p][5] + 1; k++)
				{
					// Add variable:
					char nnn[50];
					// sprintf(nnn, "X_p%d_%d_%d_%d", p, int(i + pieceIndices[p][0] - delta[p]), int(j + pieceIndices[p][1] - delta[p]), int(k + pieceIndices[p][2] - delta[p]) );
					sprintf(nnn, "X_%d_%d_%d_%d", p, i, j, k);

					IloNumVar var(env, 0, 1, ILOBOOL, nnn);
					(*vars).add(var);
					model.add(var);

					X_pos[p][i - boxDelta[p][0]][j - boxDelta[p][2]][k - boxDelta[p][4]] = vars->getSize() - 1;
					// X_pos[p][i][j][k] = -1;

				} // loop of k
			} // loop of j
		} // loop of i


		// Find the minimum H we can get
		int minHofP = boxDelta[p][2*od] + instance->pieces[p]->voxel.get_grid_size(od);
		// cout << "Min H for p = " << p << " is H = " << minHofP << "(" << pieceCoordinates[p][od] << " + " <<
			// instance->pieces[p]->voxel.get_grid_size(od) << " - " << delta[p] << ")" << endl;
		if (minHofP > bestPosH)
			bestPosH = minHofP;

	} // end of loop for each piece

	if (!silent)
	{
		if (noObj)
			cout << "Solving the problem with objective 0" << endl;
		else
			cout << "Solving the problem with the objective of minimising H." << endl;
			cout << "The best possible H we can get is: " << bestPosH  << "(current is H = " << Hmax << ")" << endl;
	}

	//DEFINING H (Objective Function):
	char nH[30];

	if (!noObj)
	{
		sprintf(nH, "H");
		IloNumVar var(env, bestPosH, Hmax, ILOINT, nH);
		(*vars).add(var);
		model.add(var);
		H_pos = vars->getSize() - 1;
		
		if (!silent)
			cout << "H was given the range [" << bestPosH << ", " << Hmax << "]" << endl;
	}


	// OBJECTIVE FUNCTION
	IloExpr expobj(env);

	if (noObj)
		expobj += 0;
	else
		expobj += (*vars)[H_pos];

	obj->setExpr(expobj);
	model.add(*obj);
	//obj->setSense(IloObjective::Maximize);
	expobj.end();


	// SUBJECT TO...(CONSTRAINTS)
	
	//SATISFYING H:
	if (!noObj)
	{
		for (int p = 0; p < ntp ; p++)
		{
			if (ignorePiece[p])
				continue;

			//////IMPROVE///////////////////////////////////////
			int pHeight = instance->pieces[p]->voxel.get_grid_size(od);
			for (int i = boxDelta[p][0]; i < boxDelta[p][1] + 1; i++)
			{
				for (int j = boxDelta[p][2]; j < boxDelta[p][3] + 1; j++)
				{
					for (int k = boxDelta[p][4]; k < boxDelta[p][5] + 1; k++)
					{
						IloExpr Lexpr(env);
						int odval;
						switch (od)
						{
						case 0:
							odval = i;
						case 1:
							odval = j;
						case 2:
							odval = k;
						}
						// Lexpr += (*vars)[H_pos] - ((part1Expr + odval)* (*vars)[X_pos[p][i][j][k]]);
						Lexpr += (*vars)[H_pos] - ((pHeight + odval)* (*vars)[X_pos[p][i - boxDelta[p][0]][j - boxDelta[p][2]][k - boxDelta[p][4]]]);
						char Lrest_name[60];
						sprintf(Lrest_name, "HCONST_%d_in_%d_%d_%d", p, i, j, k);
						IloRange Lrest(env, 0, Lexpr, IloInfinity, Lrest_name);
						(*rngs).add(Lrest);
						model.add(Lrest);
						Lexpr.end();
					}
				}
			}
		}
	}

	//SATISFYING DEMAND:
	for (int p = 0; p < ntp; p++)
	{
		if (ignorePiece[p])
			continue;
		IloExpr Lexpr(env);

		for (int i = boxDelta[p][0]; i < boxDelta[p][1] + 1; i++)
		{
			for (int j = boxDelta[p][2]; j < boxDelta[p][3] + 1; j++)
			{
				for (int k = boxDelta[p][4]; k < boxDelta[p][5] + 1; k++)
				{
					// Lexpr += (*vars)[X_pos[p][i][j][k]];
					Lexpr += (*vars)[X_pos[p][i - boxDelta[p][0]][j - boxDelta[p][2]][k - boxDelta[p][4]]];
				}
			}
		}

		char Lrest_name[60];
		sprintf(Lrest_name, "Demand_of_%d", p);
		IloRange Lrest(env, 1, Lexpr, 1, Lrest_name);
		(*rngs).add(Lrest);
		model.add(Lrest);
		Lexpr.end();
	}

	// AVOIDING OVERLAP (NFV CONSTRAINTS)
	if (!silent)
		cout << "Adding constraints for piece ";
	for (int p = 0; p < ntp; p++)
	{
		if (ignorePiece[p])
			continue;

		if (! silent)
			cout << p << ", ";

			for (int q = p + 1; q < ntp; q++)
			{

			if (ignorePiece[q])
				continue;

			// If both pieces are fixed, we asume they cannot overlap!
			if (!pieceCanMove[p] && !pieceCanMove[q])
				continue;

				// If bounding box too far -> continue

				//vector<int> posFinalP(3);
				//for (int coord = 0; coord < 3; coord++)
				//	posFinalP[coord] = pieceIndices[p][coord] - delta[p] - 1;

				////int posFinalQ[3];
				//vector<int> posFinalQ(3);
				//for (int coord = 0; coord < 3; coord++)
				//	posFinalQ[coord] = pieceIndices[q][coord] - delta[q] - 1;
				// Loop through all the possible positions of p
				for (int ip = boxDelta[p][0]; ip < boxDelta[p][1] + 1; ip++)
				{
					for (int jp = boxDelta[p][2]; jp < boxDelta[p][3] + 1; jp++)
					{
						for (int kp = boxDelta[p][4]; kp < boxDelta[p][5] + 1; kp++)
						{
							// Check if bounding boxes intersect

							// Coord P: pieceIndices[solution_coord][instance->container->openDimension]
							vector<int> posFinalP(3);
						    //for (int coord = 0; coord < 3; coord++)
							posFinalP[0] = ip;// pieceIndices[p][0] - delta[p] + ip;
							posFinalP[1] = jp;// pieceIndices[p][1] - delta[p] + jp;
							posFinalP[2] = kp;// pieceIndices[p][2] - delta[p] + kp;
						
							// Start the constraint:
							IloExpr Lexpr(env);
							char Lrest_name[144];
							sprintf(Lrest_name, "NFV_of_p%d_in_%d_%d_%d__q%d", p, ip, jp, kp, q);
							// Lexpr += (*vars)[X_pos[p][ip][jp][kp]];
							Lexpr += (*vars)[X_pos[p][ip - boxDelta[p][0]][jp - boxDelta[p][2]][kp - boxDelta[p][4]]];

							// if (p == 0 && q == 1 && posFinalP[0] == 0 && posFinalP[1] == 0 && posFinalP[2] == 0
								// && posFinalQ[0] == 14 && posFinalQ[1] == 0 && posFinalQ[2] == 0)
								// cout << "AQUI" << endl;
						
							// NFV * nfvpq = allNFV.getNFV(instance->pieces[p]->index, instance->pieces[q]->index);
							// Loop through all the possible positions of q
							bool foundAny = false;
							if (pieceCanMove[p] && pieceCanMove[q])
							{
								for (int iq = boxDelta[q][0]; iq < boxDelta[q][1] + 1; iq++)
								{
									for (int jq = boxDelta[q][2]; jq < boxDelta[q][3] + 1; jq++)
									{
										for (int kq = boxDelta[q][4]; kq < boxDelta[q][5] + 1; kq++)
										{
											//posFinalQ[2]++;
											vector<int> posFinalQ(3);
											//for (int coord = 0; coord < 3; coord++)
											posFinalQ[0] = iq; // pieceIndices[q][0] - delta[q] + iq;
											posFinalQ[1] = jq; //pieceIndices[q][1] - delta[q] + jq;
											posFinalQ[2] = kq; //pieceIndices[q][2] - delta[q] + kq;
											if (bboxes_intersectOR(instance->pieces[q]->index, instance->pieces[p]->index, posFinalQ, posFinalP))
											{
												if (!allNFV->nfv_has_point(instance->pieces[q]->index, posFinalQ, instance->pieces[p]->index, posFinalP))
												{
													Lexpr += (*vars)[X_pos[q][iq - boxDelta[q][0]][jq - boxDelta[q][2]][kq - boxDelta[q][4]]];
													foundAny = true;
												}
											}
											
										} // i of q
									} // j of q
								} // k of q
							}
							else
							{
								for (int iq = boxDelta[q][0]; iq < boxDelta[q][1] + 1; iq++)
								{
									for (int jq = boxDelta[q][2]; jq < boxDelta[q][3] + 1; jq++)
									{
										for (int kq = boxDelta[q][4]; kq < boxDelta[q][5] + 1; kq++)
										{
											vector<int> posFinalQ(3);
											posFinalQ[0] = iq; // pieceIndices[q][0] - delta[q] + iq;
											posFinalQ[1] = jq; //pieceIndices[q][1] - delta[q] + jq;
											posFinalQ[2] = kq; //pieceIndices[q][2] - delta[q] + kq;
											if (bboxes_intersectOR(instance->pieces[q]->index, instance->pieces[p]->index, posFinalQ, posFinalP))
											{
												if (!allNFV->nfv_has_point(instance->pieces[q]->index, posFinalQ, instance->pieces[p]->index, posFinalP))
												{
													if (!pieceCanMove[p])
														(*vars)[X_pos[q][iq - boxDelta[q][0]][jq - boxDelta[q][2]][kq - boxDelta[q][4]]].setBounds(0, 0);
													else if (!pieceCanMove[q])
														(*vars)[X_pos[p][ip - boxDelta[p][0]][jp - boxDelta[p][2]][kp - boxDelta[p][4]]].setBounds(0, 0);

												}
											}
											
										} // i of q
									} // j of q
								} // k of q
							}
								
							if (foundAny) // Do not add restriction if there is no overlap!
							{
								IloRange Lrest(env, -IloInfinity, Lexpr, 1, Lrest_name);
								(*rngs).add(Lrest);
								model.add(Lrest);
							}

							Lexpr.end();
							
							} // i of p
						} // j of p
					} // k of p


			} // piece type q

	} // piece type p

	if (!silent)
		cout << " - done." << endl;
	//Fixing incompatible variables:
	for (int p = 0; p < ntp; p++)
	{
		if (ignorePiece[p])
			continue;

		bool piecePproblems = false;
		int posv[3];
		for (int ip = boxDelta[p][0]; ip < boxDelta[p][1] + 1; ip++)
		{
			posv[0] = ip;
			for (int jp = boxDelta[p][2]; jp < boxDelta[p][3] + 1; jp++)
			{
				posv[1] = jp;
				for (int kp = boxDelta[p][4]; kp < boxDelta[p][5] + 1; kp++)
				{
					posv[2] = kp;

					// check bounding box inside container
					if (!(instance->pieces[p]->voxel.get_grid_size(od) + posv[od] <= Hmax && posv[od] >= 0 &&
						instance->pieces[p]->voxel.get_grid_size(cd1) + posv[cd1] <= instance->container->gridSize[cd1] && posv[cd1] >= 0 &&
						instance->pieces[p]->voxel.get_grid_size(cd2) + posv[cd2] <= instance->container->gridSize[cd2] && posv[cd2] >= 0))
					{
						// cout << "WARNING: X_" << p << "_(" << ip << ", " << jp << ", " << kp << ") was set to 0; bad preprocess?" << endl;
						// (*vars)[X_pos[p][ip][jp][kp]].setBounds(0, 0);
						(*vars)[X_pos[p][ip - boxDelta[p][0]][jp - boxDelta[p][2]][kp - boxDelta[p][4]]].setBounds(0, 0);
						piecePproblems = true;
					}
				}
			}
		}
		if (piecePproblems)
		{
			cout << endl << "WARNING: Piece " << p << " was given incorrect bounds.";
			cout << endl << "Piece dims: ";
			cout_point3(instance->pieces[p]->voxel.get_grid_size(0), instance->pieces[p]->voxel.get_grid_size(1), instance->pieces[p]->voxel.get_grid_size(2));
			cout << "Piece position: ";
			cout_point3(pieceIndices[p]);
			report_piece(p);
			cout << "Delta box limits: [" << boxDelta[p][0] << ", " << boxDelta[p][1] << "], [" << boxDelta[p][2] << ", " << boxDelta[p][3] << "], [" << boxDelta[p][4] << ", " << boxDelta[p][5] << "]" << endl;
			cout << "Container: ";
			cout_point3(instance->container->gridSize[0], instance->container->gridSize[1], instance->container->gridSize[2]);
			cout << "Allowed H = " << Hmax << endl;
			cout << endl;
		}
	}


	// if (ipFormulation)
		// cplex.exportModel("modelo.lp");
	// else
	// cplex.exportModel("modelo.lp");
	
	if (!silent)
		cout << "Set up finished, solving...";

	// cplex.setDefaults();
	cplex.exportModel("packing_model.lp");
	if (noObj)
	{
		cout << "Emphasis is to find a feasible solution only!" << endl;
		cplex.setParam(IloCplex::MIPEmphasis, 1);
	}
	IloBool status = cplex.solve();
	cout << "Model solved!" << endl;

	if (!silent)
		cout << "done." << endl;

	int cplexH = -1;

	if (status)
	{
		// Retrieve solution
		methodString = "CPLEX Compaction 0.2 IP";

		cplexH = cplex.getObjValue();

		if (!silent)
			cplex.writeSolution("Solution_IP.txt");

		IloNumArray aX(env, vars->getSize());
		cplex.getValues(aX, (*vars));
		for (int p = 0; p < ntp; p++)
		{
			if (ignorePiece[p])
				continue;
			for (int i = boxDelta[p][0]; i < boxDelta[p][1] + 1; i++)
			{
				for (int j = boxDelta[p][2]; j < boxDelta[p][3] + 1; j++)
				{
					for (int k = boxDelta[p][4]; k < boxDelta[p][5] + 1; k++)
					{
						if (aX[X_pos[p][i - boxDelta[p][0]][j - boxDelta[p][2]][k - boxDelta[p][4]]] > 0.5)
						{
							pieceIndices[p][0] = i; //  -delta[p];
							pieceIndices[p][1] = j; //  - delta[p];
							pieceIndices[p][2] = k; //  - delta[p];

							// Modify i and j to break the loops:
							i =  boxDelta[p][1] + 1;
							j = boxDelta[p][3] + 1;
							break;
						}
					}
				}
			}
		}

		aX.end();
		cout << "Finished retrieving indices from variables." << endl;

		// Save the solution:
		if (!silent)
			serialise("solution_compaction.psol");
	}
	else
	{
		cout << endl << "WARNING: CPLEX couldn't solve!" << endl;
		// Reset indices:
		pieceIndices = oldIndices;
		coordinates_from_indices(resolution);
		// cplex.exportModel("infeasible_model.lp");
		// exit(-1);
		// return;
	}

	//cout << "Cleaning up X_pos..." << endl;
	delete[] X_pos;
/*	cout << "Done." << endl;
	cout << "Cleaning up cplex object..." << endl*/;
	cplex.end();
/*	cout << "Done." << endl;
	cout << "Cleaning up model object..." << endl*/;
	model.end();
	//cout << "Done." << endl;
	//cout << "Cleaning up environment..." << endl;
	env.end();
	//cout << "Done." << endl;

	coordinates_from_indices(instance->pieces[0]->voxel.resolution);
	int solH = highest_container_point();

	if (!silent)
		cout << "H = " << solH << ", cplex H = " << cplexH << endl;

	if ((solH < Hmax) && status)
		modify = true;
	else if (status && noObj)
		modify = true;
	else
	{
		modify = false;
	//	pieceIndices = oldIndices;
	//	coordinates_from_indices(resolution);
		if (!silent)
			cout << endl << "WARNING: H was not modified, returning to old inidices!" << endl;
	}

}
