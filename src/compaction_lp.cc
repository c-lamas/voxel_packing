//
//// Include CPLEX:
//#ifdef __linux__ 
//	#include </local/software/cplex/12.6.1/cplex/include/ilcplex/ilocplex.h>
//	#include </local/software/cplex/12.6.1/cplex/include/ilcplex/ilocplexi.h>
//#else
//	#include <ilcplex\ilocplex.h>
//#endif
//#include "3dlib.h"
//#include "3dpack.h"
//#include <iostream>
//#include <cmath>
//#include <algorithm>
//
//void PACKING_LAYOUT::modelo_lp(vector<vector<int> > boxDelta, double time_limit, bool &modify, int &Hmax, vector<vector<int> > oldIndices, bool noObj)
//{
//	// Start compaction:
//
//	// delta: numer of cells allowed to move each one of the pieces (starting on 0)
//	// Assumin we minimize k
//
//	/// From class
//	IloEnv env;
//	IloModel model(env);
//	IloCplex cplex(env);
//	IloNumVarArray *vars;
//	IloRangeArray *rngs;
//	IloObjective *obj; // By default: minimize
//
//	int np = instance->nPieces; //number of pieces
//
//	// track position of the variables
//	// int **pos_coord;
//	vector<int> pos_coord_aux;
//	pos_coord_aux.assign(3, -1);
//	vector<vector <int > > pos_coord;
//	pos_coord.assign(np, pos_coord_aux);
//
//
//	int H_pos;
//
//	//// above is From class
//	//IloEnv environment;
//	//IloModel mod(environment);
//	//IloCplex cpl(mod);
//
//	IloNumVarArray vars2(env);
//	IloRangeArray rngs2(env);
//	IloObjective obj2(env); // By default: minimize
//
//	//env = environment;
//	//model = mod;
//	//cplex = cpl;
//	vars = &vars2;
//	rngs = &rngs2;
//	obj = &obj2;
//
//	// Save the open and closed dimensions:
//	int od = instance->container->openDimension;
//	int cd1 = (od + 1) % 3;
//	int cd2 = (od + 2) % 3;
//
//	bool silent = true;
//	if (silent)
//	{
//		cplex.setParam(IloCplex::NumericalEmphasis, true);
//		cplex.setParam(IloCplex::MIPDisplay, 0);
//	}
//	else
//		cout << "Max. allowed height: " << Hmax << endl;
//
//	cplex.setParam(IloCplex::TiLim, time_limit);
//	cplex.setParam(IloCplex::WriteLevel, 3);
//
//	np = instance->nPieces;
//	// pos_coord = new int *[np]; // piece types
//	int bestPosH = 0;
//
//	for (int p = 0; p < np; p++)
//	{
//		// if (ignorePiece[p])
//		// continue;
//		for (int coord = 0; coord < 3; coord++)
//		{
//			// Add variable:
//			char nnn[50];
//			// sprintf(nnn, "X_p%d_%d_%d_%d", p, int(i + pieceIndices[p][0] - delta[p]), int(j + pieceIndices[p][1] - delta[p]), int(k + pieceIndices[p][2] - delta[p]) );
//			sprintf(nnn, "P_%d_coord_%d", p, coord);
//			IloNumVar var(env, 0, IloInfinity, ILOFLOAT, nnn);
//			(*vars).add(var);
//			model.add(var);
//
//			pos_coord[p][coord] = vars->getSize() - 1;
//
//		}
//
//
//		// Find the minimum H we can get
//		// int minHofP = boxDelta[p][2*od] + instance->pieces[p]->voxel.get_grid_size(od);
//		// cout << "Min H for p = " << p << " is H = " << minHofP << "(" << pieceCoordinates[p][od] << " + " <<
//			// instance->pieces[p]->voxel.get_grid_size(od) << " - " << delta[p] << ")" << endl;
//		// if (minHofP > bestPosH)
//			// bestPosH = minHofP;
//
//	} // end of loop for each piece
//
//
//	//DEFINING H (Objective Function):
//	char nH[30];
//
//	sprintf(nH, "H");
//	IloNumVar var(env, 0, Hmax, ILOINT, nH);
//	(*vars).add(var);
//	model.add(var);
//	H_pos = vars->getSize() - 1;
//
//	// OBJECTIVE FUNCTION
//	IloExpr expobj(env);
//
//	expobj += (*vars)[H_pos];
//
//	obj->setExpr(expobj);
//	model.add(*obj);
//	//obj->setSense(IloObjective::Maximize);
//	expobj.end();
//
//
//	// SUBJECT TO...(CONSTRAINTS)
//	
//	//SATISFYING H:
//	for (int p = 0; p < np ; p++)
//	{
//		if (ignorePiece[p])
//			continue;
//
//		IloExpr Lexpr(env);
//		Lexpr += (*vars)[H_pos] - instance->pieces[p]->voxel.get_grid_size(od) - (*vars)[pos_coord[p][od]];
//		char Lrest_name[60];
//		sprintf(Lrest_name, "H_limit_%d", p);
//		IloRange Lrest(env, 0, Lexpr, IloInfinity, Lrest_name);
//		(*rngs).add(Lrest);
//		model.add(Lrest);
//		Lexpr.end();
//	}
//
//
//	// AVOIDING OVERLAP (NFV CONSTRAINTS)
//	if (!silent)
//		cout << "Adding constraints for piece ";
//	for (int p = 0; p < np; p++)
//	{
//		if (ignorePiece[p])
//			continue;
//
//		if (! silent)
//			cout << p << ", ";
//
//			for (int q = p + 1; q < ntp; q++)
//			{
//
//			if (ignorePiece[q])
//				continue;
//
//			// If both pieces are fixed, we asume they cannot overlap!
//			if (!pieceCanMove[p] && !pieceCanMove[q])
//				continue;
//
//				// If bounding box too far -> continue
//
//				//vector<int> posFinalP(3);
//				//for (int coord = 0; coord < 3; coord++)
//				//	posFinalP[coord] = pieceIndices[p][coord] - delta[p] - 1;
//
//				////int posFinalQ[3];
//				//vector<int> posFinalQ(3);
//				//for (int coord = 0; coord < 3; coord++)
//				//	posFinalQ[coord] = pieceIndices[q][coord] - delta[q] - 1;
//				// Loop through all the possible positions of p
//				for (int ip = boxDelta[p][0]; ip < boxDelta[p][1] + 1; ip++)
//				{
//					for (int jp = boxDelta[p][2]; jp < boxDelta[p][3] + 1; jp++)
//					{
//						for (int kp = boxDelta[p][4]; kp < boxDelta[p][5] + 1; kp++)
//						{
//							// Check if bounding boxes intersect
//
//							// Coord P: pieceIndices[solution_coord][instance->container->openDimension]
//							vector<int> posFinalP(3);
//						    //for (int coord = 0; coord < 3; coord++)
//							posFinalP[0] = ip;// pieceIndices[p][0] - delta[p] + ip;
//							posFinalP[1] = jp;// pieceIndices[p][1] - delta[p] + jp;
//							posFinalP[2] = kp;// pieceIndices[p][2] - delta[p] + kp;
//						
//							// Start the constraint:
//							IloExpr Lexpr(env);
//							char Lrest_name[144];
//							sprintf(Lrest_name, "NFV_of_p%d_in_%d_%d_%d__q%d", p, ip, jp, kp, q);
//							// Lexpr += (*vars)[X_pos[p][ip][jp][kp]];
//							Lexpr += (*vars)[X_pos[p][ip - boxDelta[p][0]][jp - boxDelta[p][2]][kp - boxDelta[p][4]]];
//
//							// if (p == 0 && q == 1 && posFinalP[0] == 0 && posFinalP[1] == 0 && posFinalP[2] == 0
//								// && posFinalQ[0] == 14 && posFinalQ[1] == 0 && posFinalQ[2] == 0)
//								// cout << "AQUI" << endl;
//						
//							// NFV * nfvpq = allNFV.getNFV(instance->pieces[p]->index, instance->pieces[q]->index);
//							// Loop through all the possible positions of q
//							bool foundAny = false;
//							if (pieceCanMove[p] && pieceCanMove[q])
//							{
//								for (int iq = boxDelta[q][0]; iq < boxDelta[q][1] + 1; iq++)
//								{
//									for (int jq = boxDelta[q][2]; jq < boxDelta[q][3] + 1; jq++)
//									{
//										for (int kq = boxDelta[q][4]; kq < boxDelta[q][5] + 1; kq++)
//										{
//											//posFinalQ[2]++;
//											vector<int> posFinalQ(3);
//											//for (int coord = 0; coord < 3; coord++)
//											posFinalQ[0] = iq; // pieceIndices[q][0] - delta[q] + iq;
//											posFinalQ[1] = jq; //pieceIndices[q][1] - delta[q] + jq;
//											posFinalQ[2] = kq; //pieceIndices[q][2] - delta[q] + kq;
//											if (bboxes_intersectOR(instance->pieces[q]->index, instance->pieces[p]->index, posFinalQ, posFinalP))
//											{
//												if (!allNFV.nfv_has_point(instance->pieces[q]->index, posFinalQ, instance->pieces[p]->index, posFinalP))
//												{
//													Lexpr += (*vars)[X_pos[q][iq - boxDelta[q][0]][jq - boxDelta[q][2]][kq - boxDelta[q][4]]];
//													foundAny = true;
//												}
//											}
//											
//										} // i of q
//									} // j of q
//								} // k of q
//							}
//							else
//							{
//								for (int iq = boxDelta[q][0]; iq < boxDelta[q][1] + 1; iq++)
//								{
//									for (int jq = boxDelta[q][2]; jq < boxDelta[q][3] + 1; jq++)
//									{
//										for (int kq = boxDelta[q][4]; kq < boxDelta[q][5] + 1; kq++)
//										{
//											vector<int> posFinalQ(3);
//											posFinalQ[0] = iq; // pieceIndices[q][0] - delta[q] + iq;
//											posFinalQ[1] = jq; //pieceIndices[q][1] - delta[q] + jq;
//											posFinalQ[2] = kq; //pieceIndices[q][2] - delta[q] + kq;
//											if (bboxes_intersectOR(instance->pieces[q]->index, instance->pieces[p]->index, posFinalQ, posFinalP))
//											{
//												if (!allNFV.nfv_has_point(instance->pieces[q]->index, posFinalQ, instance->pieces[p]->index, posFinalP))
//												{
//													if (!pieceCanMove[p])
//														(*vars)[X_pos[q][iq - boxDelta[q][0]][jq - boxDelta[q][2]][kq - boxDelta[q][4]]].setBounds(0, 0);
//													else if (!pieceCanMove[q])
//														(*vars)[X_pos[p][ip - boxDelta[p][0]][jp - boxDelta[p][2]][kp - boxDelta[p][4]]].setBounds(0, 0);
//
//												}
//											}
//											
//										} // i of q
//									} // j of q
//								} // k of q
//							}
//								
//							if (foundAny) // Do not add restriction if there is no overlap!
//							{
//								IloRange Lrest(env, -IloInfinity, Lexpr, 1, Lrest_name);
//								(*rngs).add(Lrest);
//								model.add(Lrest);
//							}
//
//							Lexpr.end();
//							
//							} // i of p
//						} // j of p
//					} // k of p
//
//
//			} // piece type q
//
//	} // piece type p
//
//	if (!silent)
//		cout << " - done." << endl;
//	//Fixing incompatible variables:
//	for (int p = 0; p < ntp; p++)
//	{
//		if (ignorePiece[p])
//			continue;
//
//		bool piecePproblems = false;
//		int posv[3];
//		for (int ip = boxDelta[p][0]; ip < boxDelta[p][1] + 1; ip++)
//		{
//			posv[0] = ip;
//			for (int jp = boxDelta[p][2]; jp < boxDelta[p][3] + 1; jp++)
//			{
//				posv[1] = jp;
//				for (int kp = boxDelta[p][4]; kp < boxDelta[p][5] + 1; kp++)
//				{
//					posv[2] = kp;
//
//					// check bounding box inside container
//					if (!(instance->pieces[p]->voxel.get_grid_size(od) + posv[od] <= Hmax && posv[od] >= 0 &&
//						instance->pieces[p]->voxel.get_grid_size(cd1) + posv[cd1] <= instance->container->gridSize[cd1] && posv[cd1] >= 0 &&
//						instance->pieces[p]->voxel.get_grid_size(cd2) + posv[cd2] <= instance->container->gridSize[cd2] && posv[cd2] >= 0))
//					{
//						// cout << "WARNING: X_" << p << "_(" << ip << ", " << jp << ", " << kp << ") was set to 0; bad preprocess?" << endl;
//						// (*vars)[X_pos[p][ip][jp][kp]].setBounds(0, 0);
//						(*vars)[X_pos[p][ip - boxDelta[p][0]][jp - boxDelta[p][2]][kp - boxDelta[p][4]]].setBounds(0, 0);
//						piecePproblems = true;
//					}
//				}
//			}
//		}
//		if (piecePproblems)
//		{
//			cout << endl << "WARNING: Piece " << p << " was given incorrect bounds.";
//			cout << endl << "Piece dims: ";
//			cout_point3(instance->pieces[p]->voxel.get_grid_size(0), instance->pieces[p]->voxel.get_grid_size(1), instance->pieces[p]->voxel.get_grid_size(2));
//			cout << "Delta box limits: [" << boxDelta[p][0] << ", " << boxDelta[p][1] << "], [" << boxDelta[p][2] << ", " << boxDelta[p][3] << "], [" << boxDelta[p][4] << ", " << boxDelta[p][5] << "]" << endl;
//			cout << "Container: ";
//			cout_point3(instance->container->gridSize[0], instance->container->gridSize[1], instance->container->gridSize[2]);
//			cout << "Allowed H = " << Hmax << endl;
//			cout << endl;
//		}
//	}
//
//
//	// if (ipFormulation)
//		// cplex.exportModel("modelo.lp");
//	// else
//	// cplex.exportModel("modelo.lp");
//	
//	if (!silent)
//		cout << "Set up finished, solving...";
//
//	// cplex.setDefaults();
//	IloBool status = cplex.solve();
//
//
//	if (!silent)
//		cout << "done." << endl;
//
//	int cplexH = -1;
//
//	if (status)
//	{
//		// Retrieve solution
//		methodString = "CPLEX Compaction 0.2 IP";
//
//		cplexH = cplex.getObjValue();
//
//		if (!silent)
//			cplex.writeSolution("Solution_IP.txt");
//
//		IloNumArray aX(env, vars->getSize());
//		cplex.getValues(aX, (*vars));
//		for (int p = 0; p < ntp; p++)
//		{
//			if (ignorePiece[p])
//				continue;
//			for (int i = boxDelta[p][0]; i < boxDelta[p][1] + 1; i++)
//			{
//				for (int j = boxDelta[p][2]; j < boxDelta[p][3] + 1; j++)
//				{
//					for (int k = boxDelta[p][4]; k < boxDelta[p][5] + 1; k++)
//					{
//						if (aX[X_pos[p][i - boxDelta[p][0]][j - boxDelta[p][2]][k - boxDelta[p][4]]] > 0.5)
//						{
//							pieceIndices[p][0] = i; //  -delta[p];
//							pieceIndices[p][1] = j; //  - delta[p];
//							pieceIndices[p][2] = k; //  - delta[p];
//						}
//					}
//				}
//			}
//		}
//
//		aX.end();
//
//
//		// Save the solution:
//		if (!silent)
//			serialise("solution_compaction.psol");
//	}
//	else
//	{
//		cout << endl << "WARNING: CPLEX couldn't solve!" << endl;
//		// Reset indices:
//		pieceIndices = oldIndices;
//		coordinates_from_indices(resolution);
//		// cplex.exportModel("infeasible_model.lp");
//		// exit(-1);
//		// return;
//	}
//	delete[] X_pos;
//
//	model.end();
//	cplex.end();
//	env.end();
//
//	coordinates_from_indices(instance->pieces[0]->voxel.resolution);
//	int solH = highest_container_point();
//
//	if (!silent)
//		cout << "H = " << solH << ", cplex H = " << cplexH << endl;
//
//	if ((solH < Hmax) && status)
//		modify = true;
//	else if (status && noObj)
//		modify = true;
//	else
//	{
//		modify = false;
//	//	pieceIndices = oldIndices;
//	//	coordinates_from_indices(resolution);
//		if (!silent)
//			cout << endl << "WARNING: H was not modified, returning to old inidices!" << endl;
//	}
//
//}
