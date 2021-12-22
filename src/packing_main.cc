// Include CPLEX:
#ifdef __linux__ 
	// #include </local/software/cplex/12.6.1/cplex/include/ilcplex/ilocplex.h>
	// #include </local/software/cplex/12.6.1/cplex/include/ilcplex/ilocplexi.h>
#else
	#include <ilcplex\ilocplex.h>
#endif

#include "3dlib.h"
#include "3dpack.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <random>
#include <time.h>

int main(int argc, char **argv)
{
	PACKING_OPTIONS packingOptions;
	// See if an argument was provide d:
	string runID = "";
	if (argc >= 2)
	{
		read_configuration(packingOptions, argv[1]);
	}
	else if (argc == 1)
	{
		// Read the options from the file packing.conf:
		cout << "WARNING: No options argument provided, reading from packing.conf..." << endl;
		read_configuration(packingOptions);
	}
	
	if (argc == 3)
	{
		string runIDcopy = argv[2];
		runID = runIDcopy;
		cout << "This run has been given an ID: \"" << runID.c_str() << "\" that will trail all the saved files.\n";
	}

	if (argc >= 4)
	{
		cout << "ERROR: Incorrect number of arguments (" << argc << "), exiting..." << endl;
	}

	//  ---  READ THE INPUT FILE  ---  //
	vector<string> target_files;
	vector<int> quantities;
	vector<double> containerInfo;
	// cout << "Reading configuration from file: \"" << packingOptions.file.c_str() << "\"...\n";
	read_input(packingOptions.file.c_str(), target_files, quantities, containerInfo);
	cout << "Done." << endl;
	packingOptions.idOfRun = runID;


	if (containerInfo.size() < 3)
	{
		cout << "Error: Container was not provided" << endl;
		exit(-13423);
	}

	// Finish the instance and create a packing layout that will be the solution:
	CONTAINER problemContainer;
	for (int i = 0; i < 3; i++)
	{
		problemContainer.sideSize[i] = containerInfo[i];
		if (containerInfo[i] < 0)
			problemContainer.openDimension = i;
	}

	problemContainer.find_grid(packingOptions.resolution);

	// Create the instance:
	INSTANCE problemInstance(target_files, quantities, packingOptions, &problemContainer);



	// problemInstance.container = &problemContainer;

	problemInstance.type = STRIP;

	PACKING_LAYOUT packing_solution(problemInstance, packingOptions);

		/////////////////// DEBUG ///////////////////////////////
		//packing_solution.instance->nPieces = 5;
		//int pairs = 0;
		//for (int p = 0; p < 5; p++)
		//-1
		//	for (int q = p + 1; q < 5; q++)
		//	{
		//		cout << "REAL: p = " << p << ", q = " << q << ", pair = " << pairs << endl;
		//		pairs++;
		//	}

		//	int pairIdx = -1;
		//	packing_solution.update_pair_index(p, pairIdx);
		//	for (int q = 0; q < 5; q++)
		//	{
		//		packing_solution.update_pair_index(p, q, pairIdx);
		//		if (p == q)
		//			continue;
		//		cout << "CALC: p = " << p << ", q = " << q << ", pair = " << pairIdx << endl;
		//	}
		//}

		/////////////////// DEBUG ///////////////////////////////

	NFVSTRUCTURE allnfvs;
	// Generate or read nfv:
	if (packingOptions.nfvfile.compare(":") == 0)
	{
		cout << "Generating NFVs..." << endl;
		allnfvs = packing_solution.generate_nfvs();
		allnfvs.fix_addresses();
		allnfvs.serialise(packingOptions.file.append(".nfv"));
		packing_solution.allNFV = &allnfvs;
		packing_solution.allNFV->fix_addresses();
	}
	else
	{
		cout << "Reading NFVs..." << endl;
		allnfvs.deserialise(packingOptions.nfvfile, packing_solution.instance->originalPieces);
		// allnfvs->deserialise(packingOptions.nfvfile, packing_solution.instance->originalPieces);
		packing_solution.allNFV = &allnfvs;
		packing_solution.allNFV->fix_addresses();
	}
	cout << "Done." << endl;


	// PACKING_LAYOUT bestSolution = packing_solution;
	INSTANCE  bestInstance = (*packing_solution.instance);

	double probChange = packingOptions.probChange;
	double randomNum = 0.0;
	vector<int> bestN;
	int saveThisMany = 3;
	/*=========================================================================================================*/
	/*=========================================================================================================*/
	/*=========================================================================================================*/
	/*=========================================================================================================*/
	if (packingOptions.method == 0) // Only BLF
	{
		// Sort by volume:
		std::sort(packing_solution.instance->pieces.begin(), packing_solution.instance->pieces.end(),  sort_piece_pointers_decreasing_depth);
		// std::sort(packing_solution.instance->pieces.begin(), packing_solution.instance->pieces.end(),  sort_piece_pointers_decreasing);
		PACKING_LAYOUT emptySolution = packing_solution;
		PACKING_LAYOUT bestSolution = packing_solution;
		packing_solution = emptySolution;
		packing_solution.allNFV->fix_addresses();
		vector<int> containerHeights;
		vector<int> bestHeights;
		packing_solution.strip_voxel_packing_first_fit_nfv();
		std::stringstream sstm;
		sstm << packingOptions.file << "_" << "FFD.psol";
		string initialSname = sstm.str();
		packing_solution.serialise(initialSname.c_str());
		cout << "Finished!" << endl;
		cout << "Time: " << packing_solution.elapsedTime << endl;

	}
	else if (packingOptions.method == 1 || packingOptions.method == 3) // Random search (kick + local)
	{
		/////// SEARCH PARAMETERS ///////
		int maxKicks = packingOptions.maxKicks;
		int maxKicksOR = maxKicks;
		int maxPerturbations = 1; // Just want to try different piece orders
		// int maxPerturbations = packingOptions.maxPerturbations;
		int maxPerturbationsOR = maxPerturbations;

		////////////////////////////////

		std::sort(packing_solution.instance->pieces.begin(), packing_solution.instance->pieces.end(), sort_piece_pointers_decreasing_depth);

		// Generate NFV's here:
		// NFVSTRUCTURE allnfvs(packing_solution.instance->originalPieces);
		// Point to it in packingLayout:
		PACKING_LAYOUT emptySolution = packing_solution;
		PACKING_LAYOUT bestSolution = packing_solution;
		packing_solution = emptySolution;
		packing_solution.allNFV->fix_addresses();
		vector<int> containerHeights;
		vector<int> bestHeights;

		packing_solution.strip_voxel_packing_first_fit_nfv();
		// packing_solution.strip_voxel_packing_lowest_fit_nfv();


		int bestHeight = packing_solution.highest_container_point();

		bestN.assign(saveThisMany, bestHeight);

		// Select this as the best solution so far:
		cout << "Best solution in iteration: " << "INITIAL" << "; Height: " << bestHeight << endl;
		cout << "Piece order: ";
		for (int j = 0; j < bestSolution.instance->pieces.size(); j++)
		{
			cout << bestSolution.instance->pieces[j]->index << "\t";
		}
		cout << endl << endl;

		bestSolution = packing_solution;
		bestInstance = (*packing_solution.instance);
		bestHeight = bestSolution.highest_container_point();

		cout << "Start random sortings..." << endl;
		clock_t heuristicClockInit, heuristicClock;
		heuristicClockInit = clock();
		int lastKick;
		float rsearchTime = 0; 
		float rsearchTimeOld = 0; 
		for (int i = 0; i < maxKicks; i++)
		{
			lastKick = i;
			// cout << "Kick " << i << endl;
			for (int perturbations = 0; perturbations < maxPerturbations; perturbations++)
			{
				packing_solution = emptySolution;
				packing_solution.allNFV->fix_addresses();
				// packing_solution.random_placement_rules();

				packing_solution.strip_voxel_packing_first_fit_nfv();
				// packing_solution.serialise("trial.psol");
				// packing_solution.strip_voxel_packing_lowest_fit_nfv();

				containerHeights.push_back(packing_solution.highest_container_point());
				// cout << "H = " << containerHeights.back() << endl;
				// Before saving the result, check if we have exceeded the time

				heuristicClock = clock();
				// double soFarElapsed = float(initialSolutionTime - InitialPackingClock) / CLOCKS_PER_SEC;
				rsearchTime =  float(heuristicClock - heuristicClockInit) / CLOCKS_PER_SEC;

				if (rsearchTime >= packingOptions.maxTime)
				{
					cout << "Elapsed time: " << rsearchTime << " s. ---> Exit" << endl;
					break;
				}
				else
					rsearchTimeOld = rsearchTime;

				if (containerHeights.back() < bestN.back() && containerHeights.back() != bestN[saveThisMany - 2])
				{
					// Update one of the best N
					bestN.back() = containerHeights.back();
					std::sort(bestN.begin(), bestN.end());
				}

				if (containerHeights.back() < bestHeight)
				{
					bestHeight = containerHeights.back();
					cout << "Best solution kick: " << i << "; Perturbation " << perturbations << "; Height: " << bestHeight << endl;
					cout << "Elapsed time so far: " << rsearchTime << " seconds." << endl;
					bool reportSolution = false;
					if (reportSolution)
					{
						cout << "Piece order: ";
						cout << "{";
						for (int j = 0; j < bestSolution.instance->pieces.size(); j++)
						{
							cout << bestSolution.instance->pieces[j]->index << ",\t";
						}
						cout << "}" << endl;
						cout << "placeRight =  " << "{";
						for (int plac_vec = 0; plac_vec < packing_solution.placeFront.size() - 1; plac_vec++)
						{
							if (packing_solution.placeRight[plac_vec])
								cout << "true, ";
							else
								cout << "false, ";
						}
						if (packing_solution.placeRight.back())
							cout << "true";
						else
							cout << "false";


						// Print placefront:
						cout << endl << "placeFront: " << endl << "{ ";
						for (int plac_vec = 0; plac_vec < packing_solution.placeFront.size() - 1; plac_vec++)
						{
							if (packing_solution.placeFront[plac_vec])
								cout << "true, ";
							else
								cout << "false, ";
						}
						if (packing_solution.placeFront.back())
							cout << "true}";
						else
							cout << "false}";

						cout << endl;
					}
					bestSolution = packing_solution;
					bestInstance = (*packing_solution.instance);
					bestHeights.push_back(containerHeights.back());
					// Serialise such a great achievement:
					std::stringstream stringbestsol;
					stringbestsol << "randomSearch" << bestHeights.size() << ".psol";
					string bestsolname = stringbestsol.str();
					packing_solution.serialise(bestsolname.c_str());

					// Add a few more kicks and perturbations:

					maxKicks = i  + maxKicksOR;
					maxPerturbations = perturbations + maxPerturbationsOR;


				}
				// maxPerturbations = maxPerturbationsOR + 10;
			}
			// Kick!
			std::random_shuffle(packing_solution.instance->pieces.begin(), packing_solution.instance->pieces.end());
			if (rsearchTime >= packingOptions.maxTime)
			{
				cout << "STOP: Max time elapsed! (" << rsearchTimeOld << ")" << endl;
				break;
			}


		}

		cout << "Finally performed " << lastKick << " kicks in " << rsearchTime << " seconds." << endl;
		packing_solution = bestSolution;
		(*packing_solution.instance) = bestInstance;

		cout << "Best solution has height: " << packing_solution.highest_container_point() << endl;
		double averageHeight = 0;
		for (int nsols = 0; nsols < containerHeights.size(); ++nsols)
		{
			averageHeight += containerHeights[nsols];	
		}
		averageHeight = averageHeight/containerHeights.size();
		cout << "Average height: " << averageHeight << endl;

		// cout << "Relocate the vertices in the solution polytopes..." << endl;
		packing_solution.coordinates_from_indices(packingOptions.resolution);
	}

	if (packingOptions.method == 2 || packingOptions.method == 3) // Compaction
	{
		if (packingOptions.method == 2)
		{
			// Initial solution can be given in solutionFile or needs to be calculated:
			if (packingOptions.solutionFile.compare(":") == 0)
			{
				cout << "Initial solution from First Fit BLB algorithm..." << endl;
				packing_solution.strip_voxel_packing_first_fit_nfv();
				cout << "Done." << endl;
			}
			else
			{
				cout << "Initial solution from file: " << packingOptions.solutionFile << endl;
				packing_solution.deserialise(packingOptions.solutionFile);
				packing_solution.report_on_screen();
				cout << "Done." << endl;
			}

			bestN.assign(saveThisMany, packing_solution.highest_container_point());

		}

		// Create the delta and parameters:
		vector<int> delta, deltazero, deltap1, deltap2;
		int deltaVal = packingOptions.initDelta;
		for (int pt = 0; pt < packing_solution.instance->nPieces; pt++)
		{
			deltazero.push_back(deltaVal);
			deltap1.push_back(deltaVal + 1);
			deltap2.push_back(deltaVal + 2);
		}

		double time_limit = packingOptions.maxTime;
		bool modify = false;
		int H = packing_solution.highest_container_point();
		int oldH = H;
		int bestEverH = H;
		int newH;
		int maxTrials = 100 * packingOptions.maxIters;
		int totalTrials = 0;
		for (int ii = 0; ii < packingOptions.maxIters; ii++)
		{
			H = packing_solution.highest_container_point();
			delta = deltazero;
			int oportunities = 0; // Number of times we increase the delta:
			// Run model
				// Don't run if the initial solution is not good enough:
			double allowance = 1.25;
			if (H <= bestN.back()*allowance)
			{
				for (int i = 0; i < 100; i++)
				{

				
						cout << "Solving for init. solution " << H << ". Allowed gap is (" << bestN.front() << ", " << allowance*bestN.back() << ")" << endl;
						/* Serialise the initial point */
						std::stringstream sstm;
						sstm << packingOptions.file << "k_" << ii << "_RANDOM_" << i << ".psol";
						string initialSname = sstm.str();
						packing_solution.serialise(initialSname.c_str());

						vector < vector < int> > oldIndices = packing_solution.pieceIndices;
						packing_solution.preprocess(delta, H);
						packing_solution.init_cplex(delta, time_limit, modify, H, oldIndices);
						newH = packing_solution.highest_container_point();
						if (newH - H < 0)
						{
							cout << endl << endl << "Max height after iteration " << i << " is " << newH << " (" << H - newH << " better than previous)" << endl << endl;
							cout << "Delta for this was: " << delta[0] << endl;
							H = newH;

							// Reset deltas if necessary:
							delta = deltazero;
							oportunities = 0;
							if (newH < bestN.back() && newH != bestN[saveThisMany - 2])
							{
								// Update one of the best N
								bestN.back() = newH;
								std::sort(bestN.begin(), bestN.end());
								cout << "Worst best N is: " << bestN.back() << endl;
							}
						}
						else
						{
							cout << endl << endl << "Max height after iteration " << i << " is " << newH << " (did not improve)" << endl;
							oportunities++;
							if (oportunities > 0)
								{
									cout << "First attempt (delta = " << delta[0] << ") did not find anything. Exit." << endl;
									break;
								}
							else if (oportunities == 1)
								delta = deltap1;
							else if (oportunities == 2)
								delta = deltap2;

							cout << "Delta increased to: " << delta[0] << endl;
						}

						if (newH < bestEverH)
						{
							bestEverH = newH;
							cout << "New best H is: " << bestEverH << endl;
							cout << "Piece order was: " << endl;
							for (int pzi = 0; pzi < packing_solution.instance->nPieces; pzi++)
								cout << packing_solution.instance->pieces[pzi]->index << "\t" << endl;

							packing_solution.show_indices();
							std::stringstream sstm;
							sstm << packingOptions.file << "k_" << ii << "_CPLEX_" << i << ".psol";
							string bestsolname = sstm.str();
							packing_solution.serialise(bestsolname.c_str());
							cout << bestsolname << " was the best at this time" << endl;
						}
						else
						{
							std::stringstream sstm;
							sstm << packingOptions.file << "kick" << ii << "_sol_" << i << ".psol";
							string bestsolname = sstm.str();
							packing_solution.serialise(bestsolname.c_str());
						}

					
				}
			}
			else
			{
				// cout << "Tried H = " << H << " not good..." << endl;
				// cout << "Solving for init. solution " << H << ". Allowed gap is (" << bestN.front() << ", " << allowance*bestN.back() << ")" << endl;
				ii--;
				totalTrials++;
				if (totalTrials >= maxTrials)
				{
					cout << "Reached maximum trials of init. solutions (" << maxTrials << "). STOP" << endl;
					break;
				}
			}

			// Create random order:
			// cout << "Creating new sol..." << endl;
			// cout << "Worst best N is: " << bestN.back() << endl;
			std::random_shuffle(packing_solution.instance->pieces.begin(), packing_solution.instance->pieces.end());
			packing_solution.random_placement_rules();

			packing_solution.strip_voxel_packing_first_fit_nfv();
			// cout << "Solution had " << packing_solution.highest_container_point() << " height." << endl;
		}
		cout << endl << "Max height after compacting: " << newH << "(" << oldH - newH << " better than initial)" << endl;

	} // End compaction method
	if (packingOptions.method == 4) // Test BLB with all possible permutations in the placement rules
	{
		// Do this in a random shuffle:
		std::random_shuffle(packing_solution.instance->pieces.begin(), packing_solution.instance->pieces.end());

		int np = packing_solution.instance->nPieces;
		if (np > 15)
		{
			cout << "This will take a long time, exiting..." << endl;
			exit(-1);
		}

		std::stringstream stringbestsol;
		stringbestsol << "ennum_" << packingOptions.file << ".txt";
		string bestsolname = stringbestsol.str();

		ofstream fi;
		fi.open(bestsolname.c_str());

		int loopSize = 2 * np;
		float powerSize = pow(2,loopSize);

		vector<bool> b(2*np);
		for (int i = 0; i < powerSize; i++)
		{
			for (int j = 0; j < loopSize; j++)
			{
				b[j] = i & (1 << j);
			}


			// Set the correct bits to each vector (placeFront and placeRight)
			for (int bit = 0; bit < np; bit++)
			{
				packing_solution.placeFront[bit] = (b[bit] == 1);
				packing_solution.placeRight[bit] = (b[bit + np] == 1);
			}

			// Solve with NFV:
			packing_solution.strip_voxel_packing_first_fit_nfv();

			// cout << "Solution for iteration: " << i << endl;
			int H = packing_solution.highest_container_point();
			fi << H << endl;
			cout << "it: " << i << " - " << H << endl;

			if (i % 10000 == 0)
			{
				std::stringstream sstm;
				sstm << packingOptions.file << "perm" << i << "_sol_" << ".psol";
				string bestsolname = sstm.str();
				packing_solution.serialise(bestsolname.c_str());
			}

			// Before next iteration, reset everything:
			packing_solution.reset();
		}

		fi.close();
	}
	if (packingOptions.method == 5) // Hill climb on placement rules (placeFront and placeRight)
	{

		// Give an initial solution:
		// Initial solution can be given in solutionFile or needs to be calculated:
		packing_solution.initial_solution_first_fit(packingOptions);

		// Find the local optimum around it:
		packing_solution.hill_climb();

		std::stringstream sstm;
		sstm << packingOptions.file << "_" << "HC_local_opt.psol";
		string hc_solname = sstm.str();
		packing_solution.serialise(hc_solname);

	}
	int oldTime = packingOptions.maxTime;
	if (packingOptions.method == 67)
		packingOptions.maxTime = 20; // Set the initial solution time to 1 min

	if (packingOptions.method == 6 || packingOptions.method == 67) // Iterated local search (Kick: change piece order and random placements)
	{
		packing_solution.initial_solution_first_fit(packingOptions);
		packing_solution.ils();
		std::stringstream sstm;
		sstm << packingOptions.file << "_" << "ILS_final.psol";
		string hc_solname = sstm.str();
		packing_solution.serialise(hc_solname);
	}
	if (packingOptions.method == 7 || packingOptions.method == 67) // Iterated compaction new:
	{
		// Initial solution can be given in solutionFile or needs to be calculated:
		if (packingOptions.method != 67)
			packing_solution.initial_solution_first_fit(packingOptions);
		else
			packingOptions.maxTime = oldTime; // Return to the real time

		packing_solution.iterated_compaction(packingOptions.maxIters,
			packingOptions.initDelta, 2, 0, packingOptions.maxTime);

	}
	if (packingOptions.method == 8) // Simulated Annealing (SA)
	{
		clock_t InitialPackingClock = clock();
		int od = packing_solution.instance->container->openDimension;

		// We would like to start with a random solution, rather than always the same!
		// std::random_shuffle(packing_solution.instance->pieces.begin(), packing_solution.instance->pieces.end());
		// packing_solution.random_placement_rules();



		packing_solution.initial_solution_first_fit(packingOptions);
		// int k = 0;
		int cantImprove = 0;
		int compactBy = packingOptions.maxPerturbations;
		packing_solution.serialise("IteratedSA_0.psol");


		clock_t initialSolutionTime = clock();
		double soFarElapsed = float(initialSolutionTime - InitialPackingClock) / CLOCKS_PER_SEC;

		int lastBest = packing_solution.maxHeightVoxels;
		packing_solution.UpperBoundH = lastBest;



		// In case we do compaction:
		bool doAModel = false;
		vector<int> deltazero;
		int deltaVal = packingOptions.initDelta;
		for (int pt = 0; pt < packing_solution.instance->nPieces; pt++)
		{
			deltazero.push_back(deltaVal);
		}
		bool modify = false;
		double time_limit = 12000;
		/////

		// Start the iterated procedure:
		for (int trial = 1; trial < packingOptions.maxKicks + 2; trial++)
		{
			cout << endl << endl << "*** ITERATION " << trial << ": H = " << packing_solution.maxHeightVoxels;
			cout << ", O = " << packing_solution.overlapCount << ", elapsedTime = " << round(soFarElapsed) << "s. ***" << endl << endl;
			if (packing_solution.overlapCount == 0)
			{
				// We can move to the minimum...
				// packingOptions.int1 = packingOptions.int2;
				cantImprove = 0;
				lastBest = packing_solution.maxHeightVoxels;
				

				for (int p = 0; p < packing_solution.instance->nPieces; p++)
				{
					if (packing_solution.pieceIndices[p][od] + packing_solution.instance->pieces[p]->voxel.get_grid_size(od) >=
						packing_solution.maxHeightVoxels - compactBy)
						packing_solution.pieceIndices[p][od] = max(packing_solution.pieceIndices[p][od] - compactBy, 0);
				}
				packing_solution.UpperBoundH = packing_solution.maxHeightVoxels - compactBy;
					// packing_solution.pieceIndices[p][od] = max(packing_solution.pieceIndices[p][od] - compactBy, 0);
			}
			else if (trial > 2)
			{
				// int specCompact = ceil((lastBest - packing_solution.UpperBoundH) / 2);
				// packing_solution.UpperBoundH = lastBest - specCompact;
				// cout << "Return to compact by " << specCompact << " rather than " << compactBy << endl;
				// packingOptions.sat0 = packingOptions.sat0 * 2; // Heat up...
				// packingOptions.int1 = min(packingOptions.int3, packingOptions.int1 + 1);
				packing_solution.UpperBoundH++;
				cantImprove++;
				// Move up the ones touching the last row
				 for (int p = 0; p < packing_solution.instance->nPieces; p++)
				 {
					 if (packing_solution.pieceIndices[p][od] + packing_solution.instance->pieces[p]->voxel.get_grid_size(od) >=
						 packing_solution.maxHeightVoxels - 2)
						 packing_solution.pieceIndices[p][od] ++;
				}
				packing_solution.highest_container_point();

				if (doAModel && packing_solution.overlapCount <= 3)
				{
					// It's a small thing, try to solve with compaction:
					vector < vector < int> > oldIndices = packing_solution.pieceIndices;
					cout << "Solving a model with delta = " << deltaVal << "..." << endl;
					packing_solution.ignorePiece.assign(packing_solution.instance->nPieces, false);
					packing_solution.preprocess(deltazero, packing_solution.UpperBoundH);
					bool noObj = true;
					packing_solution.init_cplex(deltazero, time_limit, modify, packing_solution.UpperBoundH, oldIndices, noObj);
					if (!modify)
					{
						cout << "Didn't work!" << endl;
						packing_solution.pieceIndices = oldIndices;
					}
					packing_solution.is_feasible_voxel();
					packing_solution.serialise("after_model", trial);
				}
			}

			// if (cantImprove >= compactBy - 1)
			// {
				// packing_solution.serialise("IteratedSA_", trial);
				// break;
			// }

			packing_solution.SA(packingOptions.maxIters, packingOptions.sat0, packingOptions.satype);
			packing_solution.serialise("IteratedSA_", trial);

			clock_t IterationEnd = clock();
			soFarElapsed = float(IterationEnd - InitialPackingClock) / CLOCKS_PER_SEC;
		}
		
		cout << endl << endl << "Iterated SA finished! Total time: " << soFarElapsed << endl;

	}
	if (packingOptions.method == 9) // Start with everything in (0,0)...
	{
		int od = packing_solution.instance->container->openDimension;
		clock_t InitialPackingClock = clock();
		clock_t initialSolutionTime = clock();
		double soFarElapsed = float(initialSolutionTime - InitialPackingClock) / CLOCKS_PER_SEC;
		int nonImproving = 0;
		for (int p = 0; p < packing_solution.instance->nPieces; p++)
		{
			packing_solution.piecePlaced[p] = true;
			packing_solution.ignorePiece[p] = false;
		}
		int startH = packing_solution.highest_container_point();
		int bestFeasible = RAND_MAX;

		// Lower bound here!
		packing_solution.UpperBoundH = startH * 2;

		// Start to iterate!
		cout << endl << endl << "*** ITERATION " << 0 << ": H = " << packing_solution.maxHeightVoxels;
		cout << ", O = " << packing_solution.overlapCount << ", ";
		cout << "bestH = " << bestFeasible << "elapsedTime = " << round(soFarElapsed) << "s. ***" << endl << endl;
		packing_solution.serialise("growingSA", 0);
		bool firstTimeRepeat = false;
		for (int trial = 1; trial < packingOptions.maxKicks + 1; trial++)
		{
			packing_solution.SA(packingOptions.maxIters, packingOptions.sat0, packingOptions.satype);
			cout << endl << endl << "*** ITERATION " << trial << ": H = " << packing_solution.maxHeightVoxels;
			cout << ", O = " << packing_solution.overlapCount << ", ";
			cout << "bestH = " << bestFeasible << ", elapsedTime = " << round(soFarElapsed) << "s. ***" << endl << endl;
			packing_solution.serialise("growingSA", trial);

			// Did we find a feasible solution with this H?
			if (packing_solution.overlapCount == 0)
			{
				bestFeasible = packing_solution.maxHeightVoxels;
				firstTimeRepeat = false;
				nonImproving = 0;
				packing_solution.serialise("nonOvSol", trial);
				cout << endl << "(reduce H by " << packingOptions.int3 << ")" << endl;
				packing_solution.UpperBoundH -= packingOptions.int3;

				for (int p = 0; p < packing_solution.instance->nPieces; p++)
					packing_solution.pieceIndices[p][od] = min(packing_solution.pieceIndices[p][od],
						packing_solution.UpperBoundH - packing_solution.instance->pieces[p]->voxel.get_grid_size(od));

				packing_solution.highest_container_point();
				packing_solution.is_feasible_voxel();
			}
			else if (packing_solution.UpperBoundH >= bestFeasible - 1) // We do not want to explore things worse than we had already...
			{
				cout << endl << ">>> Exchanging position of two pieces!!!";


				// Move 3 iterations down to recorver...
				packing_solution.UpperBoundH -= packingOptions.int2;
				packing_solution.highest_container_point();
				packing_solution.is_feasible_voxel();

				firstTimeRepeat = true;

				// Find the suspects:
				int p1 = -1;
				int p2 = -1;
				while (true)
				{
					int ri = (int) double(rand()) % packing_solution.instance->nPieces;
					if (packing_solution.overlaps[ri])
					{
						if (p1 < 0)
							p1 = ri;
						else if (ri != p1)
						{
							p2 = ri;
							break;
						}
					}
				}

				cout << endl << "    P =  " << p1 << ", Q = " << p2 << endl;
				cout << endl << "    (and reduce H by " << packingOptions.int2 << ")" << endl;
				// Exchange:
				vector<int> p1WasAt = packing_solution.pieceIndices[p1];
				packing_solution.pieceIndices[p1] = packing_solution.pieceIndices[p1];
				packing_solution.pieceIndices[p2] = p1WasAt;

				// Move down if any piece goes over the top:
				for (int p = 0; p < packing_solution.instance->nPieces; p++)
					packing_solution.pieceIndices[p][od] = min(packing_solution.pieceIndices[p][od],
						packing_solution.UpperBoundH - packing_solution.instance->pieces[p]->voxel.get_grid_size(od));

			}
			else
			{
				firstTimeRepeat = false;
				nonImproving++;
				packing_solution.UpperBoundH++;

				// Apply a kick if necessary:
				// if (nonImproving >= 3)
				// {
					// // Nothing
				// }

			}
			clock_t IterationEnd = clock();
			soFarElapsed = float(IterationEnd - InitialPackingClock) / CLOCKS_PER_SEC;
		}



	}
	if (packingOptions.method == 10) // Hill climbing, neighbourhood structure
	{
		// Don't do initial sol:
		packing_solution.initial_solution_first_fit(packingOptions);

		packing_solution.serialise("SolBeforeHC.psol");
		// Start with all solutions in 0 instead:
		//for (int p = 0; p < packing_solution.instance->nPieces; p++)
		//{
		//	packing_solution.piecePlaced[p] = true;
		//	packing_solution.ignorePiece[p] = false;
		//}


		// Get the fitness:
		packing_solution.is_feasible_voxel();

		// Hill climb!
		packing_solution.hill_climb_n_class();
		packing_solution.serialise("SolAfterHC.psol");
	}
	if (packingOptions.method == 11) // Tabu search
	{
		// Don't do initial sol:
		// packing_solution.initial_solution_first_fit(packingOptions);

		// packing_solution.serialise("SolBeforeTS.psol");
		// Start with all solutions in 0 instead:
		packing_solution.UpperBoundH = packingOptions.int3;
		for (int p = 0; p < packing_solution.instance->nPieces; p++)
		{
			packing_solution.piecePlaced[p] = true;
			packing_solution.ignorePiece[p] = false;
			cout << "This piece can be put whithin: ";
			cout_point3((packing_solution.UpperBoundH - packing_solution.instance->pieces[p]->voxel.get_grid_size(packing_solution.od)),
				packing_solution.instance->maxPositions[packing_solution.instance->pieces[p]->index][packing_solution.cd1],
				packing_solution.instance->maxPositions[packing_solution.instance->pieces[p]->index][packing_solution.cd2]);

			// Start placing them randomly...
			packing_solution.pieceIndices[p][packing_solution.od] =
				(int) double(rand()) % (packing_solution.UpperBoundH -
				packing_solution.instance->pieces[p]->voxel.get_grid_size(packing_solution.od));
			packing_solution.pieceIndices[p][packing_solution.cd1] = (int) double(rand()) %
				packing_solution.instance->maxPositions[packing_solution.instance->pieces[p]->index][packing_solution.cd1];
			packing_solution.pieceIndices[p][packing_solution.cd2] = (int) double(rand()) %
				packing_solution.instance->maxPositions[packing_solution.instance->pieces[p]->index][packing_solution.cd2];


		}

		// Do this iteratively until we find a feasible solution:
		// Where we start growing up:
		int trial = 0;
		int bestUPbound = 200;
		int od = packing_solution.instance->container->openDimension;
		int it_counter = 0;
		while (true)
		{
			it_counter++;
			cout << "Tabu search iteration no. " << it_counter << " best feasible H is " << bestUPbound << endl;
			packing_solution.tabu_search(trial);
			packing_solution.serialise("TS_", trial);
			bool solutionInfeasible = !packing_solution.is_feasible_voxel();
			if (solutionInfeasible)
			{
				packing_solution.UpperBoundH += 1;
				packingOptions.int2 = 3;
				packing_solution.highest_container_point();
				cout << "Fitness is: " << packing_solution.full_fitness_overlap() << endl;
				packing_solution.fix_overlaps_from_pairs();
				
			}

			if (!solutionInfeasible || (packing_solution.UpperBoundH >= bestUPbound))
			{
				if ((packing_solution.UpperBoundH >= bestUPbound))
				{
					cout << "Warning, going somewhere where we have been before... Exit!" << endl;
				}
				else
				{
					bestUPbound = packing_solution.highest_container_point();
					cout << endl << "*** found a feasible solution ***" << endl;
					cout << "H = " << bestUPbound << endl << endl;
				}

				int updateBy = ceil(0.2*bestUPbound);
				cout << "Shrinking solution by " << updateBy << " voxels!" << endl;
				packing_solution.UpperBoundH -= updateBy;
				// Move down if any piece goes over the top:
				bool justMoveDown = false;
				if (solutionInfeasible & !justMoveDown)
					cout << "Destroying everything..." << endl;

				for (int p = 0; p < packing_solution.instance->nPieces; p++)
				{
					if (!solutionInfeasible || justMoveDown)
					{
						int ri = (int) double(rand()) % packing_solution.UpperBoundH;
						if (packing_solution.pieceIndices[p][od] > ri)
							packing_solution.pieceIndices[p][od] = max(0, packing_solution.pieceIndices[p][od] - updateBy);
						else // Adjust it not to go over the upper bound
							packing_solution.pieceIndices[p][od] = min(packing_solution.pieceIndices[p][od],
								packing_solution.UpperBoundH - packing_solution.instance->pieces[p]->voxel.get_grid_size(od));
					}
					else
					{
						// Randomly assign coordinates:
						packing_solution.pieceIndices[p][od] = 
							(int) double(rand()) % (packing_solution.UpperBoundH - packing_solution.instance->pieces[p]->voxel.get_grid_size(od));
						packing_solution.pieceIndices[p][packing_solution.cd1] = (int) double(rand()) %
							packing_solution.instance->maxPositions[packing_solution.instance->pieces[p]->index][packing_solution.cd1];
						packing_solution.pieceIndices[p][packing_solution.cd2] = (int) double(rand()) %
							packing_solution.instance->maxPositions[packing_solution.instance->pieces[p]->index][packing_solution.cd2];
					}


				}
				packing_solution.highest_container_point();
				packing_solution.full_fitness_overlap();
				packing_solution.fix_overlaps_from_pairs();
			}

			trial++;
			if (trial >= packingOptions.maxKicks)
			{
				cout << "Max. number of kicks reached. Exiting!" << endl;
				break;
			}
		}
	}

	if (packingOptions.method == 12) // Iterated Variable Neighbourhood Search
	{
		std::sort(packing_solution.instance->pieces.begin(), packing_solution.instance->pieces.end(),  sort_piece_pointers_decreasing_depth);
		packing_solution.ivns();
	}

	if (packingOptions.method == 13) // Iterated Tabu Search
	{
		// std::sort(packing_solution.instance->pieces.begin(), packing_solution.instance->pieces.end(),  sort_piece_pointers_decreasing_depth);
		packing_solution.initial_solution_first_fit(packingOptions);
		packing_solution.its();
	}

	if (packingOptions.method == 14) // Multistart Iterated Tabu Search
	{
		bool switchToCompaction = false;
		int maxKicks = 100;
		int bestH = packing_solution.highest_container_point();
		cout << "Starting ILS for " << maxKicks << " kicks or " << packingOptions.maxTime << " seconds." << endl;
		cout << "Initial H = " << bestH << endl;
		for (int i = 0; i < maxKicks; i++)
		{
			// Generate a random initial solution:
			packing_solution.reset();
			std::random_shuffle(packing_solution.instance->pieces.begin(), packing_solution.instance->pieces.end());
			packing_solution.random_placement_rules();

			// Give an initial solution:
			// packing_solution.strip_voxel_packing_first_fit_nfv();
			packing_solution.its();

			// Find the local optimum around it:
			// cout << " -- HC running for kick: " << i << " --" << endl;
			// packing_solution.hill_climb();

			if (switchToCompaction)
			{
				packing_solution.iterated_compaction(packingOptions.maxIters, 1, 2, 0, packingOptions.maxTime);
			}

			int foundH = packing_solution.highest_container_point();

			if (foundH < bestH && packing_solution.overlapCount == 0)
			{

				cout << endl << endl << " ### ILS new best, H = " << foundH << " (better than " << bestH << ") ###" << endl << endl << endl;
				bestH = foundH;
				std::stringstream sstm;
				sstm << packingOptions.file << "_" << "HC_K_" << i << "H_" << foundH << ".psol";
				string hc_solname = sstm.str();
				packing_solution.serialise(hc_solname);
			}

		}
	}

	if (packingOptions.method == 15) // VNS February Deprecated
	{
		// Bounds:
		int lob = 0;
		int upb = 0;
		packing_solution.find_simple_bounds_voxel(lob, upb);
		
		// Algorithm:
		packing_solution.vns_february(lob, upb);

		// Save best layout:
		std::stringstream sstm;
		sstm << packingOptions.file << "vns_feb_" << packing_solution.highest_container_point() << ".psol";
		string hc_solname = sstm.str();
		packing_solution.serialise(hc_solname);
		cout << "Saved file: " << hc_solname.c_str() << endl;
	}

	if (packingOptions.method == 16) // Try to solve model to optimality
	{
		double time_limit = packingOptions.maxTime;
		// Bounds:
		int lob = 0;
		int upb = 0;
		packing_solution.find_simple_bounds_voxel(lob, upb);
		packing_solution.initial_solution_first_fit(packingOptions);
		packingOptions.maxTime = 60;
		packing_solution.ils();
		packingOptions.maxTime = time_limit;
		int H = packing_solution.highest_container_point();
		if (H < upb)
			upb = H;

		cout << "Starting model with bounds: lower " << lob << ", upper " << upb << endl; 
		cout << "The highest container point is: " << H << endl;


		// Preparing and solving the model

		int box_size = upb;

		// Check if any of the sides is larger than the upper bound
		// if so, use it for the box size
		for (int j = 0; j < 3; j++)
		{
			if (packing_solution.instance->container->gridSize[j] > upb)
				box_size = packing_solution.instance->container->gridSize[j];
		}

		vector<int> delta;
		for (int pt = 0; pt < packing_solution.instance->nPieces; pt++)
			delta.push_back(box_size);

		bool modify = false;

		vector < vector < int> > oldIndices = packing_solution.pieceIndices;
		packing_solution.preprocess(delta, H);
		packing_solution.init_cplex(delta, time_limit, modify, H, oldIndices);
		int newH = packing_solution.highest_container_point();
		cout << "After the model, the highest container point is: " << H << endl;


		// Save layout:
		std::stringstream sstm;
		sstm << packingOptions.file << "standalone_model_" << packing_solution.highest_container_point() << ".psol";
		string hc_solname = sstm.str();
		packing_solution.serialise(hc_solname);
		cout << "Saved file: " << hc_solname.c_str() << endl;
	}


	if (packingOptions.method == 99) // Testing
	{
		// Just so they do not appear in the middle when showing:
		for (int i = 0; i < packing_solution.instance->nPieces; i++)
			packing_solution.pieceIndices[i][packing_solution.instance->container->openDimension] = -1000;

		std::random_shuffle(packing_solution.instance->pieces.begin(), packing_solution.instance->pieces.end());
		packing_solution.random_placement_rules();
		packing_solution.matheuristic(packingOptions.maxKicks, packingOptions.maxPerturbations);

		std::stringstream sstm;
		sstm << packingOptions.file << "_" << "MH_" << packingOptions.maxKicks <<
			"_" << packingOptions.maxPerturbations << "_H_" << packing_solution.highest_container_point() << ".psol";
		string sa_solname = sstm.str();
		packing_solution.serialise(sa_solname);
	}


	/*  ######   EXIT    ######   */

	// Whatever the method, report: // We do this later, maybe on screen, maybe on a file!
	// packing_solution.report_on_screen();

	if (packingOptions.doCompaction)
	{
		cout << "*** Switching to compaction ***" << endl;
		int H_value = packing_solution.highest_container_point();
		cout << "Initial H = " << H_value << " (real: " << H_value*packing_solution.resolution << ")" << endl;

		packing_solution.iterated_compaction(packingOptions.maxIters,
			packingOptions.initDelta, 2, 0, packingOptions.maxTime);
		H_value = packing_solution.highest_container_point();

		cout << "Final H = " << H_value << " (real: " << H_value*packing_solution.resolution << ")" << endl;
		packing_solution.report_on_screen();

		// Serialise it:
		std::stringstream sstm;
		sstm << packingOptions.file << "_" << "Final_Compaction_H_" << packing_solution.highest_container_point() << ".psol";
		string co_solname = sstm.str();
		packing_solution.serialise(co_solname);
	}

	// Make this to keep allnfvs in memory...
	NFV * dummy = allnfvs.getNFV(0, 0);

	// If we had a run ID, save a file with the name and the results //
	// Otherwise, just show on screen:
	std::stringstream rrss;

	char date[40];
	time_t t = time(0);
	strftime(date, sizeof(date), "On %d-%m-%Y at %H:%M:%S", gmtime(&t));

	rrss << "\n-------------------------------------------------------------------\n";
	if (argc == 3)
		rrss << "Result for run: " << runID;
	else
		rrss << "Run with no ID " << runID;
	rrss << " - " << date;
	rrss << "\n-------------------------------------------------------------------\n";
	rrss << "File being run: " << packingOptions.file << endl;
	rrss << "Solution method: " << packingOptions.method << " (" << packing_solution.methodString << ")" << endl;
	rrss << "The solution found ";
	if (packing_solution.is_feasible_voxel())
		rrss << "is feasible";
	else
		rrss << "is INFEASIBLE";
	rrss << " with a height of: " << packing_solution.highest_container_point() << " voxels." << endl << endl;
	rrss << ">>> Full report:" << endl;
	packing_solution.report_on_screen(rrss);
	// Dump this on screen:
	cout << rrss.str().c_str();
	rrss << "\n------------------------------------------\n";
	// If a runID was given, also do it in a file with the same name:
	if (argc == 3)
	{
		std::stringstream reportFilename;
		reportFilename << packingOptions.idOfRun << "_report.txt";
		string reportFilenameString = reportFilename.str();
		std::ofstream ofs(reportFilenameString, std::ofstream::app);
		ofs << rrss.str().c_str();
		ofs.close();

		// Serialise the solution:
		std::stringstream solutionFilename;
		solutionFilename << packingOptions.idOfRun << "_output.psol";
		string solutionFilenameString = solutionFilename.str();
		packing_solution.serialise(solutionFilenameString);
	}

	return 0;
}
