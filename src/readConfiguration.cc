//#include "testpolytopemain.h"
#include "3dlib.h"
#include <string>
#include <fstream>
#include <iostream>

void read_configuration(PACKING_OPTIONS &packing_options)
{
	// If no filename is provided as argument, read packing.conf
	string filename = "packing.conf";
	read_configuration(packing_options, filename);
}
void read_configuration(PACKING_OPTIONS &packing_options, string filename)
{
	string cW;
	cW.reserve(500);
	ifstream in(filename);
	if (!in.is_open())
	{
		cout << "There was a problem opening the configuration file: " << filename << endl;
		cout << "Exiting..." << endl;
		exit(-12432);
	}
	else
		cout << "Started reading from: " << filename << endl;

	double tempDouble; // Auxiliar var to read doubles.
	int tempInt;


	while (in >> cW)
	{
		if (cW.compare("resolution") == 0)
		{
			in >> packing_options.resolution;
			cout << "Reslution set to: " << packing_options.resolution << endl;
		}
		else if (cW.compare("method") == 0)
			in >> packing_options.method;		
		else if (cW.compare("randomSeed") == 0)
		{
			in >> packing_options.randomSeed;
			cout << "Set randomseed to " << packing_options.randomSeed << endl;
		}
		else if (cW.compare("sat0") == 0)
			in >> packing_options.sat0;
		else if (cW.compare("satype") == 0)
			in >> packing_options.satype;
		else if (cW.compare("doCompaction") == 0)
		{
			in >> tempInt;
			packing_options.doCompaction = !(tempInt == 0);
		}
		else if (cW.compare("file") == 0)
			in >> packing_options.file;
		else if ((cW.compare("nfv") == 0) || (cW.compare("nfvfile") == 0))
			in >> packing_options.nfvfile;
		else if (cW.compare("solutionFile") == 0)
			in >> packing_options.solutionFile;
		else if (cW.compare("maxIters") == 0)
			in >> packing_options.maxIters;
		else if (cW.compare("maxKicks") == 0)
			in >> packing_options.maxKicks;
		else if (cW.compare("maxPerturbations") == 0)
			in >> packing_options.maxPerturbations;
		else if (cW.compare("probChange") == 0)
			in >> packing_options.probChange;
		else if (cW.compare("maxTime") == 0)
			in >> packing_options.maxTime;
		else if (cW.compare("fitnessType") == 0)
			in >> packing_options.fitnessTypePar;
		else if (cW.compare("fitnessString") == 0)
			in >> packing_options.fitnessString;
		else if (cW.compare("initDelta") == 0)
			in >> packing_options.initDelta;
		else if (cW.compare("deltaPumps") == 0)
			in >> packing_options.deltaPumps;
		else if (cW.compare("int1") == 0)
			in >> packing_options.int1;
		else if (cW.compare("int2") == 0)
			in >> packing_options.int2;
		else if (cW.compare("int3") == 0)
			in >> packing_options.int3;
		else if (cW.compare("double1") == 0)
			in >> packing_options.double1;
		else if (cW.compare("double2") == 0)
			in >> packing_options.double2;
		else if (cW.compare("oscillationMode") == 0)
			in >> packing_options.oscillationMode;
		else if (cW.compare("decPercentage") == 0)
			in >> packing_options.decPercentage;
		else if (cW.compare("double3") == 0)
			in >> packing_options.double3;
			
		// Size of the tabu list
		else if (cW.compare("tabuSize") == 0)
			in >> packing_options.tabuSize;
		// How much we change the increase and decrease parameters in iterated tabu search
		else if (cW.compare("itsIterationMultiplier") == 0)
			in >> packing_options.itsIterationMultiplier;
		else if (cW.compare("itsDecreaseMultiplier") == 0)
			in >> packing_options.itsDecreaseMultiplier;
		else if (cW.compare("modelBoxOverlap") == 0)
			in >> packing_options.modelBoxOverlap;
		else if (cW.compare("modelBoxNonOverlap") == 0)
			in >> packing_options.modelBoxNonOverlap;
		else if (cW.compare("modelMaxOverlapPairs") == 0)
			in >> packing_options.modelMaxOverlapPairs;
		else if (cW.compare("HCtype") == 0)
		{
			in >> packing_options.HCtype;
			if (!(packing_options.HCtype.compare("closest") == 0) && !(packing_options.HCtype.compare("steepest") == 0))
			{
				cout << "Unknown HCtype option \"" << packing_options.HCtype << "\"" << endl;
				cout << "Reset to default \"closest\"" << endl;
				packing_options.HCtype = "closest";
			}
		}
		else
			cout << endl << "WARNING: option \"" << cW << "\" not recognised, skipped." << endl;
		
			
	}
	cout << endl << "Done reading configuration options." << endl;
}