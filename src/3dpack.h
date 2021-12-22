#ifndef __TDPACK_H_INCLUDED__
#define __TDPACK_H_INCLUDED__

#include "3dlib.h"



#include <vector>
#include <deque>

// class POLYTOPE;
class PACKING_LAYOUT;


enum instanceType {STRIP, BIN, FEASIBILITY};
enum packingType{BottomLeftBack, PushAndSlide};


class CONTAINER
{
public:
	// Dimensions of the container:
	double sideSize[3];
	double baseArea;


	// Grid size:
	int gridSize[3];
	int baseAreaVoxel;


	// Depending on the type of problem:
	int openDimension; // 0, 1, 2; -1 if not applicable (e.g. bin packing)



	/* Methods */
	CONTAINER();

	void find_grid(double resolution);

	// This is done in SCENE!
	// Display 2 walls of container:
	// void render(vector<float> color);
};

class INSTANCE
{
public:
	INSTANCE();
	INSTANCE(vector<string> target_files, vector<int> quantities, PACKING_OPTIONS &packingOptions);
	INSTANCE(vector<string> target_files, vector<int> quantities, PACKING_OPTIONS &packingOptions, CONTAINER * container_par);

	instanceType type;

	CONTAINER * container;

// private:
	int nPieces;
	int nPairs;
	int nPiecesOr;
	int voxelLowerBound;
	int voxelUpperBound;
	vector<POLYTOPE> allElements;
	vector<POLYTOPE*> pieces;
	vector<POLYTOPE*> originalPieces;
	vector<int> demands;
	vector<vector< int> > maxPositions;
	int voxelVolume;
	int largestVoxelBox;
	int largestVoxelSide;
	double volume;


	// Sorting:
	void sort_pieces_by(vector<int> sortingOrder);
	void swap_items(int i, int j);
	double voxel_volume_double();
	int voxel_volume_int();
};

class PACKING_LAYOUT
{
public:

	// Pointers to data:
	INSTANCE * instance;
	PACKING_OPTIONS * options;


	// For convenience, have the resolution and solution height (which might be outdated):
	double resolution;
	double maxHeight;
	double fitness;
	int maxHeightVoxels;
	int UpperBoundH;
	int fitnessType;
	
	// Dimensions (so we do not need to keep track of the open one!)
	int od;
	int cd1;
	int cd2;

	// Layout features:
	bool feasible;
	int overlapCount;
	bool complete;

	// Destroy-related:
	int nDisruptions;
	vector<string> destroyNames;
	vector<float> destroyScores;

	vector < vector<float> > rotationAngles;
	vector < vector<float> > pieceCoordinates;
	vector < vector<int> > pieceIndices;
	vector <bool > overlaps; // True if the piece overlaps another one
	vector <double> relBoxOverlapPairVoxel; // Overlap amount for each pair of pieces (relative, from voxel boxes)
	vector <double> relPenetrationDepth; // Overlap amount for each pair of pieces (relative, from voxel boxes)
	vector < vector < vector<int> > > pieceIndicesType;
	vector <int> lastAt; // Record the height of last piece of this type, so we start checking from there in NFV packing

	vector <bool> placeRight; // Indicate if the piece has to go to opposite side in X coord (BLB algorithms)
	vector <bool> placeFront; // Indicate if the piece has to go to opposite side in Y coord (BLB algorithms)

	vector <int> placementOrder;
	vector < bool> piecePlaced;
	vector < bool> ignorePiece;
	vector < bool> pieceCanMove;
	vector <int> instancePieceOrder;
	// Packing algorithm info:
	float elapsedTime;
	string methodString;
	string destroyInfo;

	// If needed, NFVSTRUCTURE:
	NFVSTRUCTURE * allNFV;

	// Methods:
	PACKING_LAYOUT();
	PACKING_LAYOUT(INSTANCE &instance, PACKING_OPTIONS &packingOptions);
	int solution_distance(PACKING_LAYOUT &anotherSolution);
	bool is_feasible_voxel();
	void find_simple_bounds_voxel(int &lower, int &upper, bool saveSolution, bool doBLB);
	void find_simple_bounds_voxel(int &lower, int &upper, bool saveSolution);
	void find_simple_bounds_voxel(int &lower, int &upper);
	void fix_overlaps_from_pairs();
	bool is_complete();
	void reset();
	void reset_after_index(int i);

	// Packing tools:
	int slice_usage(int slice_idx);
	int slice_usage(int slice_idx, int slice_coord);
	bool bboxes_intersect(int &newPiece, int &placedPiece, vector<int> &proposedPoint);
	bool box_overlap_rel(int p, int q, double &relAmount);
	// bool box_overlap(int p, int q, float &relAmount, int pairIdx);

	void iterated_heuristics();

	int highest_container_point();
	bool bboxes_intersectOR(int newPiece, int placedPiece, vector<int> proposedPoint, vector<int> placedPoint);
	int highest_container_point(int open_dimension);
	int is_legal_position(vector<int> &targetPos, int pz); // Tells if the piece pz can be placed in targetPovector<int> (negative if not)
	bool get_piece_layout_value(int pieceIndex, int layoutx, int layouty, int layoutz);
	bool get_piece_layout_value_with_pos(int pieceIndex, int layoutx, int layouty, int layoutz, int posx, int posy, int posz);

	int slice_free(int slice_idx);

	void coordinates_from_indices(double resolution);
	void indices_to_indices_type();
	void show_indices();
	bool can_p_move_to(int p, int i, int j, int k);
	void report_on_screen(stringstream &onStream);
	void report_on_screen();
	// void voxel_grid_starts_at(int pieceIdx, vector<int> &grid_index);

	// Save / Load from file
	void deserialise(string  filename);

	void serialise(string filename);
	void serialise(string filename, int index);

	// NFVs
	NFVSTRUCTURE generate_nfvs();


	// Packing algorithms (constructive):
	void strip_voxel_packing_sliding();
	void strip_voxel_packing_first_fit_nfv();
	void initial_solution_first_fit(PACKING_OPTIONS packingOptions);
	void strip_voxel_packing_lowest_fit_nfv();

	// Meta-heuristics:
	void hill_climb();
	void ils();
	void ils_kick(int number_of_changes);
	void hill_climb_n_class();
	void tabu_search(int iteration);
	void tabu_search(int iteration, clock_t &initialTimer);
	void ivns(int lob, int upb); // Iterated Variable Neighbourhood Search
	void ivns();
	void its(int lob, int upb); // Iterated Tabu Search
	void its();
	void tabu_search();
	void SA(int maxIter, double T0);
	void SA(int maxIter, double T0, int SAtype);
	void matheuristic(int moveLast, int moveLessLast);
	void vns_february(int lob, int upb);

	// Neighbourhood related:
	void select_neighbourhood_type(int &nType, vector<double> &f_values, int &SAType);
	void random_neighbour(int neighbourType, double &f_star, int &k, vector<double> &f_values);
	int penetration_depth(int p, int q);
	double full_fitness();
	double full_fitness(int updateOnlyP, bool updateThings);
	double full_fitness(bool updateThings);
	double full_fitness_closest_point(int updateOnlyP);
	double full_fitness_closest_point(int updateOnlyP, bool updateThings);
	double nfv_closest_point(int p, int q);
	double full_fitness_closest_point();
	double full_fitness_overlap();
	double full_fitness_overlap(int &k);
	void strategic_oscillation();
	void strategic_oscillation(int lob);
	double decrease_by_insert(int decreaseTo, bool randomizeInserts);

	// Destroy-related:
	void destroy_n(int n, vector<int> &overlapList);
	int select_item(vector<float> &probabilities);
	void decrease_n_by_perc(int n, float perc, vector<float> &probabilities);
	void decrease_all_but_n_by_perc(int n, float perc, vector<float> &probabilities);



	// Local search:
	void refine_positions();

	// Tools
	void random_placement_rules();
	void update_pair_index(int &p, int &pairIdx);
	void update_pair_index(int &p, int &q, int &pairIdx);
	void report_piece(int p);


	// CLASS WITH CPLEX
	//class PACKING_LAYOUT_CPLEX: public PACKING_LAYOUT
	//{
	//public:
	//IloEnv env;
	//IloModel model;
	//IloCplex cplex;
	//IloNumVarArray *vars;
	//IloRangeArray *rngs;
	//IloObjective *obj; // By default: minimize

	//// track position of the variables
	//int ****X_pos;
	//int H_pos;
	//int ntp; //number of types

	//private:
	void preprocess(vector<int> delta, int Hmax);
	void preprocess_box(vector<vector<int>> &boxDelta, vector<int> &orDelta, int Hmax);
	void expand_delta(vector<vector<int>> &boxDelta, vector<int> &orDelta);
	// void init_cplex(vector<int> delta, double time_limit, bool &modify, int &H);

	void init_cplex(vector<int> delta, double time_limit, bool &modify, int &H, vector< vector <int> > oldIndices);
	void init_cplex(vector<int> delta, double time_limit, bool &modify, int &H, vector< vector <int> > oldIndices, bool noObj);
	void init_cplex(vector<vector<int> > boxDelta, double time_limit, bool &modify, int &Hmax, vector<vector<int> > oldIndices);
	void init_cplex(vector<vector<int> > boxDelta, double time_limit, bool &modify, int &Hmax, vector<vector<int> > oldIndices, bool noObj);
	void modelo_lp(vector<vector<int> > boxDelta, double time_limit, bool &modify, int &Hmax, vector<vector<int> > oldIndices, bool noObj);

	void iterated_compaction(int maxIters, int deltaZero, int oportunities, int mode, double maxTime);



};

class NEIGHBOURHOOD 
{
private:
	int ntype; // Neighbourhood type
	int ftype; // Fitness type
	int cp; // Current piece
	int parameter; // Current piece
	PACKING_LAYOUT * ly; // Current layout
	PACKING_LAYOUT prevLy; // Store here things we might need to go back to
	int * od;
	int * cd1;
	int * cd2;


	int cneig_mod_6;

	// Neighbourhood generic functions:
	double test_move_to();
	void do_move_to();
	void undo_move();
	int opposite_neighbour_to_tabu();

	// Neighbourhood specific functions:
	void n20_next_neighbour(bool test);
	void n30_next_neighbour(bool test);
	void n40_next_neighbour(bool test);
	void n50_next_neighbour(bool test);
	// void n20_undo_neighbour();

	// DATA:
	// int n20_directions[5][3];
	vector < vector < int> >  n20_directions;
	vector< vector < int > > n30_points; // These points might change depending on the parameter!

	deque<int> tabuPiece;
	deque<int> tabuMove;


public:
	// Accessible vars:
	int cneig; // Current neighbour
	double cfit; // currentFitness
	double pfit; // previousFitness
	double tfit; // test fitness
	// Movement info:
	int dir_i;
	int dir_j;
	int dir_k;

	bool enabledTabu;
	int tabuSize;

	int rejectedMovements;
	int upperBoundary; // Forbid movements that go further than this value in the open dimension

	// Accessible functions:
	// NEIGHBOURHOOD(int neighbourhoodType, PACKING_LAYOUT * layout);
	NEIGHBOURHOOD(int neighbourhoodType, PACKING_LAYOUT * layout, int givenParameter);
	// ~NEIGHBOURHOOD();
	double swap_to(bool test);
	void set_type(int t);
	void set_fitness();
	void set_fitness(double fitness);
	int get_type();
	int get_parameter();
	int get_piece();
	void change_piece_to(int p);
	void change_parameter_to(int newPar);
	bool next_neighbour(bool test);
	bool next_random_neighbour(bool test, vector<int> &overlapList);
	double get_fitness(bool update);
	bool undo_neighbour();
	bool random_neighbour();
	void add_tabu();
	void clear_tabu_list();
	void cout_tabu_list(bool horizontal);

};



extern void read_input(const char *filename, vector<string> &target_files, vector<int> &quantities, vector<double> &containerSizes);
extern bool sort_piece_pointers_increasing(POLYTOPE * i, POLYTOPE * j);
extern bool sort_piece_pointers_decreasing(POLYTOPE * i, POLYTOPE * j);
extern bool sort_piece_pointers_decreasing_depth(POLYTOPE * i, POLYTOPE * j);

#endif
