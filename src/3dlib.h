#ifndef DLIB_HEADER
#define DLIB_HEADER
#include <stdio.h>
#include <vector>
#include <time.h> 
#include <string> 
// #include <math.h>
#include <cmath>
#include<random>
// #include "3dpack.h"
// #include "3dshow.h"

#define EPS 1e-16
#define TOL 1e-8

using namespace std;
using std::vector;
using std::string;
typedef vector<vector<double>> DOUBLEMAT;
typedef vector<vector<int>> INTMAT;
typedef unsigned char byte;

class PACKING_LAYOUT;
class PACKING_OPTIONS;

extern PACKING_LAYOUT * finalLayout;
extern std::default_random_engine randomNumGenerator;

class POINTCLOUD
{
public:
	// Properties:
	int 		n_vertices;
	//int 		n_faces;
	DOUBLEMAT 	vertices;
	//INTMAT		faces;
	//DOUBLEMAT	facenormals;
	//INTMAT		vertextrack; // Track in which faces the vertex is used

	void move_cloud(vector<double> mvec);
	void move_cloud(vector<float> mvec);
	void rotate_cloud(int ax, double alpha);
};

class BOX : public POINTCLOUD
{
public:
	int			n_faces;
	INTMAT		faces;
	DOUBLEMAT	facenormals;
	vector<double> * minpoint;
	vector<double> * maxpoint;
	vector<double> centre;

	// Constructor and object related:
	BOX();
	BOX duplicate();
	void rotate(vector<double> &angle);
	void locate_at(vector<double> &mvec);
	void locate_at(vector<float> &mvec);


	// Fill the box from two points:
	void expand(vector<double> bboxmin, vector<double> bboxmax);
	void expand(vector<int> bboxmin, vector<int> bboxmax);

	// Render
	// void render_to_buffer();
	// void render_to_buffer(double extra_border);
};

class VOXEL
{
public:
	// vector<BOX> cubes;
	vector< vector< vector<bool> > > map;
	vector<double> corner;
	vector<int> cornerInd;
	bool init;
	double resolution;
	int BoxVoxelVolume;
	double BoxRealVolume;

	int volume;
	double volumeDouble;

	double creationTime;
	vector<double> rotAngle;
	string name; // Name of the file it comes from

	// Binvox related stuff:
	int depth, width, height, wxh, size, version;
	byte *voxels;
	float tx, ty, tz, scale;
	
	// Read from file:
	void readBinvox(string filename);
	int get_binvox_index(int x, int y, int z);
	void voxelsTomap();
	void remove_empty_slices();

	// Track the triangles on the shell:
	vector<vector<vector<vector<int>>>> shellCubes;

	// Constructor:
	VOXEL();

	// Transformations:
	void move(vector<double> mvec);
	void rotate(int ax, double alpha);

	// Phi function:
	double phi_voxel_no_rot(VOXEL V2, int &mvdir);
	double phi_voxel(VOXEL V2, int &mvdir);

	// TOOLS: 
	bool voxel_voxel_intersection(VOXEL &V2);
	bool voxel_voxel_intersection_res(VOXEL &V2);
	bool voxel_voxel_intersection(VOXEL &V2, vector<int> indV1, vector<int> indV2);
	int slice_usage(int sliceIdx, int sliceCoord);

	// GL: // This was moved to SCENE
	// void render(vector<double> voxel_color);

	// Render to file!
	void render_to_file(string dest_file);

	// Getters, setters...
	bool get_map_value(int i, int j, int k);
	int get_grid_size(int coord);

};

class POLYTOPE: public POINTCLOUD
{
public:
	// From POINTCLOUD:
	//int 		n_vertices;
	//DOUBLEMAT 	vertices;
	string modelname;
	int 		n_faces;
	int 		index;
	double volume;
	
	INTMAT		faces;
	DOUBLEMAT	facenormals; // Normal vector at each face
	DOUBLEMAT	vertexnormals; // Normal vector at each vertex (averaged)
	INTMAT		vertextrack; // Track in which faces the vertex is used

	vector<float> colour;
	// Bounding box:
	BOX box;

	// Voxel:
	VOXEL voxel;

	//vector< vector<double> > bboxvertex;
	//vector<vector<double>> bboxvertex(8, vector<double>(3));

	// Constructor and object related functions:
	POLYTOPE();
	POLYTOPE duplicate();

	// Representation aid:
	void voxelize(double resolution);
	void voxelizationExt(double resolution);
	void update_from_voxel();

	// Loading / importing functions:
	void readPLY(string filename);
	void read3DS(string filename);

	// Rendering functions:
	// void render_to_buffer(bool debugColors);
	// void render_bbox();

	// Construction (find bounding box, convex hull, etc...)
	// void expand_bbox();

	// Transformations:
	void move(vector<double> mvec);
	void place_at(vector<float> mvec);
	void place_at(vector<double> mvec);
	void move(vector<double> mvec, bool onlyVoxel);
	void rotate(int ax, double alpha);

	// Intersection, etc...
	bool intersect(POLYTOPE target);
	bool in_bbox(vector<double> point);
	bool moller_intersection(int faceNumber, POLYTOPE target, int targetFaceNumber);
	// Save / Export functions:

};

class PACKING_OPTIONS
{
public:
	double resolution;
	int method;
	string file;
	string nfvfile;
	string solutionFile;

	// Common for heuristic methods:
	int maxIters;
	int maxKicks;
	int tabuSize;
	double itsIterationMultiplier;
	double itsDecreaseMultiplier;
	int maxPerturbations;
	double probChange;
	double maxTime;
	bool doCompaction;
	int oscillationMode;
	double decPercentage;
	int fitnessTypePar;
	string fitnessString;
	string idOfRun;


	// --- Algorithm specific --- //

	// Hill climbing:
	string HCtype; // "closest" or "steepest"

	// Compaction IP
	int initDelta;
	int deltaPumps;

	// Constructor
	PACKING_OPTIONS();
	~PACKING_OPTIONS();

	// SA options
	double sat0;
	int satype;

	// IVNS
	int modelBoxOverlap = 10;
	int modelBoxNonOverlap = 1;
	int modelMaxOverlapPairs = 2;

	// Few parameters that might come in handy:
	int int1;
	int int2;
	int int3;
	double double1;
	double double2;
	double double3;

	// Viewer options:
	bool vLoadNFV;
	string vleadString;
	int vfirstFile;
	int vlastFile;

	int randomSeed;
// private:

};

class NFVXNODE
{
public:

	NFVXNODE(int value);
	int value;
// private:
	vector<int> zpoints;
	static bool sort_less_than(NFVXNODE i, NFVXNODE j);
	static bool sort_less_than_zpoints(int i, int j);
};

class NFVYNODE
{
	// This is the top Node!
public:
	NFVYNODE(int value);
	int value;
// private:
	vector<NFVXNODE> xpoints;
	static bool sort_less_than(NFVYNODE i, NFVYNODE j);
};

class NFV // No-fit Voxel
{
	public:
		vector<NFVYNODE> ypoints;
		NFV(int pfix, int pmov, vector<POLYTOPE*> pc, bool fromFile);
		NFV(int i, int j, vector<POLYTOPE*> pieces);
		// NFV();
		// vector<vector<int>> validPoints;
		
		int nPoints;

		// Check if it contains a point or not:
		bool get_grid_size(int dim);
		bool has_point(int i, int j, int k);
		bool has_point(int i, int j, int k, bool reverse);
		bool has_point_reverse(int i, int j, int k);
		int closest_point(int &i, int &j, int &k);
		int closest_point(int &i, int &j, int &k, int &firstMinDisp);

		void add_point(int i, int j, int k);
	private:
		// NFV properties:
		int idx_i;
		int idx_j;
		int size_i;
		int size_j;
		int size_k;
		float density;
		float creationTime;

		// Points:


};

class NFVSTRUCTURE
{
	// Structure that is created from an instance and holds all the NFVs
	// and contains functions to access them easily:

public:
	NFVSTRUCTURE();
	NFVSTRUCTURE(vector<POLYTOPE> collection);
	NFVSTRUCTURE(vector<POLYTOPE*> collection);
	NFVSTRUCTURE(string filename, vector<POLYTOPE*> pc);

	// Read the contents from a file:
	NFVSTRUCTURE(string filename);

	void fix_addresses();
	bool serialise(string filename);
	bool deserialise(string filename, vector<POLYTOPE*> pc);
	static void ds_error(string foundstring, string comparestring);

	// Access members:
	bool nfv_has_point(int nfv1, int i1, int j1, int k1, int nfv2, int i2, int j2, int k2);
	// bool nfv_has_point(int nfv1, int nfv2, vector < int > point);
	bool nfv_has_point(int &nfv1, vector<int> &placedHere, int &nfv2, vector < int > &placedThere);
// 	int getNFV(NFV &myNFV);
	NFV *  getNFV(int i, int j);
private:
	void generateNFVs(vector<POLYTOPE*> collection);
	vector<NFV> NFVvec;
	vector< vector < NFV* > > NFVaddresses;

};



// Read configuration:
extern void read_configuration(PACKING_OPTIONS &packing_options);
extern void read_configuration(PACKING_OPTIONS &packing_options, string filename);

// extern void read_configuration(double &resolution, int &method);


// External functions for basic and repetitive scene tasks:
extern void modify_view(double c_x, double c_y, double c_z, double diam, double zNear);

// Basic math functions:
extern double dotprod(vector<double> a, vector<double> b);
extern void crossprod(vector<double> v1, vector<double> v2, vector<double> &result);
void rotate_vertex(vector<double> &vertex, vector<double> angle);

//extern int rc_evaluateBinomialCoefficient(int nValue, int nValue2);
//extern int rc_factorial(int nValue);



// Formatting functions:
extern void cout_point3(vector<int> point);
extern void cout_point3(vector<double> point);
extern void cout_point3(vector<float> point);
extern void cout_point3(int i, int j, int k);
extern void cout_point3(double x, double y, double z);
/*extern const char * print_point3(vector<double> point);
extern const char * print_point3(vector<int> point)*/;

// Global variables:
// extern vector<POLYTOPE*>  globalElements;

// Functions from external code:
// extern int triBoxOverlap(float boxcenter[3], float boxhalfsize[3],float triverts[3][3]);


#endif
