#include "3dlib.h"
#include <string>
#include <fstream>
#include <iostream>
#include <cmath>

using namespace std;

VOXEL::VOXEL()
{
	init = false;
	resolution = 0.1;
	creationTime = -1;
	rotAngle.assign(3, 0);
	cornerInd.assign(3, 0);
	corner.assign(3, 0);
	name = "empty dir";
	// cout << "Initialized voxel!" << endl;
}

void VOXEL::move(vector<double> mvec)
{
	for (int i = 0; i < 3; i++)
	{
		corner[i] += mvec[i];
	}
	
}

void VOXEL::render_to_file(string dest_file)
{
	// Open file:
	FILE * vfile = fopen(dest_file.c_str(), "w");


	// Initialize vars for the loop:
	int mapsize_i = map.size();
	int mapsize_j = map[0].size();
	int mapsize_k = map[0][0].size();

	// Start writing:
	fprintf(vfile, "function load_thing() {\n\tvar coordinates = [\n");

	// Write header:
	fprintf(vfile, "// res\t%.4f\n", resolution);
	fprintf(vfile, "// pos\t%.4f\t%.4f\t%.4f\n", corner[0], corner[1], corner[2]);

	bool isFirst = true;
	for (int i = 0; i < mapsize_i; ++i)
	{
		for (int j = 0; j < mapsize_j; ++j)
		{
			for (int k = 0; k < mapsize_k; ++k)
			{
				if (!map[i][j][k])
				{
					// ++negative;
					// glColor4f(1.0f / div, 1.0f / div, 1.0f / div, 0.2f);
					continue;
				}

				// If i, j and k are > 0 and less than the border, do further checks:
				if (i > 0 && j > 0 && k > 0 && i < mapsize_i - 1 && j < mapsize_j -1 && k < mapsize_k - 1)
				{
					// If it's fully surrounded, we don't need to draw it:
					if (map[i - 1][j][k] && map[i][j - 1][k] && map[i][j][k - 1] && map[i + 1][j][k] && map[i][j + 1][k] && map[i][j][k + 1])
						continue;
				}

				if (isFirst)
				{
					fprintf(vfile, "\t\t[%d,\t%d,\t%d]", i, j, k);
					isFirst = false;
				}
				else
				{
					fprintf(vfile, ",\n\t\t[%d,\t%d,\t%d]", i, j, k);
				}

			}
		}
	}

	fprintf(vfile, "\n];\n");
	fprintf(vfile, "\nreturn coordinates;\n");
	fprintf(vfile, "}\n");
	fclose(vfile);
}


double VOXEL::phi_voxel_no_rot(VOXEL V2, int &mvdir)
{
	// mvdir returns one guess where you could move V1 to make contact with V2 (!)
	double phi_val = 1e30;
	double aux_phi_val, voxel_phi_val;
	double twoRes = resolution/2 + V2.resolution/2;
	int aux_mdir;

	for (int i = 0; i < map.size(); ++i)
	{
		for (int j = 0; j < map[i].size(); ++j)
		{
			for (int k = 0; k < map[i][j].size(); ++k)
			{
				// If this voxel is not used in V1, skip it:
				if (!map[i][j][k])
					continue;

				// If it is used, check it against all the voxels in V2:
				for (int i2 = 0; i2 < V2.map.size(); ++i2)
				{
					for (int j2 = 0; j2 < V2.map[i2].size(); ++j2)
					{
						for (int k2 = 0; k2 < V2.map[i2][j2].size(); ++k2)
						{
							// If this voxel in V2 is not used, skip the test
							if (!V2.map[i2][j2][k2])
								continue;
							// Both voxels are used, check the phi_value between them, and store the maximum of the three:
						
							// coord x
							voxel_phi_val = abs((corner[0] + i*resolution) - (V2.corner[0] + i2*V2.resolution)) - twoRes;
							aux_mdir = 0;
							// coord y
							aux_phi_val = abs((corner[1] + j*resolution) - (V2.corner[1] + j2*V2.resolution)) - twoRes;
							if (voxel_phi_val < aux_phi_val)
							{
								voxel_phi_val = aux_phi_val;
								aux_mdir = 1;
							}
							

							// coord z
							aux_phi_val = abs((corner[2] + k*resolution) - (V2.corner[2] + k2*V2.resolution)) - twoRes;
							if (voxel_phi_val < aux_phi_val)
							{
								aux_mdir = 2;
								voxel_phi_val = aux_phi_val;
							}
							

							// If this value is smaller than the original, store it:
							if (voxel_phi_val < phi_val)
							{
								phi_val = voxel_phi_val;
								mvdir = aux_mdir;
							}
							//if (phi_val < TOL && phi_val > -TOL)
							//printf("Found a 0!\n");
							// printf("[%i, %i, %i] vs [%i, %i, %i]: %f (dir: %i)\n", i, j, k, i2, j2, k2, phi_val, mvdir);
							


						} // end of i2, V2
					} // end of j2, V2
				} // end of k2, V2


			} // end of i, V1
		} // end of j, V1
	} // end of k, V1


	return phi_val;
}

bool VOXEL::voxel_voxel_intersection_res(VOXEL &V2)
{
	vector<int> V1cornerIndex(3);
	vector<int> V2cornerIndex(3);
	for (int i = 0; i < 3; i++)
	{
		V1cornerIndex[i] = round(corner[i] / resolution);
		V2cornerIndex[i] = round(V2.corner[i] / V2.resolution);
	}

	cornerInd = V1cornerIndex;
	V2.cornerInd = V2cornerIndex;

	return voxel_voxel_intersection(V2);
}

bool VOXEL::voxel_voxel_intersection(VOXEL &V2)
{
	return voxel_voxel_intersection(V2, cornerInd, V2.cornerInd);
}

bool VOXEL::voxel_voxel_intersection(VOXEL &V2, vector<int> indV1, vector<int> indV2)
{

	vector<int> voxel2coord(3);
	const int V2mapSizeMin0 = V2.map.size() - 1;
	const int V2mapSizeMin1 = V2.map[0].size() - 1;
	const int V2mapSizeMin2 = V2.map[0][0].size() - 1;

	const int V1mapSize0 = map.size();
	const int V1mapSize1 = map[0].size();
	const int V1mapSize2 = map[0][0].size();
	//V1cornerIndex = { round(corner[0] / resolution), round(corner[1] / resolution), round(corner[2] / resolution) };

	// for (int i = 0; i < map.size(); ++i)
	for (int i = 0; i < V1mapSize0; ++i)
	{
		voxel2coord[0] = indV1[0] + i - indV2[0];
        // Find the corresponding coordinate of this voxel in V2:
        if (voxel2coord[0] < 0 || voxel2coord[0] > V2mapSizeMin0)
            continue;

		// for (int j = 0; j < map[i].size(); ++j)
		for (int j = 0; j < V1mapSize1; ++j)
		{
			voxel2coord[1] = indV1[1] + j - indV2[1];
            if (voxel2coord[1] < 0 || voxel2coord[1] > V2mapSizeMin1)
                continue;

			for (int k = 0; k < V1mapSize2; ++k)
			{
				voxel2coord[2] = indV1[2] + k - indV2[2];

				// If this voxel is not used in V1, skip it:
				if (!map[i][j][k])
					continue;

				if (voxel2coord[2] < 0 || voxel2coord[2] > V2mapSizeMin2)
					continue;

				// if (V2.map[voxel2coord[0]][voxel2coord[1]][voxel2coord[2]])
				if (V2.get_map_value(voxel2coord[0], voxel2coord[1], voxel2coord[2]))
					return true;


			} // end of i, V1
		} // end of j, V1
	} // end of k, V1

	return false;
}


double VOXEL::phi_voxel(VOXEL V2, int &mvdir)
{


	double minPointPlaneDist = 0; // When comparing two cubes, the minimum vertex to face distance
	double minCubeCubeDist, minPointPlaneDistV2; // The minimum distance between two cubes
	vector<int> coordsV1(3), coordsV2(3); // Store the integer coordinates of the cubes being analysed.
	vector<double> cubeCornerV1(3), cubeCornerV2(3); // Store the actual coordinates here.
	double aux, aux2;

	// Aux. vars for the edge-edge contact:
	vector<double> nu(3), tau(3);

	for (int i = 0; i < map.size(); ++i)
	{
		for (int j = 0; j < map[i].size(); ++j)
		{
			for (int k = 0; k < map[i][j].size(); ++k)
			{
				// If this voxel is not used in V1, skip it:
				if (!map[i][j][k])
					continue;

				coordsV1 = { i, j, k };
				// rotate_vertex(coordsV1, rotAngle);

				// Enter now the second voxel:
				for (int iv2 = 0; iv2 < map.size(); ++iv2)
				{
					for (int jv2 = 0; jv2 < map[i].size(); ++jv2)
					{
						for (int kv2 = 0; kv2 < map[i][j].size(); ++kv2)
						{
							// If this voxel is not used in V1, skip it:
							if (!map[iv2][jv2][kv2])
								continue;

							coordsV2 = { iv2, jv2, kv2 };
							

							// STEP 1: 
							// Check all the vertices of the one cube from V1 against the faces of the cube in V2 and vice-versa:
							//minPointPlaneDist = 1e32; // Set the distance to Inf. as we want to know the max.
							//minPointPlaneDistV2 = 1e32; // Same for the V2 vs V1 computation.
							
							
							for (int axis = 0; axis < 3; axis++)
							{
								// Find the corner of the two cubes:
								cubeCornerV1[axis] = corner[coordsV1[axis]] + resolution*coordsV1[axis];
								cubeCornerV2[axis] = V2.corner[coordsV2[axis]] + V2.resolution*coordsV2[axis];
								//rotate_vertex(coordsV2, V2.rotAngle);
								// Go through the 8 points of each cube and look for the minimum point-plane distance

								// +0
								minPointPlaneDist = cubeCornerV2[axis] - cubeCornerV1[axis];
								aux = cubeCornerV2[axis] + 2 * resolution - cubeCornerV1[axis];
								if (aux < minPointPlaneDist)
									{
										minPointPlaneDist = aux;
										mvdir = axis;
									}

								minPointPlaneDistV2 = cubeCornerV1[axis] - cubeCornerV2[axis];
								aux = cubeCornerV1[axis] + 2 * resolution - cubeCornerV2[axis];
								if (aux < minPointPlaneDistV2)
									{
										minPointPlaneDistV2 = aux;
										mvdir = axis;
									}

								// +2
								minPointPlaneDist = cubeCornerV2[axis] - (cubeCornerV1[axis] + 2*resolution);
								aux = cubeCornerV2[axis] + 2 * resolution - (cubeCornerV1[axis] + 2 * resolution);
								if (aux < minPointPlaneDist)
									{
										minPointPlaneDist = aux;
										mvdir = axis;
									}

								minPointPlaneDistV2 = cubeCornerV1[axis] - (cubeCornerV2[axis] + 2 * V2.resolution);
								aux = cubeCornerV1[axis] + 2 * resolution - (cubeCornerV2[axis] + 2 * V2.resolution);
								if (aux < minPointPlaneDistV2)
									{
										minPointPlaneDistV2 = aux;
										mvdir = axis;
									}

							}
							
							// STEP 2:
							// Check edge-edge contact:




						} // end of i, V2
					} // end of j, V2
				} // end of k, V2


			} // end of i, V1
		} // end of j, V1
	} // end of k, V1

	// DEBUG:
	return minPointPlaneDist;

}

int VOXEL::get_binvox_index(int x, int y, int z)
{
	int index = x * wxh + z * width + y;  // wxh = width * height = d * d
	return index;
}

int VOXEL::slice_usage(int sliceIdx, int sliceCoord)
{
	int startValues[3];
	int endValues[3];
	int voxelsThere = 0;
	for (int coord = 0; coord < 3; coord++)
	{
		if (sliceCoord == coord)
		{
			startValues[coord] = sliceIdx;
			endValues[coord] = sliceIdx + 1;
		}
		else
		{
			startValues[coord] = 0;
			endValues[coord] = get_grid_size(coord);
		}

		// cout << "Coordinate " << coord << " iterates from " << startValues[coord] << " to " << endValues[coord] << endl;
	}

	for (int i = startValues[0]; i < endValues[0]; i++)
	{
		for (int j = startValues[1]; j < endValues[1]; j++)
		{
			for (int k = startValues[2]; k < endValues[2]; k++)
			{
				if (get_map_value(i, j, k))
					voxelsThere++;
				// {
				// }
				// else
				// {
					// cout << "Map for " << i << " " << j << " " << k << " was " << map[i][j][k] << endl;
				// }
			}
		}

	}

	return voxelsThere;


}

int VOXEL::get_grid_size(int coord)
{
	switch (coord)
	{
	case 0 : 
		return map.size();
		// return width;
	case 1 :
		return map[0].size();
		// return height;
	case 2 : 
		return map[0][0].size();
		// return depth;
	default:
		return -1;
	}
}
bool VOXEL::get_map_value(int i, int j, int k)
{
	// Easy way at the moment...
	return map[i][j][k];

}

void VOXEL::voxelsTomap()
{
	// Reads the voxels array and writes the map() 3d matrix

	// Reserve the necessary memory:
	// vector<bool> line(width);
	// depth = width;
	// cout << endl << depth << endl;
	// printf("\nVoxel dimensions %d, %d, %d", width, height, depth);
	vector<bool> line;
	depth = size / wxh;
	cout << endl << "\t\tVoxel dimensions " << width << ", " << height << ", " << depth << endl;
	line.assign(width, false);
	vector<vector<bool>> slice;
	slice.assign(height, line);
	map.assign(depth, slice);
	cout << endl << "\t\tMap dimensions " << map.size() << ", " << map[0].size() << ", " << map[0][0].size() << endl;
	// line.assign(width, false);
	float volumePerc = 0;
	int volumeCount = 0;
	double res3 = pow(resolution, 3);
	// printf("\nSice Vol: \n");
	for (int i = 0; i < width; i++)
	{
		// printf("%.2f\t", ((float) volumeCount) / size);
		for (int j = 0; j < height; j++)
		{
			for (int k = 0; k < depth; k++)
			{
				if (voxels[get_binvox_index(i, j, k)] == '\x1')
				{
					// cout << "true (" << i << ", " << j << ", " << k << ")\n";
					map[i][j][k] = true;
					++volumeCount;
				}
				// else
					// cout << "false\n";
			}

		}
	}

	volumePerc = float(volumeCount) / float(size);
	volume = volumeCount;
	volumeDouble = double(volume) * res3;
	// printf("\n\t\tVolume was %.2f%%\n", volumePerc);
	// cout << "\t\t(Volume count: " << volume << " )" << endl;
	// cout << "\t\t(Real volume: " << volumeDouble << ", resolution " << resolution << ")" << endl;

	// Remove the empty slices we might have:
	// cout << "\t\tremoving empty slices..." << endl;
	cout << "-----" << endl;
	remove_empty_slices();
	// cout << "\t\tdone." << endl;
	size = width * depth * height;
	cout << "\t\tVoxel size: " << size << " (" << width << " x " << height << " x " << depth << ")" << endl;
	cout << "\t\tVoxel real size: " << double(size) * res3;
	cout << " (" << width*resolution << " x " << height*resolution << " x " << depth*resolution << ")" << endl;
	volumePerc = ((float)volumeCount) / size;
	printf("\t\tVolume after removing slices is %.2f%% ", volumePerc*100);
	cout << "(Volume count: " << volume << " )" << endl;
}

void VOXEL::remove_empty_slices()
{
	// cout << endl << endl << "WARNING! Slice removal to be implemented..." << endl << endl;
	// Start removing YZ slices:
	int slicesRemoved = 0;
	int slicesRemovedFront = 0;
	for (int sliceCoord = 0; sliceCoord < 3; sliceCoord++)
	{
		for (int i = get_grid_size(sliceCoord) - 1; i >= 0; i--)
		{
			int sliceUsageValue = slice_usage(i, sliceCoord);
			// cout << "Slice " << i << " for coordinate " << 0 << " usage value is: " << sliceUsageValue << endl;
			if (sliceUsageValue == 0) 
			{
				slicesRemoved++;
				// cout << "Found an empty slice, removing..." << endl;
				if (sliceCoord == 0)
				{
					map.pop_back();
					width -= 1;
				}
				else if (sliceCoord == 1)
				{
					for (int xcoord = 0; xcoord < width; xcoord++)
					{
						map[xcoord].pop_back();
					}
					height -= 1;
				}
				else if (sliceCoord == 2)
				{
					for (int xcoord = 0; xcoord < width; xcoord++)
					{
						for (int ycoord = 0; ycoord < height; ycoord++)
						{
							map[xcoord][ycoord].pop_back();
						}
					}
					depth -= 1;
				}
			}
			else 
			{
				break;
			}

		}
	}

	// Now, do it backwards in case something is empty from the other side:
	// cout << "Current size: " << get_grid_size(0) << " x " << get_grid_size(1) << " x " << get_grid_size(2) << endl;
	int slicesChecked = 0;
	for (int sliceCoord = 0; sliceCoord < 3; sliceCoord++)
	{
		for (int i = 0; i <= get_grid_size(sliceCoord) - 1; i++)
		{
			int sliceUsageValue = slice_usage(i, sliceCoord);
			slicesChecked++;
			// cout << "Slice " << i << " for coordinate " << sliceCoord << " usage value is: " << sliceUsageValue << endl;
			if (sliceUsageValue == 0) 
			{
				i = -1;
				slicesRemovedFront++;
				// cout << "SliceCoord " << sliceCoord << endl;
				// cout << "Found an empty slice, removing..." << endl;
				if (sliceCoord == 0)
				{
					map.erase(map.begin());
					width -= 1;
				}
				else if (sliceCoord == 1)
				{
					for (int xcoord = 0; xcoord < width; xcoord++)
					{
						map[xcoord].erase(map[xcoord].begin());
					}
					height -= 1;
				}
				else if (sliceCoord == 2)
				{
					for (int xcoord = 0; xcoord < width; xcoord++)
					{
						for (int ycoord = 0; ycoord < height; ycoord++)
						{
							map[xcoord][ycoord].erase(map[xcoord][ycoord].begin());
						}
					}
					depth -= 1;
				}
			}
			else
			{
				// cout << "Exit on break. (Coord " << sliceCoord << ", index " << i << ", usage was " << sliceUsageValue << " )" << endl;
				break;
			}

		}
	}

	// Bounding box volumes:
	BoxVoxelVolume = width*depth*height;
	BoxRealVolume = BoxVoxelVolume*pow(resolution, 3);

	// cout << "Current size: " << get_grid_size(0) << " x " << get_grid_size(1) << " x " << get_grid_size(2) << endl;
	// cout << "Checked " << slicesChecked << " front slices." << endl;
	// cout << "\t\t\tRemoved " << slicesRemoved + slicesRemovedFront << "slices." << endl;
	// cout << "\t\t\t(" << slicesRemoved << " back + " <<  slicesRemovedFront << " front)" << endl;
}

void VOXEL::readBinvox(string filespec)
{

// Code from: http://www.cs.princeton.edu/~min/binvox/read_binvox.cc
// (Modified for compatibility)

	/* 
	OUTPUTS:
	
	- depth
	- version
	- height
	- width
	- tx, ty, tz
	- scale
	- size
	- voxels
	*/
	// int version;
	// int depth, height, width;
	// int size;
	// byte *voxels = 0;
	// float tx, ty, tz;
	// float scale;
	cout << "Reading binvox file: " << filespec.c_str() << endl;
	ifstream *input = new ifstream(filespec.c_str(), ios::in | ios::binary);
	name.assign(filespec);
	//
	// read header
	//
	string line;
	
	// line.reserve(200);
	*input >> line;  // #binvox
	if (line.compare("#binvox") != 0) {
		cout << "Error: first line reads [" << line << "] instead of [#binvox]" << endl;
		delete input;
		return;
		// return 0;
	}
	*input >> version;
	// cout << "\tbinvox version " << version << endl;

	int depth = -1;
	int done = 0;
	while (input->good() && !done) {
		*input >> line;
		if (line.compare("data") == 0) done = 1;
		else if (line.compare("dim") == 0) {
			*input >> depth >> height >> width;
		}
		else if (line.compare("translate") == 0) {
			*input >> tx >> ty >> tz;
		}
		else if (line.compare("scale") == 0) {
			*input >> scale;
		}
		else {
			cout << "  unrecognized keyword [" << line << "], skipping" << endl;
			char c;
			do {  // skip until end of line
				c = input->get();
			} while (input->good() && (c != '\n'));

		}
	}
	if (!done) {
		cout << "  error reading header" << endl;
		return;
		// return 0;
	}
	if (depth == -1) {
		cout << "  missing dimensions in header" << endl;
		return;
		// return 0;
	}

	size = width * height * depth;
	// printf("\nVoxel dimensions %d, %d, %d", width, height, depth);
	// wxh = width * height;
	wxh = width * height;
	voxels = new byte[size];
	if (!voxels) {
		cout << "  error allocating memory" << endl;
		// return 0;
		return;
	}

	//
	// read voxel data
	//
	byte value;
	byte count;
	int index = 0;
	int end_index = 0;
	int nr_voxels = 0;

	input->unsetf(ios::skipws);  // need to read every byte now (!)
	*input >> value;  // read the linefeed char

	while ((end_index < size) && input->good()) {
		*input >> value >> count;

		if (input->good()) {
			end_index = index + count;
			if (end_index > size)
				exit(-1);
				// return 0;
			for (int i = index; i < end_index; i++) 
				voxels[i] = value;

			if (value) nr_voxels += count;
			index = end_index;
		}  // if file still ok

	}  // while

	input->close();
	line.clear();
	// cout << "\tread " << nr_voxels << " voxels" << endl;

	// Convert to map:
	corner.assign(3, 0);
	// corner.reserve(3);

	// corner.push_back(tx);
	// corner.push_back(ty);
	// corner.push_back(tz);

	// cout << "\tconverting voxels to map..." << endl;
	voxelsTomap();
	// cout << "\tdone." << endl;

	cout << "Done." << endl << endl;

}
