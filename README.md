# Introduction
This repository contains the C++ code that implements the algorithms described in the article:

C.Lamas-Fernandez, A. Martinez-Sykora, J. Bennell, _Voxel-based solution approaches to the 3D irregular packing problem_, 2021

It implements a number of algorithms that aim to pack three dimensional irregular items (discretised in voxel representation) in a three dimensional container where two dimensions are fixed and one is open.

# Code compilation
## Windows:
The compilation in Windows has been tested with Microsoft Visual Studio's command line ```cl.exe``` tool. It requires ```cplex``` to be installed in ```<CPLEX_FOLDER>```. The command below can be used with ```cplex 12.8```:
```bash
cl /EHsc /W2 /DIL_STD /Ox /MD /I "<CPLEX_FOLDER>concert\include" /I "<CPLEX_FOLDER>cplex\include" *.cc cplex1280.lib ilocplex.lib concert.lib /link /LIBPATH:"<CPLEX_FOLDER>cplex\lib\x64_windows_vs2017\stat_mda" /LIBPATH:"<CPLEX_FOLDER>concert\lib\x64_windows_vs2017\stat_mda"
```
## Linux:
The compilation in Linux has been tested with 
 ```gcc 6.1.0``` and ```cplex 12.6.1```. If cplex is installed in ```<CPLEX_FOLDER>``` the command below can be used to compile the code (from the ```src``` folder):

```bash
g++ -std=c++11 -fpermissive -DIL_STD -I <CPLEX_FOLDER>/concert/include -I <CPLEX_FOLDER>/cplex/include *.cc -O2 -fPIC -lilocplex -lconcert -lcplex -lm -lpthread -L <CPLEX_FOLDER>/cplex/lib//x86-64_linux/static_pic/ -L <CPLEX_FOLDER>/concert/lib/x86-64_linux/static_pic
```

# Problem instances

The code works with voxelised representations of items in the ```binvox``` format. You can learn more about this format and how to convert 3D mesh models to it in [Patrick Min's website](https://www.patrickmin.com/binvox/).
The actual datasets used in the article can be accessed from the [ESICUP website](https://www.euro-online.org/websites/esicup/data-sets/).
The data instances are given in ```.txt``` files that follow the following format:
```
CONTAINER	sizeX	sizeY	-1
P1_filename.binvox	Number_of_times_P1_is_included
P2_filename.binvox	Number_of_times_P2_is_included
...
```

The size of the container is in "metric" units (as opposed to number of voxels), and independent of the voxelisation of the pieces. The actual match between these units and the voxel size is controlled by a ```resolution``` parameter.

# Quick guide

The main algorithms are implemented in the following methods of the ```PACKING_LAYOUT``` class:

## Iterated Local Search
File: ```3dpack.cc```
Relevant functions:
```c++
void PACKING_LAYOUT::ils()
```

## Iterated Tabu Search
File: ```3dpack.cc```
Relevant functions:
```c++
void PACKING_LAYOUT::its()
void PACKING_LAYOUT::its(int lob, int upb)
```

## Variable Neighbourhood Search
File: ```3dpack.cc```
Relevant functions:
```c++
void PACKING_LAYOUT::ivns()
void PACKING_LAYOUT::ivns(int lob, int upb)
```
Note: The code often refers to this method as _Iterated_ Variable Neighbourhood Search and its acronym, ivns.

