#pragma once
#include "common.h"
#include "ArapDeform.h"
#include <Eigen/Dense>
#include <vector>

using namespace std;
using namespace zzs;

class Geometry2D
{
public:
	Geometry2D(void);
	~Geometry2D(void);

	/* 2D triangle mesh 
		triangulateio is the basic structure of Triangle. The tasks are:
		1. what's the form of your input?
		2. change your input to get its corresponding triangulateio
		3. what's the form of your output?
		4. get your output from triangulateio
	*/
	static void					InitTriangulateio			(triangulateio ** pptio);
	static void					FreeTriangulateio			(triangulateio ** pptio, bool in = false);
	
	static triangulateio *		InputToTriangulateio		(const std::vector<v2d> & input_points, const std::vector<v2i> & input_segments);
	static triangulateio *		ComputeMeshByTriangle		(triangulateio * tio);
	static void					TriangulateioToOutput		(triangulateio * tio, vector<Vertex> & vertices, vector<Triangle>& triangles);
};

