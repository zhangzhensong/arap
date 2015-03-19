#include "Geometry2D.h"
#include <malloc.h>

Geometry2D::Geometry2D(void)
{
}


Geometry2D::~Geometry2D(void)
{
}

void Geometry2D::InitTriangulateio(triangulateio ** pptio)
{
	(*pptio) = (triangulateio*)malloc(sizeof(triangulateio));

	struct triangulateio * tio = (*pptio);

	tio->pointlist = 0;                      
	tio->pointattributelist = 0;             
	tio->pointmarkerlist = 0;                
	tio->numberofpoints = 0;                 
	tio->numberofpointattributes = 0;        

	tio->trianglelist = 0;                   
	tio->triangleattributelist = 0;          
	tio->trianglearealist = 0;               
	tio->neighborlist = 0;                   
	tio->numberoftriangles = 0;              
	tio->numberofcorners = 0;                
	tio->numberoftriangleattributes = 0;     

	tio->segmentlist = 0;                    
	tio->segmentmarkerlist = 0;              
	tio->numberofsegments = 0;               

	tio->holelist = 0;                       
	tio->numberofholes = 0;                  

	tio->regionlist = 0;                     
	tio->numberofregions = 0;                

	tio->edgelist = 0;               
	tio->edgemarkerlist = 0;         
	tio->normlist = 0;               
	tio->numberofedges = 0;          
}

void Geometry2D::FreeTriangulateio(triangulateio ** pptio, bool in)
{
	struct triangulateio * tio = (*pptio);
	if(tio->pointlist != 0) free(tio->pointlist);                     
	if(tio->pointattributelist != 0) free(tio->pointattributelist);
	if(tio->pointmarkerlist != 0) free(tio->pointmarkerlist);               
	
	if(tio->trianglelist != 0) free(tio->trianglelist);                  
	if(tio->triangleattributelist != 0) free(tio->triangleattributelist);          
	if(tio->trianglearealist != 0) free(tio->trianglearealist);               
	if(tio->neighborlist != 0) free(tio->neighborlist);                   
	
	if(tio->segmentlist != 0) free(tio->segmentlist);                    
	if(tio->segmentmarkerlist != 0) free(tio->segmentmarkerlist);             

	if(in) // only allocalte mem for "in" triangulateio
		if(tio->holelist != 0) 
			free(tio->holelist);                      

	if(in) // only allocalte mem for "in" triangulateio
		if(tio->regionlist != 0) 
			free(tio->regionlist);                     

	if(tio->edgelist != 0) free(tio->edgelist);               
	if(tio->edgemarkerlist != 0) free(tio->edgemarkerlist);        
	if(tio->normlist != 0) free(tio->normlist);              

	free(*pptio);
	(*pptio) = 0;
}

triangulateio * Geometry2D::InputToTriangulateio(const std::vector<v2d> & input_points, const std::vector<v2i> & input_segments)
{
	struct triangulateio * ans;
	Geometry2D::InitTriangulateio(&ans);
	
	ans->numberofpoints = (int)(input_points.size());

	if(ans->numberofpoints == 0) return ans;
	
	ans->pointlist = (REAL *) malloc(ans->numberofpoints * 2 * sizeof(REAL));

	for(int i = 0;i<ans->numberofpoints;i++)
	{
		ans->pointlist[i*2]	 = input_points[i][0];
		ans->pointlist[i*2+1]= input_points[i][1];
	}

	ans->numberofsegments = (int)(input_segments.size());

	if (ans->numberofsegments > 0)
	{
		ans->segmentlist = (int *)malloc(ans->numberofsegments * 2 * sizeof(int));

		for(int i = 0;i<ans->numberofsegments;i++)
		{
			ans->segmentlist[i*2]   = input_segments[i][0];
			ans->segmentlist[i*2+1] = input_segments[i][1];
		}
	}
	

	return ans;
}

triangulateio* Geometry2D::ComputeMeshByTriangle(triangulateio * tio)
{
	struct triangulateio * ans, * vorout;
	Geometry2D::InitTriangulateio(&ans);
	Geometry2D::InitTriangulateio(&vorout);

	//triangulate("zpqa500Q",tio,ans,vorout);
	triangulate("zpcQY",tio,ans,vorout);

	Geometry2D::FreeTriangulateio(&vorout);

	return ans;
}

void Geometry2D::TriangulateioToOutput(triangulateio * tio, vector<Vertex> & vertices, vector<Triangle>& triangles)
{
	for(int i = 0;i<tio->numberofpoints;i++)
	{
		Vertex v(tio->pointlist[i*2], tio->pointlist[i*2+1]);
		vertices.push_back(v);
	}

	for(int i = 0;i<tio->numberoftriangles;i++)
	{
		unsigned int tri[3] = { tio->trianglelist[i * 3] , tio->trianglelist[i * 3 + 1], tio->trianglelist[i * 3 + 2] };
		Triangle t;
		for (int j = 0; j < 3; j++)
		{
			t.nVertices[j] = tri[j];
		}
		triangles.push_back(t);
	}
}
