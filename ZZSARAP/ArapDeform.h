#pragma once
#include <Eigen/Dense>
#include <vector>
#include <set>

#include "common.h"

using namespace Eigen;
typedef Eigen::Vector2f Point;

namespace zzs {
	struct Vertex
	{
		v2f vPosition;
		Vertex(){vPosition << 0,0;}

		Vertex(float x, float y)
		{
			vPosition << x,y;
		}

	};

	struct Triangle
	{
		size_t nVertices[3];

		// definition of each vertex in triangle-local coordinate system
		v2f vTriCoords[3];
	};

	struct Constraint
	{
		size_t nVertex;      // point index
		v2f vConstrainedPos; // the current position of the constraint point

		Constraint(){nVertex = 0; vConstrainedPos = Eigen::Vector2f::Zero();}
		Constraint(size_t nVert, const v2f &vPos)
		{
			nVertex = nVert;
			vConstrainedPos = vPos;
		}

		bool operator <(const Constraint& c2) const
		{
			return nVertex < c2.nVertex;
		}
	};

	class ArapDeform
	{
	public:
		ArapDeform(void);
		~ArapDeform(void);

		// add point to build mesh
		void setAddPoint();         
		void addPoint(Vertex &);

		// use the newly add point to build mesh
		// precompute the triangle local coordinate
		void buildMesh();

		void clearData();
		const std::vector<Vertex> & getVerts() const;
		const std::vector<Vertex> & getDeformedVerts() const;
		const std::vector<Triangle> & getTriangles() const;

		size_t findHitVertex(float x, float y);
		void setVertex(size_t index, Point position);

		// Precompute Gprime and B
		void updateConstraints(const std::set<size_t>& selected);
		void updateMesh(bool isRigid);

		bool isAddPoint() const;

	private:
		// initial input
		std::vector<Vertex> m_vInitialVertices;
		std::vector<Triangle> m_vTriangles;

		// output
		std::vector<Vertex> m_vDeformedVertices;
		
		// control points
		std::set<Constraint> m_vConstraints;

		bool bAddPt;
		int nBoundaryPoints;

		// these three matrix should be precompute according to Eq(6)-(8)
		MatrixXd m_G;
		MatrixXd m_Gprime;
		MatrixXd m_B;

		// m_FirstMatrix = -G'.inverse() * B
		MatrixXd m_FirstMatrix;

		std::vector<size_t> m_vVertexMap;
	};
}