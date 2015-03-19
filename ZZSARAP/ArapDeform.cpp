#include <iostream>

#include "ArapDeform.h"
#include "Geometry2D.h"
#include "triangle.h"
#include "common.h"

using namespace zzs;
using namespace std;

ArapDeform::ArapDeform(void)
{
	// initial input
	clearData();
	nBoundaryPoints = 10;
}


ArapDeform::~ArapDeform(void)
{
}

void ArapDeform::setAddPoint()
{
	bAddPt = !bAddPt;
}

bool ArapDeform::isAddPoint() const
{
	return bAddPt;
}

void ArapDeform::buildMesh()
{
	if(m_vInitialVertices.size() < 3)
	{
		cout << "Not enough points for triangle, please add more points!"<< endl;
		return;
	}

	vector<v2d> vPoints;
	vPoints.clear();

	for (int i = 0; i < m_vInitialVertices.size(); i++)
	{
		vPoints.push_back(v2d(m_vInitialVertices[i].vPosition[0], m_vInitialVertices[i].vPosition[1]));
	}

	std::vector<v2i> segments;
	segments.clear();
	triangulateio *in = Geometry2D::InputToTriangulateio(vPoints, segments);
	triangulateio *out=	Geometry2D::ComputeMeshByTriangle(in);

	vector<Vertex> vertices;
	vertices.clear();
	m_vTriangles.clear();

	Geometry2D::TriangulateioToOutput(out, vertices, m_vTriangles);
	//cout << "Compare initial vertices and final vertices" << endl;
	//cout << vertices.size() << " " << m_vInitialVertices.size() << endl;
	//
	//for (int i = 0; i < vertices.size(); i++)
	//{
	//	cout << vertices[i].vPosition.transpose() << " <=> " << m_vInitialVertices[i].vPosition.transpose() << endl;
	//}

	// let's setup triangle-local coordinate systems
	size_t nTriangle = m_vTriangles.size();
	for (size_t i = 0; i < nTriangle; i++)
	{
		Triangle &t = m_vTriangles[i];
		for (size_t j = 0; j < 3; j++)
		{
			size_t n0 = j;
			size_t n1 = (j + 1) % 3;
			size_t n2 = (j + 2) % 3;

			v2f v0 = m_vInitialVertices[t.nVertices[n0]].vPosition;
			v2f v1 = m_vInitialVertices[t.nVertices[n1]].vPosition;
			v2f v2 = m_vInitialVertices[t.nVertices[n2]].vPosition;

			// find coordinate system
			v2f v01(v1 - v0);
			v2f v01N(v01); 
			v01N.normalize();
			v2f v01Rot90(v01.y(), -v01.x());
			v2f v01Rot90N(v01Rot90); 
			v01Rot90N.normalize();

			// express v2 in coordinate system
			v2f vLocal(v2 - v0);
			float fX = vLocal.dot(v01) / v01.squaredNorm();
			float fY = vLocal.dot(v01Rot90) / v01Rot90.squaredNorm();
#if 0
			// check v2 is right or not
			v2f v2test(v0 + fX * v01 + fY * v01Rot90);
			float fLength  = (v2test - v2).norm();

			cout <<  "compare v2: " << fLength << endl;
#endif
			t.vTriCoords[j] = v2f(fX, fY);

		}
	}


	Geometry2D::FreeTriangulateio(&in);
	Geometry2D::FreeTriangulateio(&out);
}

void ArapDeform::clearData()
{
	// initial input
	m_vInitialVertices.clear();
	m_vTriangles.clear();

		// output
	m_vDeformedVertices.clear();
		
		// control points
	m_vConstraints.clear();

	bAddPt = true;
}

void ArapDeform::addPoint(Vertex &v)
{
	m_vInitialVertices.push_back(v);
	m_vDeformedVertices.push_back(v);
}

const std::vector<Vertex> & ArapDeform::getVerts() const
{
	return m_vInitialVertices;
}

const std::vector<Triangle> & ArapDeform::getTriangles() const
{
	return m_vTriangles;
}

const std::vector<Vertex> & ArapDeform::getDeformedVerts() const
{
	return m_vDeformedVertices;
}

size_t ArapDeform::findHitVertex(float x, float y)
{
	size_t nVerts = m_vDeformedVertices.size();
	float fx, fy;

	for (size_t i = 0; i < nVerts; i++)
	{
		fx = m_vDeformedVertices[i].vPosition[0];
		fy = m_vDeformedVertices[i].vPosition[1];
		double disSquare = (x - fx) * (x - fx) + (y - fy) * (y - fy);
		if (disSquare < 25)
		{
			return i;
		}
	}

	return std::numeric_limits<size_t>::max();
}

void ArapDeform::setVertex(size_t index, Point position)
{
	m_vDeformedVertices[index].vPosition = position;



}

void ExtractSubMatrix( MatrixXd & mFrom, int nRowOffset, int nColOffset, MatrixXd & mTo )
{
	int nRows = mTo.rows();
	int nCols = mTo.cols();

	for ( int i = 0; i < nRows; ++i ) {
		for ( int j = 0; j < nCols; ++j ) {
			mTo(i,j) = mFrom( i + nRowOffset, j + nColOffset );
		}
	}
}


// In this function, we are going to calculate the  Gprime and B of Eq(8)
void ArapDeform::updateConstraints(const std::set<size_t>& selected)
{
	std::set<size_t>::iterator cur(selected.begin()), end(selected.end());
	size_t nSelected;
	m_vConstraints.clear();
	
	while (cur != end)
	{
		nSelected = *cur++;
		Constraint c(nSelected, m_vInitialVertices[nSelected].vPosition);
		m_vConstraints.insert(c);
	}

	cout << "Finished update constraints! --> " << m_vConstraints.size() << endl;
	size_t nDeformedVerts = m_vDeformedVertices.size();
	size_t nInitialVerts = m_vInitialVertices.size();

	for (size_t i = 0; i < nDeformedVerts; i++)
	{
		if (selected.find(i) == end)
		{
			m_vDeformedVertices[i].vPosition = m_vInitialVertices[i].vPosition;
		}
	}


	cout << "No. of deformed vertices and intial vertices: " << nDeformedVerts << " " << nInitialVerts << endl;

	// resize matrix G and clear to zero
	size_t nGsize = 2 * nInitialVerts;
	m_G.resize(nGsize, nGsize);

	for (size_t i = 0; i < nGsize; i++)
	{
		for (size_t j = 0; j < nGsize; j++)
		{
			m_G(i,j) = 0;
		}
	}

	size_t nConstraints = m_vConstraints.size();
	size_t nFreeVerts = nInitialVerts - nConstraints;

	// m_vVertexMap is used to reorder the v', the free vertices are put in the front
	// and the control vertices are put in the back
	m_vVertexMap.resize(nInitialVerts);
	size_t nRow = 0;
	for (size_t i = 0; i < nInitialVerts; i++)
	{
		Constraint c(i, v2f::Zero());
		if (m_vConstraints.find(c) != m_vConstraints.end())
		{
			continue;
		}
		m_vVertexMap[i] = nRow++;
	}

	if (nRow != nFreeVerts)
	{
		cerr << "Free vertices number doesn't match." << endl;
	}

	std::set<Constraint>::iterator cur1(m_vConstraints.begin()), end1(m_vConstraints.end());
	while (cur1 != end1)
	{
		const Constraint &c = (*cur1);
		m_vVertexMap[c.nVertex] = nRow++;
		cur1++;
	}

	if (nRow != nInitialVerts)
	{
		cerr << "Total vertices number doesn't match." << endl;
	}

	// Now, let's fill in the matrix G
	// let's explain a little logic here
	// according to Eq(1), let the coordinate of v0, v1 and v2 be (v0x, v0y), (v1x, v1y) and (v2x, v2y), respectively
	// so Eq(1) can be rewrited as
	// (v2x, v2y) = (v0x, v0y) + x * (v1x - v0x, v1y - v0y) + y * (v1y - v0y, v0x - v1x)
	// thus Eq(3) can be rewrited as
	// E = ||((1 - x) * v0x - y * v0y + x * v1x + y * v1y - v2x, y * v0x + (1 - x) * v0y - y * v1x + x * v1y - v2y)||^2
	//   = v' * G * v
	//   = (v0x, v0y, v1x, v1y, v2x, v2y)' * G * (v0x, v0y, v1x, v1y, v2x, v2y)
	// where G = A' * A, and 
	// A = [1 - x,    -y,  x, y, -1,  0;
	//          y, 1 - x, -y, x,  0, -1]
	// So G = 
	//[ (x - 1)*(x - 1) + y*y,                     0, - x*(x - 1) - y*y,                 y, x - 1,    -y;
	//                      0, (x - 1)*(x - 1) + y*y,                -y, - x*(x - 1) - y*y,     y, x - 1;
	//      - x*(x - 1) - y*y,                    -y,         x*x + y*y,         y*x - x*y,    -x,     y;
	//        			    y,     - x*(x - 1) - y*y,         x*y - y*x,         x*x + y*y,    -y,    -x;
	//                  x - 1,                     y,                -x,                -y,     1,     0;
	//                     -y,                 x - 1,                 y,                -x,     0,     1]
	// and E = v' * G * v
	//       = v' * Gtri * v,
	// where Gtri = 
	//[ (x - 1)*(x - 1) + y*y,                     0, -2x*(x - 1) - 2y*y,                 2y,   2x - 2,    -2y;
	//                      0, (x - 1)*(x - 1) + y*y,                -2y, -2x*(x - 1) - 2y*y,       2y, 2x - 2;
	//						0,                     0,          x*x + y*y,                  0,      -2x,     2y;
	//        			    0,                     0,                  0,          x*x + y*y,      -2y,    -2x;
	//                      0,                     0,                  0,                  0,        1,      0;
	//                      0,                     0,                  0,                  0,        0,      1]


	const size_t nTriangles = m_vTriangles.size();
	for (size_t i = 0; i < nTriangles; i++)
	{
		Triangle & t = m_vTriangles[i];
		for (int j= 0; j < 3; j++)
		{
			int n0x = 2 * m_vVertexMap[ t.nVertices[j] ];
			int n0y = n0x + 1;
			int n1x = 2 * m_vVertexMap[ t.nVertices[(j+1)%3] ];
			int n1y = n1x + 1;
			int n2x = 2 * m_vVertexMap[ t.nVertices[(j+2)%3] ];
			int n2y = n2x + 1;
			float x = t.vTriCoords[j].x();
			float y = t.vTriCoords[j].y();

			// n0x,n?? elems, the first line of matrix Gtri as explained earlier
			m_G(n0x, n0x) += 1 - 2*x + x*x + y*y;
			m_G(n0x, n1x) += 2*x - 2*x*x - 2*y*y;		
			m_G(n0x, n1y) += 2*y;						
			m_G(n0x, n2x) += -2 + 2*x;					
			m_G(n0x, n2y) += -2 * y;

			// n0y,n?? elems, the second line of matrix Gtri
			m_G(n0y, n0y) += 1 - 2*x + x*x + y*y;
			m_G(n0y, n1x) += -2*y;						
			m_G(n0y, n1y) += 2*x - 2*x*x - 2*y*y;		
			m_G(n0y, n2x) += 2*y;						
			m_G(n0y, n2y) += -2 + 2*x;	

			// n1x,n?? elems, the third line of matrix Gtri
			m_G(n1x, n1x) += x*x + y*y;
			m_G(n1x, n2x) += -2*x;						
			m_G(n1x, n2y) += 2*y;

			// n1y,n?? elems, the fourth line of matrix Gtri
			m_G(n1y, n1y) += x*x + y*y;
			m_G(n1y, n2x) += -2*y;						
			m_G(n1y, n2y) += -2*x;

			// final 2 elems, the fifth and sixth line of Gtri
			m_G(n2x, n2x) += 1;
			m_G(n2y, n2y) += 1;
		}
	}

	//cout << "G's size is " << m_G.rows() << " * "<<m_G.cols() << endl; 
	//cout << "Print G:" << endl << m_G << endl;

	// extract G00, G01 and G10 from G, and then compute Gprime and B
	MatrixXd mG00(2 * nFreeVerts, 2 * nFreeVerts);
	MatrixXd mG01(2 * nFreeVerts, 2 * nConstraints);
	MatrixXd mG10(2 * nConstraints, 2 * nFreeVerts);

	ExtractSubMatrix( m_G, 0, 0, mG00 );
	ExtractSubMatrix( m_G, 0, 2*nFreeVerts, mG01 );
	ExtractSubMatrix( m_G, 2*nFreeVerts, 0, mG10 );

	// ok, now compute GPrime = G00 + Transpose(G00) and B = G01 + Transpose(G10)
	m_Gprime = mG00 + mG00.transpose();
	m_B = mG01 + mG10.transpose();

	MatrixXd m_GprimeInverse = m_Gprime.inverse();
	MatrixXd mFinal = m_GprimeInverse * m_B;

	m_FirstMatrix = (-1) * mFinal;
	cout << "first matrix's size: " << m_FirstMatrix.rows() << " * " << m_FirstMatrix.cols() << endl;

}

void ArapDeform::updateMesh(bool isRigid)
{
	// perform step 1
	size_t nConstraints = m_vConstraints.size();
	Eigen::VectorXd vQ(2 * nConstraints);
	int k = 0;
	std::set<Constraint>::iterator cur(m_vConstraints.begin()), end(m_vConstraints.end());
	while ( cur != end ) {
		const Constraint & c = *cur++;
		//vQ[ 2*k ] = c.vConstrainedPos.x();
		//vQ[ 2*k + 1] = c.vConstrainedPos.y();
		vQ[2 * k] = m_vDeformedVertices[c.nVertex].vPosition.x();
		vQ[2 * k + 1] = m_vDeformedVertices[c.nVertex].vPosition.y();
		++k;
	}

	Eigen::VectorXd vU = m_FirstMatrix * vQ;
	const size_t nVerts = m_vDeformedVertices.size();

	//cout << "=================================" << endl;
	//cout << "update new positions" << endl;
	for (size_t i = 0; i < nVerts; i++)
	{
		Constraint c(i, v2f::Zero());
		if (m_vConstraints.find(c) != m_vConstraints.end())
		{
			continue;
		}

		int nRow = m_vVertexMap[i];
		double fX = vU[2 * nRow];
		double fY = vU[2 * nRow + 1];

		//cout << "new vs old positions: (" << fX << ", " << fY << " ) <--> " << m_vDeformedVertices[i].vPosition.transpose() << endl;

		m_vDeformedVertices[i].vPosition = v2f(fX, fY);


	}
	
	if (isRigid)
	{
		// perform step 2 - scaling and step 3 fitting
	}
}