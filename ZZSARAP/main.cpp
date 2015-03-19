#include <iostream>
#include <Eigen/Dense>
#include <GL/glut.h>
#include <set>

#include "ArapDeform.h"
#include "common.h"

using namespace std;
using namespace zzs;

using Eigen::MatrixXd;

int winID;
int winWidth = 800;
int winHeight = 600;

set<size_t> m_Selected;
size_t m_nSelected = std::numeric_limits<size_t>::max();

ArapDeform m_deform;

void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT);
	glViewport (0, 0, (GLsizei) winWidth, (GLsizei) winHeight);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0, winWidth, winHeight, 0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glPushMatrix();
	
	glPointSize(5.0f);

	const std::vector<Vertex>& verts = m_deform.getDeformedVerts();
	const size_t np = verts.size();

	if (m_deform.isAddPoint())
	{
		glColor3f(0.0, 1.0, 0.0);
		glBegin(GL_POINTS);

		for (int i = 0; i < np; i++)
		{
			glVertex2f(verts[i].vPosition[0], verts[i].vPosition[1]);
		}
		glEnd();
	}else
	{
		glColor3f(0.0, 0.0, 1.0);
		glBegin(GL_POINTS);

		for (int i = 0; i < np; i++)
		{
			glVertex2f(verts[i].vPosition[0], verts[i].vPosition[1]);
		}
		glEnd();

		// todo: draw triangles
		const std::vector<Triangle>& vTriangles = m_deform.getTriangles();
		const size_t nt = vTriangles.size();

		glBegin(GL_LINES);
		for (int i = 0; i < nt; i++)
		{
			const size_t * tri = vTriangles[i].nVertices;

			glVertex2f(verts[tri[0]].vPosition[0], verts[tri[0]].vPosition[1]);
			glVertex2f(verts[tri[1]].vPosition[0], verts[tri[1]].vPosition[1]);

			glVertex2f(verts[tri[1]].vPosition[0], verts[tri[1]].vPosition[1]);
			glVertex2f(verts[tri[2]].vPosition[0], verts[tri[2]].vPosition[1]);

			glVertex2f(verts[tri[2]].vPosition[0], verts[tri[2]].vPosition[1]);
			glVertex2f(verts[tri[0]].vPosition[0], verts[tri[0]].vPosition[1]);			
		}
		glEnd();
	}
	
	// draw controllers
	glColor3f(1.0, 0.0, 0.0);
	std::set<size_t>::iterator cur(m_Selected.begin()), end(m_Selected.end());
	glBegin(GL_POINTS);
	
	while ( cur != end ) {
		size_t nSelected = *cur++;
		glVertex2f(verts[nSelected].vPosition[0], verts[nSelected].vPosition[1]);
	}
	glEnd();

	//glFlush();
	glPopMatrix();
 
	glutSwapBuffers();
}

void keyboardCommon(unsigned char key, int x, int y){

	if (key == 27)
	{
		glutDestroyWindow(winID);
	}
	if ((key == 'A')||(key == 'a'))
	{
		m_deform.setAddPoint();
		if (!m_deform.isAddPoint())
		{
			m_deform.buildMesh();
		}
	}

	if ((key == 'C')||(key == 'c'))
	{
		cout << "Clear every thing!" << endl;

		m_nSelected = std::numeric_limits<size_t>::max();
		m_Selected.clear();
		m_deform.clearData();
	}

	if ((key == 'v')||(key == 'V'))
	{
		cout << "Clear constraints!" << endl;
		m_Selected.clear();
		m_nSelected = std::numeric_limits<size_t>::max();
	}

	glutPostRedisplay();
}


void spinDisplay(void)
{
   glutPostRedisplay();
}

void init(void) 
{
   glClearColor (1.0, 1.0, 1.0, 0.0);
   glShadeModel (GL_FLAT);
}

void reshape(int w, int h)
{
	winWidth = w;
	winHeight = h;
}

void mouse(int button, int state, int x, int y) 
{
   switch (button) {
      case GLUT_LEFT_BUTTON:
         if (state == GLUT_DOWN)
		 {
			//glutIdleFunc(spinDisplay);
			cout << "Current mouse position: " << x << " " <<y << endl;
			if (m_deform.isAddPoint())
			{
				Vertex point((float)x, (float)y);
				cout << "add point:" << x << " " <<y << endl;
				point.vPosition << (float)x, (float)y;

				m_deform.addPoint(point);

				m_Selected.clear();
				m_nSelected = std::numeric_limits<size_t>::max();
				m_deform.updateConstraints(m_Selected);
			}else
			{
				m_nSelected = m_deform.findHitVertex((float)x, (float)y);
				if(m_Selected.find(m_nSelected) == m_Selected.end())
					m_nSelected = std::numeric_limits<size_t>::max();
			}
		 }else
		 {
			 m_nSelected = std::numeric_limits<size_t>::max();
		 }
            
         break;
      case GLUT_MIDDLE_BUTTON:
      case GLUT_RIGHT_BUTTON:
         if (state == GLUT_DOWN)
         {
			 cout << "Add control point at point: " << x << " " << y << endl;
			 size_t nHit = m_deform.findHitVertex((float)x, (float)y);
			 if (nHit < std::numeric_limits<size_t>::max())
			 {
				 cout << "Find the vertex" << endl;
				 if(m_Selected.find(nHit) == m_Selected.end())
					 m_Selected.insert(nHit);
				 else
				 {
					 m_Selected.erase(nHit);
				 }
				 // once modified the constraints, we should update them
				// i.e., precompute Gprime and B
				m_deform.updateConstraints(m_Selected);
			 }
			 else
			 {
				 cout << "Doesn't find the vertex" << endl;
			 }
			 
		 }

         break;
      default:
         break;
   }

   glutPostRedisplay();
}


void OnMouseMove(int x, int y)
{
	if ( m_nSelected != std::numeric_limits<size_t>::max() ) {
		Point newPos(x, y);
		m_deform.setVertex(m_nSelected, newPos);
		m_deform.updateMesh(false);
		glutPostRedisplay();
	}
}

void main(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize (winWidth, winHeight); 
	glutInitWindowPosition (100, 100);
	winID = glutCreateWindow ("Test OpenGL");
	init ();
	glutDisplayFunc(display); 
	glutReshapeFunc(reshape); 
	glutKeyboardFunc(keyboardCommon);
	glutMotionFunc( OnMouseMove );
	glutMouseFunc(mouse);
	glutMainLoop();
}