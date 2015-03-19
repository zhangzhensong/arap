This is a direct implementation of Igarashi's Siggraph 05 paper[1]. Just to understand how to calculate Eq(5)-(8), so I just implement the first step of the algorithm, i.e., I just do the scale free construction. There are some derivations on how to express the matrix G in Eq(5) in my source code.

The libraries I used in this code include: (1) Jonathan Richard Shewchuk's Triangle[3], the code is contained in my source code; (2) Freeglut library [4]; and (3) Eigen [5]. If you want to use this code, please download these library accordingly.

Operations: Left click - add mesh vertices "A" or "a" - build mesh Right click - add control points Left drag - mesh deformation "C" or "c" - clear the mesh "V" or "v" - clear the control points

If you want to know how to implement the whole paper, please refer to Ryan Schmidt's implementation [2].

Reference

[1] Takeo Igarashi, Tomer Moscovich, John F. Hughes, "As-Rigid-As-Possible Shape Manipulation", ACM Transactions on Computer Graphics, Vol.24, No.3, ACM SIGGRAPH 2005, Los Angels, USA, 2005.

[2] http://www.dgp.toronto.edu/~rms/software/Deform2D/

[3] https://www.cs.cmu.edu/~quake/triangle.html

[4] http://freeglut.sourceforge.net/

[5] http://eigen.tuxfamily.org/