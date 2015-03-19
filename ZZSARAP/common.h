#ifndef COMMON_H_
#define COMMON_H_
#include <Eigen/Dense>

using Eigen::Vector2i;
using Eigen::Vector2d;
using Eigen::Vector2f;

extern "C"
{
#include "triangle.h"
}

#ifdef ANSI_DECLARATORS
extern "C" void triangulate(char *, struct triangulateio *, struct triangulateio *,
                 struct triangulateio *);
extern "C" void trifree(VOIDD *memptr);
#else /* not ANSI_DECLARATORS */
void triangulate();
void trifree();
#endif /* not ANSI_DECLARATORS */

typedef Vector2i v2i;
typedef Vector2d v2d;
typedef Vector2f v2f;

#endif