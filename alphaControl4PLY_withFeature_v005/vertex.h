////////////////////
///// vertex.h /////
////////////////////

#ifndef __VERTEX_HH
#define __VERTEX_HH

#include<vector>
#include<kvs/Type>

#include "ply_customize.h"

//-------------//
struct Vertex
//-------------//
{
  // Position (x,y,z)
  kvs::Real32 vertex_x;
  kvs::Real32 vertex_y;
  kvs::Real32 vertex_z;

  // Normal vector (nx, ny, nz)
  kvs::Real32 normal_x;
  kvs::Real32 normal_y;
  kvs::Real32 normal_z;

  // Color (r, g, b)
  int r;
  int g;
  int b;

  // Number of facets which has this vertex
  int faceNum;


  // Default constructor
  Vertex( void ) :
    vertex_x(0), vertex_y(0), vertex_z(0), 
    normal_x(0), normal_y(0), normal_z(0), 
    r(DEFAULT_POINT_COLOR_R), 
    g(DEFAULT_POINT_COLOR_G), 
    b(DEFAULT_POINT_COLOR_B), 
    faceNum(0)
  {}

  // Constructor
  Vertex( kvs::Real32 x,  kvs::Real32 y,  kvs::Real32 z, 
	  kvs::Real32 nx, kvs::Real32 ny, kvs::Real32 nz, 
	  int cr, int cg, int cb, 
	  int fNum = 0                                     )
  {
    vertex_x = x;
    vertex_y = y;
    vertex_z = z;

    normal_x = nx;
    normal_y = ny;
    normal_z = nz;

    r		 = cr;
    g		 = cg;
    b		 = cb;

    faceNum  = fNum;
  }


  // Copy constructor
  Vertex(const Vertex & v)
  {
    vertex_x = v.vertex_x;
    vertex_y = v.vertex_y;
    vertex_z = v.vertex_z;

    normal_x = v.normal_x;
    normal_y = v.normal_y;
    normal_z = v.normal_z;

    r		 = v.r;
    g		 = v.g;
    b		 = v.b;
    faceNum  = v.faceNum;
  }
	
	
  // Comparison operator "=="
  int operator == ( const Vertex &v ) const
  {
    return vertex_x == v.vertex_x && vertex_y == v.vertex_y && vertex_z == v.vertex_z \
    && normal_x == v.normal_x && normal_y == v.normal_y && normal_z == v.normal_z \
    && r == v.r && g == v.g && b == v.b \
    && faceNum == v.faceNum;
  }

  // Comparison operator "<" (for sorting)
  int operator < ( const Vertex &v ) const
  {
    return (vertex_x*vertex_x+vertex_y*vertex_y+vertex_z*vertex_z) < (v.vertex_x*v.vertex_x+v.vertex_y*v.vertex_y+v.vertex_z*v.vertex_z);
  }

};// class Vertex

#endif
// end of vertex.h
