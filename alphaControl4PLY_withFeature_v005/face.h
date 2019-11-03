//////////////////
///// face.h /////
//////////////////

#ifndef __FACE__
#define __FACE__
#include<vector>

// Triangles are assumed.

//-------------//
struct face{
//-------------//

  // Center 
  double center_x;
  double center_y;
  double center_z;

  // normal vector
  double normal_x;
  double normal_y;
  double normal_z;

  //
  int index1;
  int index2;
  int index3;

  //
  double distList;

  // Area 
  double area;
};


//------------------//
struct connections{
//------------------//

  //
  int index1;
  int index2;
  int index3;

  //
  int numOfSplit;
};

//------------------//
struct polygon_edge{
//------------------//
  int vertexNum1; //
  int vertexNum2; //
};

#endif
// end of face.h
