#ifndef __create_octree
#define __create_octree

#include <vector>
using namespace std;

struct octreeNode {
  vector<size_t> pInd;
  double c[3];
  octreeNode *cOctreeNode[2][2][2];
};

void create_octree(octreeNode *node, float points[], int nMin,
		   double xMin, double xMax, double yMin, double yMax,
		   double zMin, double zMax);

void search_points(double p[], double R, float points[],
                   octreeNode *Node, vector<size_t> *nearIndPtr,
                   vector<double> *dist );

#endif
