#include "octree.h"

octree::octree(float points[], size_t np, double range[], int nMin)
{

  octreeRoot = new octreeNode;

  for (size_t i = 0; i < np; i++) {
    octreeRoot->pInd.push_back(i);
  }

  create_octree(octreeRoot, points, nMin,
 		range[0], range[1], range[2], range[3], range[4], range[5]);

  

}
