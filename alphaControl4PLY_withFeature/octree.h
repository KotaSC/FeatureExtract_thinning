#ifndef __octree
#define __octree

#include <vector>
#include "create_octree.h"
using namespace std;


// 8ʬ�ڤΥΡ��ɤ�ɽ����¤��
// struct octreeNode {
//   vector <int> pInd;
//   double c[3];
//   octreeNode *cOctreeNode[2][2][2];
// };


// ���饹 octree �����
class octree {
private:
  vector <int> pInd;
public:
  octreeNode *octreeRoot;
  octree(float points[], size_t np, double range[], int nMin);
};

#endif
