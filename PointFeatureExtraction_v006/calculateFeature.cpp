#include "calculateFeature.h"
#include "octree.h"

#include <vector>
#include <Accelerate/Accelerate.h>
#include <kvs/BoxMuller>

const int INTERVAL = 1000000;
const double EPSILON = 1.0e-16;
const int MIN_NODE = 15;

calculateFeature::calculateFeature(void) : m_type(PointPCA),
                                           m_isNoise(false),
                                           m_noise(0.0),
                                           m_searchRadius(0.01)
{
}

calculateFeature::calculateFeature(const FeatureType type,
                                   const double distance,
                                   kvs::PolygonObject *ply) : m_type(type),
                                                              m_isNoise(false),
                                                              m_noise(0.0),
                                                              m_searchRadius(distance)
{
  calc(ply);
}

void calculateFeature::setFeatureType(FeatureType type)
{
  m_type = type;
}

void calculateFeature::addNoise(double noise)
{
  m_isNoise = true;
  m_noise = noise;
  std::cout << "ADD NOISE : " << noise << std::endl;
}

void calculateFeature::setSearchRadius(double distance)
{
  m_searchRadius = distance;
}
void calculateFeature::setSearchRadius(double divide,
                                       kvs::Vector3f bbmin,
                                       kvs::Vector3f bbmax)
{
  kvs::Vector3f bb = bbmax - bbmin;
  double b_leng = bb.length();
  m_searchRadius = b_leng / divide;
}

void calculateFeature::calc(kvs::PolygonObject *ply)
{
  std::vector<float> normal;
  kvs::BoxMuller normRand;

  kvs::ValueArray<kvs::Real32> coords = ply->coords();
  kvs::ValueArray<kvs::Real32> normals = ply->normals();

  size_t num = ply->numberOfVertices();
  m_number = num;
  double d_noise = sqrt(m_searchRadius) * m_noise;
  bool hasNormal = false;
  if (num == ply->numberOfNormals())
    hasNormal = true;
  if (!hasNormal)
  {
    if (m_isNoise || m_type == NormalPCA || m_type == NormalDispersion)
    {
      std::cout << "Cannot add Noise" << std::endl;
      exit(1);
    }
  }

  for (size_t i = 0; i < num; i++)
  {
    float x = coords[3 * i];
    float y = coords[3 * i + 1];
    float z = coords[3 * i + 2];
    float nx = 0.0;
    float ny = 0.0;
    float nz = 0.0;
    if (hasNormal)
    {
      nx = normals[3 * i];
      ny = normals[3 * i + 1];
      nz = normals[3 * i + 2];
    }
    if (m_isNoise)
    {
      double dt = normRand.rand(0.0, d_noise * d_noise);
      x += dt * nx;
      y += dt * ny;
      z += dt * nz;
    }
    normal.push_back(nx);
    normal.push_back(ny);
    normal.push_back(nz);
  }

  if (m_type == PointPCA)
  {
    calcPointPCA(ply);
  }
  else if (m_type == NormalPCA)
  {
    calcNormalPCA(ply, normal);
  }
  else if (m_type == NormalDispersion)
  {
    calcNormalDispersion(ply, normal);
  }
}

void calculateFeature::calcPointPCA(kvs::PolygonObject *ply)
{

  ply->updateMinMaxCoords();
  kvs::ValueArray<kvs::Real32> coords = ply->coords();
  float *pdata = coords.data();
  size_t numVert = ply->numberOfVertices();
  kvs::Vector3f minBB = ply->minObjectCoord();
  kvs::Vector3f maxBB = ply->maxObjectCoord();

  double *mrange = new double[6];
  mrange[0] = (double)minBB.x();
  mrange[1] = (double)maxBB.x();
  mrange[2] = (double)minBB.y();
  mrange[3] = (double)maxBB.y();
  mrange[4] = (double)minBB.z();
  mrange[5] = (double)maxBB.z();

  // create octree
  std::cout << "Creating Octree... (Number of Vertex : " << numVert << std::endl;
  std::cout << minBB << " \n"
            << maxBB << std::endl;
  octree *myTree = new octree(pdata, numVert, mrange, MIN_NODE);

  kvs::MersenneTwister uniRand;
  double sigMax = 0.0;

  std::cout << "Start OCtree Search..... " << std::endl;
  for (size_t i = 0; i < numVert; i++)
  {
    if (i == numVert)
      --i;
    double point[3] = {coords[3 * i],
                       coords[3 * i + 1],
                       coords[3 * i + 2]};

    vector<size_t> nearInd;
    vector<double> dist;
    search_points(point, m_searchRadius, pdata, myTree->octreeRoot, &nearInd, &dist);
    int n0 = (int)nearInd.size();

    //--- Standardization for x, y, z
    double xb = 0.0, yb = 0.0, zb = 0.0;
    for (int j = 0; j < n0; j++)
    {
      // 近傍点の(x, y, z)座標を格納
      double x = coords[3 * nearInd[j]];
      double y = coords[3 * nearInd[j] + 1];
      double z = coords[3 * nearInd[j] + 2];

      xb += x;
      yb += y;
      zb += z;
    }
    // 平均値を計算
    xb /= (double)n0;
    yb /= (double)n0;
    zb /= (double)n0;

    //--- Calculaton of covariance matrix
    double x2 = 0.0, y2 = 0.0, z2 = 0.0;
    double xy = 0.0, yz = 0.0, zx = 0.0;
    for (int j = 0; j < n0; j++)
    {
      // 近傍点の(x, y, z)座標と，平均値との差を計算
      double nx = (coords[3 * nearInd[j]] - xb);
      double ny = (coords[3 * nearInd[j] + 1] - yb);
      double nz = (coords[3 * nearInd[j] + 2] - zb);
      x2 += nx * nx;
      y2 += ny * ny;
      z2 += nz * nz;
      xy += nx * ny;
      yz += ny * nz;
      zx += nz * nx;
    }
    // 分散・共分散の計算
    double s_x2 = x2 / (double)(n0);
    double s_y2 = y2 / (double)(n0);
    double s_z2 = z2 / (double)(n0);
    double s_xy = xy / (double)(n0);
    double s_yz = yz / (double)(n0);
    double s_zx = zx / (double)(n0);

    //--- Preparation for LAPACK
    char jovz = 'V';    // 'V': Compute eigenvalues and eigenvectors, 'N': Compute eigenvalues only
    char uplo = 'U';    // 'U': Upper triangle of A is stored, 'L': Lower triangle of A is stored
    int n = 3;          // dimension
    double A[n * n];    // the symmetric matrix(dimension: n)
    double W[n];        // eigenvalues
    int lwork = n * n;  // working area size
    double WORK[n * n]; // working area
    int info;           // status(0: success)

    //---- Covariance matrix
    A[0] = s_x2;
    A[3] = s_xy;
    A[6] = s_zx;
    A[1] = 0.0;
    A[4] = s_y2;
    A[7] = s_yz;
    A[2] = 0.0;
    A[5] = 0.0;
    A[8] = s_z2;

    //---- Calcuation of eigenvalues and egenvectors
    dsyev_(&jovz, &uplo, (__CLPK_integer *)&n, A, (__CLPK_integer *)&n,
           W, WORK, (__CLPK_integer *)&lwork, (__CLPK_integer *)&info);

    // W[0]: 第3固有値, W[1]: 第2固有値, W[2]: 第1固有値
    double sum = W[0] + W[1] + W[2]; // Sum of eigenvalues
    // double var = searchPoint.x;
    double var = W[0] / sum; // Change of curvature
    // double var = (W[2] - W[1]) / W[2]; // Linearity
    // double var = W[1] - W[0] / W[2];             // Planarity
    // double var = 1 - ( ( W[1] - W[0] ) / W[2] ); // Aplanarity
    // double var = W[2];

    if (sum < EPSILON)
      var = 0.0;
    if (info > 0)
      var = 0.0; // If LAPACK doesn't converge
    //---　Contributing rate of 3rd(minimum) component
    m_feature.push_back(var);
    if (sigMax < var)
      sigMax = var;
    if (!((i + 1) % INTERVAL))
      std::cout << i + 1 << ", " << n0 << ": " << var << std::endl;
  }

  m_maxFeature = sigMax;
  std::cout << "Maximun of Sigma : " << sigMax << std::endl;
}

void calculateFeature::calcNormalPCA(kvs::PolygonObject *ply,
                                     std::vector<float> &normal)
{
  ply->updateMinMaxCoords();
  kvs::ValueArray<kvs::Real32> coords = ply->coords();
  float *pdata = coords.data();
  size_t numVert = ply->numberOfVertices();
  kvs::Vector3f minBB = ply->minObjectCoord();
  kvs::Vector3f maxBB = ply->maxObjectCoord();

  double *mrange = new double[6];
  mrange[0] = (double)minBB.x();
  mrange[1] = (double)maxBB.x();
  mrange[2] = (double)minBB.y();
  mrange[3] = (double)maxBB.y();
  mrange[4] = (double)minBB.z();
  mrange[5] = (double)maxBB.z();

  // create octree
  std::cout << "Creating Octree... (Number of Vertex : " << numVert << std::endl;
  std::cout << minBB << " \n"
            << maxBB << std::endl;
  octree *myTree = new octree(pdata, numVert, mrange, MIN_NODE);

  kvs::MersenneTwister uniRand;
  double sigMax = 0.0;

  std::cout << "Start OCtree Search..... " << std::endl;
  for (size_t i = 0; i < numVert; i++)
  {
    if (i == numVert)
      --i;
    double point[3] = {coords[3 * i],
                       coords[3 * i + 1],
                       coords[3 * i + 2]};

    vector<size_t> nearInd;
    vector<double> dist;
    search_points(point, m_searchRadius, pdata, myTree->octreeRoot, &nearInd, &dist);
    int n0 = (int)nearInd.size();

    //--- Calculaton of covariance matrix
    double x2 = 0.0, y2 = 0.0, z2 = 0.0;
    double xa = 0.0, ya = 0.0, za = 0.0;
    double xy = 0.0, yz = 0.0, zx = 0.0;
    for (int j = 0; j < n0; j++)
    {
      double nx = normal[3 * nearInd[j]];
      double ny = normal[3 * nearInd[j] + 1];
      double nz = normal[3 * nearInd[j] + 2];
      x2 += nx * nx;
      y2 += ny * ny;
      z2 += nz * nz;
      xa += nx;
      ya += ny;
      za += nz;
      xy += nx * ny;
      yz += ny * nz;
      zx += nz * nx;
    }
    double s_x2 = (x2 - xa * xa / (double)n0) / (double)n0;
    double s_y2 = (y2 - ya * ya / (double)n0) / (double)n0;
    double s_z2 = (z2 - za * za / (double)n0) / (double)n0;
    double s_xy = (xy - xa * ya / (double)n0) / (double)n0;
    double s_yz = (yz - ya * za / (double)n0) / (double)n0;
    double s_zx = (zx - za * xa / (double)n0) / (double)n0;

    //--- Preparation for LAPACK
    char jovz = 'V';
    char uplo = 'U';
    int n = 3;
    double A[n * n];
    double W[n];
    int lwork = n * n;
    double WORK[n * n];
    int info;

    //---- Covariance matrix
    A[0] = s_x2;
    A[3] = s_xy;
    A[6] = s_zx;
    A[1] = 0.0;
    A[4] = s_y2;
    A[7] = s_yz;
    A[2] = 0.0;
    A[5] = 0.0;
    A[8] = s_z2;

    //---- Calcuation of eigenvalues and egenvectors
    dsyev_(&jovz, &uplo, (__CLPK_integer *)&n, A, (__CLPK_integer *)&n,
           W, WORK, (__CLPK_integer *)&lwork, (__CLPK_integer *)&info);

    double var = W[0] + W[1] + W[2]; // Sum of eigenvalues
    if (info > 0)
      var = 0.0; // If LAPACK doesn't converge
    m_feature.push_back(var);
    if (sigMax < var)
      sigMax = var;
    if (!((i + 1) % INTERVAL))
      std::cout << i + 1 << ", " << n0 << ": " << var << std::endl;
  }
  m_maxFeature = sigMax;
  std::cout << "Maximun of Sigma : " << sigMax << std::endl;
}

void calculateFeature::calcNormalDispersion(kvs::PolygonObject *ply,
                                            std::vector<float> &normal)
{
  ply->updateMinMaxCoords();
  kvs::ValueArray<kvs::Real32> coords = ply->coords();
  float *pdata = coords.data();
  size_t numVert = ply->numberOfVertices();
  kvs::Vector3f minBB = ply->minObjectCoord();
  kvs::Vector3f maxBB = ply->maxObjectCoord();

  double *mrange = new double[6];
  mrange[0] = (double)minBB.x();
  mrange[1] = (double)maxBB.x();
  mrange[2] = (double)minBB.y();
  mrange[3] = (double)maxBB.y();
  mrange[4] = (double)minBB.z();
  mrange[5] = (double)maxBB.z();

  // create octree
  std::cout << "Creating Octree... (Number of Vertex : " << numVert << std::endl;
  std::cout << minBB << " \n"
            << maxBB << std::endl;
  octree *myTree = new octree(pdata, numVert, mrange, MIN_NODE);

  kvs::MersenneTwister uniRand;
  double sigMax = 0.0;

  std::cout << "Start OCtree Search..... " << std::endl;

  for (size_t i = 0; i < numVert; i++)
  {
    if (i == numVert)
      --i;
    double point[3] = {coords[3 * i],
                       coords[3 * i + 1],
                       coords[3 * i + 2]};

    vector<size_t> nearInd;
    vector<double> dist;
    search_points(point, m_searchRadius, pdata, myTree->octreeRoot, &nearInd, &dist);
    int n0 = (int)nearInd.size();

    int index = nearInd[0];
    float ni[3] = {normal[3 * index], normal[3 * index + 1], normal[3 * index + 2]};

    double sum = 0.0;
    double sum2 = 0.0;
    for (int j = 1; j < n0; j++)
    {
      index = nearInd[j];
      float nt[3] = {normal[3 * index], normal[3 * index + 1], normal[3 * index + 2]};
      double dot = ni[0] * nt[0] + ni[1] * nt[1] + ni[2] * nt[2];
      sum += dot;
      sum2 += dot * dot;
    }

    double mean = sum / (double)(n0 - 1);
    double var = (sum2 - (double)(n0 - 1) * mean * mean) / (double)(n0 - 1);

    m_feature.push_back(var); //
    if (sigMax < var)
      sigMax = var;
    if (!((i + 1) % INTERVAL))
      std::cout << i + 1 << ", " << n0 << ": " << var << std::endl;
  }
  m_maxFeature = sigMax;
  std::cout << "Maximun of Sigma : " << sigMax << std::endl;
}
