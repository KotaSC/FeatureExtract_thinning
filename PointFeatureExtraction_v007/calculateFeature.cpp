#include "calculateFeature.h"
#include "octree.h"

#include <Accelerate/Accelerate.h> //CLAPACK
#include <vector>
#include <cmath>
#include <kvs/BoxMuller>
#include <kvs/EigenDecomposer>
#include <kvs/Vector3>
#include <kvs/Matrix33>
#include <kvs/Matrix>


const int INTERVAL   = 1000000;
const double EPSILON = 1.0e-16;
const int MIN_NODE   = 15;
const int DIM        = 3;

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
  m_noise   = noise;
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
  double b_leng    = bb.length();
  m_searchRadius   = b_leng / divide;

  std::cout << "==================================" << std::endl;
  std::cout << "m_searchRadius = " << m_searchRadius << std::endl;
  std::cout << "==================================" << std::endl;

}

void calculateFeature::calc(kvs::PolygonObject *ply)
{
  std::vector<float> normal;
  kvs::BoxMuller normRand;

  kvs::ValueArray<kvs::Real32> coords  = ply->coords();
  kvs::ValueArray<kvs::Real32> normals = ply->normals();

  size_t num = ply->numberOfVertices();
  m_number   = num;
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
    float x  = coords[3 * i];
    float y  = coords[3 * i + 1];
    float z  = coords[3 * i + 2];
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
  else if (m_type == RDoCFeature)
  {
    calcRDoCFeature(ply);
  }
  else if (m_type == MinimumEntropy)
  {
    calcMinimumEntropy(ply);
  }
  else if (m_type == MSFeature)
  {
    calcMSFeature(ply);
  }
  else if (m_type == PlaneBasedFeature)
  {
    calcPlaneBasedFeature(ply);
  }

}



void calculateFeature::calcPointPCA(kvs::PolygonObject *ply)
{
  m_feature = calcFeatureValues( ply, m_searchRadius );
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
    double xx = 0.0, yy = 0.0, zz = 0.0;
    double xa = 0.0, ya = 0.0, za = 0.0;
    double xy = 0.0, yz = 0.0, zx = 0.0;
    for (int j = 0; j < n0; j++)
    {
      double nx = normal[3 * nearInd[j]];
      double ny = normal[3 * nearInd[j] + 1];
      double nz = normal[3 * nearInd[j] + 2];
      xx += nx * nx;
      yy += ny * ny;
      zz += nz * nz;
      xa += nx;
      ya += ny;
      za += nz;
      xy += nx * ny;
      yz += ny * nz;
      zx += nz * nx;
    }
    double s_xx = (xx - xa * xa / (double)n0) / (double)n0;
    double s_yy = (yy - ya * ya / (double)n0) / (double)n0;
    double s_zz = (zz - za * za / (double)n0) / (double)n0;
    double s_xy = (xy - xa * ya / (double)n0) / (double)n0;
    double s_yz = (yz - ya * za / (double)n0) / (double)n0;
    double s_zx = (zx - za * xa / (double)n0) / (double)n0;

    //---- Covariance matrix
    kvs::Matrix<double> M( 3, 3 );
    M[0][0] = s_xx; M[0][1] = s_xy; M[0][2] = s_zx;
    M[1][0] = s_xy; M[1][1] = s_yy; M[1][2] = s_yz;
    M[2][0] = s_zx; M[2][1] = s_yz; M[2][2] = s_zz;

    //---- Calcuation of eigenvalues
    kvs::EigenDecomposer<double> eigen( M );

    const kvs::Vector<double>& L = eigen.eigenValues();

    // L[0]: 第1固有値, L[1]: 第2固有値, L[2]: 第3固有値
    double sum = L[0] + L[1] + L[2]; // Sum of eigenvalues
    // double var = searchPoint.x;
    double var = L[2] / sum; // Change of curvature
    // double var = (L[0] - L[1]) / L[0]; // Linearity
    // double var = L[1] - L[2] / L[0];             // Planarity
    // double var = 1 - ( ( L[1] - L[2] ) / L[0] ); // Aplanarity
    // double var = L[0];

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



void calculateFeature::calcRDoCFeature(kvs::PolygonObject *ply)
{
  std::vector<float> smallRadFeature;
  std::vector<float> largeRadFeature;

  double RDoC;
  double M;


  std::cout << "=========================================================" << std::endl;
  std::cout << "Input Neighbor Radius Maginification M ( 0 < M < 1 ) : ";
  std::cin >> M;
  std::cout << "=========================================================" << std::endl;


  double smallRadius  = m_searchRadius*M;
  double largeRadius  = m_searchRadius;

  size_t numVert = ply->numberOfVertices();

  std::cout << "============================================" << std::endl;
  std::cout << "smallRadius : " << smallRadius << std::endl;
  std::cout << "Start feature calculation with small radius" << std::endl;

  smallRadFeature  = calcFeatureValues( ply, smallRadius );

  std::cout << "Finish feature calculation with small radius" << std::endl;
  std::cout << "============================================" << std::endl;

  std::cout << "============================================" << std::endl;
  std::cout << "largeRadius : " << largeRadius << std::endl;
  std::cout << "Start feature calculation with large radius" << std::endl;

  largeRadFeature  = calcFeatureValues( ply, largeRadius );

  std::cout << "Finish feature calculation with large radius" << std::endl;
  std::cout << "============================================" << std::endl;


  for( int i = 0; i < numVert; i++ ) {

    // Calculate DoC feature
    RDoC = std::fabs( smallRadFeature[i] - largeRadFeature[i] );

    m_feature.push_back( RDoC );

  }

  m_maxFeature = *std::max_element( m_feature.begin(), m_feature.end() );;
  std::cout << "Maximun of Sigma : " << m_maxFeature << std::endl;
}

void calculateFeature::calcMinimumEntropy(kvs::PolygonObject *ply)
{

  std::vector<double> eigenValues;

  std::vector<float> tmpEigentropy;
  std::vector<float> selectedFeature;

  std::vector<vector<float>> eigentropy;
  std::vector<vector<float>> featureValues;

  size_t numVert = ply->numberOfVertices();

  int numItr;
  double itrRadius;

  float sigMax = 0.0;

  std::cout << "========================" << std::endl;
  std::cout << "Input Number of Iteration : ";
  std::cin  >> numItr;
  std::cout << "========================" << std::endl;

  for ( int j = 0; j < numItr; j++ )
  {
    itrRadius   = m_searchRadius * ( (double)( j+1 ) / (double)numItr );
    eigenValues = calcEigenValues( ply, itrRadius );

    for ( size_t i = 0; i < numVert; i++ )
    {
      double sum = eigenValues[i*3] + eigenValues[i*3 + 1] + eigenValues[i*3 + 2];

      // Change of curvature
      double ft = eigenValues[i*3 + 2] / sum;

      // Eigentropy
      double lambda1 = eigenValues[i*3] / sum;
      double lambda2 = eigenValues[i*3 + 1] / sum;
      double lambda3 = eigenValues[i*3 + 2] / sum;
      double et      = -( lambda1*log(lambda1) + lambda2*log(lambda2) + lambda3*log(lambda3) );

      if (sum < EPSILON)
      {
        ft = 0.0;
        et = 0.0;
      }

      if (!((i + 1) % INTERVAL))
        std::cout << i + 1 << ", " << "Feature Value: "  << ft  << ", " << "Eigentropy: " << et << std::endl;


      featureValues[j].push_back( ft );
      eigentropy[j].push_back( et );
    }

    eigenValues.clear();
  }

  for ( size_t i = 0; i < numVert; i++ )
  {
    for ( int j = 0; j < numItr; j++ )
    {
      tmpEigentropy.push_back( eigentropy[j][i] );
    }

    std::vector<float>::iterator minIt = std::min_element( tmpEigentropy.begin(), tmpEigentropy.end() );
    size_t minEigentropyIndex          = std::distance( tmpEigentropy.begin(), minIt );

    tmpEigentropy.clear();

    selectedFeature.push_back( featureValues[minEigentropyIndex][i] );

    if ( sigMax < featureValues[minEigentropyIndex][i] )
      sigMax = featureValues[minEigentropyIndex][i];
  }

  m_maxFeature = 1.0;
  std::cout << "Maximun of Sigma : " << sigMax << std::endl;

  // Normalize feature values
  float normFt;

  for ( float f : selectedFeature )
  {
    normFt = f / sigMax;
    m_feature.push_back( normFt );
  }
}

void calculateFeature::calcMSFeature(kvs::PolygonObject *ply)
{
}

void calculateFeature::calcPlaneBasedFeature(kvs::PolygonObject *ply)
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

  double allowableError;

  std::cout << "==================================" << std::endl;
  std::cout << "Input Allowable Error : ";
  std::cin >> allowableError;
  std::cout << "==================================" << std::endl;

  // create octree
  std::cout << "Creating Octree... (Number of Vertex : " << numVert << std::endl;
  std::cout << minBB << " \n"
            << maxBB << std::endl;
  octree *myTree = new octree(pdata, numVert, mrange, MIN_NODE);

  kvs::MersenneTwister uniRand;
  double sigMax = 0.0;
  std::vector<float> featureValues;

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

      double x = coords[3 * nearInd[j]];
      double y = coords[3 * nearInd[j] + 1];
      double z = coords[3 * nearInd[j] + 2];

      xb += x;
      yb += y;
      zb += z;
    }

    xb /= (double)n0;
    yb /= (double)n0;
    zb /= (double)n0;

    //--- Calculaton of covariance matrix
    double xx = 0.0, yy = 0.0, zz = 0.0;
    double xy = 0.0, yz = 0.0, zx = 0.0;
    for (int j = 0; j < n0; j++)
    {

      double nx = (coords[3 * nearInd[j]] - xb);
      double ny = (coords[3 * nearInd[j] + 1] - yb);
      double nz = (coords[3 * nearInd[j] + 2] - zb);
      xx += nx * nx;
      yy += ny * ny;
      zz += nz * nz;
      xy += nx * ny;
      yz += ny * nz;
      zx += nz * nx;
    }

    double s_xx = xx / (double)(n0);
    double s_yy = yy / (double)(n0);
    double s_zz = zz / (double)(n0);
    double s_xy = xy / (double)(n0);
    double s_yz = yz / (double)(n0);
    double s_zx = zx / (double)(n0);

    // double s_yx = s_xy;
    // double s_zy = s_yz;
    // double s_xz = s_zx;

     /***

    // Caluculate Covariance matrix and EigenVectors using KVS
    //---- Covariance matrix
    kvs::Matrix<double> M( 3, 3 );
    M[0][0] = s_xx; M[0][1] = s_xy; M[0][2] = s_xz;
    M[1][0] = s_yx; M[1][1] = s_yy; M[1][2] = s_yz;
    M[2][0] = s_zx; M[2][1] = s_zy; M[2][2] = s_zz;

    //---- Calcuation of eigenvalues
    kvs::EigenDecomposer<double> eigen( M );
    const kvs::Matrix<double>& E = eigen.eigenVectors();

    ***/

     // Caluculate Covariance matrix and EigenVectors using LAPACK
    //--- Preparation for LAPACK
    char jovz = 'V';
    char uplo = 'U';
    int n     = DIM;
    double A[n*n];
    double W[n];
    int lwork = n*n;
    double WORK[n*n];
    int info;

    //---- Covariance matrix
    A[0] = s_xx; A[3] = s_xy; A[6] = s_zx;
    A[1] = 0.0 ; A[4] = s_yy; A[7] = s_yz;
    A[2] = 0.0 ; A[5] = 0.0 ; A[8] = s_zz;

    //---- Calcuation of eigenvalues and egenvectors
    dsyev_( &jovz, &uplo, (__CLPK_integer *) &n, A, (__CLPK_integer *) &n,
            W, WORK, (__CLPK_integer *) &lwork, (__CLPK_integer *) &info );

    int notOnLocalPlane = 0;

    for ( int j = 0; j < n0; j++ )
    {

      // Calculate Local Plane
      // double px = coords[3 * nearInd[j]]     - point[0];
      // double py = coords[3 * nearInd[j] + 1] - point[1];
      // double pz = coords[3 * nearInd[j] + 2] - point[2];

      // Calculate Least Squares Plane
      double px = coords[3 * nearInd[j]]     - xb;
      double py = coords[3 * nearInd[j] + 1] - yb;
      double pz = coords[3 * nearInd[j] + 2] - zb;

      double localPlane = std::fabs( px*A[0] + py*A[1] + pz*A[2] );

      if ( localPlane > allowableError )
        notOnLocalPlane++;

    }

    double var = (double)notOnLocalPlane/(double)n0;

    featureValues.push_back(var);

    if (sigMax < var)
      sigMax = var;
    if (!((i + 1) % INTERVAL))
      std::cout << i + 1 << ", " << n0 << ": " << var << std::endl;
  }

  m_maxFeature = 1.0;
  std::cout << "Maximun of Sigma : " << sigMax << std::endl;

  // Normalize feature values
  float normFt;

  for ( float f : featureValues )
  {
    normFt = f / sigMax;
    m_feature.push_back( normFt );
  }

}

std::vector<float> calculateFeature::calcFeatureValues(kvs::PolygonObject* ply, double radius)
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

  std::vector<float> featureValues;
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
    search_points(point, radius, pdata, myTree->octreeRoot, &nearInd, &dist);
    int n0 = (int)nearInd.size();

    //--- Standardization for x, y, z
    double xb = 0.0, yb = 0.0, zb = 0.0;
    for (int j = 0; j < n0; j++)
    {

      double x = coords[3 * nearInd[j]];
      double y = coords[3 * nearInd[j] + 1];
      double z = coords[3 * nearInd[j] + 2];

      xb += x;
      yb += y;
      zb += z;
    }

    xb /= (double)n0;
    yb /= (double)n0;
    zb /= (double)n0;

    //--- Calculaton of covariance matrix
    double xx = 0.0, yy = 0.0, zz = 0.0;
    double xy = 0.0, yz = 0.0, zx = 0.0;
    for (int j = 0; j < n0; j++)
    {

      double nx = (coords[3 * nearInd[j]] - xb);
      double ny = (coords[3 * nearInd[j] + 1] - yb);
      double nz = (coords[3 * nearInd[j] + 2] - zb);
      xx += nx * nx;
      yy += ny * ny;
      zz += nz * nz;
      xy += nx * ny;
      yz += ny * nz;
      zx += nz * nx;
    }

    double s_xx = xx / (double)(n0);
    double s_yy = yy / (double)(n0);
    double s_zz = zz / (double)(n0);
    double s_xy = xy / (double)(n0);
    double s_yz = yz / (double)(n0);
    double s_zx = zx / (double)(n0);

    // double s_yx = s_xy;
    // double s_zy = s_yz;
    // double s_xz = s_zx;

    /***

    // Caluculate Covariance matrix and EigenValues using KVS
    //---- Covariance matrix
    kvs::Matrix<double> M( 3, 3 );
    M[0][0] = s_xx; M[0][1] = s_xy; M[0][2] = s_xz;
    M[1][0] = s_yx; M[1][1] = s_yy; M[1][2] = s_yz;
    M[2][0] = s_zx; M[2][1] = s_zy; M[2][2] = s_zz;

    //---- Calcuation of eigenvalues
    kvs::EigenDecomposer<double> eigen( M );
    const kvs::Vector<double>& L = eigen.eigenValues();

    ***/

    // Caluculate Covariance matrix and EigenValues using LAPACK
    //--- Preparation for LAPACK
    char jovz = 'V';
    char uplo = 'U';
    int n     = DIM;
    double A[n*n];
    double W[n];
    int lwork = n*n;
    double WORK[n*n];
    int info;

    //---- Covariance matrix
    A[0] = s_xx; A[3] = s_xy; A[6] = s_zx;
    A[1] = 0.0 ; A[4] = s_yy; A[7] = s_yz;
    A[2] = 0.0 ; A[5] = 0.0 ; A[8] = s_zz;

    //---- Calcuation of eigenvalues and egenvectors
    dsyev_( &jovz, &uplo, (__CLPK_integer *) &n, A, (__CLPK_integer *) &n,
            W, WORK, (__CLPK_integer *) &lwork, (__CLPK_integer *) &info );


    // W[2]: 第1固有値, W[1]: 第2固有値, W[0]: 第3固有値
    // Sum of eigenvalues
    double sum = W[2] + W[1] + W[0];

    // Change of curvature
    // double var = W[0] / sum;

    // Linearity
    // double var = ( W[2] - W[1] ) / W[2];

    // Planarity
    // double var = ( W[1] - W[0]) / W[2];

    // Aplanarity
    double var = 1 - ( ( W[1] - W[0]) / W[2] );

    // Eigentropy
    // double lambda1 = W[2] / sum;
    // double lambda2 = W[1] / sum;
    // double lambda3 = W[0] / sum;
    // double var = -( lambda1*log(lambda1) + lambda2*log(lambda2) + lambda3*log(lambda3) );

    // double var = W[0];
    // double var = coords[3 * i];

    if (sum < EPSILON)
      var = 0.0;

    //--- Contributing rate of 3rd(minimum) component
    featureValues.push_back(var);
    if (sigMax < var)
      sigMax = var;

    if (!((i + 1) % INTERVAL))
      std::cout << i + 1 << ", " << n0 << ": " << var << std::endl;

  }

  m_maxFeature = 1.0;
  std::cout << "Maximun of Sigma : " << sigMax << std::endl;

  // Normalize feature values
  float normFt;
  std::vector<float> ft;

  for ( float f : featureValues )
  {
    normFt = f / sigMax;
    ft.push_back( normFt );
  }

  return ft;
}

std::vector<double> calculateFeature::calcEigenValues(kvs::PolygonObject* ply, double radius)
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

  std::vector<double> eigenValues;

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
    search_points(point, radius, pdata, myTree->octreeRoot, &nearInd, &dist);
    int n0 = (int)nearInd.size();

    //--- Standardization for x, y, z
    double xb = 0.0, yb = 0.0, zb = 0.0;
    for (int j = 0; j < n0; j++)
    {

      double x = coords[3 * nearInd[j]];
      double y = coords[3 * nearInd[j] + 1];
      double z = coords[3 * nearInd[j] + 2];

      xb += x;
      yb += y;
      zb += z;
    }

    xb /= (double)n0;
    yb /= (double)n0;
    zb /= (double)n0;

    //--- Calculaton of covariance matrix
    double xx = 0.0, yy = 0.0, zz = 0.0;
    double xy = 0.0, yz = 0.0, zx = 0.0;
    for (int j = 0; j < n0; j++)
    {

      double nx = (coords[3 * nearInd[j]] - xb);
      double ny = (coords[3 * nearInd[j] + 1] - yb);
      double nz = (coords[3 * nearInd[j] + 2] - zb);
      xx += nx * nx;
      yy += ny * ny;
      zz += nz * nz;
      xy += nx * ny;
      yz += ny * nz;
      zx += nz * nx;
    }

    double s_xx = xx / (double)(n0);
    double s_yy = yy / (double)(n0);
    double s_zz = zz / (double)(n0);
    double s_xy = xy / (double)(n0);
    double s_yz = yz / (double)(n0);
    double s_zx = zx / (double)(n0);

    // double s_yx = s_xy;
    // double s_zy = s_yz;
    // double s_xz = s_zx;

    /***

    // Caluculate Covariance matrix and EigenValues using KVS
    //---- Covariance matrix
    kvs::Matrix<double> M( 3, 3 );
    M[0][0] = s_xx; M[0][1] = s_xy; M[0][2] = s_xz;
    M[1][0] = s_yx; M[1][1] = s_yy; M[1][2] = s_yz;
    M[2][0] = s_zx; M[2][1] = s_zy; M[2][2] = s_zz;

    //---- Calcuation of eigenvalues
    kvs::EigenDecomposer<double> eigen( M );
    const kvs::Vector<double>& L = eigen.eigenValues();

    ***/

    // Caluculate Covariance matrix and EigenValues using LAPACK
    //--- Preparation for LAPACK
    char jovz = 'V';
    char uplo = 'U';
    int n     = DIM;
    double A[n*n];
    double W[n];
    int lwork = n*n;
    double WORK[n*n];
    int info;

    //---- Covariance matrix
    A[0] = s_xx; A[3] = s_xy; A[6] = s_zx;
    A[1] = 0.0 ; A[4] = s_yy; A[7] = s_yz;
    A[2] = 0.0 ; A[5] = 0.0 ; A[8] = s_zz;

    //---- Calcuation of eigenvalues and egenvectors
    dsyev_( &jovz, &uplo, (__CLPK_integer *) &n, A, (__CLPK_integer *) &n,
            W, WORK, (__CLPK_integer *) &lwork, (__CLPK_integer *) &info );


    eigenValues.push_back(W[2]);
    eigenValues.push_back(W[1]);
    eigenValues.push_back(W[0]);

    if (!((i + 1) % INTERVAL))
      std::cout << i + 1 << ", " << n0 << " EigenValues: ( " << eigenValues[0] << ", "  << eigenValues[1] << ", " << eigenValues[2] << " )" << std::endl;

  }

  return eigenValues;
}
