#include "FeaturePointExtraction.h"
#include "alp_customize.h"
#include "octree.h"
#include <vector>
#include <numeric>
#include <cmath>
#include <algorithm>
#include <kvs/BoxMuller>

const int INTERVAL = 1000000;
const int MIN_NODE = 15;

FeaturePointExtraction::FeaturePointExtraction( void ) {
}

FeaturePointExtraction::FeaturePointExtraction( kvs::PolygonObject* ply,
                                                std::vector<float> &ft,
                                                double smallFth,
                                                int repeatLevel,
                                                kvs::Vector3f BBMin,
                                                kvs::Vector3f BBMax,
                                                AlphaControlforPLY *fpoint )
{
  alpbaControl4Feature( ply, ft, smallFth, repeatLevel, BBMin, BBMax, fpoint );
}

void FeaturePointExtraction::alpbaControl4Feature( kvs::PolygonObject* ply,
                                                   std::vector<float> &ft,
                                                   double smallFth,
                                                   int repeatLevel,
                                                   kvs::Vector3f BBMin,
					                                         kvs::Vector3f BBMax,
                                                   AlphaControlforPLY *fpoint )
{
  size_t numVert = ply->numberOfVertices();
  std::vector<int> ind;

  int num = 0;
  for( size_t i = 0; i < numVert; i++ ) {
    if( ft[i] >= smallFth ) {
      ind.push_back( i );
      num++;
    }
  }

  kvs::ValueArray<kvs::Real32> coords  = ply->coords();
  kvs::ValueArray<kvs::Real32> normals = ply->normals();
  kvs::ValueArray<kvs::UInt8>  colors  = ply->colors();

  std::vector<kvs::Real32> SetCoords;
  std::vector<kvs::Real32> SetNormals;
  std::vector<kvs::UInt8>  SetColors;

  double alphaMin     = ALPHA_MIN;
  double alphaMax     = ALPHA_MAX;
  double largeFth     = LARGE_F_TH;
  double dim          = DIMENSION;
  double initialPoint = largeFth - smallFth;
  double grad         = ( alphaMax - alphaMin ) / std::pow( initialPoint, dim );

  std::cout << "==================================="     << std::endl;
  std::cout << "Minimum opacity          : " << alphaMin << std::endl;
  std::cout << "Maximum opacity          : " << alphaMax << std::endl;
  std::cout << "Small threshold          : " << smallFth << std::endl;
  std::cout << "Large threshold          : " << largeFth << std::endl;
  std::cout << "Dimension                : " << dim      << std::endl;
  std::cout << "Number of feature points : " << num      << std::endl;
  std::cout << "==================================="     << std::endl;

  for ( int i = 0; i < num; i++ ) {

    size_t index = ind[ i ];

    double x     = ft[index] - smallFth;
    double alpha = grad*std::pow( x, dim ) + alphaMin;

    if ( ft[index] >= LARGE_F_TH )
      alpha = alphaMax;

    // Caluculate Point Number and Increase Ratio according to Feature Value
    double a_num     = fpoint->calculateRequiredPartcleNumber( alpha, repeatLevel, BBMin, BBMax );
    double ratio     = fpoint->pointRatio( a_num );
    double createNum = 1.0*ratio;


    if ( !((i + 1) % INTERVAL) )
    {
      std::cout << i + 1 << std::endl;
      std::cout << "Feature Value:        " << ft[index] << std::endl;
      std::cout << "Alpha:                " << alpha     << std::endl;
      std::cout << "Analytical Point Num: " << a_num     << std::endl;
      std::cout << "Point Ratio:          " << ratio     << std::endl;
      std::cout << "Create Point Num:     " << createNum << std::endl;
    }

    for( int j = 0; j < createNum; j++ ) {

      SetCoords.push_back( coords[3*index] );
      SetCoords.push_back( coords[3*index+1] );
      SetCoords.push_back( coords[3*index+2] );

      SetNormals.push_back( normals[3*index] );
      SetNormals.push_back( normals[3*index+1] );
      SetNormals.push_back( normals[3*index+2] );

      SetColors.push_back( colors[3*index] );
      SetColors.push_back( colors[3*index+1] );
      SetColors.push_back( colors[3*index+2] );

      // SetColors.push_back( 255 );
      // SetColors.push_back( 0 );
      // SetColors.push_back( 0 );
    }
  }

  SuperClass::setCoords ( kvs::ValueArray<kvs::Real32>( SetCoords  ) );
  SuperClass::setNormals( kvs::ValueArray<kvs::Real32>( SetNormals ) );
  SuperClass::setColors ( kvs::ValueArray<kvs::UInt8> ( SetColors  ) );
}

void FeaturePointExtraction::adaptiveAlphaControl4Feature( kvs::PolygonObject *ply,
                                                           std::vector<float> &ft,
                                                           double smallFth,
                                                           int repeatLevel,
                                                           kvs::Vector3f BBMin,
                                                           kvs::Vector3f BBMax,
                                                           AlphaControlforPLY *fpoint )
{
  size_t numVert = ply->numberOfVertices();
  std::vector<int> ind;

  int num = 0;
  for ( size_t i = 0; i < numVert; i++ )
  {
    if ( ft[i] >= smallFth )
    {
      ind.push_back(i);
      num++;
    }
  }

  double alphaMin = ALPHA_MIN;
  double alphaMax = ALPHA_MAX;
  double largeFth = LARGE_F_TH;
  double dim = DIMENSION;
  double initialPoint = largeFth - smallFth;
  double grad = (alphaMax - alphaMin) / std::pow(initialPoint, dim);

  std::cout << "===================================" << std::endl;
  std::cout << "Minimum opacity          : " << alphaMin << std::endl;
  std::cout << "Maximum opacity          : " << alphaMax << std::endl;
  std::cout << "Small threshold          : " << smallFth << std::endl;
  std::cout << "Large threshold          : " << largeFth << std::endl;
  std::cout << "Dimension                : " << dim << std::endl;
  std::cout << "Number of feature points : " << num << std::endl;
  std::cout << "===================================" << std::endl;

  ply->updateMinMaxCoords();

  kvs::ValueArray<kvs::Real32> coords  = ply->coords();
  kvs::ValueArray<kvs::Real32> normals = ply->normals();
  kvs::ValueArray<kvs::UInt8> colors   = ply->colors();

  std::vector<kvs::Real32> SetCoords;
  std::vector<kvs::Real32> SetNormals;
  std::vector<kvs::UInt8>  SetColors;

  float *pdata = coords.data();
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
  octree *myTree = new octree( pdata, numVert, mrange, MIN_NODE );

  kvs::MersenneTwister uniRand;

  double div;

  std::cout << "Input Division : ";
  std::cin >> div;

  kvs::Vector3f bb = maxBB - minBB;
  double b_leng    = bb.length();
  double radius    = b_leng / div;

  std::cout << "Start OCtree Search..... " << std::endl;
  for ( size_t i = 0; i < num; i++ )
  {
    if ( i == num )
      --i;

    size_t index = ind[i];

    double point[3] = { coords[3 * index],
                        coords[3 * index + 1],
                        coords[3 * index + 2] };

    vector<size_t> nearInd;
    vector<double> dist;
    search_points( point, radius, pdata, myTree->octreeRoot, &nearInd, &dist );
    int n0 = (int)nearInd.size();

    double sumNearestFt = 0.0;
    double aveNearestFt = 0.0;

    for ( size_t j = 0; j < n0; j++ ) {
      sumNearestFt += ft[ nearInd[j] ];
    }

    aveNearestFt = sumNearestFt / n0;

    double x = ft[index] - smallFth;
    double alpha = grad * std::pow(x, dim) + alphaMin;

    if (ft[index] >= LARGE_F_TH)
      alpha = alphaMax;

    // Caluculate Point Number and Increase Ratio according to Feature Value
    double a_num = fpoint->calculateRequiredPartcleNumber(alpha, repeatLevel, BBMin, BBMax);
    double ratio = fpoint->pointRatio(a_num);
    double createNum = 1.0 * ratio;

    if (!((i + 1) % INTERVAL))
    {
      std::cout << i + 1 << std::endl;
      std::cout << "Feature Value:        " << ft[index] << std::endl;
      std::cout << "Alpha:                " << alpha << std::endl;
      std::cout << "Analytical Point Num: " << a_num << std::endl;
      std::cout << "Point Ratio:          " << ratio << std::endl;
      std::cout << "Create Point Num:     " << createNum << std::endl;
    }

    for (int j = 0; j < createNum; j++)
    {

      SetCoords.push_back(coords[3 * index]);
      SetCoords.push_back(coords[3 * index + 1]);
      SetCoords.push_back(coords[3 * index + 2]);

      SetNormals.push_back(normals[3 * index]);
      SetNormals.push_back(normals[3 * index + 1]);
      SetNormals.push_back(normals[3 * index + 2]);

      SetColors.push_back(colors[3 * index]);
      SetColors.push_back(colors[3 * index + 1]);
      SetColors.push_back(colors[3 * index + 2]);

      // SetColors.push_back( 255 );
      // SetColors.push_back( 0 );
      // SetColors.push_back( 0 );
    }
  }
  SuperClass::setCoords(kvs::ValueArray<kvs::Real32>(SetCoords));
  SuperClass::setNormals(kvs::ValueArray<kvs::Real32>(SetNormals));
  SuperClass::setColors(kvs::ValueArray<kvs::UInt8>(SetColors));
}