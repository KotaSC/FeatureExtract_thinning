#include "FeaturePointExtraction.h"
#include "alp_customize.h"
#include "octree.h"
#include <vector>
#include <numeric>
#include <cmath>
#include <algorithm>

const int INTERVAL = 1000000;
const int MIN_NODE = 15;

FeaturePointExtraction::FeaturePointExtraction( void ) {
}

FeaturePointExtraction::FeaturePointExtraction(kvs::PolygonObject *ply,
                                               std::vector<float> &ft,
                                               double smallFth,
                                               double alphaMin,
                                               int repeatLevel,
                                               kvs::Vector3f BBMin,
                                               kvs::Vector3f BBMax,
                                               AlphaControlforPLY *fpoint)
{
  int colorID;
  int pfeID;

  // Select feature point color
  std::cout << "\nHighlight color" << std::endl;
  std::cout << "ORIGINAL: " << ORIGINAL_COLOR_ID << ", ";
  std::cout << "RED: " << RED_COLOR_ID << ", ";
  std::cout << "BLACK: " << BLACK_COLOR_ID << std::endl;

  std::cout << "Select an ID >> ";
  std::cin >> colorID;

  std::cout << "Highlight color ==> " << colorID << std::endl;
  std::cout << std::endl;

  // Select point feature extraction type
  std::cout << "Feature extraction type" << std::endl;
  std::cout << "Normal point feature extraction: " << NORMAL_PFE_ID << ", ";
  std::cout << "Adaptive point feature extraction: " << ADAPTIVE_PFE_ID << std::endl;

  std::cout << "Select an ID >> ";
  std::cin >> pfeID;

  std::cout << "Feature extraction type ==> " << pfeID << std::endl;
  std::cout << std::endl;

  if ( pfeID == NORMAL_PFE_ID )
    alpbaControl4Feature( ply, ft, smallFth, alphaMin, colorID, repeatLevel, BBMin, BBMax, fpoint );
  else if ( pfeID == ADAPTIVE_PFE_ID )
    adaptiveAlphaControl4Feature( ply, ft, smallFth, alphaMin, colorID, repeatLevel, BBMin, BBMax, fpoint );
}

void FeaturePointExtraction::alpbaControl4Feature(kvs::PolygonObject *ply,
                                                  std::vector<float> &ft,
                                                  double smallFth,
                                                  double alphaMin,
                                                  int colorID,
                                                  int repeatLevel,
                                                  kvs::Vector3f BBMin,
                                                  kvs::Vector3f BBMax,
                                                  AlphaControlforPLY *fpoint)
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

  std::cout << "\nInput opacity function parameters" << std::endl;
  std::vector<double> alphaVec = calcOpacity( num, smallFth, alphaMin, ft, ind );

  for ( int i = 0; i < num; i++ ) {

    size_t index = ind[ i ];

    // Caluculate Point Number and Increase Ratio according to Feature Value
    double alpha     = alphaVec[i];
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

      if ( colorID == ORIGINAL_COLOR_ID ) {
        SetColors.push_back( colors[3*index] );
        SetColors.push_back( colors[3*index+1] );
        SetColors.push_back( colors[3*index+2] );
      }
      else if ( colorID == RED_COLOR_ID ) {
        SetColors.push_back( 255 );
        SetColors.push_back( 0 );
        SetColors.push_back( 0 );
      }
      else if ( colorID == BLACK_COLOR_ID ) {
        SetColors.push_back(0);
        SetColors.push_back(0);
        SetColors.push_back(0);
      }
    }
  }

  SuperClass::setCoords ( kvs::ValueArray<kvs::Real32>( SetCoords  ) );
  SuperClass::setNormals( kvs::ValueArray<kvs::Real32>( SetNormals ) );
  SuperClass::setColors ( kvs::ValueArray<kvs::UInt8> ( SetColors  ) );
}

void FeaturePointExtraction::adaptiveAlphaControl4Feature( kvs::PolygonObject *ply,
                                                           std::vector<float> &ft,
                                                           double smallFth,
                                                           double alphaMin,
                                                           int colorID,
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

  double div;

  std::cout << "\nInput Division : ";
  std::cin >> div;
  std::cout << std::endl;

  kvs::Vector3f bb = maxBB - minBB;
  double b_leng    = bb.length();
  double radius    = b_leng / div;

  std::cout << "\nInput Type(b) function parameters" << std::endl;
  std::vector<double> alphaVecTypeB = calcOpacity( num, smallFth, alphaMin, ft, ind );

  std::cout << "\nInput Type(c) function parameters" << std::endl;
  std::vector<double> alphaVecTypeC = calcOpacity( num, smallFth, alphaMin, ft, ind );

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

    double alpha;

    if ( aveNearestFt < 0.5 )
      alpha = alphaVecTypeC[i];
    else
      alpha = alphaVecTypeB[i];

    // Caluculate Point Number and Increase Ratio according to Feature Value
    double a_num     = fpoint->calculateRequiredPartcleNumber( alpha, repeatLevel, BBMin, BBMax );
    double ratio     = fpoint->pointRatio( a_num );
    double createNum = 1.0 * ratio;

    if ( !((i + 1) % INTERVAL) )
    {
      std::cout << i + 1 << std::endl;
      std::cout << "Feature Value:        " << ft[index] << std::endl;
      std::cout << "Alpha:                " << alpha     << std::endl;
      std::cout << "Analytical Point Num: " << a_num     << std::endl;
      std::cout << "Point Ratio:          " << ratio     << std::endl;
      std::cout << "Create Point Num:     " << createNum << std::endl;
    }

    for (int j = 0; j < createNum; j++)
    {
      SetCoords.push_back( coords[3 * index] );
      SetCoords.push_back( coords[3 * index + 1] );
      SetCoords.push_back( coords[3 * index + 2] );

      SetNormals.push_back( normals[3 * index] );
      SetNormals.push_back( normals[3 * index + 1] );
      SetNormals.push_back( normals[3 * index + 2] );

      if ( colorID == ORIGINAL_COLOR_ID )
      {
        SetColors.push_back(colors[3 * index]);
        SetColors.push_back(colors[3 * index + 1]);
        SetColors.push_back(colors[3 * index + 2]);
      }
      else if ( colorID == RED_COLOR_ID )
      {
        SetColors.push_back(255);
        SetColors.push_back(0);
        SetColors.push_back(0);
      }
      else if ( colorID == BLACK_COLOR_ID )
      {
        SetColors.push_back(0);
        SetColors.push_back(0);
        SetColors.push_back(0);
      }
    }
  }
  SuperClass::setCoords( kvs::ValueArray<kvs::Real32>(SetCoords) );
  SuperClass::setNormals( kvs::ValueArray<kvs::Real32>(SetNormals) );
  SuperClass::setColors( kvs::ValueArray<kvs::UInt8>(SetColors) );
}

std::vector<double> FeaturePointExtraction::calcOpacity( int featurePointNum,
                                                         double smallFth,
                                                         double alphaMin,
                                                         std::vector<float> &featureValue,
                                                         std::vector<int> &featurePointIndex )
{

  double alphaMax;
  double largeFth;
  double dim;

  std::cout << "Maximum opacity: ";
  std::cin >> alphaMax;

  std::cout << "Large feature value threshold: ";
  std::cin >> largeFth;

  std::cout << "Dimension: ";
  std::cin >> dim;
  std::cout << std::endl;

  double initialPoint = largeFth - smallFth;
  double grad         = ( alphaMax - alphaMin ) / std::pow( initialPoint, dim );

  std::vector<double> alphaVec;

  for ( size_t i = 0; i < featurePointNum; i++ )
  {
    if ( i == featurePointNum )
      --i;

    size_t index = featurePointIndex[i];

    double f     = featureValue[index] - smallFth;
    double alpha = grad * std::pow( f, dim ) + alphaMin;

    if ( featureValue[index] >= largeFth )
      alpha = alphaMax;

    alphaVec.push_back( alpha );
  }

  return alphaVec;
}