#include "FeaturePointExtraction.h"
#include "alp_customize.h"
#include <vector>
#include <numeric>
#include <cmath>
#include <algorithm>

const int INTERVAL = 1000000;

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

  std::cout << "Max opacity              : " << alphaMax << std::endl;
  std::cout << "Min opacity              : " << alphaMin << std::endl;
  std::cout << "Dimension                : " << dim      << std::endl;
  std::cout << "Number of feature points : " << num      << std::endl;

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