#include "FeaturePointExtraction.h"
#include "feature_customize.h"
#include <vector>
#include <numeric>
#include <cmath>
#include <algorithm>
#include <kvs/MersenneTwister>



FeaturePointExtraction::FeaturePointExtraction( void ) {
}

FeaturePointExtraction::FeaturePointExtraction( kvs::PolygonObject* ply,
                                                std::vector<float> &ft,
                                                double ft_ratio,
                                                double threshold,
                                                int repeatLevel,
                                                kvs::Vector3f BBMin,
                                                kvs::Vector3f BBMax,
                                                AlphaControlforPLY *fpoint )
{
  alpbaControl4Feature( ply, ft, ft_ratio, threshold, repeatLevel, BBMin, BBMax, fpoint );

}

void FeaturePointExtraction::alpbaControl4Feature( kvs::PolygonObject* ply,
                                                   std::vector<float> &ft,
                                                   double ft_ratio,
                                                   double threshold,
                                                   int repeatLevel,
                                                   kvs::Vector3f BBMin,
					                                         kvs::Vector3f BBMax,
                                                   AlphaControlforPLY *fpoint )
{
  size_t numVert = ply->numberOfVertices();
  std::vector<int> ind;

  // 特徴量の最大値を求める
  double maxFt = *std::max_element( ft.begin(), ft.end() );

  int num = 0;
  for( size_t i = 0; i < numVert; i++ ) {
    if( ft[i] > threshold ) {
      ind.push_back( i );
      num++;
    }
  }

  // 特徴量の合計値
  // double sumFt = std::accumulate( ftNorm.begin(), ftNorm.end(), 0.0 );
  // std::cout << "Sum of feature value    : " << sumFt << std::endl;

  // 特徴量の平均値
  // double aveFt = sumFt/ftNorm.size();
  // std::cout << "Avarage of feature value: " << aveFt << std::endl;

  kvs::ValueArray<kvs::Real32> coords  = ply->coords();
  kvs::ValueArray<kvs::Real32> normals = ply->normals();
  kvs::ValueArray<kvs::UInt8>  colors  = ply->colors();

  std::vector<kvs::Real32> SetCoords;
  std::vector<kvs::Real32> SetNormals;
  std::vector<kvs::UInt8>  SetColors;

  double alphaMax = ALPHA_MAX;
  double alphaMin = ALPHA_MIN;
  double dim      = DIMENSION;
  double xMax     = X_MAX;

  double initialPoint = xMax - ( threshold/maxFt );
  double grad         = ( alphaMax - alphaMin ) / std::pow( initialPoint, dim );

  std::cout << "===========================================" << std::endl;
  std::cout << "Max opacity              : " << alphaMax     << std::endl;
  std::cout << "Min opacity              : " << alphaMin     << std::endl;
  std::cout << "Dimension                : " << dim          << std::endl;
  std::cout << "Gradient                 : " << grad         << std::endl;
  std::cout << "Max feature value        : " << maxFt        << std::endl;
  std::cout << "Number of feature points : " << num          << std::endl;
  std::cout << "===========================================" << std::endl;


  kvs::MersenneTwister uniRand;

  for ( int i = 0; i < num; i++ ) {

    size_t index = ind[ i ];

    double x     = ( ft[index] - threshold ) / maxFt;
    double alpha = grad*std::pow( x, dim ) + alphaMin;

    // 任意の不透明度を実現するために必要な点数・増減率を計算する
    double a_num     = fpoint->calculateRequiredPartcleNumber( alpha, repeatLevel, BBMin, BBMax );
    double ratio     = fpoint->pointRatio( a_num );
    double createNum = 1.0*ratio;


    for( int j = 0; j < createNum; j++ ) {

      SetCoords.push_back( coords[3*index] );
      SetCoords.push_back( coords[3*index+1] );
      SetCoords.push_back( coords[3*index+2] );

      SetNormals.push_back( normals[3*index] );
      SetNormals.push_back( normals[3*index+1] );
      SetNormals.push_back( normals[3*index+2] );

      // SetColors.push_back( colors[3*index] );
      // SetColors.push_back( colors[3*index+1] );
      // SetColors.push_back( colors[3*index+2] );

      SetColors.push_back( 255 );
      SetColors.push_back( 0 );
      SetColors.push_back( 0 );

      // SetColors.push_back( 255 * ft[index]/maxFt );
      // SetColors.push_back( 0 );
      // SetColors.push_back( 0 );
    }
  }

  SuperClass::setCoords ( kvs::ValueArray<kvs::Real32>( SetCoords  ) );
  SuperClass::setNormals( kvs::ValueArray<kvs::Real32>( SetNormals ) );
  SuperClass::setColors ( kvs::ValueArray<kvs::UInt8> ( SetColors  ) );
}