#include "FeaturePointExtraction.h"
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
                                                double threshold )
{
  alpbaControl4Feature( ply, ft, ft_ratio, threshold );

}

void FeaturePointExtraction::alpbaControl4Feature( kvs::PolygonObject* ply,
                                                   std::vector<float> &ft,
                                                   double ft_ratio,
                                                   double threshold )
{
  size_t numVert = ply->numberOfVertices();
  std::vector<int> ind;

  // 特徴量の最大値を求める
  double maxFt = *std::max_element( ft.begin(), ft.end() );
  std::cout << "Max feature: " << maxFt << std::endl;

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

  double gmax  = 1.0;
  double gmin  = 0.0;
  double dim   = 2.0;
  double gx    = 1.0 - ( threshold/maxFt );
  double denom = std::pow( gx, dim );

  // 関数の傾き
  double grad = ( gmax - gmin )/denom;

  std::cout << "Gradient                : " << grad << std::endl;
  std::cout << "Number of feature points: " << num << std::endl;

  double createNum = ft_ratio*(double)num;
 
  std::cout << "createNum: " << createNum << std::endl;

  kvs::ValueArray<kvs::Real32> coords  = ply->coords();
  kvs::ValueArray<kvs::Real32> normals = ply->normals();
  kvs::ValueArray<kvs::UInt8>  colors  = ply->colors();

  std::vector<kvs::Real32> SetCoords;
  std::vector<kvs::Real32> SetNormals;
  std::vector<kvs::UInt8>  SetColors;

  kvs::MersenneTwister uniRand;


  int i = 0;

  while( true ) {

    if ( i > createNum )
      break;

    size_t id = ( size_t )( ( double )num*uniRand() );

    if (id == num)
      --id;

    size_t index = ind[ id ];

    // ここら辺をいじって点の生成数とか変える
    double x = ( ft[index] - threshold ) / maxFt;

    double xdim = std::pow( x, dim );
    double f = grad*xdim + gmin;

    double r = uniRand();

    if( f >= r ){

      i++;

      SetCoords.push_back ( coords[3*index]   );
      SetCoords.push_back ( coords[3*index+1] );
      SetCoords.push_back ( coords[3*index+2] );

      SetNormals.push_back( normals[3*index]   );
      SetNormals.push_back( normals[3*index+1] );
      SetNormals.push_back( normals[3*index+2] );

      // SetColors.push_back ( colors[3*index]   );
      // SetColors.push_back ( colors[3*index+1] );
      // SetColors.push_back ( colors[3*index+2] );
     
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
