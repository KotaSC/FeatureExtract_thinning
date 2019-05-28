#include "FeaturePointExtraction.h"

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

  int num = 0;
  for( size_t i=0; i< numVert; i++ ) {
    if( ft[i] > threshold ) {
      ind.push_back( i );
      num++;
    }
  }
  std::cout << "Number of feature points: " << num << std::endl;

  double createNum = ft_ratio*(double)num;

  kvs::ValueArray<kvs::Real32> coords  = ply->coords();
  kvs::ValueArray<kvs::Real32> normals = ply->normals();
  kvs::ValueArray<kvs::UInt8>  colors  = ply->colors();

  std::vector<kvs::Real32> SetCoords;
  std::vector<kvs::Real32> SetNormals;
  std::vector<kvs::UInt8>  SetColors;

  kvs::MersenneTwister uniRand;

  for( int i=0; i<createNum; i++ ) {

    size_t id = (size_t)((double)num*uniRand());
    size_t index = ind[ id ];

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

  }

  SuperClass::setCoords( kvs::ValueArray<kvs::Real32>( SetCoords ) );
  SuperClass::setNormals( kvs::ValueArray<kvs::Real32>( SetNormals ) );
  SuperClass::setColors( kvs::ValueArray<kvs::UInt8>( SetColors ) );
}
