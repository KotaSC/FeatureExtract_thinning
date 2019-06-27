#ifndef _FeaturePointExtractionThinning_H__
#define _FeaturePointExtractionThinning_H__

#include <kvs/PolygonObject>
#include <kvs/PointObject>
#include <kvs/Module>
#include <vector>

class FeaturePointExtractionThinning: public kvs::PointObject {
  kvsModuleSuperClass( kvs::PointObject );

 public:
  FeaturePointExtractionThinning( void );
  FeaturePointExtractionThinning( kvs::PolygonObject* ply,
						  std::vector<float> &ft,
						  double ft_ratio,
						  double threshold );
 private:
  void alpbaControl4Feature( kvs::PolygonObject* ply,
						     std::vector<float> &ft,
						     double ft_ratio,
						     double threshold );
};




#endif
