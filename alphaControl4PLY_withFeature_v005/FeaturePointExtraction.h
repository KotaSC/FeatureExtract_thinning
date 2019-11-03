#ifndef _FeaturePointExtraction_H__
#define _FeaturePointExtraction_H__

#include <kvs/PolygonObject>
#include <kvs/PointObject>
#include <kvs/Module>
#include <vector>

class FeaturePointExtraction: public kvs::PointObject {
  kvsModuleSuperClass( kvs::PointObject );

 public:
  FeaturePointExtraction( void );
  FeaturePointExtraction( kvs::PolygonObject* ply,
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
