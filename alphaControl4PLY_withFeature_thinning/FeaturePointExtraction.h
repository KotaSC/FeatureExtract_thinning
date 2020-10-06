#ifndef _FeaturePointExtraction_H__
#define _FeaturePointExtraction_H__

#include <kvs/PolygonObject>
#include <kvs/PointObject>
#include <kvs/Module>
#include <vector>

#include "AlphaControlforPLY.h"

class FeaturePointExtraction: public kvs::PointObject {
  kvsModuleSuperClass( kvs::PointObject );

 public:
  FeaturePointExtraction( void );
  FeaturePointExtraction( kvs::PolygonObject* ply,
						              std::vector<float> &ft,
						              double threshold,
						              int repeatLevel,
						              kvs::Vector3f BBMin,
					   	            kvs::Vector3f BBMax,
						              AlphaControlforPLY *fpoint );
 private:
  void alpbaControl4Feature( kvs::PolygonObject* ply,
						                 std::vector<float> &ft,
						                 double threshold,
								             int repeatLevel,
							               kvs::Vector3f BBMin,
							               kvs::Vector3f BBMax,
							               AlphaControlforPLY *fpoint );

	void adaptiveAlphaControl4Feature( kvs::PolygonObject *ply,
																		 std::vector<float> &ft,
																		 double threshold,
																		 int repeatLevel,
																		 kvs::Vector3f BBMin,
																		 kvs::Vector3f BBMax,
																		 AlphaControlforPLY *fpoint );

	std::vector<double> calcOpacity( int featurePointNum,
                                   std::vector<float> &featureValue,
                                   std::vector<int> &featurePointIndex );
};

#endif
