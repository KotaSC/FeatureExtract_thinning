#ifndef _FeaturePointExtraction_H__
#define _FeaturePointExtraction_H__

#include <kvs/PolygonObject>
#include <kvs/PointObject>
#include <kvs/Module>

#include <fstream>

#include "AlphaControlforPLY.h"
class FeaturePointExtraction: public kvs::PointObject {
  kvsModuleSuperClass( kvs::PointObject );

	public:
		FeaturePointExtraction( void );
  	FeaturePointExtraction( kvs::PolygonObject* ply,
						              	std::vector<float> &ft,
						              	double threshold,
														double alphaMin,
						              	int repeatLevel,
														int imageResolution,
						              	kvs::Vector3f BBMin,
					   	            	kvs::Vector3f BBMax,
						              	AlphaControlforPLY *fpoint,
														std::string dirName );

	private:
		void alpbaControl4Feature( kvs::PolygonObject *ply,
															 std::vector<float> &ft,
															 double threshold,
															 double alphaMin,
															 int colorID,
															 int repeatLevel,
															 int imageResolution,
															 kvs::Vector3f BBMin,
															 kvs::Vector3f BBMax,
															 AlphaControlforPLY *fpoint,
															 std::string dirName );

		void adaptiveAlphaControl4Feature( kvs::PolygonObject *ply,
																		   std::vector<float> &ft,
																		   double threshold,
																			 double alphaMin,
																			 int colorID,
																		   int repeatLevel,
																			 int imageResolution,
																		   kvs::Vector3f BBMin,
																		   kvs::Vector3f BBMax,
																		   AlphaControlforPLY *fpoint,
																			 std::string dirName );

		std::vector<double> calcOpacity( int featurePointNum,
																		 double threshold,
																		 double alphaMin,
																		 std::vector<float> &featureValue,
																		 std::vector<int> &featurePointIndex );

		void writeParameterList( double smallFth,
														 double largeFth,
														 double alphaMin,
														 double alphaMax,
														 double d,
														 int repeatLevel,
														 int imageResolution,
														 std::string parameterList,
														 std::string dirName );

		void writeParameterList4AdaptivePFE( double smallFth,
																				 double largeFth,
																				 double alphaMin,
																				 double alphaMax,
																				 double d,
																				 int repeatLevel,
																				 int imageResolution,
																				 double div,
																				 double functionSwitchingThreshold,
																				 std::string parameterList,
																				 std::string dirName );

	private:
		double alphaMax;
		double largeFth;
		double dim;
		double div;
		double functionSwitchingThreshold;
};

#endif
