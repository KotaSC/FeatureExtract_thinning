#ifndef _calculateFeature_H__
#define _calculateFeature_H__

#include <kvs/PolygonObject>
#include <vector>

class calculateFeature
{

 public:

  enum FeatureType {
    PointPCA             = 0,
    NormalPCA            = 1,
    NormalDispersion     = 2,
    InnerProductOfNormal = 3,
    DepthDisplacement    = 4,
    CurvatureDifference  = 5,
    MinimumEntropy       = 7
  };
 public:
  calculateFeature( void );
  calculateFeature( const FeatureType type,
		    const double distance,
		    kvs::PolygonObject* ply );

  std::vector<float> feature( void ) { return m_feature; }
  void setFeatureType( FeatureType type );
  void addNoise( double noise );
  void setSearchRadius( double distance );
  void setSearchRadius( double divide,
			kvs::Vector3f bbmin,
			kvs::Vector3f bbmax  );
  void calc( kvs::PolygonObject* ply );
  double maxFeature( void ) { return m_maxFeature; }
  double minFeature( void ) { return m_minFeature; }

 private:
  size_t m_number;
  FeatureType m_type;
  std::vector<float> m_feature;	/* Feature Data */
  bool m_isNoise;
  double m_noise;
  double m_searchRadius;
  double m_maxFeature;
  double m_minFeature;

 private:
  void calcPointPCA( kvs::PolygonObject* ply );
  void calcNormalPCA( kvs::PolygonObject* ply,
		      std::vector<float> &normal );
  void calcNormalDispersion( kvs::PolygonObject* ply ,
		       std::vector<float> &normal );
  void calcInnerProductOfNormal( kvs::PolygonObject* ply );
  void calcDepthDisplacement( kvs::PolygonObject* ply );
  void calcCurvatureDifference( kvs::PolygonObject* ply );
  void calcMinimumEntropy( kvs::PolygonObject* ply );

  std::vector<float> calcFeature( kvs::PolygonObject* ply, double bbDiv );

};

#endif

