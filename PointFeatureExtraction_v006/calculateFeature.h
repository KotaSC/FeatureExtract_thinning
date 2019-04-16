#ifndef _calculateFeature_H__
#define _calculateFeature_H__

#include <kvs/PolygonObject>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>

class calculateFeature
{

 public:

  enum FeatureType {
    PointPCA = 0,
    NormalPCA = 1,
    NormalDispersion = 2
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

 private:
  size_t m_number;
  FeatureType m_type;
  std::vector<float> m_feature;	/* Feature Data */
  bool m_isNoise;
  double m_noise;
  double m_searchRadius;
  double m_maxFeature;

 private:
  void calcPointPCA( std::vector<pcl::PointXYZ> &point );
  void calcNormalPCA( std::vector<pcl::PointXYZ> &point,
		      std::vector<float> &normal );
  void calcNormalDispersion( std::vector<pcl::PointXYZ> &point,
		       std::vector<float> &normal );

};

#endif

