#ifndef _calculateFeature_H__
#define _calculateFeature_H__

#include <kvs/PolygonObject>
#include <vector>

class calculateFeature
{

public:
  enum FeatureType
  {
    PointPCA              = 0,
    MinimumEntropyFeature = 1,
    NormalPCA             = 2,
    NormalDispersion      = 3,
    PlaneBasedFeature     = 4
  };

  enum FeatureValueID
  {
    CHANGE_OF_CURVATURE_ID = 0,
    APLANARITY_ID          = 1,
    LINEARITY_ID           = 2,
    EIGENTROPY_ID          = 3,
    SUM_OF_EIGENVALUES_ID  = 4,
    PLANARITY_ID           = 5,
  };

public:
  calculateFeature( void );
  calculateFeature( const FeatureType type,
                    const FeatureValueID id,
                    const double distance,
                    kvs::PolygonObject *ply );

  std::vector<float> feature( void ) { return m_feature; }
  void setFeatureType( FeatureType type );
  void setFeatureValueID( FeatureValueID id );
  void addNoise( double noise );
  void setSearchRadius( double distance );
  void setSearchRadius( double divide,
			                  kvs::Vector3f bbmin,
			                  kvs::Vector3f bbmax  );
  double setMinMaxSearchRadius( double divide,
			                    kvs::Vector3f bbmin,
			                    kvs::Vector3f bbmax  );
  void calc( kvs::PolygonObject *ply );
  double maxFeature( void ) { return m_maxFeature; }
  double minFeature( void ) { return m_minFeature; }

private:
  size_t m_number;
  FeatureType m_type;
  FeatureValueID m_feature_id;
  std::vector<float> m_feature; // Feature Data
  bool m_isNoise;
  double m_noise;
  double m_searchRadius;
  double m_maxFeature;
  double m_minFeature;

 private:
   void calcPointPCA( kvs::PolygonObject *ply );
   void calcNormalPCA( kvs::PolygonObject *ply, std::vector<float> &normal );
   void calcNormalDispersion( kvs::PolygonObject *ply, std::vector<float> &normal );

   void calcMinimumEntropyFeature( kvs::PolygonObject *ply );
   void calcPlaneBasedFeature( kvs::PolygonObject *ply );

   std::vector<float> calcFeatureValues( kvs::PolygonObject *ply, double radius );
   std::vector<double> calcEigenValues( kvs::PolygonObject *ply, double radius );


};

#endif