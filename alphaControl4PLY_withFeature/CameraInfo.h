////////////////////////
///// CameraInfo.h /////
////////////////////////

#if !defined  SSM__CAMERA_INFO_H_INCLUDE
#define       SSM__CAMERA_INFO_H_INCLUDE

#include<kvs/Vector3>
#include <kvs/PointObject>
#include <kvs/CellByCellParticleGenerator>

class CameraInfo {

 public:
 void set ( kvs::Camera*  camera, 
            double min_x, double min_y, double min_z , 
            double max_x, double max_y, double max_z      );
 /*******
  const float  pointDepth        ( const kvs::Vector3d& P ) ;
  double crossSectionSpreadRatio ( const kvs::Vector3d& P ) ; 
 *******/

  GLdouble* modelviewMatrix ( void ) { return m_modelview  ; }
  GLdouble* projectionMatrix( void ) { return m_projection ; }
  GLint*    viewportMatrix  ( void ) { return m_viewport   ; }
  float     bbCenterDepth   ( void ) { return m_bb_center_depth ;}

 private:
  kvs::Vector3f m_bb_min ;
  kvs::Vector3f m_bb_max ;

  kvs::PointObject* m_bb_obj ;

  GLdouble m_modelview [16]; 
  GLdouble m_projection[16]; 
  GLint    m_viewport  [ 4]; 

  float    m_bb_center_depth ;


};

//-----
inline 
void CameraInfo::set ( kvs::Camera*  camera, 
                       double min_x , 
                       double min_y , 
                       double min_z , 
                       double max_x , 
                       double max_y , 
                       double max_z             )
{
  // min and max coordinates of the bounding box
  m_bb_min.set ( (float)min_x, (float)min_y, (float)min_z ); 
  m_bb_max.set ( (float)max_x, (float)max_y, (float)max_z ); 

  // Create a dummy object
  m_bb_obj = new kvs::PointObject();
  m_bb_obj->setMinMaxExternalCoords( m_bb_min, m_bb_max );
  m_bb_obj->updateNormalizeParameters();

  // Calc OpenGL matrices
  /*::GetModelviewMatrix ( camera, m_bb_obj, &(m_modelview) );
  ::GetProjectionMatrix( camera, &(m_projection) );
  ::GetViewport( camera, &(m_viewport) );*/
  kvs::detail::GetModelviewMatrix ( camera, m_bb_obj, &(m_modelview) );
  kvs::detail::GetProjectionMatrix( camera, &(m_projection) );
  kvs::detail::GetViewport( camera, &(m_viewport) );


  // Calc distance betwen the eye and the object center: 
  //  It is normalized to have value between 0 and 1.
  m_bb_center_depth  = 
   kvs::detail::CalculateObjectDepth(m_bb_obj,
    //   kvs::CellByCellParticleGenerator::CalculateObjectDepth(m_bb_obj,
                                                          m_modelview, 
                                                          m_projection,
                                                          m_viewport   );

}// set()



/***************
//-----
inline 
const float CameraInfo::pointDepth( const kvs::Vector3d& P )
{
    // calculate suitable depth.
    GLdouble x, y, z;

    ::Project( P.x(), P.y(), P.z(),
               m_modelview, m_projection, m_viewport,
               &x, &y, &z );

    // Calc depth
    const float point_depth = static_cast<float>( z );

    return( point_depth );
}


//-----
inline 
double CameraInfo::crossSectionSpreadRatio ( const kvs::Vector3d& P )
{
  float point_depth = pointDepth ( P );

  return ( m_bb_center_depth / point_depth ) ;

} 
****************/


#endif

/*****
inline const float CalculateObjectDepth(
    const kvs::ObjectBase* object,
    const GLdouble         modelview[16],
    const GLdouble         projection[16],
    const GLint            viewport[4] )
{
    // calculate suitable depth.
    GLdouble x, y, z;

    ::Project(
        object->objectCenter().x(),
        object->objectCenter().y(),
        object->objectCenter().z(),
        modelview, projection, viewport,
        &x, &y, &z );

    const float object_depth = static_cast<float>( z );

    return( object_depth );
}

*****/
