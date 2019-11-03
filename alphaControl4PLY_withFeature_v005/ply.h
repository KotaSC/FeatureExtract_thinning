/////////////////
///// ply.h /////
/////////////////

#ifndef __PLY__
#define __PLY__

#include <fstream>
#include <vector>
#include "vertex.h"
#include "face.h"
#include <kvs/Vector3>
#include <kvs/PointObject>
#include <kvs/PolygonObject>
#include <kvs/ValueArray>
#include <cstdlib>
#include <ctime>

#include "ply_customize.h"

//-----------//
class Ply {
//-----------//
private:
  // surface geometry data
  std::vector<Vertex> m_verts; //vertex positions
  std::vector<face>   m_faces; //facets
  int m_nvertex ; //number of vertices
  int m_nface   ; // number of facets written in PLY file.
                  //  Note: The number of polygons may change 
                  //        through the polygon decomposition.
  double m_S_area;  // surface area

  // surface properties
  double m_alpha; //alpha (opasity) value
  bool   m_haveNormal;//Does input PLY file have normal-vector info?
  bool   m_haveColor ;//Does input PLY file have color info?

  // max-min coordinates of input polygon mesh
  kvs::Vector3d m_centerCoord;
  kvs::Vector3d m_maxCoord;
  kvs::Vector3d m_minCoord;

  // "View baounding box" to calculate camera parameter.
  kvs::Vector3d m_vbbMax;
  kvs::Vector3d m_vbbMin;

  // Rendering parameters
  //  double m_zoomFactor ;
  //  bool   m_flagShading ;

  // Other paramters
  /*  unsigned int m_bgGrayColor ;
  bool   m_flagFPS     ;
  bool   m_flagLOD     ;
  bool   m_flagShuffle ;*/

  bool ReadBody( kvs::PolygonObject *ply, 
		 kvs::Vector3ui default_color);

 public:
  // Constructors and destructor
  Ply(void);
  Ply( kvs::PolygonObject *ply, 
       kvs::Vector3ui default_color);

  ~Ply(){ m_verts.clear(); m_faces.clear(); }

  // Access methods
  double Alpha  ( void ) { return m_alpha  ; }
  double Nvertex( void ) { return m_nvertex; }
  double Nface  ( void ) { return m_nface  ; }
  double Area   ( void ) { return m_S_area ; }
  void   SetViewBoundingBox ( const kvs::Vector3d& min, 
                              const kvs::Vector3d& max ) 
         { m_vbbMin = min;  m_vbbMax = max; } 

  // Does the input PLY file have color info?
  bool HaveColorFromFile(void){ return m_haveColor;};

  // Generate Point data
  void ConvertToParticles( double  alpha       , 
                           double  pixel_width  , 
                           int     repeat_level , 
                           // int     resolution  , 
                           kvs::PointObject* particles );
	
  // Center and  Bounding box of the polygon object
  kvs::Vector3d ObjectCenter(void){ return m_centerCoord; } 
  kvs::Vector3d MaxCoord    (void){ return m_maxCoord   ; }	
  kvs::Vector3d MinCoord    (void){ return m_minCoord   ; }

};

#endif
// end of ply.h
