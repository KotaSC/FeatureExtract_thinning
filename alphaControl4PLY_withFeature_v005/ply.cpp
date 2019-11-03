///////////////////
///// ply.cpp /////
///////////////////

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iterator>
#define _USE_MATH_DEFINES
#include <cmath>
#include <kvs/RotationMatrix33>
#include <kvs/MersenneTwister>
#include <kvs/Timer>
#include <set>
#include <deque>
#include <list>
#include "ply.h"
#include "shuffle.h"

//----- DEBUG control
//#define DEBUG
//-----
Ply::Ply(void):
  m_nvertex(0), 
  m_nface(0), 
  m_S_area(0.0), 
  m_alpha(1.0), 
  m_haveNormal(false), 
  m_haveColor(false),
  m_centerCoord(0, 0, 0), 
  m_maxCoord(-100000, -1000000, -1000000), 
  m_minCoord(1000000,1000000,1000000), 
  m_vbbMax (1.0, 1.0, 1.0 ), 
  m_vbbMin (-1.0, -1.0, -1.0) 
{}

Ply::Ply(kvs::PolygonObject *ply, 
	 kvs::Vector3ui deaful_color ):
  m_nvertex(0), 
  m_nface(0), 
  m_S_area(0.0), 
  m_alpha(1.0), 
  m_haveNormal(false), 
  m_haveColor(false),
  m_centerCoord(0, 0, 0), 
  m_maxCoord(-100000, -1000000, -1000000), 
  m_minCoord(1000000,1000000,1000000), 
  m_vbbMax (1.0, 1.0, 1.0 ), 
  m_vbbMin (-1.0, -1.0, -1.0) 
{
  
  ReadBody( ply, deaful_color );

}

bool Ply::ReadBody(kvs::PolygonObject *ply, 
		   kvs::Vector3ui deaful_color )
{
  m_nvertex = static_cast<int>( ply->numberOfVertices() );
  m_nface   = static_cast<int>( ply->numberOfConnections() );
  if( static_cast<int>( ply->numberOfNormals() ) == m_nvertex )
    m_haveNormal = true; 
  if( static_cast<int>( ply->numberOfColors() ) == m_nvertex )
    m_haveColor  = true;
  
  kvs::ValueArray<kvs::Real32> vert = ply->coords();  
  kvs::ValueArray<kvs::Real32> norm = ply->normals();
  kvs::ValueArray<kvs::UInt8>  col  = ply->colors();  
  m_centerCoord.set(0.0, 0.0, 0.0);  
  kvs::Vector3f default_nom( 0.0, 0.0, 0.0 );
  for(int i=0; i<m_nvertex; i++) { 
    Vertex v;   
    v.vertex_x = vert[ 3*i ];  
    v.vertex_y = vert[ 3*i+1 ];
    v.vertex_z = vert[ 3*i+2 ];
    if( m_haveNormal ) {
      v.normal_x = norm[ 3*i ];  
      v.normal_y = norm[ 3*i+1 ];
      v.normal_z = norm[ 3*i+2 ];
    } else {
      v.normal_x = default_nom.x();
      v.normal_y = default_nom.y();
      v.normal_z = default_nom.z();
    }
    if( m_haveColor ) {
      v.r = col[ 3*i   ];
      v.g = col[ 3*i+1 ];
      v.b = col[ 3*i+2 ];      
    } else {
      v.r        = deaful_color.x();  
      v.g        = deaful_color.y();  
      v.b        = deaful_color.z();  
    }

    v.faceNum = 0;                                              
    m_centerCoord.set( m_centerCoord.x() + v.vertex_x, 
		       m_centerCoord.y() + v.vertex_y, 
		       m_centerCoord.z() + v.vertex_z );

    if(m_minCoord.x() > v.vertex_x) m_minCoord.set(v.vertex_x, m_minCoord.y(), m_minCoord.z());
    if(m_minCoord.y() > v.vertex_y) m_minCoord.set(m_minCoord.x(), v.vertex_y, m_minCoord.z());
    if(m_minCoord.z() > v.vertex_z) m_minCoord.set(m_minCoord.x(), m_minCoord.y(), v.vertex_z);
                                                                                                               
    if(m_maxCoord.x() < v.vertex_x) m_maxCoord.set(v.vertex_x, m_maxCoord.y(), m_maxCoord.z());
    if(m_maxCoord.y() < v.vertex_y) m_maxCoord.set(m_maxCoord.x(), v.vertex_y, m_maxCoord.z());
    if(m_maxCoord.z() < v.vertex_z) m_maxCoord.set(m_maxCoord.x(), m_maxCoord.y(), v.vertex_z);
    
    m_verts.push_back(v); 
  }
  m_centerCoord = m_centerCoord / m_nvertex;
  
  // Face
  kvs::ValueArray<kvs::UInt32>  cnect  = ply-> connections();  
  for( int i=0; i<m_nface; i++ ) {  
    face f; 
    f.index1 = cnect[ 3*i ];  
    f.index2 = cnect[ 3*i+1 ];
    f.index3 = cnect[ 3*i+2 ];
    f.center_x = (m_verts[f.index1].vertex_x + m_verts[f.index2].vertex_x + m_verts[f.index3].vertex_x) /3;
    f.center_y = (m_verts[f.index1].vertex_y + m_verts[f.index2].vertex_y + m_verts[f.index3].vertex_y) /3;
    f.center_z = (m_verts[f.index1].vertex_z + m_verts[f.index2].vertex_z + m_verts[f.index3].vertex_z) /3;

    kvs::Vector3d vec_0_1(m_verts[f.index2].vertex_x - m_verts[f.index1].vertex_x, 
			  m_verts[f.index2].vertex_y - m_verts[f.index1].vertex_y, 
			  m_verts[f.index2].vertex_z - m_verts[f.index1].vertex_z);
    kvs::Vector3d vec_0_2(m_verts[f.index3].vertex_x - m_verts[f.index1].vertex_x, 
			  m_verts[f.index3].vertex_y - m_verts[f.index1].vertex_y, 
			  m_verts[f.index3].vertex_z - m_verts[f.index1].vertex_z);
    kvs::Vector3d vec_0_3(m_verts[f.index3].vertex_x - m_verts[f.index2].vertex_x, 
			  m_verts[f.index3].vertex_y - m_verts[f.index2].vertex_y, 
			  m_verts[f.index3].vertex_z - m_verts[f.index2].vertex_z);

    double edge0_1 = vec_0_1.length();
    double edge0_2 = vec_0_2.length();
    double edge0_3 = vec_0_3.length();
    double s_0 = (edge0_1 + edge0_2 + edge0_3)/2.0; 
    f.area = sqrt(fabs(s_0*(s_0-edge0_1)*(s_0-edge0_2)*(s_0-edge0_3)));
    m_S_area += f.area;

    //---- Calculation of normal
    kvs::Vector3d nol_0 = vec_0_1.cross(vec_0_2);
    nol_0.normalize();                           
    f.normal_x = nol_0.x();                      
    f.normal_y = nol_0.y();                      
    f.normal_z = nol_0.z();                      
    m_verts[f.index1].normal_x += nol_0.x();     
    m_verts[f.index1].normal_y += nol_0.y();     
    m_verts[f.index1].normal_z += nol_0.z();     
    m_verts[f.index1].faceNum++;                 
                                             
    m_verts[f.index2].normal_x += nol_0.x();     
    m_verts[f.index2].normal_y += nol_0.y();     
    m_verts[f.index2].normal_z += nol_0.z();     
    m_verts[f.index2].faceNum++;                 
                                             
    m_verts[f.index3].normal_x += nol_0.x();     
    m_verts[f.index3].normal_y += nol_0.y();     
    m_verts[f.index3].normal_z += nol_0.z();     
    m_verts[f.index3].faceNum++;
                 
    m_faces.push_back(f);
  }
  return true; 
  
}

//////////////////////////////
/////  Polygon sampling  /////
//////////////////////////////
//---------------------------------------------
// For each polygon in the given polygon mesh, 
// generate uniformly distributed  points 
// with distance "d = sqrt(S/n_sum), where
//   S     = total polygon area,
//   n_sum = total number of particles.
// The n_sum is defined by using 
// the user-defined opacity "alpha" and 
// repeat level "L_R":
//  n_sum = log(1-alpha)/log(1-(s/S))*L_R, 
// where 
//  s = particle_width * particle_width
//    = pixel_3D_width * pixel_3D_width  
// is the particle cross section. 
//---------------------------------------------
////////////////////////////////
//          p2
//            *
//           * *
//   vec21  *   *   vec32 
//  (1->2) *     *  (2->3)
//        *       * 
//       * * * * * * 
//     p1   vec31   p3
//          (1->3)
////////////////////////////////
//---------------------------------------------------------
// Loop structure:
//
//  for ( each polygon i) {
//    //***********************************//
//    // PART 1: Create particles on edges //
//    //***********************************//
//    for (each particle j on edge 21 ) { SAMPLING }
//    for (each particle j on edge 31 ) { SAMPLING }
//    for (each particle j on edge 32 ) { SAMPLING }
//
//    //**********************************************//
//    // PART 2: Create particles inside each polygon //
//    //**********************************************//
//    for (each scanline k ) {
//      for (each particle j ) { SAMPLING }
//  }
//---------------------------------------------------------
//-----
void Ply::ConvertToParticles( double alpha      , 
                              double pixel_width , 
                              int    repeat_level, 
			      // int    resolution , 
                              kvs::PointObject* particles )
{	
  // Constants 
  double epsilon     = 1.0e-3 ; // to avoid overlapping sampling points
  double epsilon_cos = 1.0e-4 ; // to treat triangles with zero area

  // Values used in the formula of SPBR
  //..... Particle cross section "s"
  double s_particleCrossSection = pixel_width * pixel_width;

  //..... n_sum: total number of particles (= n * L_R) 
  double n_sum = log(1-alpha)/log(1-(s_particleCrossSection/m_S_area))*repeat_level;

  //.....  inter-particle distance "d" and "2d"
  double d_particleDistance = sqrt(m_S_area/n_sum)    ; // d
  double d2              = 2.0 * d_particleDistance ; // 2*d 

  // Message
  /*  std::cout << "Number of vertices:"  << m_nvertex   << std::endl;
  std::cout << "Number of faces:   "  << m_nface     << std::endl;
  std::cout << "pixel width:"     << pixel_width << std::endl;
  std::cout << "S(area):"         << m_S_area    << std::endl;
  std::cout << "point_dist:"      << d_particleDistance << std::endl;
  std::cout << "n_sum (theoretical):"   << n_sum << std::endl;*/

  // Message 
  kvs::Timer timer1;
  timer1.start();

  // list of vertices in the given polygon mesh
  std::deque<Vertex> point_list ;
  
  // copy: vector<Vertex> m_verts ==> deque<Vertex> point_list
  for( std::vector<Vertex>::iterator i = m_verts.begin(); 
       i != m_verts.end()                               ; 
       i++                                                )
  {
    point_list.push_back((*i));
  } 
  

  ////////////////////////////////////
  /////  Polygon-sampling loop  /////
  ///////////////////////////////////
  //---------------------------------------------
  // For each polygon in the given polygon mesh, 
  //  generate uniformly-distributed particles 
  //  with distance "d_particleDistance".
  //---------------------------------------------------------
  //... i is the polygon index
  //
  for( std::vector<face>::iterator i = m_faces.begin(); 
       i != m_faces.end(); 
       i++) 
  {
    // position, normal vector, and color of Vertex 1 of the i-th polygon
    kvs::Vector3d p1Vertex( m_verts[(*i).index1].vertex_x, m_verts[(*i).index1].vertex_y, m_verts[(*i).index1].vertex_z );
    kvs::Vector3d p1Normal( m_verts[(*i).index1].normal_x, m_verts[(*i).index1].normal_y, m_verts[(*i).index1].normal_z );
    kvs::Vector3d p1Color ( m_verts[(*i).index1].r, m_verts[(*i).index1].g, m_verts[(*i).index1].b );

    // position, normal vector, and color of Vertex 2 of the i-th polygon
    kvs::Vector3d p2Vertex( m_verts[(*i).index2].vertex_x, m_verts[(*i).index2].vertex_y, m_verts[(*i).index2].vertex_z );
    kvs::Vector3d p2Normal( m_verts[(*i).index2].normal_x, m_verts[(*i).index2].normal_y, m_verts[(*i).index2].normal_z );
    kvs::Vector3d p2Color ( m_verts[(*i).index2].r, m_verts[(*i).index2].g, m_verts[(*i).index2].b );

    // position, normal vector, and color of Vertex 3 of the i-th polygon
    kvs::Vector3d p3Vertex( m_verts[(*i).index3].vertex_x, m_verts[(*i).index3].vertex_y, m_verts[(*i).index3].vertex_z );
    kvs::Vector3d p3Normal( m_verts[(*i).index3].normal_x, m_verts[(*i).index3].normal_y, m_verts[(*i).index3].normal_z );
    kvs::Vector3d p3Color ( m_verts[(*i).index3].r, m_verts[(*i).index3].g, m_verts[(*i).index3].b );

    //***********************************//
    // PART 1: Create particles on edges //
    //***********************************//

    //---------------------------------------//
    // Part 1-1  Create particles on edge 21 //
    //---------------------------------------//

    // edge vector of edge 21 and its length
    const kvs::Vector3d vec21( p2Vertex - p1Vertex ); // vector 1-->2
    const double        vec21_length = vec21.length();

    // normalized edge vector for edge 21
    kvs::Vector3d vec21_hat = vec21;  vec21_hat.normalize(); 

    // number of generated particles on edge 21
    int edge21PointNum = (int)( vec21_length / d2 ); 

    // Avoid a particle just on vertex 2. 
    if( fabs( vec21_length - d2*edge21PointNum ) < epsilon*d2 ) {//TANAKA
      edge21PointNum--; 
    }

    // Create particles on edge 21 with distance 2*d 
    //  p1+1*(2d)*vec21_hat, p1+ 2*(2d)*vec21_hat, p1+3*(2d)*vec21_hat, ... 
    //.... j is the particle index on edge 21 
    for( int j = 1; j <= edge21PointNum; j++ ) {//TANAKA
      // the j-th particle position
      Vertex v;
      v.vertex_x = (vec21_hat.x()*d2*j) + m_verts[(*i).index1].vertex_x;
      v.vertex_y = (vec21_hat.y()*d2*j) + m_verts[(*i).index1].vertex_y;
      v.vertex_z = (vec21_hat.z()*d2*j) + m_verts[(*i).index1].vertex_z;

      //----- Phong shading -----//
      // normal-vector interpolation on edge 21 (1 ---> s ---> 2)
      kvs::Vector3d psVertex(v.vertex_x, v.vertex_y, v.vertex_z);//current pos
      kvs::Vector3d vs2( psVertex - p2Vertex );
      kvs::Vector3d v1s( p1Vertex - psVertex );

      kvs::Vector3d normal_s = 
        (1/vec21_length)*( vs2.length()*p1Normal + v1s.length()*p2Normal );

      v.normal_x = normal_s.x();
      v.normal_y = normal_s.y();
      v.normal_z = normal_s.z();

      // color for Phong shading. 
      //.... Use the color read from input PLY file.
      kvs::Vector3d color_s = 
        (1/vec21_length)*( vs2.length()*p1Color + v1s.length()*p2Color );

      v.r = color_s.x();
      v.g = color_s.y();
      v.b = color_s.z();
      //----- end of Phong shading -----//

      // Store the generated particle to the list
      point_list.push_back(v);

    }// for (j): particle generation on edge 21 

    //---------------------------------------//
    // Part 1-2  Create particles on edge 31 //
    //---------------------------------------//

    // edge vectors of edge 31 and its length
    const kvs::Vector3d vec31( p3Vertex - p1Vertex ); // vector 1-->3
    const double        vec31_length = vec31.length();

    // normalized edge vector for edge 31
    kvs::Vector3d vec31_hat = vec31;  vec31_hat.normalize(); 

    // number of generated particles on edge 31
    int edge31PointNum = (int)( vec31_length / d2 );

    // Avoid a particle just on vertex 3. 
    if( fabs( vec31_length - d2*edge31PointNum ) < epsilon*d2 ) {//TANAKA
      edge31PointNum--;
    }

    // Create particles on edge 31 with distance 2*d 
    //  p1+1*(2d)*vec31_hat, p1+ 2*(2d)*vec31_hat, p1+3*(2d)*vec31_hat, ... 
    //.... j is the particle index on edge 31 
    for( int j = 1; j <= edge31PointNum; j++ ) {//TANAKA
      // the j-th particle position
      Vertex v;
      v.vertex_x = (vec31_hat.x()*d2*j) + m_verts[(*i).index1].vertex_x;
      v.vertex_y = (vec31_hat.y()*d2*j) + m_verts[(*i).index1].vertex_y;
      v.vertex_z = (vec31_hat.z()*d2*j) + m_verts[(*i).index1].vertex_z;

      //----- Phong shading -----//
      // normal-vector interpolation on edge 31 (1 ---> s ---> 3)
      kvs::Vector3d psVertex(v.vertex_x, v.vertex_y, v.vertex_z);//current pos
      kvs::Vector3d vs3( psVertex - p3Vertex );
      kvs::Vector3d v1s( p1Vertex - psVertex );

      kvs::Vector3d normal_s = 
        (1/vec31_length)*( vs3.length()*p1Normal + v1s.length()*p3Normal );

      v.normal_x = normal_s.x();
      v.normal_y = normal_s.y();
      v.normal_z = normal_s.z();

      // color for Phong shading. 
      //.... Use the color read from input PLY file.
      kvs::Vector3d color_s = 
        (1/vec31_length)*( vs3.length()*p1Color + v1s.length()*p3Color );

      v.r = color_s.x();
      v.g = color_s.y();
      v.b = color_s.z();
      //----- end of Phong shading -----//

      // Store the generated particle to the list
      point_list.push_back(v);

    } // for (j): particle generation on edge 31 

    //---------------------------------------//
    // Part 1-3  Create particles on edge 32 //
    //---------------------------------------//

    // edge vectors of edge 32 and its length
    const kvs::Vector3d vec32( p3Vertex - p2Vertex ); // vector 2-->3
    const double        vec32_length = vec32.length();

    // normalized edge vector for edge 32
    kvs::Vector3d vec32_hat = vec32;  vec32_hat.normalize(); 

    // number of generated particles on edge 32
    int edge32PointNum = (int)( vec32_length / d2 );

    // Avoid a particle just on vertex 3. 
    if( fabs( vec32_length - d2*edge32PointNum ) < epsilon*d2 ) {//TANAKA
      edge32PointNum--;
    }

    // Create particles on edge 32 with distance 2*d 
    //  p2+1*(2d)*vec32_hat, p2+ 2*(2d)*vec32_hat, p2+3*(2d)*vec32_hat, ... 
    //.... j is the particle index on edge 32 
    for( int j = 1; j <= edge32PointNum; j++ ) {//TANAKA
      // the j-th particle position
      Vertex v;
      v.vertex_x = (vec32_hat.x()*d2*j) + m_verts[(*i).index2].vertex_x;
      v.vertex_y = (vec32_hat.y()*d2*j) + m_verts[(*i).index2].vertex_y;
      v.vertex_z = (vec32_hat.z()*d2*j) + m_verts[(*i).index2].vertex_z;

      //----- Phong shading -----//
      // normal-vector interpolation on edge 32 (2 ---> s ---> 3)
      kvs::Vector3d psVertex(v.vertex_x, v.vertex_y, v.vertex_z);//current pos
      kvs::Vector3d vs3( psVertex - p3Vertex );
      kvs::Vector3d v2s( p2Vertex - psVertex );

      kvs::Vector3d normal_s = 
        (1/vec32_length)*( vs3.length()*p2Normal + v2s.length()*p3Normal );

      v.normal_x = normal_s.x();
      v.normal_y = normal_s.y();
      v.normal_z = normal_s.z();

      // color for Phong shading. 
      //.... Use the color read from input PLY file.
      kvs::Vector3d color_s = 
        (1/vec32_length)*( vs3.length()*p2Color + v2s.length()*p3Color );

      v.r = color_s.x();
      v.g = color_s.y();
      v.b = color_s.z();
      //----- end of Phong shading -----//

      // Store the generated particle to the list
      point_list.push_back(v);

    }// for (j): particle generation on edge 32 

    //**********************************************//
    // PART 2: Create particles inside each polygon //
    //**********************************************//

    // particle distance along edge 31
    double d_31 = 0.0; // initialization 

    // number of scanlines
    int num_scanlines = 0 ; // initialized to "skip sampling"

    // Starting position of the first scanline (k=1).
    kvs::Vector3d p_cross_31 ;

    // polygon normal
    kvs::Vector3d polygon_normal((*i).normal_x, (*i).normal_y, (*i).normal_z);
    polygon_normal.normalize();

    // moving direction of scan line (perpendicular to edge 21)
    kvs::Vector3d vec_m = vec21.cross( polygon_normal );  vec_m.normalize();

    // Check direction of the scanline movement:
    //  The movement should be in the direction to inside of 
    //  the polygon.
    if( vec31_hat.dot( vec_m ) < 0 ) { vec_m *= (-1.0); }

    // Calc the number of scanlines
    const double cos_factor = fabs( vec31_hat.dot( vec_m  ) );

    if( cos_factor > epsilon_cos ) { 
      // calc particle distance along edge 31
      d_31 = d_particleDistance / cos_factor ;

      // Calc the number of scanlines
      num_scanlines = vec31_length / d_31 ;

      // Avoid a scanline passing through vertex 3.
      if( fabs( vec31_length - d_31 * num_scanlines ) < epsilon*d2 ) {//TANAKA
        num_scanlines--;
      }

      // Starting position of the first scanline (k=1).
      p_cross_31 = p1Vertex + d_31 * vec31_hat ;

    } 

    // Generate sampling points on each scanline.
    //.... k is the scanline index.  
    for( int k = 1; k<= num_scanlines; k++ ) { // TANAKA

      // beginning point of the current scanline
      kvs::Vector3d p_scanline_begin = (p_cross_31 - p1Vertex)*k + p1Vertex;

      // scanline length //TANAKA
      //.... Use the formula of the homologous triangle  
      double scanline_length = ( 1.0 - (d_31/vec31_length)*k ) * vec21_length;

      // ending point of the current scanline // TANAKA
      kvs::Vector3d p_scanline_end = p_scanline_begin + vec21_hat * scanline_length;

      // Calc the number of sampling points on the current scanline.
      int particleNumOnScanline = (int)(scanline_length / d_particleDistance);

      // Avoid a sampling point just on edge 32
      if( fabs( scanline_length - d_particleDistance * particleNumOnScanline ) < epsilon*d_particleDistance ) 
	{//TANAKA
	  particleNumOnScanline--;
	}
      
      // Generate sampling points on the k-the scanline
      // .... j is a particle index on the current scanline.
      for( int j=1; j<= particleNumOnScanline ; j++ ) {//TANAKA 
	
	// current sampling position 
        kvs::Vector3d p = p_scanline_begin + vec21_hat*d_particleDistance*j;

        // Set the current sampling position to a Vertex object
        Vertex v;
        v.vertex_x = p.x();
        v.vertex_y = p.y();
        v.vertex_z = p.z();


        //----- Phong shading -----//TANAKA

        // interpolation of normal vectors on edge 31
        double vec_b3_length = ( p_scanline_begin - p3Vertex ).length();
        double vec_1b_length = ( p1Vertex - p_scanline_begin ).length();

        kvs::Vector3d normal_begin = 
          (1/vec31_length)*(vec_b3_length*p1Normal + vec_1b_length*p3Normal);

        // interpolation of normal vectors on edge 32
        double vec_e3_length = ( p_scanline_end - p3Vertex ).length();
        double vec_2e_length = ( p2Vertex - p_scanline_end ).length();

        kvs::Vector3d normal_end = 
          (1/vec32_length)*(vec_e3_length*p2Normal + vec_2e_length*p3Normal);

        // interpolation of normal vectors on the current scanline
        double vec_pb_length = ( p - p_scanline_begin ).length();
        double vec_ep_length = ( p_scanline_end - p   ).length();

        kvs::Vector3d normal_p = 
          (1/scanline_length)*(vec_ep_length*normal_begin + vec_pb_length*normal_end);

        // set the interpolated normal vector to the vertex object
        v.normal_x = normal_p.x();
        v.normal_y = normal_p.y();
        v.normal_z = normal_p.z(); // TANAKA (BUG in Shimokubo's code)

	// interpolation of color 
        kvs::Vector3d color_begin = 
          (1/vec31_length)*(vec_b3_length*p1Color + vec_1b_length*p3Color);
        kvs::Vector3d color_end = 
          (1/vec32_length)*(vec_e3_length*p2Color + vec_2e_length*p3Color);
        kvs::Vector3d color_p = 
          (1/scanline_length)*(vec_ep_length*color_begin + vec_pb_length*color_end);

        // set the interpolated color to the vertex object
        v.r = color_p.x();
        v.g = color_p.y();
        v.b = color_p.z(); 

        //----- end of Phong shading -----//

        point_list.push_back(v);

      } // for(j) (for each particle)

    } // for (k) (for each scanline)

  } // for (i) (for each polygon)

  //////////////////////////////////////////
  /////  End of polygon-sampling loop  /////
  //////////////////////////////////////////



  ////////////////////////
  /////  Final step  /////
  ////////////////////////
  //-------------------------------------------------------
  // Below, we create ValuedArray objects 
  //  for the created particles.
  //  Then we set them back to the argument, "pointObject.
  // We also clean up the temporary "point_list" object.
  // Optionally we write the result to a PBR-format file, 
  // calling the "WritePBRFile ()" method
  //-------------------------------------------------------

  // Message 
  timer1.stop();
  std::cout << "Num of created particles: "<< point_list.size() <<std::endl;
  std::cout << "Particle-creation time (sec): " << timer1.sec() <<std::endl;

  // Message 
  std::cout << "** Start particle shuffling" << std::endl;
  kvs::Timer timer2;
  timer2.start();

  // Particle shuffling 
  RandomGenerator rnd;
  std::random_shuffle( point_list.begin(), point_list.end(), rnd );

  // Message 
  timer2.stop();
  std::cout << "** Shuffling completed" << std::endl;
  std::cout << "Shuffling time (sec): " << timer2.sec()<< std::endl;

  // memory allocation for the ValuedArray objects.
  kvs::ValueArray<kvs::Real32> pointCoords ;
  kvs::ValueArray<kvs::Real32> pointNormals;
  kvs::ValueArray<kvs::UInt8>  pointColors ;
  pointCoords.allocate  ( point_list.size()*3 );
  pointNormals.allocate ( point_list.size()*3 );
  pointColors.allocate  ( point_list.size()*3 );

  // Message 
  //  std::cout<<"Number of created particles: "<< point_list.size() <<std::endl;

  // Set the created particles to the ValuedArray objects
  int j=0;
  for( std::deque<Vertex>::iterator i = point_list.begin(); 
       i != point_list.end(); 
       i++                                                 ) 
  {
    // x-coord 
    pointCoords[j] = (*i).vertex_x;
    pointNormals[j] = (*i).normal_x;
    pointColors[j] = (*i).r;

    j++;

    // y-coord 
    pointCoords[j] = (*i).vertex_y;
    pointNormals[j] = (*i).normal_y;
    pointColors[j] = (*i).g;

    j++;

    // z-coord 
    pointCoords[j] = (*i).vertex_z;
    pointNormals[j] = (*i).normal_z;
    pointColors[j] = (*i).b;

    j++;

  } // for (i)

  // Set the created particles 
  //  back to the argument, "particles", 
  //  which is a pointObject type. 
  particles->setCoords(pointCoords);
  particles->setNormals(pointNormals);
  particles->setColors(pointColors);
  particles->updateMinMaxCoords();

  // Clear temporary particle list
  point_list.clear();

}// ConvertToParticles() 

// end of ply.cpp
