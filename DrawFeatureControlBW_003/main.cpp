#include <kvs/glut/Application> 
#include <kvs/glut/Screen> 
#include <kvs/PointObject> 
#include <kvs/PointRenderer>
#include <kvs/glut/Slider> 
#include <kvs/ObjectManager> 

#include <iostream>
#include <fstream> 
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm> 
#include <iterator>

#include "ImportPointClouds.h"
#include "event_control.h" 

class FeatureSlider : public kvs::glut::Slider 
{
  std::vector<float> m_ft;
  double m_pSize;
  //  kvs::PointObject* m_object; 
public:
  FeatureSlider( kvs::glut::Screen* screen ):
    kvs::glut::Slider( screen )
  {  }
  
  void setFeatureValue( const std::vector<float> &f ) {
    copy( f.begin(), f.end(), back_inserter(m_ft) );
  }
  void setPoitSize( double size ) {
    m_pSize = size;
  }  

  void valueChanged( void ) {
    kvs::glut::Screen* glut_screen = static_cast<kvs::glut::Screen*>( screen() );
    kvs::PointObject* obj =
      kvs::PointObject::DownCast( glut_screen->scene()->objectManager()->object( "PP") ); 
    size_t num = obj->numberOfVertices();

    //    kvs::PointObject *newpt = new kvs::PointObject();
    double thr = this->value();
    std::cout << "Feature Value : " << thr << std::endl;
    std::vector<unsigned char> cl;
    std::vector<float> sp;

    for( size_t i=0; i<num; i++ ) {
      if( m_ft[i] >= thr ) {
	cl.push_back( 0 );
	cl.push_back( 0 );
	cl.push_back( 0 );
	sp.push_back( m_pSize  );
      } else {
	cl.push_back( 255 );
	cl.push_back( 255 );
	cl.push_back( 255 );
	sp.push_back( m_pSize  );
      } 
    }
    obj->setColors( kvs::ValueArray<kvs::UInt8>( cl ) ); 
    obj->setSizes( kvs::ValueArray<float>( sp ));
    screen()->redraw();
  }

};


int main(int argc, char** argv ) 
{
  int pointSize = 3;
  if ( argc < 2 ) {
    std::cout << "Usage: " << argv[0] << "  datafile" << std::endl;
    exit(1);
  }
  if( argc == 3 ) {
    pointSize = atoi( argv[2] );
  }  

  ImportPointClouds *ply = new ImportPointClouds( argv[1] ) ;
  ply->updateMinMaxCoords(); 
  std::cout << "PLY Mim, Max " << std::endl;
  std::cout << ply->minObjectCoord() << std::endl;
  std::cout << ply->maxObjectCoord() << std::endl;

  std::vector<float> ft = ply->featureData( );
  std::cout << "SIZE: " << ft.size() << std::endl;
  
  //--- Convert PolygonObject to PointObject
  kvs::PointObject* object = new kvs::PointObject( *ply );

  object->setSize(pointSize); 
  object->updateMinMaxCoords();
  object->setName("PP"); 
  double ft_max = *max_element( ft.begin(), ft.end() );

  kvs::glut::Application app( argc, argv );
  
  kvs::glut::Screen screen( &app ); 
  kvs::PointRenderer* renderer = new kvs::PointRenderer();
  //renderer->disableShading();
  renderer->enableTwoSideLighting();
  //  renderer->enableAntiAliasing();
  screen.setTitle( "Point Object" );
  screen.setBackgroundColor( kvs::RGBColor( 255, 255, 255) );
  screen.registerObject( object, renderer);

  screen.show();   

  InitializeEvent  init; 
  KeyPressEvent    key;
  screen.addEvent( &init ); 
  screen.addEvent( &key );

  FeatureSlider slider( &screen );
  slider.setMargin( 10 ); 
  slider.setCaption("Feature"); 
  //  slider.setPointObject(object);
  slider.setPoitSize( pointSize );
  slider.setFeatureValue( ft );
  slider.setRange( 0.0, ft_max);  
  slider.show(); 
 
  return app.run();

}
