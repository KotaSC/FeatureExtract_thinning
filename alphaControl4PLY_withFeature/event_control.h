////////////////////////////
///// event_control.h  /////
////////////////////////////

#if !defined  SSM__EVENT_CONTROL_INCLUDE
#define       SSM__EVENT_CONTROL_INCLUDE

#include <kvs/Version> //KVS2

#if KVS_VERSION_MAJOR == 2
  #include <kvs/Scene> //KVS2
#endif

#include <kvs/InitializeEventListener>
#include <kvs/KeyPressEventListener>
#include <kvs/Key>
#include <kvs/glut/Screen>
#include <kvs/ColorImage>
#include <kvs/Camera>

#include "single_inputfile.h"

const kvs::Vector3f DEFAULT_LIGHT_POSITION (12.0, 12.0, 12.0) ;

//------------------------------------------------------------//
class InitializeEvent : public kvs::InitializeEventListener {
//------------------------------------------------------------//

  void update( void )
  {
    // default light position
    kvs::Vector3f P0 = DEFAULT_LIGHT_POSITION ;

    // Set the light position 
    //... The default setting is (0, 0, 12) 
#if KVS_VERSION_MAJOR == 1
    screen()->light()->setPosition( P0.x(), P0.y(), P0.z() );//KVS1
#elif KVS_VERSION_MAJOR == 2
    static_cast<kvs::glut::Screen*>(screen())->scene()->light()->setPosition( P0.x(), P0.y(), P0.z() );//KVS2
#endif

  }

};

//------------------------------------------------------------//
class KeyPressEvent : public kvs::KeyPressEventListener
//------------------------------------------------------------//
{
   void update( kvs::KeyEvent* event )
   {
       switch ( event->key() )
       {
       // push l: light control
       // push o: object controls
       // push s: snapshot (BMP)
       // push S: snapshot (PPM)
       // push G: snapshot (PGM)
         case kvs::Key::l: 
           std::cerr << "\n** Light-control mode" << std::endl;

#if KVS_VERSION_MAJOR == 1
           screen()->controlTarget() = kvs::glut::Screen::TargetLight; //KVS1
#elif KVS_VERSION_MAJOR == 2
           static_cast<kvs::glut::Screen*>( screen() )->scene()->controlTarget() = kvs::Scene::TargetLight; //KVS2
#endif

           this->displayMenu();
           break;
         case kvs::Key::o: 
           std::cerr << "\n** Object-control mode" << std::endl;

#if KVS_VERSION_MAJOR == 1
           screen()->controlTarget() = kvs::glut::Screen::TargetObject; //KVS1
#elif KVS_VERSION_MAJOR == 2
	   static_cast<kvs::glut::Screen*>( screen() )->scene()->controlTarget() = kvs::Scene::TargetObject; //KVS2
#endif

           this->displayMenu();
           break;
         case kvs::Key::s: {
           char filename_bmp [256];
           SingleInputFile* p  = SingleInputFile::GetInstance();
           p->GetBMPName ( filename_bmp );

           std::cerr << "\n** Snapshot (BMP)" << std::endl;
           kvs::ColorImage snapshot_image; 

#if KVS_VERSION_MAJOR == 1
	   snapshot_image = screen()->camera()->snapshot(); //KVS1
#elif KVS_VERSION_MAJOR == 2
	   snapshot_image = static_cast<kvs::glut::Screen*>( screen() )->scene()->camera()->snapshot();//KVS2
#endif


	   snapshot_image.write( filename_bmp );
           this->displayMenu();
           break;
	 }
         case kvs::Key::S: {
           char filename_ppm [256];
           SingleInputFile* p  = SingleInputFile::GetInstance();
           p->GetPPMName ( filename_ppm );

           std::cerr << "\n** Snapshot (PPM) " << std::endl;
           kvs::ColorImage snapshot_image; 

#if KVS_VERSION_MAJOR == 1
	   snapshot_image = screen()->camera()->snapshot();//KVS1
#elif KVS_VERSION_MAJOR == 2
	   snapshot_image = static_cast<kvs::glut::Screen*>( screen() )->scene()->camera()->snapshot();//KVS2
#endif


	   snapshot_image.write( filename_ppm );
           this->displayMenu();
           break;
	 }
         case kvs::Key::G: {
           char filename_pgm [256];
           SingleInputFile* p  = SingleInputFile::GetInstance();
           p->GetPGMName ( filename_pgm );

           std::cerr << "\n** Snapshot (PGM) " << std::endl;
           kvs::ColorImage snapshot_image; 

#if KVS_VERSION_MAJOR == 1
	   snapshot_image = screen()->camera()->snapshot(); //KVS1
#elif KVS_VERSION_MAJOR == 2
	   snapshot_image = static_cast<kvs::glut::Screen*>( screen() )->scene()->camera()->snapshot();//KVS2
#endif


	   snapshot_image.write( filename_pgm );
           this->displayMenu();
           break;
	 }
         default: 
           break;
       } // switch


   }

 public:
   void displayMenu (void ) 
   {
     std::cerr << "\nKeyboard menu:" << std::endl;
     std::cerr << "  o-key: object control, l-key: light control" << std::endl;
     std::cerr << "  s-key: snapshot image (BMP)" << std::endl;
     std::cerr << "  S-key: snapshot image (PPM)" << std::endl;
     std::cerr << "  G-key: snapshot image (PGM)" << std::endl;
     std::cerr << "  q-key: quit" << std::endl;
   }

};




#endif

