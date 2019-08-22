#ifndef _writePBRfile_H__
#define _writePBRfile_H__

#include <kvs/PointObject>
#include <iostream>
#include <fstream>
#include <cstring>
#include "fileList.h" 
#include "spcomment.h"

const int  DISPLAY_INTERVAL = 10000000;

void writePBRfile( fileList *files, 
		   int repeatLevel,
		   int imageResolution,
		   std::string outptFile,
		   kvs::PointObject* object )
{
  unsigned int num_particles = object->numberOfVertices();
  std::vector<std::string> pbrCommand = files->pbrCommand();
  //--- File Open
  std::ofstream fout;
  fout.open( outptFile.c_str() );
  if( !fout ) {
    std::cerr << "ERROR: Cannot Open File " << outptFile << std::endl;
    exit(1);
  }
  
  //--- Header
  fout << SPBR_BINARY_DATA_COMMAND << std::endl;
  if( !(files->isSetRepeatLevel()) ) 
    fout << REPEAT_LEVEL_COMMAND << " " << repeatLevel << std::endl;
  if( !(files->isImageResolution()) )
    fout << IMAGE_RESOLUTION_COMMAND << " " << imageResolution << std::endl; 
  if( !(files->isBackGroundColor()) )
    fout << BG_COLOR_BYTE_COMMAND << " 0 0 0" << std::endl;
  fout << SHADING_COMMAND << " 0" << std::endl;
  fout << PARTICLE_ZOOM_COMMAND << " 0" << std::endl;
  fout << NUM_PARTICLES_COMMAND << " " << num_particles << std::endl;
  for( int i=0; i<pbrCommand.size(); i++ ) //--- Output from LST-file 
    fout << pbrCommand[i] << std::endl;
  fout << BG_COLOR_BYTE_COMMAND << " 0 0 0" << std::endl;

  fout << END_HEADER_COMMAND << std::endl;

  //--- Data
  //  Get point data
  kvs::ValueArray<kvs::Real32> coords = object->coords();
  kvs::ValueArray<kvs::UInt8>  colors = object->colors();
  kvs::ValueArray<kvs::Real32> normals = object->normals();
  //---- Data
  kvs::Real32 tmp;
  //  kvs::UInt8 col[3] ={ 0, 255, 0 };
  kvs::UInt8 col;
  for( unsigned int i=0; i<num_particles; i++ ) {
    //---- Coords
    tmp = coords[3*i];
    fout.write( (char*)&tmp, sizeof(kvs::Real32) );
    tmp = coords[3*i+1];
    fout.write( (char*)&tmp, sizeof(kvs::Real32) );
    tmp = coords[3*i+2];
    fout.write( (char*)&tmp, sizeof(kvs::Real32) );
    //---- Normals
    tmp = normals[3*i];
    fout.write( (char*)&tmp, sizeof(kvs::Real32) );
    tmp = normals[3*i+1];
    fout.write( (char*)&tmp, sizeof(kvs::Real32) );
    tmp = normals[3*i+2];
    fout.write( (char*)&tmp, sizeof(kvs::Real32) );
    //---- Colors
    //fout.write( (char*)&col, sizeof(kvs::UInt8)*3 );
    col = colors[3*i];
    //col = 255;
    fout.write( (char*)&col, sizeof(kvs::UInt8) );
    col = colors[3*i+1];
    fout.write( (char*)&col, sizeof(kvs::UInt8) );
    col = colors[3*i+2];
    fout.write( (char*)&col, sizeof(kvs::UInt8) );
    
    // message
    if( !( (i+1) % DISPLAY_INTERVAL ) ) {
      std::cerr << (i+1) << "\t" << "particles (" ;
      std::cerr << ( 100.0 * (double)i / (double)num_particles ) ;
      std::cerr << ") \%" << std::endl;
    }      
  }
  
  fout.close();
  
  std::cout << "** File " << outptFile << "  is generated." << std::endl;
  
}


#endif
