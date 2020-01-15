#ifndef _writeFeature_H__
#define _writeFeature_H__

#include <kvs/PolygonObject>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <kvs/ColorMap>

enum WritingDataType {
    Ascii = 0,
    Binary = 1
};

const float NORMAL[3] ={ 0.0, 0.0, 0.0 };
const int COLOR[3] = {0, 255, 255};

void writeFeature( kvs::PolygonObject *ply,
	      std::vector<float> &ft,
	      char* filename,
	      WritingDataType type = Ascii )
{
  size_t num = ply->numberOfVertices();
  bool hasNormal = false, hasColor = false;;
  if( num == ply->numberOfNormals() ) hasNormal = true;
  if( num == ply->numberOfColors() ) hasColor = true;
  kvs::ValueArray<kvs::Real32> coords = ply->coords();
  kvs::ValueArray<kvs::Real32> normals = ply->normals();
  kvs::ValueArray<kvs::UInt8> colors = ply->colors();

  float ftMax = *std::max_element(ft.begin(), ft.end());
  kvs::ColorMap cmap( 256.0, 0.0, ftMax );
  cmap.create();

  std::ofstream fout( filename);
  if( type == Binary ) {
    fout << "#/XYZ_BinaryData" << std::endl;
    fout << "#/NumParticles  " << num << std::endl;
    fout << "#/XYZDataType  XYZNormalColorFeature" << std::endl;
    fout << "#/EndHeader" << std::endl;
  }

  for(int i=0; i<num; i++ ) {
    float x = coords[3*i];
    float y = coords[3*i+1];
    float z = coords[3*i+2];
    float nx = NORMAL[0];
    float ny = NORMAL[1];
    float nz = NORMAL[2];
    int r = COLOR[0];
    int g = COLOR[1];
    int b = COLOR[2];
    if( hasNormal ) {
      nx = normals[ 3*i ];
      ny = normals[ 3*i+1 ];
      nz = normals[ 3*i+2];
    }
    if( hasColor ) {

      kvs::RGBColor color( cmap.at( ft[i] ) );

      r = (int)color.r();
      g = (int)color.g();
      b = (int)color.b();

      // Color Edge
      // if(ft[i] > 0.3){
      //   r = (int)255;
      //   g = (int)0;
      //   b = (int)0;
      // } else {
      //   r = (int)255;
      //   g = (int)255;
      //   b = (int)255;
      // }
    }
    if( type == Binary ) {
      fout.write( (char*)&x, sizeof(float) );
      fout.write( (char*)&y, sizeof(float) );
      fout.write( (char*)&z, sizeof(float) );
      fout.write( (char*)&nx, sizeof(float) );
      fout.write( (char*)&ny, sizeof(float) );
      fout.write( (char*)&nz, sizeof(float) );
      unsigned char cl = (unsigned char)r;
      fout.write( (char*)&cl, sizeof(unsigned char) );
      cl = (unsigned char)g;
      fout.write( (char*)&cl, sizeof(unsigned char) );
      cl = (unsigned char)b;
      fout.write( (char*)&cl, sizeof(unsigned char) );
      float tmp = ft[i];
      fout.write( (char*)&tmp, sizeof(float) );

    }
    else {
      fout << x << " " << y << " " << z << " "
	   << nx << " " << ny << " " << nz << " "
	   << r << " " << g << " " << b  << " "
	   << ft[i] << std::endl;
    }
  }

  fout.close();
}


#endif
