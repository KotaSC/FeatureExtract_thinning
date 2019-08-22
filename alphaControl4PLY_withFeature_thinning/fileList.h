#ifndef _fileList_HH_
#define _fileList_HH_

#include <cstring>
#include <vector>
#include <kvs/Vector3>

const int MAX_FILENAME = 1024;

class fileList{
 private:
  std::vector<std::string> m_ply_files;
  std::vector<std::string> m_pbr_files;

  std::vector<float> m_alpha;
  std::vector<float> m_alpha_ft;
  std::vector<float> m_thresholds;

  std::string m_input_path;
  std::string m_output_path;
  std::vector<std::string> m_pbr_command;	/* PBR Command */

  int m_numberOfFiles;

  bool m_isSetRepeatLevel;
  int m_repeatLevel;
  bool m_isImageResolution;
  int m_imageResolution;
  bool m_isBoundingBox;
  kvs::Vector3f m_BB_min;
  kvs::Vector3f m_BB_max;
  bool m_isLookAt;
  kvs::Vector3f m_lookAt;
  bool m_isCameraPosition;
  kvs::Vector3f m_cameraPosition;
  bool m_isViewAngle;
  float m_viewAngle;
  bool m_isCameraZoom;
  float m_cameraZoom;
  bool m_isBackGroundColor;
  bool m_hasFeature;

 public:
  fileList( void );
  fileList( char* file );
  ~fileList( ) {  }
 private:
  int breakWord( char* buf, std::string *parse );
  void fileRead( char* file );
 public:
  bool isSetRepeatLevel( void )  { return m_isSetRepeatLevel; }
  bool isImageResolution( void ) { return m_isImageResolution; }
  bool isBoundingBox( void )     { return m_isBoundingBox; }
  bool isLookAt( void )          { return m_isLookAt; }
  bool isCameraPosition( void )  { return m_isCameraPosition; }
  bool isViewAngle( void )       { return m_isViewAngle; }
  bool isCameraZoom( void )      { return m_isCameraZoom; }  
  bool isBackGroundColor( void ) { return m_isBackGroundColor; }  
  bool isFeature( void )         { return m_hasFeature; }
  
  int repeatLevel() { return m_repeatLevel; }
  int imageResolution() { return m_imageResolution; }
  kvs::Vector3f boundingBoxMin() { return m_BB_min; }
  kvs::Vector3f boundingBoxMax() { return m_BB_max; }
  kvs::Vector3f lookAt() { return m_lookAt; }
  kvs::Vector3f cameraPosition() { return m_cameraPosition; }
  float viewAngle() { return m_viewAngle; }
  float cameraZoom() {return m_cameraZoom; }

  std::vector<std::string> plyFiles() { return m_ply_files; }
  std::vector<std::string> pbrFiles() { return m_pbr_files; }
  std::vector<float> alphas()         { return m_alpha; }
  std::vector<float> alphas_ft()      { return m_alpha_ft; }
  std::vector<float> thresholds()     { return m_thresholds;}
  std::vector<std::string> pbrCommand() { return m_pbr_command; }
  int numberOfFiles() { return m_numberOfFiles; }

};

#endif
