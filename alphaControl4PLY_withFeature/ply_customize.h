///////////////////////////
///// ply_customize.h /////
///////////////////////////

#ifndef __PLY_CUSTOMIZE_H
#define __PLY_CUSTOMIZE_H


//----- BACKGROUND COLOR control
const unsigned int DEFAULT_BG_GRAY_COLOR = 0 ; //black

//----- OBJECT COLOR control
//.....  If "PHONG" macro is active, 
//.....  colors read from an input PLY file.
//.....  If no color is defined, DEFAULT_POINT_COLOR_X's below 
//.....  are used.
const int DEFAULT_POINT_COLOR_R = 255;
const int DEFAULT_POINT_COLOR_G = 255;
const int DEFAULT_POINT_COLOR_B = 255;

//----- RESOLUTION control 
const int DEFAULT_IMAGE_RESOLUTION = 512;

//----- ALPHA control
const double DEFAULT_ALPHA = 0.1;
const double MAX_ALPHA = 0.99 ;

//----- REPEAT-LEVEL control
const int DEFAULT_REPEAT_LEVEL = 200;

//----- CAMERA control
const  double DEFAULT_CAMERA_DISTANCE = 12.0 ;
const  double DEFAULT_ZOOM_FACTOR     = 1.0 ; 
const  double MAX_ZOOM_FACTOR = 3.0 ;
const  double MIN_ZOOM_FACTOR = 0.3 ;

const int PBSR_FPS = 1;

#endif
// end of ply_customize.h
