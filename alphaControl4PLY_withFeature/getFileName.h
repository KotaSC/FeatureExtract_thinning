#ifndef _getFileName_H__
#define _getFileName_H__

#include <iostream>
#include <cstring>
#include <sstream> 

inline std::string getFileName(const std::string &path)
{
  size_t pos1; 
  pos1 = path.rfind('\\');
  if(pos1 != std::string::npos){
    return path.substr(pos1+1, path.size()-pos1-1);
  }
 
  pos1 = path.rfind('/');
  if(pos1 != std::string::npos){
    return path.substr(pos1+1, path.size()-pos1-1);
  }
 
  return path;
}

#endif
