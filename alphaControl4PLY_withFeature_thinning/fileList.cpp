#include <iostream>
#include <fstream>
#include <cstring>
#include <cstdlib>

#include "fileList.h"
#include "spcomment.h"
#include "getExtension.h"

fileList::fileList(void) : m_isSetRepeatLevel(false),
                           m_isImageResolution(false),
                           m_isBoundingBox(false),
                           m_isLookAt(false),
                           m_isCameraPosition(false),
                           m_isViewAngle(false),
                           m_isCameraZoom(false),
                           m_isBackGroundColor(false),
                           m_hasFeature(false)
{
  m_numberOfFiles = 0;
}

fileList::fileList(char *file) : m_isSetRepeatLevel(false),
                                 m_isImageResolution(false),
                                 m_isBoundingBox(false),
                                 m_isLookAt(false),
                                 m_isCameraPosition(false),
                                 m_isViewAngle(false),
                                 m_isCameraZoom(false),
                                 m_isBackGroundColor(false),
                                 m_hasFeature(false)
{
  m_numberOfFiles = 0;

  if (getExtension(file) == "lst")
    fileRead(file);
}

int fileList::breakWord(char *buf, std::string *str)
{
  char *data;
  int n = 0;
  data = strtok(buf, " \t");
  while (data != NULL)
  {
    str[n] = data;
    data = strtok(NULL, " \t");
    n++;
  }
  return n;
}

void fileList::fileRead(char *file)
{

  std::ifstream reader;
  reader.open(file);
  if (!reader)
  {
    std::cerr << "ERROR : Cannot Open " << file << std::endl;
    exit(1);
  }

  //-- Read data
  const int BUF_MAX = 1024;
  char buf[BUF_MAX];
  std::string word[15];

  while (reader.getline(buf, BUF_MAX - 1, '\n'))
  {
    //---- Header
    std::string cmd(buf);
    if (!strlen(buf))
    {
      // Blank Line
      continue;
    }
    //--- Command
    else if (buf[0] == '#')
    {
      if (buf[1] == '/')
      {
        breakWord(buf, word);
        //---- Input File path
        if (!strncmp(word[0].c_str(), "#/DATA_IMPORT_PATH", 18))
        {
          m_input_path = word[1];
          if (m_input_path[m_input_path.size() - 1] != '/')
          {
            m_input_path += '/';
          }
        }
        else if (!strncmp(word[0].c_str(), "#/DATA_EXPORT_PATH", 18))
        {
          m_output_path = word[1];
          if (m_output_path[m_output_path.size() - 1] != '/')
          {
            m_output_path += '/';
          }
          else if (!strncmp(word[0].c_str(), "#/HAS_FEATURE", 13))
          {
            if (atoi(word[1].c_str()))
              m_hasFeature = true;
            std::cout << m_hasFeature << std::endl;
          }
        }
        else if (!strncmp(word[0].c_str(), REPEAT_LEVEL_COMMAND, strlen(REPEAT_LEVEL_COMMAND)))
        {
          m_pbr_command.push_back(cmd);
          m_isSetRepeatLevel = true;
          m_repeatLevel = atoi(word[1].c_str());
        }
        else if (!strncmp(word[0].c_str(), IMAGE_RESOLUTION_COMMAND, strlen(IMAGE_RESOLUTION_COMMAND)))
        {
          m_pbr_command.push_back(cmd);
          m_isImageResolution = true;
          m_imageResolution = atoi(word[1].c_str());
        }
        else if (!strncmp(word[0].c_str(), BOUNDING_BOX_COMMAND, strlen(BOUNDING_BOX_COMMAND)))
        {
          m_pbr_command.push_back(cmd);
          m_isBoundingBox = true;
          m_BB_min.set(atof(word[1].c_str()), atof(word[2].c_str()), atof(word[3].c_str()));
          m_BB_max.set(atof(word[4].c_str()), atof(word[5].c_str()), atof(word[6].c_str()));
        }
        else if (!strncmp(word[0].c_str(), LOOK_AT_COMMAND, strlen(LOOK_AT_COMMAND)))
        {
          m_pbr_command.push_back(cmd);
          m_isLookAt = true;
          m_lookAt.set(atof(word[1].c_str()), atof(word[2].c_str()), atof(word[3].c_str()));
        }
        else if (!strncmp(word[0].c_str(), CAMERA_POSITION_COMMAND, strlen(CAMERA_POSITION_COMMAND)))
        {
          m_pbr_command.push_back(cmd);
          m_isCameraPosition = true;
          m_cameraPosition.set(atof(word[1].c_str()), atof(word[2].c_str()), atof(word[3].c_str()));
        }
        else if (!strncmp(word[0].c_str(), VIEW_ANGLE_COMMAND, strlen(VIEW_ANGLE_COMMAND)))
        {
          m_isViewAngle = true;
          m_viewAngle = atof(word[1].c_str());
        }
        else if (!strncmp(word[0].c_str(), CAMERA_ZOOM_COMMAND, strlen(CAMERA_ZOOM_COMMAND)))
        {
          m_pbr_command.push_back(cmd);
          m_isCameraZoom = true;
          m_cameraZoom = atof(word[1].c_str());
        }
        else if (!strncmp(buf, BG_COLOR_BYTE_COMMAND, strlen(BG_COLOR_BYTE_COMMAND)))
        {
          m_isBackGroundColor = true;
        }
        else if (!strncmp(buf, END_HEADER_COMMAND, strlen(END_HEADER_COMMAND)))
        {
          break;
        }
        else
          m_pbr_command.push_back(cmd);
      }
    }
  }

  //---- file list
  int fnum = 0;
  double a0 = 0.0;
  double a1 = 0.001;
  double fthr = 1.0;
  while (reader.getline(buf, BUF_MAX - 1, '\n'))
  {
    //    std::cout << "==== " << buf << std::endl;
    if (!strlen(buf))
    {
      // Blank Line
      continue;
    }
    else if (buf[0] == '#')
    {
      continue;
    }
    else
    {
      int nw = breakWord(buf, word);
      if (nw < 2)
        continue;
      else
      {
        std::string ply = m_input_path + word[0];
        std::string pbr = m_output_path + word[1];
        if (nw >= 3)
        {
          if (!strncmp(word[2].c_str(), "#", 1))
            continue;
          else
          {
            a0 = atof(word[2].c_str());
          }
          if (nw >= 4)
          {
            a1 = atof(word[3].c_str());

            if (nw >= 5)
            {
              fthr = atof(word[4].c_str());
            }
          }
        }
        m_ply_files.push_back(ply);
        m_pbr_files.push_back(pbr);
        m_alpha.push_back(a0);
        m_alpha_ft.push_back( a1 );
        m_thresholds.push_back( fthr );
        fnum++;
      }
    }
  }
  m_numberOfFiles = fnum;
}
