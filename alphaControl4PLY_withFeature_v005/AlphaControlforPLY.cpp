#include <algorithm>
#include <cmath>

#include <kvs/CellByCellParticleGenerator>
#include <kvs/MersenneTwister>

#include "AlphaControlforPLY.h"
#include "CameraInfo.h"
#include "ply.h"
#include "octree.h"

const double RADIUS = 50.0;                       // Range of Search as (triangle of bounding box)/RADIUS
const int SEARCH_NUM = 100;                       // Number of Points for Counting Sphere
const int NEAR_POINT = 10;                        // Number of Minimun of Nearest Point into a Counting Sphere
const kvs::Vector3ui DEFAUT_COLOR(255, 255, 255); // If a data dosen't have color
const int MIN_NODE = 15;                          // Minimun Number of Learf node for octree

AlphaControlforPLY::AlphaControlforPLY(void) : kvs::PointObject(),
                                               m_searchRadius(0.0),
                                               m_ratio(1.0),
                                               m_pixel_width(0.0)
{
}

AlphaControlforPLY::AlphaControlforPLY(kvs::PolygonObject *plyObject,
                                       kvs::Camera *camera,
                                       kvs::Vector3f BBMin,
                                       kvs::Vector3f BBMax,
                                       int repeatLevel,
                                       double alpha,
                                       std::vector<float> &ft,
                                       double threshold,
                                       bool hasFace) : kvs::PointObject(),
                                                       m_searchRadius(0.0),
                                                       m_ratio(1.0),
                                                       m_pixel_width(0.0)
{
  // pixel width
  calculatePixelWidht(camera, BBMin, BBMax);

  if (hasFace)
  {
    std::cerr << "PLY data has polygons" << std::endl;
    kvs::PointObject *object = new kvs::PointObject();
    Ply ply(plyObject, DEFAUT_COLOR);
    ply.SetViewBoundingBox(kvs::Vector3d(BBMin[0], BBMin[1], BBMin[2]),
                           kvs::Vector3d(BBMax[0], BBMax[1], BBMax[2]));
    ply.ConvertToParticles(alpha, m_pixel_width, repeatLevel, object);

    SuperClass::setCoords(kvs::ValueArray<kvs::Real32>(object->coords()));
    SuperClass::setNormals(kvs::ValueArray<kvs::Real32>(object->normals()));
    SuperClass::setColors(kvs::ValueArray<kvs::UInt8>(object->colors()));
  }
  else
  {
    std::cerr << "PLY data dosen't have polygons" << std::endl;
    int num = calculateRequiredPartcleNumber(alpha, repeatLevel,
                                             BBMin, BBMax);

    size_t numVert = plyObject->numberOfVertices();
    std::vector<int> ind;

    size_t notFeaturePointNum = 0;
    for( size_t i = 0; i < numVert; i++ ) {
      if( ft[i] <= threshold ) {
        ind.push_back( i );
        notFeaturePointNum++;
      }
    }
    calculatePointRaio(num, plyObject);
    setParticles(plyObject, ind, notFeaturePointNum);
  }
}

//-----------------------------------------------------------------------
//  Calculating the pixel width for SPBR
//-----------------------------------------------------------------------
void AlphaControlforPLY::calculatePixelWidht(kvs::Camera *camera,
                                             kvs::Vector3f BBMin,
                                             kvs::Vector3f BBMax)
{
  // Setting Bounding Box
  double xwidth = fabs(BBMax.x() - BBMin.x());
  double ywidth = fabs(BBMax.y() - BBMin.y());
  double zwidth = fabs(BBMax.z() - BBMin.z());

  double xmin = -0.5 * xwidth;
  double ymin = -0.5 * ywidth;
  double zmin = -0.5 * zwidth;
  double xmax = 0.5 * xwidth;
  double ymax = 0.5 * ywidth;
  double zmax = 0.5 * zwidth;

  CameraInfo cameraInfo;
  cameraInfo.set(camera,
                 xmin, ymin, zmin,
                 xmax, ymax, zmax);

  int subpixel_level = 1;

  // Pixel Width
  m_pixel_width =
      kvs::detail::CalculateSubpixelLength(subpixel_level,
                                           cameraInfo.bbCenterDepth(),
                                           cameraInfo.modelviewMatrix(),
                                           cameraInfo.projectionMatrix(),
                                           cameraInfo.viewportMatrix());

  std::cout << "L: " << m_pixel_width << std::endl;
}

//-----------------------------------------------------------------------
//
//  Creating Particles ( in case of Non-connection )
//
//-----------------------------------------------------------------------
double AlphaControlforPLY::calculateRequiredPartcleNumber(double alpha,
                                                          double repeatLevel,
                                                          kvs::Vector3f BBMin,
                                                          kvs::Vector3f BBMax)
{
  // Check Parameter for alpha
  if (alpha > 0.99)
    alpha = 0.99;
  else if (alpha < 0.0)
    alpha = 0.0;

  double pixel_width2 = m_pixel_width * m_pixel_width;
  double diag = (BBMax - BBMin).length();
  m_searchRadius = diag / RADIUS;
  double area = m_searchRadius * m_searchRadius * M_PI;
  double num = log(1 - alpha) / log(1 - (pixel_width2 / area)) * repeatLevel;

  return num;
}

void AlphaControlforPLY::calculatePointRaio(const double analyticalNum,
                                            kvs::PolygonObject *ply)
{
  ply->updateMinMaxCoords();
  kvs::ValueArray<kvs::Real32> coords = ply->coords();
  float *pdata = coords.data();
  size_t numVert = ply->numberOfVertices();
  kvs::Vector3f minBB = ply->minObjectCoord();
  kvs::Vector3f maxBB = ply->maxObjectCoord();

  double *mrange = new double[6];
  mrange[0] = (double)minBB.x();
  mrange[1] = (double)maxBB.x();
  mrange[2] = (double)minBB.y();
  mrange[3] = (double)maxBB.y();
  mrange[4] = (double)minBB.z();
  mrange[5] = (double)maxBB.z();

  // create octree
  std::cout << "Creating Octree... (Number of Vertex : " << numVert << std::endl;
  std::cout << minBB << " \n"
            << maxBB << std::endl;
  octree *myTree = new octree(pdata, numVert, mrange, MIN_NODE);
  int num = 0;
  int execSearchNum = 0;
  double averageNearDist = 0.0;
  std::vector<float> nearDistList;

  kvs::MersenneTwister uniRand;

  std::cout << "Start OCtree Search..... " << std::endl;
  for (int i = 0; i < SEARCH_NUM; i++)
  {
    size_t index = (size_t)((double)numVert * uniRand());
    if (index == numVert)
      --index;
    double point[3] = {coords[3 * index],
                       coords[3 * index + 1],
                       coords[3 * index + 2]};

    vector<size_t> nearInd;
    vector<double> dist;
    search_points(point, m_searchRadius, pdata, myTree->octreeRoot, &nearInd, &dist);
    int n0 = (int)nearInd.size();

    if (n0 < NEAR_POINT)
      continue;

    std::sort(dist.begin(), dist.end());

    double nearDist = 0.0;
    int nCountNear = 0;

    for (int j = 0; j < NEAR_POINT; j++)
    {
      if (dist[j] > 0.0)
      {
        nearDist += dist[j];
        nCountNear++;
        nearDistList.push_back(dist[j]);
      }
    }
    nearDist /= (double)nCountNear;

    averageNearDist += nearDist;
    num += n0;
    execSearchNum++;
  }
  std::sort(nearDistList.begin(), nearDistList.end());
  averageNearDist = nearDistList[(int)(nearDistList.size() * 0.5)]; // intermediate value
  // Avarage for TRY_NUM calucrate sphere
  //  averageNearDist /= (double)execSearchNum; // d_real <-- mesurement
  double averageNum = (double)num / (double)execSearchNum;

  // Analytical Number of caluculating from real nearest distans (d_real)
  double anaNumfromDreal = (m_searchRadius * m_searchRadius * M_PI) / (averageNearDist * averageNearDist);

  double offset = averageNum / anaNumfromDreal;

  std::cout << "==========================================" << std::endl;
  std::cout << "Ana: " << analyticalNum << ", execNum: " << averageNum << std::endl;
  std::cout << "Dist; " << averageNearDist << ", offset: " << offset << std::endl;
  ;
  std::cout << "anaNumfromDreal: " << anaNumfromDreal << std::endl;
  std::cout << "Ratio_Ana: " << (double)(analyticalNum / averageNum) << std::endl;
  std::cout << "==========================================" << std::endl;

  if (offset > 1.0)
    offset = 1.0;

  m_coeff4Ratio = ( anaNumfromDreal * offset );
  m_ratio = analyticalNum / (anaNumfromDreal * offset);

  std::cout << "Ratio for Alpha Control : " << m_ratio
            << ", Diff : " << averageNum - analyticalNum << std::endl;
}

void AlphaControlforPLY::setParticles(kvs::PolygonObject *ply, std::vector<int> &ind, size_t notFeaturePointNum)
{
  size_t numVert = notFeaturePointNum;
  size_t createNum = (unsigned int)numVert * m_ratio;
  size_t multiNum = (unsigned int)m_ratio;
  size_t oddNum = createNum - multiNum * numVert;
  std::cout << "Number of Non-Feature Points : " << numVert << std::endl;
  std::cout << "Number of setting Particles  : " << createNum << "( " << multiNum * numVert << " + " << oddNum << " )" << std::endl;
  kvs::ValueArray<kvs::Real32> coords = ply->coords();
  kvs::ValueArray<kvs::Real32> normals = ply->normals();
  kvs::ValueArray<kvs::UInt8> colors = ply->colors();

  std::vector<kvs::Real32> SetCoords;
  std::vector<kvs::Real32> SetNormals;
  std::vector<kvs::UInt8> SetColors;
  kvs::Vector3f nom(0.0, 0.0, 0.0);
  kvs::Vector3ui col(DEFAUT_COLOR);

  //---- with Shuffle
  kvs::MersenneTwister uniRand;
  std::vector<unsigned int> counter;
  for (size_t i = 0; i < createNum; i++)
  {

    size_t id = (size_t)((double)notFeaturePointNum*uniRand());

    if(id == notFeaturePointNum)
      id--;

    size_t index = ind[ id ];

    counter.push_back(index);
    SetCoords.push_back(coords[3 * index]);
    SetCoords.push_back(coords[3 * index + 1]);
    SetCoords.push_back(coords[3 * index + 2]);
    if (ply->numberOfNormals() == numVert)
    {
      SetNormals.push_back(normals[3 * index]);
      SetNormals.push_back(normals[3 * index + 1]);
      SetNormals.push_back(normals[3 * index + 2]);
    }
    else
    {
      SetNormals.push_back(nom.x());
      SetNormals.push_back(nom.y());
      SetNormals.push_back(nom.z());
    }
    if (ply->numberOfColors() == numVert)
    {
      SetColors.push_back(colors[3 * index]);
      SetColors.push_back(colors[3 * index + 1]);
      SetColors.push_back(colors[3 * index + 2]);
    }
    else
    {
      SetColors.push_back(col.x());
      SetColors.push_back(col.y());
      SetColors.push_back(col.z());
    }
  }

  /*  std::sort( counter.begin(), counter.end() );
  unsigned int j = 0;
  for( unsigned int i=0; i<numVert; i++ ) {
    for(;;) {
      //std::cout << "j " << j << " Cout " << counter[j] << std::endl;
      if( counter[j] == counter[j+1] ) j++;
      else break;
    }
    if( counter[j] != i ) {
      std::cout <<"^^^^^^ j: "<< j << " Count: " << counter[j] << ", i " << i << std::endl;
      exit(0);
    }
    j++;
  }*/
  SuperClass::setCoords(kvs::ValueArray<kvs::Real32>(SetCoords));
  SuperClass::setNormals(kvs::ValueArray<kvs::Real32>(SetNormals));
  SuperClass::setColors(kvs::ValueArray<kvs::UInt8>(SetColors));
}

double AlphaControlforPLY::pointRatio( double anaNum ) {
  double ratio =  anaNum/m_coeff4Ratio;
  return ratio;
}
