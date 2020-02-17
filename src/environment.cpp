/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"

// KD-tree implementation
#include "kd3Dtree.h"


// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  
  Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
  Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
  Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
  Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);
  
  if(renderScene)
    {
      renderHighway(viewer);
      egoCar.render(viewer);
      car1.render(viewer);
      car2.render(viewer);
      car3.render(viewer);
    }
  
  return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display simple highway -----
  // ----------------------------------------------------
  
  // RENDER OPTIONS
  bool renderScene = false; // 'false'==highway & cars not shown
  std::vector<Car> cars = initHighway(renderScene, viewer);
  
  // DONE -- TODO:: Create lidar sensor 
  Lidar* lid = new Lidar (cars, 0.0);
  
  // DONE -- TODO:: Create point processor
  ProcessPointClouds<pcl::PointXYZ>* myPcProcessor = new ProcessPointClouds<pcl::PointXYZ>;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lid->scan ();
  
  if (false) {
    renderRays (viewer, lid->position, inputCloud);
    renderPointCloud (viewer, inputCloud, "InCloud");
  }
  
  // 200209: 40iter/0.15 leaves a few points on the road; obs are OK
  //         100iter/0.2 is perfect
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = 
    myPcProcessor->SegmentPlane (inputCloud,40, 0.15);

  if (false) {
    renderPointCloud (viewer, segmentCloud.first, "First", Color (1,0,0));
    renderPointCloud (viewer, segmentCloud.second, "Obies", Color (0,0,1));
  }
  
  // Now calling for clustering the obstacles cloud
  std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> vObClouds = \
    myPcProcessor->Clustering (segmentCloud.second, 2.0, 10, 200);

  int index = 0;
  std::vector<std::string> vNames ({"1st", "2nd", "3rd", "4th", "5th", "Sixth" });
  std::vector<Color> vCols({Color(1,0,0), Color(0,1,0), Color(0,0,1), Color(0.5,0.5,0.5), Color(0.75,0.25,0.5)});

  for (pcl::PointCloud<pcl::PointXYZ>::Ptr pObCloud : vObClouds) {

    // render the cloud, eachone in a different colour
    renderPointCloud (viewer, pObCloud , vNames[index], vCols[index]);
    index++;

    Box bb = myPcProcessor->BoundingBox (pObCloud);
    renderBox (viewer, bb, index);
    
    if (index >= vCols.size() || index >= vNames.size()) {
      std::cout << "Ran out of names or colors. Add more...\n";
      break;
    }
  }
  
  return;
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  
  viewer->setBackgroundColor (0, 0, 0);
  
  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;
  
  switch(setAngle)
    {
    case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
    case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
    case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
    case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }
  
  if(setAngle!=FPS)
    viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
  std::cout << "starting enviroment" << std::endl;
  
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);
  simpleHighway(viewer);
  
  while (!viewer->wasStopped ())
    {
      viewer->spinOnce ();
    } 
}
