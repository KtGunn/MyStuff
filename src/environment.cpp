/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"

// KD-tree implementation
#include "kd3Dtree.h"
#include "pca.h"


// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

namespace nspKT {
    // Using a separate name space for my debugging variables
    static bool g_doClustering = false;
    static bool g_doSegmentation = false;
    static bool g_doPCABoxing = false;
}


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

void cityBlock (pcl::visualization::PCLVisualizer::Ptr& viewer )
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display city block     -----
  // ----------------------------------------------------

  ProcessPointClouds<pcl::PointXYZI>* ptProcessor = new ProcessPointClouds<pcl::PointXYZI>;
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = ptProcessor->loadPcd ("../src/sensors/data/pcd/data_1/0000000000.pcd");


  /////////////////////////////////////////////////////////////////////////////////////////////
  /// CLOUD FILTERING
  //
  pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud;
  
  float fwdX  = 30.0;
  float sideY = 8.0;
  Eigen::Vector4f lowEigen (-1*fwdX, -0.8*sideY, -2.0, 1.0);
  Eigen::Vector4f highEigen (fwdX, sideY, 1.0, 1.0);
  
    
  if (false) {
      // This was used to visualize the cropping ROI
      Box roi;
      roi.x_min = -30.0;
      roi.x_max = 30.0;
      
      roi.y_min = -26.0;
      roi.y_max = 26.0;
      
      roi.z_min = -5.0;
      roi.z_max = 15.0;
      renderBox (viewer,roi,0);
  }
  
  if (true) {
      // This was used to visualize the roof top cropping ROI
      Box roi;
      roi.x_min = -2;
      roi.x_max = 2.6;
      
      roi.y_min = -1.7;
      roi.y_max = 1.7;
      
      roi.z_min = -2;
      roi.z_max = 1;
      renderBox (viewer,roi,2, Color (0.124, 0.58, 0.620));
  }
  
  /////////////////////////////////////////////////////////////////////////////////////////////
  /// CLOUD FILTERING  -- size reduction
  //
  float resolution = 0.15;
  filteredCloud = ptProcessor->FilterCloud (inputCloud, resolution,
					    lowEigen, highEigen);
  if (false) {
      // Render the filtered cloud
      renderPointCloud (viewer, filteredCloud, "FilteredCityBlock");
      
  } else if (false) {
      // Render the full point cloud
      renderPointCloud (viewer,inputCloud,"CityBlock");
  }
  

  /////////////////////////////////////////////////////////////////////////////////////////////
  /// CLOUD SEGMENTATION -- separate road from obstacles
  //
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> vRoadObs;
  if ( nspKT::g_doSegmentation ) {
      int maxIterations = 50;
      float distanceThreshold = 0.2;
      vRoadObs = ptProcessor->SegmentPlane ( filteredCloud, maxIterations, distanceThreshold);
      
      if (true) {
	  // Road is light green
	  renderPointCloud (viewer, vRoadObs.first, "First", Color (0,0.5,0));

	  if ( !nspKT::g_doClustering) {
	      // Obstacles are pink
	      renderPointCloud (viewer, vRoadObs.second, "Second", Color (1,0,1));
	  }
      }
  }
  
  //////////////////////////////////////////////////////////////////////////////////////////////
  /// CLUSTERING -- identify individual obstacles
  //
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> vClusters;
  if ( nspKT::g_doClustering) {
      
      float distTol = 0.8;
      int minSize = 10;
      int maxSize = 1500;
      vClusters = ptProcessor->Clustering ( vRoadObs.second, distTol, minSize, maxSize);
      
      // All clusters the same colour (pink) to make them stand out
      std::vector<Color> vCols({Color(1,0,1)});

      int count = 1;
      int vI = 0;
      for (pcl::PointCloud<pcl::PointXYZI>::Ptr pO : vClusters) {
	  // Render the cloud, each in a different colour (maybe)
	  renderPointCloud (viewer, pO , "Ob_"+std::to_string(vI), vCols[vI % vCols.size()]);
	  vI++;

	  //-------------------------------------------------------------------------------
	  /// BOXING
	  //
	  if ( !nspKT::g_doPCABoxing ) {
	      Box bb = ptProcessor->BoundingBox (pO);
	      renderBox (viewer, bb, vI+100);
	  } else {
	      BoxQ bbQ = k_PCA<pcl::PointXYZI> (pO);
	      renderBox (viewer, bbQ, vI+100);
	  }
      }
  }

  return;
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
	renderPointCloud (viewer, inputCloud, "InCloud");
    }
    if (false) {
	renderRays (viewer, lid->position, inputCloud);
    }
    
    // 200209: 40iter/0.15 leaves a few points on the road; obs are OK
    //         100iter/0.2 is perfect
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud;

    //////////////////////////////////////////////////////////////////////////////////////////////
    /// SEGMENTATION -- separate road from obstacles
    //
    if ( nspKT::g_doSegmentation ) {
	int maxIter = 200;
	float threshold = 0.2;
	// std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr>
	segmentCloud = 
	    myPcProcessor->SegmentPlane (inputCloud,maxIter, threshold);
	
	if (true) {
	    renderPointCloud (viewer, segmentCloud.first, "First", Color (0,1,0));
	    renderPointCloud (viewer, segmentCloud.second, "Obies", Color (0,0,1));
	}
    }

    
    //////////////////////////////////////////////////////////////////////////////////////////////
    /// CLUSTERING -- identify individual obstacles
    //
    std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> vObClouds;
    if ( nspKT::g_doClustering) {
	
	float distTol = 0.25;
	int minSize = 10;
	int maxSize = 200;
	
	// Clustering the obstacles cloud
	vObClouds = myPcProcessor->Clustering (segmentCloud.second, distTol, minSize, maxSize);
	
	int index = 0;
	std::vector<std::string> vNames ({"1st", "2nd", "3rd", "4th", "5th", "Sixth" });
	std::vector<Color> vCols ({Color (1,0,0), Color (0,1,0), Color (0,0,1), Color (0.5,0.5,0.5),
			Color (0.75,0.25,0.5)});
	
	for (pcl::PointCloud<pcl::PointXYZ>::Ptr pObCloud : vObClouds) {
	    
	    // render the cloud, eachone in a different colour
	    renderPointCloud (viewer, pObCloud , vNames[index], vCols[index]);
	    index++;
	    
	    
	    //-------------------------------------------------------------------------------
	    /// BOXING
	    //
	    if ( !nspKT::g_doPCABoxing ) {
		Box bb = myPcProcessor->BoundingBox (pObCloud);
		renderBox (viewer, bb, index);
	    } else {
		BoxQ bbQ = k_PCA<pcl::PointXYZ> (pObCloud);
		renderBox (viewer, bbQ, index);
	    }
	    
	    if (index >= vCols.size() || index >= vNames.size()) {
		std::cout << "Ran out of names or colors. Add more...\n";
		break;
	    }
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
    std::cerr << "usage: ./environment [doseg] [docluster] [doCPA]" << std::endl;
    std::cout << "starting enviroment" << std::endl;
    
    nspKT::g_doSegmentation = false;
    nspKT::g_doClustering = false;
    nspKT::g_doPCABoxing = false;
    
    switch (argc)
	{
	case 2:
	    {
		nspKT::g_doSegmentation = std::stoi(argv[1]);
	    }
	    break;
	case 3:
	    {
		nspKT::g_doSegmentation = std::stoi(argv[1]);
		nspKT::g_doClustering = std::stoi(argv[2]);
	    }
	    break;
	case 4:
	    {
		nspKT::g_doSegmentation = std::stoi(argv[1]);
		nspKT::g_doClustering = std::stoi(argv[2]);
		nspKT::g_doPCABoxing = std::stoi(argv[3]);
	    }
	    break;
	}

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    
    bool doCity = true;
    if ( !doCity ) {
	std::cout << " SIMPLE HIGHWAY \n";
	simpleHighway(viewer);
    } else {
	std::cout << " CITY BLOCK \n";
	cityBlock (viewer);
    }
    
    while (!viewer->wasStopped ()) {
	viewer->spinOnce ();
    } 
}
