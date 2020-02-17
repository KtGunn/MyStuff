/* author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../src/render/render.h"
#include "../src/render/box.h"

// Works in API
//#include "../../render/render.h"
//#include "../../render/box.h"

#include <chrono>
#include <string>
#include "kdtree.h"

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
pcl::visualization::PCLVisualizer::Ptr initScene (Box window, int zoom)
{
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));

  viewer->setBackgroundColor (0.5, 0.5, 0.5); // grey backdrop
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  viewer->addCoordinateSystem (1.0);
  
  // black background for data points
  viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 0, 0, 0, "window");
  //viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 1, 1, 1, "window");
  return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData (std::vector<std::vector<float>> points)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  
  for (int i = 0; i < points.size(); i++) {
    pcl::PointXYZ point;
    point.x = points[i][0];
    point.y = points[i][1];
    point.z = 0;
    
    cloud->points.push_back(point);
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;
  
  return cloud;
}


void render2DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{
  
  if (node != NULL) {
    Box upperWindow = window;
    Box lowerWindow = window;

    // split on x axis
    if (depth%2==0) {
      viewer->addLine(pcl::PointXYZ (node->point[0], window.y_min, 0),pcl::PointXYZ(node->point[0], window.y_max, 0),0,0,1,
		      "line"+std::to_string(iteration));
      lowerWindow.x_max = node->point[0];
      upperWindow.x_min = node->point[0];
    }
    // split on y axis
    else {
      viewer->addLine(pcl::PointXYZ(window.x_min, node->point[1], 0),pcl::PointXYZ(window.x_max, node->point[1], 0),1,0,0,
		      "line"+std::to_string(iteration));
      lowerWindow.y_max = node->point[1];
      upperWindow.y_min = node->point[1];
    }
    iteration++;
    
    render2DTree(node->left,viewer, lowerWindow, iteration, depth+1);
    render2DTree(node->right,viewer, upperWindow, iteration, depth+1);
  }
  
}

//**************************************************************************************************
/// CLUSTERING
//
void proximity (const int id, const std::vector<std::vector<float>>& Points, std::vector<int>& cluster,
		std::map<int,bool>& mapProcd, KdTree* tree, float dTol) {
    
    // Mark the point by its id, as processed
    mapProcd.insert (std::pair<int,bool>(id, true));

    // Push it into the cluster
    cluster.push_back (id);

    // Search for nearby points
    std::vector<float> testPoint = Points[id];
    std::vector<int> nearbyIds = tree->search (testPoint, dTol);

    for ( int nearIndex : nearbyIds ) {
	if ( mapProcd.count (nearIndex) == 0 ) {
	    proximity (nearIndex, Points, cluster, mapProcd, tree, dTol);
	}
    }
    return;
}

std::vector<std::vector<int>> euclideanCluster (const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
  
  // TODO: Fill out this function to return list of indices for each cluster
  
  std::vector<std::vector<int>> clusters;
  // 'map' to mark points as processed
  std::map<int,bool> mapProcessed;

  // Cycle through all points
  for (uint n=0; n<points.size(); n++) {

      // Is this point processed?
      if ( mapProcessed.count (n) == 0 ) {
	  // No ; create a new cluster
	  std::vector<int> newCluster;

	  // Look for adjacent points
	  proximity (n, points, newCluster, mapProcessed, tree, distanceTol);
	  
	  // Add the cluster to the list
	  clusters.push_back ( newCluster );
      }
  }

  return clusters;
}
//**************************************************************************************************

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/// MAIN
//
int main ()
{
  
  // Create viewer
  Box window;
  window.x_min = -10;
  window.x_max =  10;
  window.y_min = -10;
  window.y_max =  10;
  window.z_min =   0;
  window.z_max =   0;
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25);
  
  // Create data
  std::vector<std::vector<float>> points;

  // Start w/1 point
  //std::vector<std::vector<float>> points = { {-6.2,7} };
  points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3}, {7.2,6.1}, {8.0,5.3},
	     {7.2,7.1}, {0.2,-7.1}, {1.7,-6.9}, {-1.2,-7.2}, {2.2,-8.9} };
  // Class room case
  //std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3} };

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);
  
  KdTree* tree = new KdTree;
  
  for (int i=0; i<points.size(); i++) {
    tree->insert (points[i],i);
  }
  
  int it = 0;
  render2DTree(tree->root,viewer,window, it);
  
  if (false) {

      // MY Own test
      points = { {-2,4}, {-1,5}, {5,1}, {6,2} };

      std::vector<int> nearby;

      std::cout << "Test Search B" << std::endl;
      nearby = tree->search ({5.5, 1.5},3.0);
      for(int index : nearby)
	  std::cout << index << ",";
      std::cout << std::endl;

      std::cout << "Test Search A" << std::endl;
      nearby = tree->search ({-1.5, 4.5},3.0);
      for(int index : nearby)
        std::cout << index << ",";
      std::cout << std::endl;

  } else {

      std::cout << "Test Search" << std::endl;
      std::vector<int> nearby = tree->search({-6,7},3.0);
      for(int index : nearby)
	  std::cout << index << ",";
      std::cout << std::endl;
  
  }

  //*************************************************************************
  // CLUSTERING Algorithm (timed)
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
  float dTol = 3.0;
  std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, dTol);
  auto endTime = std::chrono::steady_clock::now();

  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;
  //*************************************************************************
  

  //*************************************************************************
  // CLUSTER RENDERING
  // Render clusters
  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

  for (std::vector<int> cluster : clusters) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (int indice: cluster)
      clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],0));
      renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
      ++clusterId;
  }
  
  // Show pointcloud if no clusters are found
  if (clusters.size()==0) {
    renderPointCloud(viewer, cloud, "data");
  }
  
  while (!viewer->wasStopped ()) {
    viewer->spinOnce ();
  }

  
  return (0);  
}
