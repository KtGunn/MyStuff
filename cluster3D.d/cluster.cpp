/* author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../src/render/render.h"
#include "../src/render/box.h"

#include <chrono>
#include <string>
#include "kd3Dtree.h"

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
    point.z = points[i][2];
    
    cloud->points.push_back(point);
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;
  
  return cloud;
}


//**************************************************************************************************
/// CLUSTERING
//
void  Proximity (const int id, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		 std::vector<int>& idsCluster, std::map<int,bool>& mapProcd,
		 KdTree* tree, float dTol)
{
    
    // Mark the point by its id, as processed
    mapProcd.insert (std::pair<int,bool>(id, true));
    
    // Push it into the cluster
    idsCluster.push_back (id);
    
    // Search for nearby points
    pcl::PointXYZ testPoint = cloud->points[id];
    std::vector<int> nearbyIds = tree->search (testPoint, dTol);
    
    for ( int nearIndex : nearbyIds ) {
	if ( mapProcd.count (nearIndex) == 0 ) {
	    Proximity (nearIndex, cloud, idsCluster, mapProcd, tree, dTol);
	}
    }

    return;
}


std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
    Clustering (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
					    float clusterTolerance, int minSize, int maxSize)
{
    
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // Note the clusters are 'vectors-of-clouds'
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    
    /////////////////////////////////////////////////////////////////////////
    // FIRST: create a kd-tree
    KdTree* tree = new KdTree ();

    // Insert the cloud points into the tree
    for (int i=0; i < cloud->size(); i++) {
	pcl::PointXYZ pt = cloud->points[i];
	tree->insert (pt,i);
    }


    /////////////////////////////////////////////////////////////////////////
    // SECOND: Cycle through all points
    
    // 'map' to mark points as processed
    std::map<int,bool> mapProcessed;

    float dTol = clusterTolerance; // Clustering tolerance
    
    std::cout << "Cloud size=" << cloud->size() << std::endl;
    for (uint n=0; n<cloud->size(); n++) {
	
	// Is this point processed?
	if ( mapProcessed.count (n) == 0 ) {

	    // No ; create a cluster of point IDs
	    std::vector<int> idsCluster;
	    
	    pcl::PointXYZ p = cloud->points[n];
	    std::cout << "Point[n]=" << p.x << " " << p.y << " " << p.z << std::endl; 

	    // Look for adjacent points
	    Proximity (n, cloud, idsCluster, mapProcessed, tree, dTol);
	    
	    // Add the cluster to the list
	    std::cout << " Cluster size = " << idsCluster.size() << std::endl;
	    
	    if (idsCluster.size() >= minSize && idsCluster.size() <= maxSize) {
		std::cout << " A \n";
		pcl::PointCloud<pcl::PointXYZ>::Ptr newCluster (new pcl::PointCloud<pcl::PointXYZ>());
		for (int id : idsCluster) {
		    std::cout << " A B id=" << id << std::endl;
		    newCluster->points.push_back (cloud->points[id]);
		}
		std::cout << " A B C \n";
		clusters.push_back ( newCluster );
	    }
	} else {
	    std::cout << "   NOTE #" << n << " is processed!\n";
	}
    }
    
    std::cout << " A B C D\n";
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " <<
	clusters.size() << " clusters" << std::endl;

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
  std::vector<std::vector<float>> vpoints;
  
  // MY Own test
  vpoints = { {-2,4,1}, {-1,5,1}, {5,1,1}, {6,2,1} };
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(vpoints);
  
  renderPointCloud (viewer, cloud, "data");
  
  if (false) {
      KdTree* tree = new KdTree;
      for (int i=0; i<cloud->size(); i++) {
	  tree->insert (cloud->points[i],i);
      }
  } else {

      // All happens in 'Clustering'
      Clustering (cloud, 3.0, 1, 100);
  }
  
  while (!viewer->wasStopped ()) {
      viewer->spinOnce ();
  }
  
  
  return (0);  
}
