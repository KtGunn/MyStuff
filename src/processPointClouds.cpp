// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include <unordered_set>


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud (typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                               float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
  
  // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (filterRes,filterRes,filterRes);

  // We need a return cloud
  typename pcl::PointCloud<PointT>::Ptr filteredCloud (new typename pcl::PointCloud<PointT>());
  sor.filter (*filteredCloud);
  std::cout << " Voxel filtered size = " << filteredCloud->points.size() << std::endl;

  // Now create the ROI region-of-interest
  typename pcl::PointCloud<PointT>::Ptr filteredCroppedCloud (new pcl::PointCloud<PointT>);

  pcl::CropBox<PointT> cBox (false); // true==we want indices

  cBox.setInputCloud (filteredCloud);
  cBox.setMin (minPoint);
  cBox.setMax (maxPoint);
  
  cBox.filter (*filteredCroppedCloud);
  std::cout << " Box filtered size = " << filteredCroppedCloud->points.size() << std::endl;

  pcl::CropBox<PointT> cBroof (true); // true==we want indices
  cBroof.setInputCloud (filteredCroppedCloud);
  cBroof.setMin (Eigen::Vector4f(-2,-3,-2,1));
  cBroof.setMax (Eigen::Vector4f(4,3,1,1));
  std::vector<int> vIndices;
  cBroof.filter (vIndices);
  std::cout << " We got " << vIndices.size() << std::endl;
  
  // Create the indices object
  pcl::PointIndices::Ptr pInds {new pcl::PointIndices};
  for (int i: vIndices) {
    pInds->indices.push_back(i);
  }

  // Get the Extractor object going
  pcl::ExtractIndices<PointT> extractor;
  extractor.setInputCloud (filteredCroppedCloud);
  extractor.setIndices (pInds);
  extractor.setNegative(true);

  typename pcl::PointCloud<PointT>::Ptr finalCloud (new pcl::PointCloud<PointT>());

  extractor.filter (*finalCloud);
  

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
  
  return finalCloud;
  //return filteredCroppedCloud;
  //return filteredCloud;
  
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds (pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
  // DONE -- TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr road (new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr obstacles (new pcl::PointCloud<PointT>());
  
  for (int ind : inliers->indices ) {
    road->points.push_back(cloud->points[ind]);
  }
  
  pcl::ExtractIndices<pcl::PointXYZ> extractor;
  extractor.setInputCloud (cloud);
  extractor.setIndices (inliers);
  extractor.setNegative(true);
  extractor.filter (*obstacles);
  
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
    segResult(road, obstacles);
  return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
  // DONE -- TODO:: Fill in this function to find inliers for the cloud.
  pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
  pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
  
  // 
  std::unordered_set<int> inlierPoints;
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult;
  
  const bool PCL = false;
  
  if (PCL) {
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // PCL ransac
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
    
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    
    if (inliers->indices.size() == 0 ) {
	    std::cout << "Sorry, inliers size is 0\n";
    }
    
  } else {
    // Call the custom RANSAC funtion
    inlierPoints = ktRansac ( cloud, maxIterations, distanceThreshold);
    
    // This copy operation seems very inefficient
    for ( int ptId : inlierPoints ) {
	    inliers->indices.push_back (ptId);
    }
  }
  
  segResult = SeparateClouds (inliers,cloud);
  
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
  
  return segResult;
}


//******************************************************************************************************************
// My implementation of RANSAC
//
template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::ktRansac (typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
  std::unordered_set<int> inliersResult;
  srand (time(NULL));
  
  // For max iterations 
  for (short n=0; n<maxIterations; n++) {
    
    // Randomly sample subset and fit line
    std::unordered_set<int> testSet;
    while (testSet.size() < 3) {
	    int index = rand() % (cloud->points.size());
	    if ( testSet.count(index) > 0 ) {
        continue;
	    } else {
        testSet.insert(index);
	    }
    }
    
    auto it = testSet.begin();
    pcl::PointXYZ p1 = cloud->points[*it]; it++;
    pcl::PointXYZ p2 = cloud->points[*it]; it++;
    pcl::PointXYZ p3 = cloud->points[*it];
    
    // Unit vector |P2-P1|
    float a = p2.x-p1.x;
    float b = p2.y-p1.y;
    float c = p2.z-p1.z;
    float denom = sqrt(a*a+b*b+c*c);
    a /= denom;
    b /= denom;
    c /= denom;
    
    // Unit vector |P3-P1|
    float d = p3.x-p1.x;
    float e = p3.y-p1.y;
    float f = p3.z-p1.z;
    denom = sqrt(d*d+e*e+f*f);
    d /= denom;
    e /= denom;
    f /= denom;
    
    // Normal vector (uv1 x uv2)
    float A = b*f-e*c; // i
    float B = c*d-a*f; // j
    float C = a*e-b*d; // k
    float D = A*p1.x + B*p1.y + C*p1.z;
    denom = sqrt (A*A+B*B+C*C);
    
    // Measure distance between every point and fitted line
    for (short k=0; k<cloud->size(); k++) {
	    
	    // Next could point to test
	    pcl::PointXYZ pt = cloud->points[k];
	    float dist = fabs (A*pt.x + B*pt.y + C*pt.z + D)/denom;
	    
	    // If distance is smaller than threshold count it as inlier
	    if ( dist <= distanceTol ) {
        testSet.insert(k);
	    }
    }
    
    // Return indicies of inliers from fitted line with most inliers
    if ( inliersResult.size() < testSet.size() ) {
	    inliersResult = testSet;
    }
  }
  
  return inliersResult;
}

//*** PROXIMITY **********************************************************************************************
//
template<typename PointT>
void ProcessPointClouds<PointT>::Proximity (const int id, typename pcl::PointCloud<PointT>::Ptr cloud,
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

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering (typename pcl::PointCloud<PointT>::Ptr cloud,
                                        float clusterTolerance, int minSize, int maxSize)
{
  
  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();
  
  // Note the clusters are 'vectors-of-clouds'
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
  
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
  
  for (uint n=0; n<cloud->size(); n++) {
    
    // Is this point processed?
    if ( mapProcessed.count (n) == 0 ) {
      
	    // No ; create a cluster of point IDs
	    std::vector<int> idsCluster;
	    
	    // Look for adjacent points
	    Proximity (n, cloud, idsCluster, mapProcessed, tree, dTol);
	    
	    // Add the cluster to the list
	    std::cout << " Cluster size = " << idsCluster.size() << std::endl;
	    
	    if (idsCluster.size() >= minSize && idsCluster.size() <= maxSize) {
        // NOT yet compiled
        typename pcl::PointCloud<PointT>::Ptr newCluster (new pcl::PointCloud<PointT>);
        for (int id : idsCluster) {
          newCluster->points.push_back (cloud->points[id]);
        }
        clusters.push_back ( newCluster );
	    }
    }
  }
  
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " <<
    clusters.size() << " clusters" << std::endl;
  
  return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
  
  // Find bounding box for one of the clusters
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);
  
  Box box;
  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;
  
  return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
  pcl::io::savePCDFileASCII (file, *cloud);
  std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{
  
  typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  
  if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file \n");
    }
  std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;
  
  return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{
  
  std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});
  
  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());
  
  return paths;
  
}
