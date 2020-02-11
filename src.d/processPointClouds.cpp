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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  std::cout << " B 1\n";
  typename pcl::PointCloud<PointT>::Ptr road (new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr obstacles (new pcl::PointCloud<PointT>());

  for (int ind : inliers->indices ) {
      road->points.push_back(cloud->points[ind]);
  }

std::cout << " B 2\n";
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
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
std::cout <<  " A 1\n";
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};

std::cout <<  " A 2\n";
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
std::cout <<  " A 3\n";
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

std::cout <<  " A 4\n";
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

std::cout <<  " A 5\n";
    if (inliers->indices.size() == 0 ) {
        std::cout << "Sorry, inliers size is 0\n";
    }
std::cout <<  " A 6\n";
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}

std::unordered_set<int> myRansac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
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
		pcl::PointXYZ p1 = cloud->points[*it]; 		it++;
		pcl::PointXYZ p2 = cloud->points[*it];		it++;
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


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

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