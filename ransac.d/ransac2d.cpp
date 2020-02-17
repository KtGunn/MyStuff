/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../src/render/render.h"
#include "../src/processPointClouds.h"
#include <unordered_set>

// using templates for processPointClouds so also include .cpp to help linker
#include "../src/processPointClouds.cpp"

// 200213:
const static std::string sthighwayFile ("../../src/sensors/data/pcd/simpleHighway.pcd");


//#include "../../render/render.h"
//#include "../../processPointClouds.h"
//#include "../../processPointClouds.cpp"

// [KTG]
// #include <random>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Add inliers
    float scatter = 0.6;
    for(int i = -5; i < 5; i++)
  	{
	    double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
	    double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
	    pcl::PointXYZ point;
	    point.x = i+scatter*rx;
	    point.y = i+scatter*ry;
	    point.z = 0;
	    
	    cloud->points.push_back(point);
  	}
    // Add outliers
    int numOutliers = 10;
    while(numOutliers--)
  	{
	    double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
	    double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
	    pcl::PointXYZ point;
	    point.x = 5*rx;
	    point.y = 5*ry;
	    point.z = 0;
	    
	    cloud->points.push_back(point);
	    
  	}
    cloud->width = cloud->points.size();
    cloud->height = 1;
    
    return cloud;
    
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    return pointProcessor.loadPcd ( sthighwayFile );
    //return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem (1.0);
    return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    
    //std::random_device rd;
    //std::mt19937 gen (rd());
    //std::uniform_int_distribution<> dis(0, cloud->size()-1);
    
    // TODO: Fill in this function
    // For max iterations 
    for (short n=0; n<maxIterations; n++) {
	
	// Randomly sample subset and fit line
	std::unordered_set<int> testSet;
	while (testSet.size() < 3) {
	    //int index = dis (gen);
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

int main ()
{
    
    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();
    
    // Create data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
    
    
    // TODO: Change the max iteration and distance tolerance arguments for Ransac function
    std::unordered_set<int> inliers = Ransac(cloud, 30, 0.2);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());
    
    for(int index = 0; index < cloud->points.size(); index++)
	{
	    pcl::PointXYZ point = cloud->points[index];
	    if(inliers.count(index))
		cloudInliers->points.push_back(point);
	    else
		cloudOutliers->points.push_back(point);
	}
    
    
    // Render 2D point cloud with inliers and outliers
    if(inliers.size())
	{
	    renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
	    renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
    else
  	{
	    renderPointCloud(viewer,cloud,"data");
  	}
    
    while (!viewer->wasStopped ())
  	{
	    viewer->spinOnce ();
  	}
    
}
