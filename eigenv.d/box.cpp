

#include <iostream>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>


// Function to create a point cloud
// Cloud is randomly distributed on the surface of a cylinder
// Cylinder is oriented along z-axis, with base at (0,0,0)
//
pcl::PointCloud<pcl::PointXYZ>::Ptr CylinderCloud (float len, float radius, int count) {
  //std::cout << " L=" << len << " r=" << radius << " c=" << count << std::endl;
  
  // Create the cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud (new pcl::PointCloud<pcl::PointXYZ>);

  // Initialize randomizer
  srand (time(NULL));
  float toRad = atan2(0,-1)/180.0;

  for (int n=0; n<count; n++) {
    // Random height along cylinder
    float z = (rand() % 100)/100.0 * len;

    // Random angle around the perimeter
    float theta = (rand() % 360) * toRad;
    
    float x = radius * cos (theta);
    float y = radius * sin (theta);
    
    pcl::PointXYZ pt (x,y,z);
    pCloud->points.push_back (pt);
  }
  return pCloud;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr
tranformCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr& srcCloud, Eigen::Affine3f& xf) {
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr dstCloud (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud (*srcCloud, *dstCloud, xf);
  return dstCloud;
}


std::vector<pcl::PointXYZ> BoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster)
{
  // Find bounding box for one of the clusters
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D (*cluster, minPoint, maxPoint);
  return std::vector<pcl::PointXYZ>({minPoint, maxPoint});
}


//////////////////////////////////////////////////////////////////////////////////
///  MAIN
//
int main () {

  // Create a cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pC = CylinderCloud (1.0, .10, 500);


  if (false) {

    // ************** THIS WORKS  -- 'Ptr' pointer, on heap
    // Create a viewer
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Viewer"));
    
    viewer->addPointCloud<pcl::PointXYZ> (pC, "Cyl");
    viewer->addCoordinateSystem (1.0);
    while (!viewer->wasStopped()) {
      viewer->spinOnce ();
    }
  }

  
  // ************** THIS WORKS  ALSO!  -- non-Ptr, on stack
  // Create a viewer
  pcl::visualization::PCLVisualizer viewer ("Viewer");
  

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Original cloud
  //
  viewer.addPointCloud<pcl::PointXYZ> (pC, "Cyl");
  viewer.addCoordinateSystem (1.0);
  
  std::vector<pcl::PointXYZ> vB = BoundingBox (pC);
  viewer.addCube (vB[0].x, vB[1].x, vB[0].y, vB[1].y, vB[0].z, vB[1].z,50,75,33,"Orig");
  
  
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Transformed cloud
  //
  // Create a transformation
  Eigen::Affine3f xF = Eigen::Affine3f::Identity();
  float t[3] = {0.75, 0, 0};
  float rY = 1.0;
  xF.translation () << t[0], t[1], t[2];
  xF.rotate (Eigen::AngleAxisf (rY, Eigen::Vector3f::UnitY() ));
  
  // Create a transformed cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pCxF = tranformCloud (pC, xF);
  viewer.addPointCloud<pcl::PointXYZ> (pCxF, "xfCyl");
  
  vB.empty();
  vB = BoundingBox (pCxF);
  viewer.addCube (vB[0].x, vB[1].x, vB[0].y, vB[1].y, vB[0].z, vB[1].z,50,75,33, "xF");


  while (!viewer.wasStopped()) {
    viewer.spinOnce ();
  }

  return (0);
}
