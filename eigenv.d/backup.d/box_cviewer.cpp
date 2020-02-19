

#include <iostream>
#include <pcl/point_types.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

/*
  // NOT NEEDED
  #include <pcl/io/pcd_io.h>
  #include <pcl/ModelCoefficients.h>
  #include <pcl/sample_consensus/method_types.h>
  #include <pcl/sample_consensus/model_types.h>
  #include <pcl/segmentation/sac_segmentation.h>
  #include <pcl/filters/voxel_grid.h>
  #include <pcl/filters/extract_indices.h>
  
*/


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



int main () {

  pcl::PointCloud<pcl::PointXYZ>::Ptr pC = CylinderCloud (1.0, .10, 500);
  
  if (false) {
    // Let's see the cloud
    pcl::visualization::CloudViewer viewer ("CylinderCloud");
    
    viewer.showCloud ( pC );
    while ( !viewer.wasStopped() ) {
      // Nothing to do
    }
  } else {
    
    Eigen::Affine3f xF = Eigen::Affine3f::Identity();
    float t[3] = { 0,0,0 };
    float rY = 0.0;
    xF.translation () << t[0], t[1], t[2];
    xF.rotate (Eigen::AngleAxisf (rY, Eigen::Vector3f::UnitY() ));

    pcl::PointCloud<pcl::PointXYZ>::Ptr pCxF = tranformCloud (pC, xF);

    // Let's see the cloud
    pcl::visualization::CloudViewer viewer ("CylinderCloud");
    
    viewer.showCloud ( pCxF );
    while ( !viewer.wasStopped() ) {
      // Nothing to do
    }
  }
  
  return (0);
}
