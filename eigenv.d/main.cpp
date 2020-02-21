
#include "pca.h"


/*
  #include <iostream>
  #include <pcl/point_types.h>

  #include <pcl/visualization/pcl_visualizer.h>
  #include <pcl/common/common.h>
  #include <pcl/common/transforms.h>
  #include <pcl/common/pca.h>


  // We'll borrow this from Aron Brown for test/debug
  //
  struct BoxQ {
    Eigen::Vector3f bboxTransform;
    Eigen::Quaternionf bboxQuaternion;
    float cube_length, cube_width, cube_height; 
  };
*/


 //*****************************************************************************
 // CYLINDERCLOUD
 //   Function to create a point cloud
 //   Cloud is randomly distributed on the surface of a cylinder
 //   Cylinder is oriented along x-axis, with base at (0,0,0)
 //
 pcl::PointCloud<pcl::PointXYZ>::Ptr CylinderCloud (float len, float radius, int count)
 {
   // Create the cloud
   pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud (new pcl::PointCloud<pcl::PointXYZ>);
  
   // Initialize randomizer
   srand (time(NULL));
   float toRad = atan2(0,-1)/180.0;

   for (int n=0; n<count; n++) {
     // Random height along cylinder
     float x = (rand() % 100)/100.0 * len;

     // Random angle around the perimeter
     float theta = (rand() % 360) * toRad;
    
     float z = radius * cos (theta);
     float y = radius * sin (theta);
    
     pcl::PointXYZ pt (x,y,z);
     pCloud->points.push_back (pt);
   }
   return pCloud;
 }



void TestStuff () {
  
   // Create a cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pC = CylinderCloud (1.0, .10, 500);
  
  
  if (false) {
    // ************** THIS WORKS  -- 'Ptr' pointer, on heap
    // Create a viewer
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Viewer"));
  }
  
  // ************** THIS WORKS  ALSO!  -- non-Ptr, on stack
  // Create a viewer
  pcl::visualization::PCLVisualizer viewer ("Viewer");
  
  
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Original cloud
  //
  viewer.addPointCloud<pcl::PointXYZ> (pC, "Cyl");
  viewer.addCoordinateSystem (1.0);
  
  std::vector<pcl::PointXYZ> vB = k_BoundingBox (pC);
  viewer.addCube (vB[0].x, vB[1].x, vB[0].y, vB[1].y, vB[0].z, vB[1].z,50,75,33,"Orig");
  
  
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Transformed cloud
  //
  // Create a transformation
  const float PI = atan2(0,-1);
  float tr[3] = {1.0, 0.5, -0.2};
  float ro[3] = {PI/4, 0.0, 0.0};
  Eigen::Affine3f XF = k_Compose (tr,ro);
  
  // Create a transformed cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pCxF = k_transformCLoud (pC, XF);
  viewer.addPointCloud<pcl::PointXYZ> (pCxF, "xfCyl");
  
  if (false) {
    // This creates a WORLD centered box instead of a CLOUD centered box
    vB.empty();
    vB = k_BoundingBox (pCxF);
    viewer.addCube (vB[0].x, vB[1].x, vB[0].y, vB[1].y, vB[0].z, vB[1].z,50,75,33, "xF");
    
  } else {
    // Correctly sized & oriented bounding box
    BoxQ qB = k_PCA (pCxF);
    
    // pos = (x,y,z) of the MIDDLE of the cube
    // quaT - quaternion (constructed from two instances of the x-axis vector
    // xW,yH,zD = overall dimensions (not half)
    viewer.addCube (qB.bboxTransform, qB.bboxQuaternion, qB.cube_length, qB.cube_width, qB.cube_height , "Foo"); 
  }
  
  if (true) {
    while (!viewer.wasStopped()) {
      viewer.spinOnce ();
    }
  }
  
}

//////////////////////////////////////////////////////////////////////////////////
///  MAIN
//
int main () {
  TestStuff ();
  return (0);
}

