
#include "pca.h"


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
	pcl::visualization::PCLVisualizer::Ptr viewer ( new pcl::visualization::PCLVisualizer ("Viewer"));
    }
    
    // ************** THIS WORKS  ALSO!  -- non-Ptr, on stack
    // Create a viewer
    pcl::visualization::PCLVisualizer viewer ("Viewer");
    
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Original cloud
    //
    viewer.addPointCloud<pcl::PointXYZ> (pC, "Cyl");
    viewer.addCoordinateSystem (1.0);
    
    BoxQ qB = k_SimpleBoxQ<typename pcl::PointXYZ>(pC);
    viewer.addCube (qB.bboxTransform, qB.bboxQuaternion, qB.cube_length, qB.cube_width, qB.cube_height , "Foo"); 

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Transformed cloud
    //
    // Create a transformation
    const float PI = atan2(0,-1);

    float tr[3] = {1.0, 0.5, -0.2};
    float ro[3] = {0.0, 0.0, 0.0};

    if (false) {
	ro[0] = -1*PI/4;
    } else {
	ro[0] = PI/8;
    }
    Eigen::Affine3f XF = k_Compose (tr,ro);
    
    // Create a transformed cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pCxF = k_transformCloud<pcl::PointXYZ>(pC, XF);
    viewer.addPointCloud<pcl::PointXYZ> (pCxF, "xfCyl");
    

    if (false) {
	BoxQ qBr = k_SimpleBoxQ<pcl::PointXYZ> (pCxF);
	viewer.addCube (qBr.bboxTransform, qBr.bboxQuaternion, qBr.cube_length, qBr.cube_width, qBr.cube_height , "Foo_R"); 
    } else {
	BoxQ qBr2 = k_PCA<pcl::PointXYZ> (pCxF);
	viewer.addCube (qBr2.bboxTransform, qBr2.bboxQuaternion, qBr2.cube_length, qBr2.cube_width, qBr2.cube_height , "Foo_Real"); 
    }

    /*    
    */
    
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

