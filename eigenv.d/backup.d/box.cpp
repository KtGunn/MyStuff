

#include <iostream>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
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

void visualize (pcl::visualization::PCLVisualizer& viewer,
                pcl::PointCloud<pcl::PointXYZ>::Ptr& srcCloud,
                const unsigned short* aRGB, std::string label) {

  // Define a color handler
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    chandler (srcCloud,aRGB[0],aRGB[1],aRGB[2]);
  //viewer.addPointCloud (srcCloud, chandler, "my cloud");

  std::cout << "hopping into viewer\n";
  /**/
    while (!viewer.wasStopped() ) {
      viewer.spinOnce();
    }
    
    /**/
  return;
}

int main () {

  // Create a viewer
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Viewer"));

  // Create a cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pC = CylinderCloud (1.0, .10, 500);

  viewer->addPointCloud<pcl::PointXYZ> (pC, "Cyl");
  viewer->addCoordinateSystem (1.0);
  while (!viewer->wasStopped()) {
    viewer->spinOnce ();
  }

  // Create a transformation
  Eigen::Affine3f xF = Eigen::Affine3f::Identity();
  float t[3] = { 0,0,0 };
  float rY = 0.0;
  xF.translation () << t[0], t[1], t[2];
  xF.rotate (Eigen::AngleAxisf (rY, Eigen::Vector3f::UnitY() ));

  // Create a transformed cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pCxF = tranformCloud (pC, xF);
  
  ushort aC[] =  {1,0,0};
  //visualize (viewer,pC, &aC[0], "");
  //visualize (viewer,pCxF, &aC[0], "");
  return (0);
}
