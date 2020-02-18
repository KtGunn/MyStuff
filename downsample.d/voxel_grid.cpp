#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/visualization/cloud_viewer.h>

int
main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  
  // Fill in the cloud data
  pcl::PCDReader reader;

  // Replace the path below with the path where you saved your file
  reader.read ("../table_scene_lms400.pcd", *cloud); // Remember to download the file first!
  
  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
            << " data points (" << pcl::getFieldsList (*cloud) << ").\n";
  
  
  // ************************************************************************
  // [KTG] Let's see the full cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr theCloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2 (*cloud, *theCloud);
  
  pcl::visualization::CloudViewer viewer ("Full_Cloud");
  viewer.showCloud ( theCloud );
  while ( !viewer.wasStopped() ) {
    // Wait for exit command
  }
  // ************************************************************************

  
  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);
  
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
            << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").\n";
  
  pcl::PCDWriter writer;
  writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered,
                Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
  

  // ************************************************************************
  // [KTG] And now the filtered cloud
  pcl::visualization::CloudViewer viewer2 ("Sampled_Cloud");
  pcl::fromPCLPointCloud2 (*cloud_filtered, *theCloud);

  viewer2.showCloud ( theCloud );
  while ( !viewer2.wasStopped() ) {
    // Nothing to do
  }

  return (0);
}
