
#include <iostream>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>

#include "../src/render/box.h"


//*****************************************************************************
// COMPOSE
//  Create a transformation given vector of translation and RPY rotations,
//  in the order rZ,rY,rX
//
Eigen::Affine3f k_Compose (float (&t)[3], float (&r)[3]) {
  
  Eigen::Affine3f xF = Eigen::Affine3f::Identity();
  xF.translation() << t[0], t[1], t[2];

  Eigen::Matrix3f rot;
  rot = Eigen::AngleAxisf (r[0], Eigen::Vector3f::UnitZ ()) *
    Eigen::AngleAxisf (r[1], Eigen::Vector3f::UnitY ()) *
    Eigen::AngleAxisf (r[2], Eigen::Vector3f::UnitX ());

  xF.rotate (rot);
  return xF;
}


//*****************************************************************************
// BOUNDINGBOX -- created w.r.t. world frame
//
std::vector<pcl::PointXYZ> k_BoundingBox (pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster)
{
  // Find bounding box for one of the clusters
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D (*cluster, minPoint, maxPoint);
  return std::vector<pcl::PointXYZ>({minPoint, maxPoint});
}



//*****************************************************************************
// CLOUD TRANSFORMATION
//
pcl::PointCloud<pcl::PointXYZ>::Ptr
k_transformCLoud (pcl::PointCloud<pcl::PointXYZ>::Ptr& srcCloud, Eigen::Affine3f& xf) {
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr dstCloud (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud (*srcCloud, *dstCloud, xf);
  return dstCloud;
}



//*****************************************************************************
// PCA
//
//std::vector<pcl::PointXYZ> PCA (pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud)
const BoxQ k_PCA (pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud)
{

  ///////////////////////////////////////////////////////////////////////////
  /// CENTROID & EIGEN Analysis
  //
  // Start PCA analysis
  pcl::PCA<pcl::PointXYZ> pca;

  // Set the input cloud
  pca.setInputCloud (pCloud);

  // Create the cloud in eigen space
  pcl::PointCloud<pcl::PointXYZ>::Ptr pPCAcloud (new pcl::PointCloud<pcl::PointXYZ>());
  pca.project (*pCloud, *pPCAcloud);

  // Get eigen vectors & values
  Eigen::Matrix3f eigenM = pca.getEigenVectors ();
  Eigen::Vector3f eigenVs = pca.getEigenValues ();

  // Get the centroid
  Eigen::Vector4f Ctroid = pca.getMean();

  if (false) {
    std::cout << " Centroid = " << Ctroid[0] << " " << Ctroid[1] << " " << Ctroid[2]  << std::endl;
    std::cout << " 1st eigenV = " << eigenM.col(0)[0] << " " <<eigenM.col(0)[1] << " " <<eigenM.col(0)[2] << std::endl;
    std::cout << " 1st eigenVal = " << eigenVs[0] << std::endl;
  }
  
  // Eigen analysis check
  if ( fabs(eigenVs[0]) < fabs(eigenVs[1]) && fabs(eigenVs[0]) < fabs(eigenVs[2]) ) {
    // 1st eigen value is NOT dominant as we expected
    std::cout << " PCA analysis failed; returning normal bounding box\n";
    
    if (true) {
      // Compute a standard world space bounding box
      std::vector<pcl::PointXYZ> vPs = k_BoundingBox (pCloud);
      // Compute the cloud's centroid
      Eigen::Vector4f vctroid;
      pcl::compute3DCentroid ( *pCloud, vctroid);
      // Fill in a BoxQ object
      BoxQ qBox;
      qBox.cube_length = vPs[1].x - vPs[0].x;
      qBox.cube_width  = vPs[1].y - vPs[0].y;
      qBox.cube_height = vPs[1].z - vPs[0].z;
      qBox.bboxTransform = Eigen::Vector3f (vctroid[0], vctroid[1], vctroid[2]);
      return (qBox);
    }
  }
  
  
   ///////////////////////////////////////////////////////////////////////////
   /// BOUNDING BOX in PCA cloud
   //
   std::vector<pcl::PointXYZ> vB = k_BoundingBox (pPCAcloud);
   for (pcl::PointXYZ pclPt : vB) {
     std::cout << "   dbg Eigen Bound " << pclPt << std::endl;
   }
   float cube_length  = vB[1].x - vB[0].x;
   float cube_width =   vB[1].y - vB[0].y;
   float cube_height  = vB[1].z - vB[0].z;
   std::cout << "xW=" << cube_length << " yH=" << cube_width << " zD=" << cube_height <<
     " pts " << pPCAcloud->size() << std::endl;
   
   
   ///////////////////////////////////////////////////////////////////////////
   /// TRANFORMATION -- centroid & dominant direction
   //
   // Set the translation
   Eigen::Vector3f Trn (Ctroid[0],Ctroid[1],Ctroid[2]);
   // Set the rotation (z-axis only of interest)
   Eigen::Vector3f xRotated (eigenM.col(0)[0], eigenM.col(0)[1], 0);
   Eigen::Vector3f xAxis (1.0, 0.0, 0.0);
   Eigen::Quaternionf quat = Eigen::Quaternionf::FromTwoVectors (xAxis, xRotated);
  
   BoxQ qB;
   qB.bboxQuaternion = quat;
   qB.bboxTransform = Trn;
   qB.cube_length = cube_length;
   qB.cube_width = cube_width;
   qB.cube_height = cube_height;
  
   return (qB);
 }



