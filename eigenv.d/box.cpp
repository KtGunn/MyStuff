
#include <iostream>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>

// We'll borrow this from Aron Brown
//
struct BoxQ {
  Eigen::Vector3f midPos;
  Eigen::Quaternionf quatZ;
  float xWidth, yHeight, zDepth; 
};


//*****************************************************************************
// COMPOSE
//  Create a transformation given vector of translation and RPY rotations,
//  in the order rZ,rY,rX
//
Eigen::Affine3f Compose (float (&t)[3], float (&r)[3]) {
  
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
// BOUNDINGBOX
//
std::vector<pcl::PointXYZ> BoundingBox (pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster)
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
tranformCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr& srcCloud, Eigen::Affine3f& xf) {
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr dstCloud (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud (*srcCloud, *dstCloud, xf);
  return dstCloud;
}



//*****************************************************************************
// PCA
//
//std::vector<pcl::PointXYZ> PCA (pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud)
const BoxQ PCA (pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud)
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

  std::cout << " Centroid = " << Ctroid[0] << " " << Ctroid[1] << " " << Ctroid[2]  << std::endl;
  std::cout << " 1st eigenV = " << eigenM.col(0)[0] << " " <<eigenM.col(0)[1] << " " <<eigenM.col(0)[2] << std::endl;
  std::cout << " 1st eigenVal = " << eigenVs[0] << std::endl;

  // Eigen analysis check
  if ( fabs(eigenVs[0]) < fabs(eigenVs[1]) && fabs(eigenVs[0]) < fabs(eigenVs[2]) ) {
    // 1st eigen value is NOT dominant as we expected
    std::cout << " PCA analysis failed; returning normal bounding box\n";
     return (BoxQ());
   }
  

   ///////////////////////////////////////////////////////////////////////////
   /// BOUNDING BOX in PCA cloud
   //
   std::vector<pcl::PointXYZ> vB = BoundingBox (pPCAcloud);
   for (pcl::PointXYZ pclPt : vB) {
     std::cout << "   dbg Eigen Bound " << pclPt << std::endl;
   }
   float xWidth  = vB[1].x - vB[0].x;
   float yHeight = vB[1].y - vB[0].y;
   float zDepth  = vB[1].z - vB[0].z;
   std::cout << "xW=" << xWidth << " yH=" << yHeight << " zD=" << zDepth << std::endl;


   ///////////////////////////////////////////////////////////////////////////
   /// TRANFORMATION -- centroid & dominant direction
   //
   // Set the translation
   Eigen::Vector3f Trn (Ctroid[0],Ctroid[1],Ctroid[2]);
   // Set the rotation (z-axis only of interest)
   Eigen::Vector3f xRotated (eigenM.col(0)[0], eigenM.col(0)[1], 0);
   Eigen::Vector3f xAxis (1.0, 0.0, 0.0);
   Eigen::Quaternionf quat = Eigen::Quaternionf::FromTwoVectors (xAxis, xRotated);
   //quat.setFromTwoVectors (xAxis, xRotated);
  
   BoxQ qB;
   qB.quatZ = quat;
   qB.midPos = Trn;
   qB.xWidth = xWidth;
   qB.yHeight = yHeight;
   qB.zDepth = zDepth;
  
   return (qB);
 }

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
   const float PI = atan2(0,-1);
   float tr[3] = {1.0, 0.5, -0.2};
   float ro[3] = {PI/4, 0.0, 0.0};
   Eigen::Affine3f XF = Compose (tr,ro);

   // Create a transformed cloud
   pcl::PointCloud<pcl::PointXYZ>::Ptr pCxF = tranformCloud (pC, XF);
   viewer.addPointCloud<pcl::PointXYZ> (pCxF, "xfCyl");
  
   if (false) {
     // This creates a WORLD centered box instead of a CLOUD centered box
     vB.empty();
     vB = BoundingBox (pCxF);
     viewer.addCube (vB[0].x, vB[1].x, vB[0].y, vB[1].y, vB[0].z, vB[1].z,50,75,33, "xF");

   } else {
     // Correctly sized & oriented bounding box
     BoxQ qB = PCA (pCxF);
     
     // pos = (x,y,z) of the MIDDLE of the cube
     // quaT - quaternion (constructed from two instances of the x-axis vector
     // xW,yH,zD = overall dimensions (not half)
     viewer.addCube (qB.midPos, qB.quatZ, qB.xWidth, qB.yHeight, qB.zDepth , "Foo"); 
   }

  if (true) {
    while (!viewer.wasStopped()) {
      viewer.spinOnce ();
    }
  }
  
  return (0);
}
