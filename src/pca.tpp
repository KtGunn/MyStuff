
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
// COMPOSE
//  Create a transformation given vector of translation and quaternion
//
Eigen::Affine3f k_Compose (float (&t)[3], Eigen::Quaternionf quat) {
    
    Eigen::Affine3f xF = Eigen::Affine3f::Identity ();
    xF.translation() << t[0], t[1], t[2];
    
    Eigen::Matrix3f rot = quat.toRotationMatrix ();
    xF.rotate (rot);
    return xF;
}


//*****************************************************************************
// MINMAXPTS  -- Minimum and Maximum 3D points of a cloud
//
template<typename PointT>
    std::vector<PointT> k_MinMaxPts (typename pcl::PointCloud<PointT>::Ptr& cluster)
{
    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D (*cluster, minPoint, maxPoint);
    return std::vector<PointT>({minPoint, maxPoint});
}


//*****************************************************************************
// QBoxDimensions  -- Fill in the dimensions of a bounding box
//
template<typename PointT>
    void k_QBoxDimensions (BoxQ& qBox, std::vector<PointT> vPs)
{
    // Fill in a BoxQ object
    qBox.cube_length = vPs[1].x - vPs[0].x;
    qBox.cube_width  = vPs[1].y - vPs[0].y;
    qBox.cube_height = vPs[1].z - vPs[0].z;
    return;
}

//BoxQ k_SimpleBoxQ (typename pcl::PointCloud<PointT>::Ptr& cluster)
template<typename PointT>
    BoxQ k_SimpleBoxQ (typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Compute a standard world space bounding box
    std::vector<PointT> vPs = k_MinMaxPts<PointT> (cluster);
    
    // Fill in a BoxQ object
    BoxQ qBox;
    k_QBoxDimensions (qBox, vPs);
    
    // Compute the cloud's centroid
    Eigen::Vector4f vctroid;
    pcl::compute3DCentroid ( *cluster, vctroid);
    
    // Set the box's frame
    qBox.bboxQuaternion = Eigen::Quaternionf::Identity ();
    qBox.bboxTransform = Eigen::Vector3f ({vctroid[0], vctroid[1], vctroid[2]});
    
    return (qBox);
}


//*****************************************************************************
// CLOUD TRANSFORMATION
//
template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr
    k_transformCloud (typename pcl::PointCloud<PointT>::Ptr& srcCloud, Eigen::Affine3f& xf)
{
    typename pcl::PointCloud<PointT>::Ptr dstCloud (new typename pcl::PointCloud<PointT>());
    pcl::transformPointCloud (*srcCloud, *dstCloud, xf);
    return dstCloud;
}


//*****************************************************************************
// PCA
//
template<typename PointT>
    BoxQ k_PCA (typename pcl::PointCloud<PointT>::Ptr pCloud)
{
    
    ///////////////////////////////////////////////////////////////////////////
    /// CHECK CLOUD SIZE
    //
    if (pCloud->size() < 6) {
	// Cloud is too small to do a PCA analysis; go with standard box
	return k_SimpleBoxQ<PointT> (pCloud);
    }


    ///////////////////////////////////////////////////////////////////////////
    /// CENTROID & EIGEN Analysis
    //
    // Start PCA analysis
    pcl::PCA<PointT> pca;
    
    // Set the input cloud
    pca.setInputCloud (pCloud);
    
    // Create the cloud in eigen space
    typename pcl::PointCloud<PointT>::Ptr pPCAcloud (new pcl::PointCloud<PointT>());
    pca.project (*pCloud, *pPCAcloud);
    
    // Get eigen vectors & values
    Eigen::Matrix3f eigenM = pca.getEigenVectors ();
    Eigen::Vector3f eigenVs = pca.getEigenValues ();
    
    // Get the centroid
    Eigen::Vector4f Ctroid = pca.getMean();
    
    if (true) {
	std::cout << " Centroid = " << Ctroid[0] << " " << Ctroid[1] << " " << Ctroid[2]  << std::endl;
	std::cout << " 1st eigenV = " << eigenM.col(0)[0] << " " <<eigenM.col(0)[1] << " " <<eigenM.col(0)[2] << std::endl;
	std::cout << " 1st eigenVal = " << eigenVs[0] << std::endl;
    }
    

    ///////////////////////////////////////////////////////////////////////////
    /// EIGEN CHECK
    //
    Eigen::Vector3f xvEigen (eigenM.col(0)[0], eigenM.col(0)[1], eigenM.col(0)[2]);
    float zComp =  fabs (xvEigen.dot (Eigen::Vector3f(0,0,1)));
    std::cout << "   zC = " << zComp << std::endl;
    if ( zComp > 0.5) {
	// Primary component is upward; don't want that
	std::cout << " PCA analysis : EigenV z-dominant (" << zComp << "): simple box created\n";
	return k_SimpleBoxQ<PointT> (pCloud);
    }
    
    ///////////////////////////////////////////////////////////////////////////
    /// CENTROID & EIGEN Analysis
    //
    // Translation is the centroid
    float tr[3] = {Ctroid[0], Ctroid[1], Ctroid[2]};
    
    // Rotation about z-axis only
    Eigen::Vector3f xRotated (eigenM.col(0)[0], eigenM.col(0)[1], 0);
    Eigen::Vector3f xAxis (1.0, 0.0, 0.0);
    Eigen::Quaternionf quat = Eigen::Quaternionf::FromTwoVectors (xAxis, xRotated);
    
    Eigen::Affine3f  xF = k_Compose (tr, quat);
    Eigen::Affine3f  xFinv = xF.inverse ();
    

    ///////////////////////////////////////////////////////////////////////////
    /// BOUNDING BOX in transformed cloud
    //
    // Create a transformed cloud
    typename pcl::PointCloud<PointT>::Ptr pCxF = k_transformCloud<PointT> (pCloud, xFinv);
    // Get the bounds
    std::vector<PointT>  vPts = k_MinMaxPts<PointT> (pCxF);
    // Create BoxQ and fill in the dimensions;
    BoxQ qB;
    k_QBoxDimensions (qB, vPts);
    // Set the transformation
    qB.bboxQuaternion = quat;
    qB.bboxTransform = Eigen::Vector3f ({Ctroid[0], Ctroid[1], Ctroid[2]});

    return (qB);
}

/*
*/
