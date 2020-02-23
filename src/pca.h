
// Header file to use with PCA analysis for bounding boxes

#include <iostream>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include "../src/render/box.h"

#pragma once

// OK
Eigen::Affine3f k_Compose (float (&t)[3], float (&r)[3]);
template<typename PointT> BoxQ k_SimpleBoxQ (typename pcl::PointCloud<PointT>::Ptr cluster);
//template<typename PointT> BoxQ k_SimpleBoxQ (typename pcl::PointCloud<PointT>::Ptr& cluster);
template<typename PointT> BoxQ k_PCA (typename pcl::PointCloud<PointT>::Ptr pCloud);
template<typename PointT> typename pcl::PointCloud<PointT>::Ptr
    k_transformCloud (typename pcl::PointCloud<PointT>::Ptr& srcCloud, Eigen::Affine3f& xf);

template<typename PointT> std::vector<PointT> k_MinMaxPts (typename pcl::PointCloud<PointT>::Ptr& cluster);
template<typename PointT> void k_QBoxDimensions (BoxQ& qBox, std::vector<PointT> vPs);

