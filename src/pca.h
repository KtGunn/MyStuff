
// Header file to use with PCA analysis for bounding boxes

#include <iostream>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include "../src/render/box.h"

#pragma once

Eigen::Affine3f k_Compose (float (&t)[3], float (&r)[3]);

std::vector<pcl::PointXYZ> k_BoundingBox (pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster);

pcl::PointCloud<pcl::PointXYZ>::Ptr
k_transformCLoud (pcl::PointCloud<pcl::PointXYZ>::Ptr& srcCloud, Eigen::Affine3f& xf);

const BoxQ k_PCA (pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud);

