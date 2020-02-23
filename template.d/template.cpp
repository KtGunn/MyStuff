
// #include "header.h"

#include <iostream>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>

template<typename PointT>
    void DoNothing (typename pcl::PointCloud<PointT>::Ptr cloud)
{
    return;
}

