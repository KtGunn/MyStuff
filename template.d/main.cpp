
#include <iostream>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>

#include "header.h"


template<typename PointT>
    void DoNothing_A (typename pcl::PointCloud<PointT>::Ptr cloud)
{
    return;
}


int main () {

    std::cout << "Hello \n";
    // Create a cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pC (new pcl::PointCloud<pcl::PointXYZ>());
    
    DoNothing_A<pcl::PointXYZ>(pC);
    DoNothing<pcl::PointXYZ>(pC);

    // Does not compile
    // DoNothing<typename pcl::PointCloud<pcl::PointXYZ>::Ptr>(pC);
    
    return 0;
}

