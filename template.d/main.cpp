
#include "header.h"
#include "template.h"


template<typename PointT>
    void DoNothing_A (typename pcl::PointCloud<PointT>::Ptr cloud)
{
    return;
}


int main () {

    // Create a cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pC (new pcl::PointCloud<pcl::PointXYZ>());
    
    DoNothing_A<pcl::PointXYZ>(pC); // this is OK
    DoNothing<pcl::PointXYZ>(pC);   // this does not work!
    
    return 0;
}

