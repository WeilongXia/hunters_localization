#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// namespace hunters_localization
// {

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

class CloudData
{
  public:
    CloudData() : cloud_ptr(new CloudT())
    {
    }

  public:
    double time = 0.0;
    CloudT::Ptr cloud_ptr;
};
// } // namespace hunters_localization