#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// namespace hunters_localization
// {
class CloudData
{
  public:
    using POINT = pcl::PointXYZ;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;

  public:
    CloudData() : cloud_ptr(new CLOUD())
    {
    }

  public:
    double time = 0.0;
    CLOUD_PTR cloud_ptr;
};
// } // namespace hunters_localization