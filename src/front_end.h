#pragma once

#include <cmath>
#include <deque>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>

#include "cloud_data.h"
#include "localization_config.h"
#include "tic_toc.h"

#include "../ndt_omp/include/pclomp/ndt_omp.h"
#include "../ndt_omp/include/pclomp/ndt_omp_impl.hpp"

// namespace hunters_localization
// {
class Frame
{
  public:
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    CloudData cloud_data;
};

class FrontEnd
{
  public:
    FrontEnd(const ros::NodeHandle &nh);

    Eigen::Matrix4f Update(const CloudData &cloud_data);
    bool SetInitPose(const Eigen::Matrix4f &init_pose);
    bool SetPredictPose(const Eigen::Matrix4f &predict_pose);

    bool GetNewLocalMap(CloudT::Ptr &local_map_ptr);
    bool GetNewGlobalMap(CloudT::Ptr &global_map_ptr);
    bool GetCurrentScan(CloudT::Ptr &current_scan_ptr);

    LocalizationConfig config_;

  private:
    void UpdateNewFrame(const Frame &new_key_frame);

  private:
    pcl::VoxelGrid<PointT> cloud_filter_;
    pcl::VoxelGrid<PointT> local_map_filter_;
    pcl::VoxelGrid<PointT> display_filter_;
    pcl::NormalDistributionsTransform<PointT, PointT>::Ptr ndt_ptr_;
    pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr register_ptr_;

    std::deque<Frame> local_map_frames_;
    std::deque<Frame> global_map_frames_;

    bool has_new_local_map_ = false;
    bool has_new_global_map_ = false;
    CloudT::Ptr local_map_ptr_;
    CloudT::Ptr global_map_ptr_;
    CloudT::Ptr result_cloud_ptr_;
    Frame current_frame_;

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f predict_pose_ = Eigen::Matrix4f::Identity();
};
// } // namespace hunters_localization