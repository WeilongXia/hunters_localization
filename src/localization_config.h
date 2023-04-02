#pragma once

#include <ros/ros.h>

struct LocalizationConfig
{
    void GetParam(const ros::NodeHandle &nh)
    {
        nh.param<double>("cloud_leaf_size", cloud_leaf_size, 1.3);
        nh.param<double>("local_map_leaf_size", local_map_leaf_size, 0.6);
        nh.param<double>("display_leaf_size", display_leaf_size, 0.5);

        nh.param<double>("pcl_ndt_resolution", pcl_ndt_resolution, 1.0);
        nh.param<double>("pcl_ndt_step_size", pcl_ndt_step_size, 0.1);
        nh.param<double>("pcl_ndt_epsilon", pcl_ndt_epsilon, 0.01);
        nh.param<int>("pcl_ndt_max_iteration", pcl_ndt_max_iteration, 30);

        nh.param<double>("ndt_omp_resolution", ndt_omp_resolution, 1.0);

        nh.param<double>("key_pose_threshold", key_pose_threshold, 2.0);

        nh.param<std::string>("lidar_topic_name", lidar_topic_name, "/velodyne_points");

        nh.param<bool>("PCL_NDT", PCL_NDT, false);
        nh.param<bool>("NDT_OMP", NDT_OMP, false);

        nh.param<int>("local_map_size", local_map_size, 20);
        nh.param<int>("local_map_filter_threshold", local_map_filter_threshold, 10);
    }

    double cloud_leaf_size;
    double local_map_leaf_size;
    double display_leaf_size;

    double pcl_ndt_resolution;
    double pcl_ndt_step_size;
    double pcl_ndt_epsilon;
    int pcl_ndt_max_iteration;

    double ndt_omp_resolution;

    double key_pose_threshold;

    std::string lidar_topic_name;

    bool PCL_NDT;
    bool NDT_OMP;

    int local_map_size;
    int local_map_filter_threshold;
};