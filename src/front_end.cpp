#include "front_end.h"

// namespace hunters_localization
// {
FrontEnd::FrontEnd(const ros::NodeHandle &nh)
    : ndt_ptr_(new pcl::NormalDistributionsTransform<PointT, PointT>()), local_map_ptr_(new CloudT()),
      global_map_ptr_(new CloudT()), result_cloud_ptr_(new CloudT()),
      register_ptr_(new pclomp::NormalDistributionsTransform<PointT, PointT>())
{
    config_.GetParam(nh);

    cloud_filter_.setLeafSize(config_.cloud_leaf_size, config_.cloud_leaf_size, config_.cloud_leaf_size);
    local_map_filter_.setLeafSize(config_.local_map_leaf_size, config_.local_map_leaf_size,
                                  config_.local_map_leaf_size);
    display_filter_.setLeafSize(config_.display_leaf_size, config_.display_leaf_size, config_.display_leaf_size);
    ndt_ptr_->setResolution(config_.pcl_ndt_resolution);
    ndt_ptr_->setStepSize(config_.pcl_ndt_step_size);
    ndt_ptr_->setTransformationEpsilon(config_.pcl_ndt_epsilon);
    ndt_ptr_->setMaximumIterations(config_.pcl_ndt_max_iteration);

    register_ptr_->setResolution(config_.ndt_omp_resolution);
    int avalib_cpus = omp_get_max_threads();
    register_ptr_->setNumThreads(avalib_cpus);
    register_ptr_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
}

Eigen::Matrix4f FrontEnd::Update(const CloudData &cloud_data)
{
    current_frame_.cloud_data.time = cloud_data.time;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, indices);

    CloudT::Ptr filtered_cloud_ptr(new CloudT());
    cloud_filter_.setInputCloud(current_frame_.cloud_data.cloud_ptr);
    cloud_filter_.filter(*filtered_cloud_ptr);

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = init_pose_;

    // No key_frame in local_map_frames_, means the first frame
    // Set the first frame as key_frame, Update local_map and global_map
    if (local_map_frames_.size() == 0)
    {
        current_frame_.pose = init_pose_;
        UpdateNewFrame(current_frame_);
        return current_frame_.pose;
    }

    // Not the first frame, execuate matching
    if (config_.PCL_NDT)
    {
        TicToc tic_toc;
        ndt_ptr_->setInputSource(filtered_cloud_ptr);
        ndt_ptr_->align(*result_cloud_ptr_, predict_pose);
        current_frame_.pose = ndt_ptr_->getFinalTransformation();
        ROS_INFO("PCL_NDT registration cost %f ms", tic_toc.toc());
    }
    else if (config_.NDT_OMP)
    {
        TicToc tic_toc;
        register_ptr_->setInputSource(filtered_cloud_ptr);
        register_ptr_->align(*result_cloud_ptr_, predict_pose);
        current_frame_.pose = register_ptr_->getFinalTransformation();
        ROS_INFO("NDT_OMP registration cost %f ms", tic_toc.toc());
    }

    // Update relative motion between current_frame and last_frame
    step_pose = last_pose.inverse() * current_frame_.pose;
    predict_pose = current_frame_.pose * step_pose;
    last_pose = current_frame_.pose;

    // Determine if current_frame is a new key_frame, depends on the matching distance
    if (fabs(last_key_frame_pose(0, 3) - current_frame_.pose(0, 3)) +
            fabs(last_key_frame_pose(1, 3) - current_frame_.pose(1, 3)) +
            fabs(last_key_frame_pose(2, 3) - current_frame_.pose(2, 3)) >
        config_.key_pose_threshold)
    {
        UpdateNewFrame(current_frame_);
        last_key_frame_pose = current_frame_.pose;
    }

    return current_frame_.pose;
}

bool FrontEnd::SetInitPose(const Eigen::Matrix4f &init_pose)
{
    init_pose_ = init_pose;
    return true;
}

bool FrontEnd::SetPredictPose(const Eigen::Matrix4f &predict_pose)
{
    predict_pose_ = predict_pose;
    return true;
}

void FrontEnd::UpdateNewFrame(const Frame &new_key_frame)
{
    Frame key_frame = new_key_frame;

    key_frame.cloud_data.cloud_ptr.reset(new CloudT(*new_key_frame.cloud_data.cloud_ptr));
    CloudT::Ptr transformed_cloud_ptr(new CloudT());

    // update local_map
    local_map_frames_.push_back(key_frame);
    while (local_map_frames_.size() > config_.local_map_size)
    {
        local_map_frames_.pop_front();
    }
    local_map_ptr_.reset(new CloudT());
    for (size_t i = 0; i < local_map_frames_.size(); ++i)
    {
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr, *transformed_cloud_ptr,
                                 local_map_frames_.at(i).pose);
        *local_map_ptr_ += *transformed_cloud_ptr;
    }
    has_new_local_map_ = true;

    // update target cloud that will be matched
    if (local_map_frames_.size() < config_.local_map_filter_threshold)
    {
        if (config_.PCL_NDT)
        {
            ndt_ptr_->setInputTarget(local_map_ptr_);
        }
        else if (config_.NDT_OMP)
        {
            register_ptr_->setInputTarget(local_map_ptr_);
        }
    }
    else
    {
        CloudT::Ptr filtered_local_map_ptr(new CloudT());
        local_map_filter_.setInputCloud(local_map_ptr_);
        local_map_filter_.filter(*filtered_local_map_ptr);
        if (config_.PCL_NDT)
        {
            ndt_ptr_->setInputTarget(local_map_ptr_);
        }
        else if (config_.NDT_OMP)
        {
            register_ptr_->setInputTarget(local_map_ptr_);
        }
    }

    // update global_map
    global_map_frames_.push_back(key_frame);
    if (global_map_frames_.size() % 100 != 0)
    {
        return;
    }
    else
    {
        global_map_ptr_.reset(new CloudT());
        for (size_t i = 0; i < global_map_frames_.size(); ++i)
        {
            pcl::transformPointCloud(*global_map_frames_.at(i).cloud_data.cloud_ptr, *transformed_cloud_ptr,
                                     global_map_frames_.at(i).pose);
            *global_map_ptr_ += *transformed_cloud_ptr;
        }
    }
    has_new_global_map_ = true;
}

bool FrontEnd::GetNewLocalMap(CloudT::Ptr &local_map_ptr)
{
    if (has_new_local_map_)
    {
        display_filter_.setInputCloud(local_map_ptr_);
        display_filter_.filter(*local_map_ptr);
        return true;
    }
    return false;
}

bool FrontEnd::GetNewGlobalMap(CloudT::Ptr &global_map_ptr)
{
    if (has_new_global_map_)
    {
        display_filter_.setInputCloud(global_map_ptr_);
        display_filter_.filter(*global_map_ptr);
        return true;
    }
    return false;
}

bool FrontEnd::GetCurrentScan(CloudT::Ptr &current_scan_ptr)
{
    display_filter_.setInputCloud(result_cloud_ptr_);
    display_filter_.filter(*current_scan_ptr);
    return true;
}

// } // namespace hunters_localization