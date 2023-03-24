#include "front_end.h"

// namespace hunters_localization
// {
FrontEnd::FrontEnd()
    : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()),
      local_map_ptr_(new CloudData::CLOUD()), global_map_ptr_(new CloudData::CLOUD()),
      result_cloud_ptr_(new CloudData::CLOUD())
{
    // TODO: yaml file
    cloud_filter_.setLeafSize(1.3, 1.3, 1.3);
    local_map_filter_.setLeafSize(0.6, 0.6, 0.6);
    display_filter_.setLeafSize(0.5, 0.5, 0.5);
    ndt_ptr_->setResolution(1.0);
    ndt_ptr_->setStepSize(0.1);
    ndt_ptr_->setTransformationEpsilon(0.01);
    ndt_ptr_->setMaximumIterations(30);
}

Eigen::Matrix4f FrontEnd::Update(const CloudData &cloud_data)
{
    current_frame_.cloud_data.time = cloud_data.time;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, indices);

    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
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
    ndt_ptr_->setInputSource(filtered_cloud_ptr);
    ndt_ptr_->align(*result_cloud_ptr_, predict_pose);
    current_frame_.pose = ndt_ptr_->getFinalTransformation();

    // Update relative motion between current_frame and last_frame
    step_pose = last_pose.inverse() * current_frame_.pose;
    predict_pose = current_frame_.pose * step_pose;
    last_pose = current_frame_.pose;

    // Determine if current_frame is a new key_frame, depends on the matching distance
    // TODO: yaml file
    if (fabs(last_key_frame_pose(0, 3) - current_frame_.pose(0, 3)) +
            fabs(last_key_frame_pose(1, 3) - current_frame_.pose(1, 3)) +
            fabs(last_key_frame_pose(2, 3) - current_frame_.pose(2, 3)) >
        2.0)
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

    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());

    // update local_map
    local_map_frames_.push_back(key_frame);
    // TODO: yaml file
    while (local_map_frames_.size() > 20)
    {
        local_map_frames_.pop_front();
    }
    local_map_ptr_.reset(new CloudData::CLOUD());
    for (size_t i = 0; i < local_map_frames_.size(); ++i)
    {
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr, *transformed_cloud_ptr,
                                 local_map_frames_.at(i).pose);
        *local_map_ptr_ += *transformed_cloud_ptr;
    }
    has_new_local_map_ = true;

    // update target cloud matched through ndt
    if (local_map_frames_.size() < 10)
    {
        ndt_ptr_->setInputTarget(local_map_ptr_);
    }
    else
    {
        CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
        local_map_filter_.setInputCloud(local_map_ptr_);
        local_map_filter_.filter(*filtered_local_map_ptr);
        ndt_ptr_->setInputTarget(filtered_local_map_ptr);
    }

    // update global_map
    global_map_frames_.push_back(key_frame);
    if (global_map_frames_.size() % 100 != 0)
    {
        return;
    }
    else
    {
        global_map_ptr_.reset(new CloudData::CLOUD());
        for (size_t i = 0; i < global_map_frames_.size(); ++i)
        {
            pcl::transformPointCloud(*global_map_frames_.at(i).cloud_data.cloud_ptr, *transformed_cloud_ptr,
                                     global_map_frames_.at(i).pose);
            *global_map_ptr_ += *transformed_cloud_ptr;
        }
    }
    has_new_global_map_ = true;
}

bool FrontEnd::GetNewLocalMap(CloudData::CLOUD_PTR &local_map_ptr)
{
    if (has_new_local_map_)
    {
        display_filter_.setInputCloud(local_map_ptr_);
        display_filter_.filter(*local_map_ptr);
        return true;
    }
    return false;
}

bool FrontEnd::GetNewGlobalMap(CloudData::CLOUD_PTR &global_map_ptr)
{
    if (has_new_global_map_)
    {
        display_filter_.setInputCloud(global_map_ptr_);
        display_filter_.filter(*global_map_ptr);
        return true;
    }
    return false;
}

bool FrontEnd::GetCurrentScan(CloudData::CLOUD_PTR &current_scan_ptr)
{
    display_filter_.setInputCloud(result_cloud_ptr_);
    display_filter_.filter(*current_scan_ptr);
    return true;
}

// } // namespace hunters_localization