#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include "front_end.h"

std::deque<CloudData> cloud_data_buff;

bool front_end_pose_inited = false;

CloudData::CLOUD_PTR local_map_ptr(new CloudData::CLOUD());
CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
CloudData::CLOUD_PTR current_scan_ptr(new CloudData::CLOUD());

ros::Subscriber<sensor_msgs::PointCloud2::ConstPtr> cloud_sub;

void CloudCallback(const sensor_msgs::PointCloud::ConstPtr &cloud_msg_ptr)
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "front_end");
    ros::NodeHandle nh;

    std::shared_ptr<FrontEnd> front_end_ptr = std::make_shared<FrontEnd>();
}