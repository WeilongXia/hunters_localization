#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

#include "front_end.h"

std::deque<CloudData> cloud_data_buff;

bool front_end_pose_inited = false;
bool time_inited = false;
double init_time = 0.0;
double run_time = 0.0;

CloudT::Ptr local_map_ptr(new CloudT());
CloudT::Ptr global_map_ptr(new CloudT());
CloudT::Ptr current_scan_ptr(new CloudT());

void CloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr)
{
    CloudData cloud_data;
    cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));

    cloud_data_buff.push_back(cloud_data);
    ROS_INFO("cloud_data_buff size is %d", cloud_data_buff.size());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "front_end");
    ros::NodeHandle nh;

    FrontEnd front_end(nh);

    ros::Subscriber cloud_sub = nh.subscribe(front_end.config_.lidar_topic_name, 20, CloudCallback);

    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/current_scan", 20);
    ros::Publisher local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/local_map", 20);
    ros::Publisher global_map_pub = nh.advertise<sensor_msgs::PointCloud2>("global_map", 20);

    ros::Publisher lidar_odom_pub = nh.advertise<nav_msgs::Odometry>("/lidar_odom", 20);

    while (ros::ok())
    {
        ros::spinOnce();

        while (cloud_data_buff.size() > 0)
        {
            CloudData cloud_data = cloud_data_buff.front();
            cloud_data_buff.pop_front();

            if (!time_inited)
            {
                time_inited = true;
                init_time = cloud_data.time;
            }
            else
            {
                run_time = cloud_data.time - init_time;
            }

            Eigen::Matrix4f init_odom_matrix = Eigen::Matrix4f::Identity();
            if (!front_end_pose_inited)
            {
                front_end_pose_inited = true;
                front_end.SetInitPose(init_odom_matrix);
            }
            // front_end.SetPredictPose(init_odom_matrix);
            Eigen::Matrix4f lidar_odom_matrix = front_end.Update(cloud_data);
            ROS_INFO("position: %f %f %f", lidar_odom_matrix(0, 3), lidar_odom_matrix(1, 3), lidar_odom_matrix(2, 3));
            // 发布Lidar里程计消息
            // 封装数据转换
            nav_msgs::Odometry lidar_odom;

            lidar_odom.header.stamp = ros::Time::now();
            lidar_odom.header.frame_id = "map";

            lidar_odom.pose.pose.position.x = lidar_odom_matrix(0, 3);
            lidar_odom.pose.pose.position.y = lidar_odom_matrix(1, 3);
            lidar_odom.pose.pose.position.z = lidar_odom_matrix(2, 3);

            Eigen::Quaternionf q;
            q = lidar_odom_matrix.block<3, 3>(0, 0);
            lidar_odom.pose.pose.orientation.x = q.x();
            lidar_odom.pose.pose.orientation.y = q.y();
            lidar_odom.pose.pose.orientation.z = q.z();
            lidar_odom.pose.pose.orientation.w = q.w();

            lidar_odom_pub.publish(lidar_odom);

            // 发布TF消息
            // 封装数据转换
            static tf::TransformBroadcaster br;

            tf::Transform transform;
            transform.setOrigin(tf::Vector3(lidar_odom.pose.pose.position.x, lidar_odom.pose.pose.position.y,
                                            lidar_odom.pose.pose.position.z));
            tf::Quaternion tf_quaternion(q.x(), q.y(), q.z(), q.w());
            transform.setRotation(tf_quaternion);

            br.sendTransform(tf::StampedTransform(transform, lidar_odom.header.stamp, "map", "lidar_link"));

            front_end.GetCurrentScan(current_scan_ptr);
            // 发布当前帧点云
            // 封装数据转换
            sensor_msgs::PointCloud2Ptr cloud_ptr_current(new sensor_msgs::PointCloud2());
            pcl::toROSMsg(*current_scan_ptr, *cloud_ptr_current);
            cloud_ptr_current->header.stamp = ros::Time::now();
            cloud_ptr_current->header.frame_id = "map";
            cloud_pub.publish(*cloud_ptr_current);

            if (front_end.GetNewLocalMap(local_map_ptr))
            {
                sensor_msgs::PointCloud2Ptr cloud_ptr_local(new sensor_msgs::PointCloud2());
                pcl::toROSMsg(*local_map_ptr, *cloud_ptr_local);
                cloud_ptr_local->header.stamp = ros::Time::now();
                cloud_ptr_local->header.frame_id = "map";
                local_map_pub.publish(*cloud_ptr_local);
            }

            if (run_time > 60.0)
            {
                if (front_end.GetNewGlobalMap(global_map_ptr))
                {
                    sensor_msgs::PointCloud2Ptr cloud_ptr_global(new sensor_msgs::PointCloud2());
                    pcl::toROSMsg(*global_map_ptr, *cloud_ptr_global);
                    cloud_ptr_global->header.stamp = ros::Time::now();
                    cloud_ptr_global->header.frame_id = "map";
                    global_map_pub.publish(*cloud_ptr_global);
                }
            }
        }
    }

    return 0;
}