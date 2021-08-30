/*
 * @Description: 订阅激光点云信息，并解析数据
 * @Author: Xiangmy
 * @Date: 2021-08-27
 */

#include "lidar_localization/subscriber/cloud_subscriber.hpp"

#include "glog/logging.h"

namespace lidar_localization
{
    CloudSubscriber::CloudSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size)
        : nh_(nh)
    {
        subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);
    }

    // 接收数据
    void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr)
    {
        CloudData cloud_data;
        cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
        pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));

        new_cloud_data_.push_back(cloud_data);
    }

    // 解析数据
    void CloudSubscriber::ParseData(std::deque<CloudData> &cloud_data_buff)
    {
        if (new_cloud_data_.size() > 0)
        {
            cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
            new_cloud_data_.clear();
        }
    }
}