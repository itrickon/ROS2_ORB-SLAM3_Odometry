#ifndef MONO_INERTIAL_NODE_HPP
#define MONO_INERTIAL_NODE_HPP

#include <atomic>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "cv_bridge/cv_bridge.h"
#include <opencv2/imgproc.hpp>

#include <sophus/se3.hpp>

#include "System.h"
#include "utility.hpp"

class MonocularInertialNode : public rclcpp::Node
{
public:
    explicit MonocularInertialNode(
        ORB_SLAM3::System *SLAM);

    ~MonocularInertialNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using ImuMsg   = sensor_msgs::msg::Imu;

    void GrabImage(
        const ImageMsg::SharedPtr msg);

    void GrabImu(
        const ImuMsg::SharedPtr msg);

    void SyncWithImu();

    ORB_SLAM3::System *SLAM_;

    rclcpp::Subscription<ImageMsg>::SharedPtr image_sub_;
    rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
        odom_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster>
        tf_broadcaster_;

    std::queue<ImageMsg::SharedPtr> image_buf_;
    std::queue<ImuMsg::SharedPtr> imu_buf_;

    std::mutex image_mutex_;
    std::mutex imu_mutex_;

    std::thread sync_thread_;
    std::atomic<bool> running_{true};

    Sophus::SE3f prev_pose_;
    Sophus::SE3f accumulated_pose_;

    bool has_prev_pose_ = false;
    
    bool prev_tracking_ok_ = false; // Вставка
    
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8)); // Вставка
};

#endif
