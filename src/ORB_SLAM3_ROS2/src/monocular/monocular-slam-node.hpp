#ifndef MONOCULAR_SLAM_NODE_HPP
#define MONOCULAR_SLAM_NODE_HPP

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "cv_bridge/cv_bridge.h"

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"
#include "odometry_buffer.hpp"

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System* pSLAM);
    ~MonocularSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using OdomMsg = nav_msgs::msg::Odometry;

    void GrabImage(const ImageMsg::SharedPtr msg);
    void GrabOdom(const OdomMsg::SharedPtr msg);
    void UpdateScaleEstimate(float odom_trans, float visual_trans);

    ORB_SLAM3::System* m_SLAM;
    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<ImageMsg>::SharedPtr m_image_subscriber;
    rclcpp::Subscription<OdomMsg>::SharedPtr m_odom_subscriber;

    // Odometry buffer
    std::shared_ptr<OdometryBuffer> m_odom_buffer;

    // Camera-base transform
    Eigen::Matrix4f m_T_base_cam;
    Eigen::Matrix4f m_T_cam_base;

    // Tracking state
    double m_last_timestamp = -1.0;
    Sophus::SE3f m_last_pose;
    bool m_last_pose_valid = false;

    // Scale estimation
    float m_scale_estimate = 1.0f;
    bool m_scale_initialized = false;
    int m_scale_observations = 0;
    float m_scale_accumulator = 0.0f;

    // Frame counter
    int m_frame_count = 0;
};

#endif

