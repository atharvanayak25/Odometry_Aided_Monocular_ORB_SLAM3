#include "monocular-slam-node.hpp"
#include <opencv2/core/core.hpp>
#include <cmath>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
    : Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    
    // Initialize odometry buffer
    m_odom_buffer = std::make_shared<OdometryBuffer>();

    // Image subscriber
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, _1));

    // Odometry subscriber
    m_odom_subscriber = this->create_subscription<OdomMsg>(
        "/odom",
        10,
        std::bind(&MonocularSlamNode::GrabOdom, this, _1));

    // Camera-to-base transform for TurtleBot4 + OAK-D Lite
    // Camera optical frame: Z forward, X right, Y down
    // Robot base frame: X forward, Y left, Z up
    m_T_base_cam = Eigen::Matrix4f::Identity();
    
    // Rotation from camera optical frame to base_link
    m_T_base_cam(0,0) =  0.0f;  m_T_base_cam(0,1) =  0.0f;  m_T_base_cam(0,2) =  1.0f;
    m_T_base_cam(1,0) = -1.0f;  m_T_base_cam(1,1) =  0.0f;  m_T_base_cam(1,2) =  0.0f;
    m_T_base_cam(2,0) =  0.0f;  m_T_base_cam(2,1) = -1.0f;  m_T_base_cam(2,2) =  0.0f;
    
    // Translation: camera position in base frame (meters)
    m_T_base_cam(0,3) = 0.06f;   // 6cm forward
    m_T_base_cam(1,3) = 0.0f;    // centered
    m_T_base_cam(2,3) = 0.15f;   // 15cm up

    // Compute inverse
    m_T_cam_base = Eigen::Matrix4f::Identity();
    m_T_cam_base.block<3,3>(0,0) = m_T_base_cam.block<3,3>(0,0).transpose();
    m_T_cam_base.block<3,1>(0,3) = -m_T_cam_base.block<3,3>(0,0) * m_T_base_cam.block<3,1>(0,3);

    std::cout << "===================================" << std::endl;
    std::cout << "Odometry-Aided Mono SLAM Node" << std::endl;
    std::cout << "Image topic: camera" << std::endl;
    std::cout << "Odom topic:  /odom" << std::endl;
    std::cout << "===================================" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{
    m_SLAM->Shutdown();
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    m_SLAM->SaveTrajectoryTUM("CameraTrajectory.txt");
    
    std::cout << "Final scale estimate: " << m_scale_estimate << std::endl;
}

void MonocularSlamNode::GrabOdom(const OdomMsg::SharedPtr msg)
{
    m_odom_buffer->addMeasurement(msg);
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Convert ROS image to OpenCV
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    double timestamp = Utility::StampToSec(msg->header.stamp);
    
    // Get odometry delta if available
    Eigen::Matrix4f odom_delta_base = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f odom_delta_cam = Eigen::Matrix4f::Identity();
    float odom_trans = 0.0f;
    float odom_rot = 0.0f;
    bool has_odom = false;

    if (m_last_timestamp > 0 && m_odom_buffer->size() > 2) {
        has_odom = m_odom_buffer->getDeltaPose(
            m_last_timestamp, timestamp, 
            odom_delta_base, odom_trans, odom_rot);
        
        if (has_odom) {
            // Transform to camera frame
            odom_delta_cam = m_T_cam_base * odom_delta_base * m_T_base_cam;
        }
    }

    // Run SLAM tracking
    Sophus::SE3f current_pose = m_SLAM->TrackMonocular(m_cvImPtr->image, timestamp);
    
    // Check if tracking succeeded
    bool tracking_ok = (current_pose.translation().norm() > 1e-6 || 
                        current_pose.so3().log().norm() > 1e-6);

    // Update scale estimate using odometry
    if (has_odom && tracking_ok && m_last_pose_valid && odom_trans > 0.005f) {
        Sophus::SE3f delta_visual = m_last_pose.inverse() * current_pose;
        float visual_trans = delta_visual.translation().norm();
        
        if (visual_trans > 0.005f) {
            UpdateScaleEstimate(odom_trans, visual_trans);
        }
    }

    // Store state for next frame
    m_last_timestamp = timestamp;
    m_last_pose = current_pose;
    m_last_pose_valid = tracking_ok;
    m_frame_count++;

    // Log every 30 frames
    if (m_frame_count % 30 == 0) {
        std::cout << "Frame " << m_frame_count 
                  << " | Scale: " << m_scale_estimate 
                  << " (" << (m_scale_initialized ? "OK" : "init") << ")"
                  << " | Odom: " << (has_odom ? "YES" : "NO")
                  << " | Track: " << (tracking_ok ? "OK" : "LOST")
                  << std::endl;
    }
}

void MonocularSlamNode::UpdateScaleEstimate(float odom_trans, float visual_trans)
{
    // Safety checks for NaN/Inf
    if (std::isnan(odom_trans) || std::isnan(visual_trans) ||
        std::isinf(odom_trans) || std::isinf(visual_trans)) {
        return;
    }
    
    // Prevent division by zero
    if (visual_trans < 0.001f) {
        return;
    }
    
    float frame_scale = odom_trans / visual_trans;
    
    // Reject outliers and invalid values
    if (std::isnan(frame_scale) || std::isinf(frame_scale)) {
        return;
    }
    if (frame_scale < 0.01f || frame_scale > 100.0f) {
        return;
    }

    if (!m_scale_initialized) {
        m_scale_accumulator += frame_scale;
        m_scale_observations++;
        
        if (m_scale_observations >= 10) {
            m_scale_estimate = m_scale_accumulator / m_scale_observations;
            m_scale_initialized = true;
            std::cout << "*** Scale initialized: " << m_scale_estimate 
                      << " (from " << m_scale_observations << " obs) ***" << std::endl;
        }
    } else {
        float diff = std::abs(frame_scale - m_scale_estimate) / m_scale_estimate;
        if (diff < 0.5f) {
            float alpha = 0.02f;
            m_scale_estimate = (1.0f - alpha) * m_scale_estimate + alpha * frame_scale;
        }
    }
}
